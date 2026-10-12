"""pytest: protocol round trip, kinematics consistency, client <-> simulated ESP32 (no ROS needed)."""
import math
import pathlib
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from wra_driver import protocol as pr  # noqa: E402
from wra_driver import abenics_kin as AK  # noqa: E402
from wra_driver.arm_model import ArmModel, ServoMap  # noqa: E402

CAL = yaml.safe_load((pathlib.Path(__file__).resolve().parents[1] / "config" / "calibration.yaml").read_text())


def test_crc_and_framing():
    assert pr.crc16(b"123456789") == 0x29B1                    # CRC-16/CCITT-FALSE check value
    f = pr.encode(pr.GOALS, 3, pr.p_goals([(1, 2048, 500, 10)]))
    d = pr.Decoder()
    assert d.feed(b"\x00\xA5" + f[:4]) == []
    out = d.feed(f[4:] + f)
    assert [o[0] for o in out] == [pr.GOALS, pr.GOALS]


def test_sts_packet_checksum():
    pkt = pr.sts_packet(1, 0x02, bytes([56, 2]))
    assert pkt == bytes.fromhex("ffff0104023802be")


def test_abenics_ik_fk():
    assert AK.self_test() < 1e-9
    for pyr in ((-28, 0, 0), (10, -15, 60), (0, 20, -80)):
        R = AK.tool_rotation_pyr(*pyr)
        assert np.allclose(AK.tool_pyr(R), pyr)
        assert np.allclose(AK.fk(*AK.ik(R)), R)


def test_arm_ik_roundtrip():
    m = ArmModel()
    rng = np.random.default_rng(0)
    n = 0
    for _ in range(300):
        th2 = rng.uniform(-25, 95)
        q = (rng.uniform(-100, 100), th2, th2 - rng.uniform(45, 135),
             (rng.uniform(-20, 20), rng.uniform(-15, 15), rng.uniform(-80, 80)))
        if not m.check(*q)[0]:
            continue
        p, R = m.tcp(*q)
        q2 = m.ik(p[:3], R)
        p2, R2 = m.tcp(*q2)
        assert np.allclose(p2, p, atol=1e-6) and np.allclose(R2, R, atol=1e-9)
        n += 1
    assert n > 50


def test_actuator_mapping_roundtrip():
    m = ArmModel()
    sm = ServoMap(CAL)
    q = (12.0, 55.0, -20.0, (-12.0, 8.0, 40.0), 15.0)
    act = m.actuators(*q)
    counts = {s: sm.to_counts(s, d) for s, d in act.items()}
    assert all(0 <= c <= 4095 for c in counts.values())
    back = m.joints_from_actuators({s: sm.to_deg(s, c) for s, c in counts.items()})
    assert abs(back[0] - q[0]) < 0.1 and abs(back[1] - q[1]) < 0.1 and abs(back[4] - q[4]) < 0.05
    assert np.allclose(back[3], q[3], atol=0.3)


def test_client_with_fake_esp32():
    from wra_driver.fake_esp32 import FakeEsp32, FakeSerial
    from wra_driver.client import WraClient
    m = ArmModel()
    sm = ServoMap(CAL)
    h = m.g["home"]
    act = m.actuators(h["j1"], h["j2"], h["j3"], tuple(h["wt"]), 8.0)
    esp = FakeEsp32(centers={s: sm.to_counts(s, d) for s, d in act.items()})
    c = WraClient("fake", calibration=CAL, serial_obj=FakeSerial(esp), max_joint_speed_dps=120)
    try:
        assert c.wait_state().state == "SAFE_OFF"
        assert c.arm() == "OK"
        time.sleep(0.7)
        assert c.tel.state == "READY"
        assert c.arm() == "OK"
        time.sleep(0.1)
        assert c.tel.state == "ACTIVE"
        assert c.move_joints((10.0, 60.0, -15.0, (-10.0, 5.0, 20.0), 20.0))
        time.sleep(0.5)
        q = c.joints()
        assert abs(q[0] - 10) < 0.3 and abs(q[1] - 60) < 0.3 and abs(q[4] - 20) < 0.3
        esp.press_estop()
        time.sleep(0.1)
        assert c.tel.state == "FAULT" and "ESTOP_OPEN" in c.tel.faults
    finally:
        c.close()


def _fake_client():
    from wra_driver.fake_esp32 import FakeEsp32, FakeSerial
    from wra_driver.client import WraClient
    m = ArmModel()
    sm = ServoMap(CAL)
    h = m.g["home"]
    act = m.actuators(h["j1"], h["j2"], h["j3"], tuple(h["wt"]), 8.0)
    esp = FakeEsp32(centers={s: sm.to_counts(s, d) for s, d in act.items()})
    c = WraClient("fake", calibration=CAL, serial_obj=FakeSerial(esp), max_joint_speed_dps=120)
    assert c.wait_state().state == "SAFE_OFF"
    assert c.arm() == "OK"
    time.sleep(0.7)
    assert c.arm() == "OK"
    time.sleep(0.1)
    assert c.tel.state == "ACTIVE"
    return c, esp, sm


def _knob_frame():
    import numpy as np
    xk = np.array([-1.0, 0.0, 0.0])          # knob axis points out of the door, toward the chair
    zk = np.array([0.0, 0.0, 1.0])
    return np.column_stack([xk, np.cross(zk, xk), zk])


def _run_door(stiff):
    import numpy as np
    from wra_driver.tasks import DoorKnobTask, KnobParams
    c, esp, sm = _fake_client()
    log = []
    knob_d = 56.0

    def on_log(msg):
        log.append(msg)
        if msg.startswith("3/7"):              # the claw stalls on the knob
            cl = max(esp.pos)                  # claw servo = highest id
            q_knob = sm.to_counts(cl, c.model.actuators(*c.joints()[:4], knob_d / 2)[cl])
            esp.block[cl] = (q_knob - 2, q_knob + 4096) if q_knob < esp.pos[cl] else (q_knob - 4096, q_knob + 2)
        if stiff and msg.startswith("4/7"):    # latch too stiff: the Theta servos cannot move
            for sid in (5, 7):
                esp.block[sid] = (esp.pos[sid] - 3, esp.pos[sid] + 3)
    try:
        ok = DoorKnobTask(c, log=on_log).run(np.array([470.0, 0.0, 400.0]), _knob_frame(),
                                             KnobParams(knob_d=knob_d, standoff=80.0))
    finally:
        c.close()
    return ok, log


def test_door_task_fake():
    ok, log = _run_door(stiff=False)
    assert ok, log
    assert any(m.startswith("   turned to within") for m in log)


def test_door_task_stiff_knob_aborts():
    ok, log = _run_door(stiff=True)
    assert not ok
    assert any("did not turn far enough" in m for m in log), log


def test_theta_torque_cap_only_in_knob_mode():
    c, esp, sm = _fake_client()
    try:
        assert c.torque_limits({5: 300, 7: 300}) == "CLAMPED"      # outside knob mode: 25 % max
        assert esp.tlim[5] == 250 and esp.tlim[7] == 250
        assert c.knob_mode(True) == "OK"
        assert c.torque_limits({5: 300, 7: 300}) == "OK"           # knob mode: 30 %
        assert c.torque_limits({5: 500}) == "CLAMPED" and esp.tlim[5] == 300
        assert c.knob_mode(False) == "OK"
        assert esp.tlim[5] == 250 and esp.tlim[7] == 250            # back to continuous
    finally:
        c.close()


def test_user_keepout_zone():
    """docs/09 T5.1: with the side set, poses that put the arm into the user's head/torso box are refused."""
    free, right = ArmModel(), ArmModel(side="right")
    q = (104.0, 40.0, -10.0, (0.0, 0.0, 0.0))          # arm swung back toward the user's torso
    assert free.check(*q)[0]
    ok, why = right.check(*q)
    assert not ok and "keep-out" in why, why
    h = right.g["home"]
    assert right.check(h["j1"], h["j2"], h["j3"], tuple(h["wt"]))[0]   # park pose is clear
