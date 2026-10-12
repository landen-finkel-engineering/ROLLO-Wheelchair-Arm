#!/usr/bin/env python3
"""WRA-1 command line (no ROS needed) - bench tests, teaching, demos.

  python3 wra_cli.py --port /dev/ttyUSB0 status
  python3 wra_cli.py --port /dev/ttyUSB0 arm            # 1st: ARMING (press START), 2nd: torque on
  python3 wra_cli.py --port /dev/ttyUSB0 move 0 60 -10 0 0 0 10   # th1 th2 th3 pitch yaw roll claw_half_mm
  python3 wra_cli.py --port /dev/ttyUSB0 home | off | power-down | clear | claw 40
  python3 wra_cli.py --port /dev/ttyUSB0 teach-knob     # torque OFF, guide the claw onto the knob by hand
  python3 wra_cli.py --port /dev/ttyUSB0 door           # runs the door task on the taught knob
  python3 wra_cli.py --fake ...                         # same, against the simulated ESP32
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "wra_ros2" / "wra_driver"))
from wra_driver.client import WraClient, load_calibration  # noqa: E402
from wra_driver.tasks import DoorKnobTask, KnobParams  # noqa: E402

KNOB_FILE = HERE / "taught_knob.json"


def fmt(q):
    th1, th2, th3, wrist, claw = q
    return (f"th1 {th1:7.2f}  th2 {th2:7.2f}  th3 {th3:7.2f}  wrist {tuple(round(w, 1) for w in wrist)}  "
            f"claw {2 * claw:5.1f} mm")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--fake", action="store_true", help="use the simulated ESP32")
    ap.add_argument("--speed", type=float, default=30.0, help="max joint speed deg/s (default 30)")
    ap.add_argument("--no-keepout", action="store_true",
                    help="disable the user keep-out zone: BENCH STAND ONLY, never with anyone in the chair")
    ap.add_argument("cmd", choices=["status", "arm", "off", "power-down", "clear", "home", "move", "claw",
                                    "teach-knob", "door", "watch"])
    ap.add_argument("vals", nargs="*", type=float)
    a = ap.parse_args()
    cal = load_calibration()
    ser = None
    if a.fake:
        from wra_driver.fake_esp32 import FakeEsp32, FakeSerial
        from wra_driver.arm_model import ArmModel, ServoMap
        m, sm = ArmModel(), ServoMap(cal)
        h = m.g["home"]
        act = m.actuators(h["j1"], h["j2"], h["j3"], tuple(h["wt"]) if m.abenics else (h["j4"], h["j5"]), 8.0)
        esp = FakeEsp32(centers={s: sm.to_counts(s, d) for s, d in act.items()})
        ser = FakeSerial(esp)
    c = WraClient(a.port, calibration=cal, serial_obj=ser, max_joint_speed_dps=a.speed, keepout=not a.no_keepout)
    t = c.wait_state(2.0)
    if t is None:
        raise SystemExit("no telemetry from the ESP32 - check the USB cable / port / firmware")
    if a.fake:                                   # convenience: bring the simulator to ACTIVE
        c.arm(); time.sleep(0.7); c.arm(); time.sleep(0.1)
    m = c.model
    try:
        if a.cmd == "status":
            t = c.tel
            print(json.dumps({"state": t.state, "faults": t.faults, "estop_ok": t.estop_ok, "docked": t.docked,
                              "bus_V": t.bus_V, "calibrated": t.calibrated,
                              "geometry_params_sha": m.g.get("params_sha", "missing"),
                              "servos": {s.id: dict(online=s.online, pos=s.pos, temp=s.temp, volt=s.volt, load=s.load)
                                         for s in t.servos.values()}}, indent=1))
            q = c.joints()
            print(fmt(q) if q else "joints: servos offline")
        elif a.cmd == "arm":
            print(c.arm(), "->", c.wait_state().state, "(ARMING: press START on the pendant now)")
        elif a.cmd == "off":
            print(c.torque_off())
        elif a.cmd == "power-down":
            print(c.power_down())
        elif a.cmd == "clear":
            print(c.clear_faults(), c.tel.faults)
        elif a.cmd == "home":
            h = m.g["home"]
            print(c.move_joints((h["j1"], h["j2"], h["j3"], tuple(h["wt"]) if m.abenics else (h["j4"], h["j5"]), 8.0)))
        elif a.cmd == "move":
            v = a.vals
            nw = 3 if m.abenics else 2
            tgt = (v[0], v[1], v[2], tuple(v[3:3 + nw]), v[3 + nw])
            print(c.move_joints(tgt))
            print(fmt(c.joints()))
        elif a.cmd == "claw":
            q = c.joints()
            print(c.move_joints((*q[:4], max(0.0, min(65.0, a.vals[0])) / 2)))
        elif a.cmd == "teach-knob":
            print("Torque OFF.  Guide the open claw onto the knob (pads at the knob's widest part, tool axis on the "
                  "knob axis, jaws horizontal).  Press Enter when it sits right.")
            c.torque_off()
            input()
            q = c.joints()
            p, R = m.tcp(*q[:4])
            grip_depth = KnobParams().grip_depth
            xk = -R[:, 0]                                  # knob x axis = out of the door
            pK = p[:3] + xk * grip_depth
            RK = np.column_stack([xk, np.cross(R[:, 2], xk), R[:, 2]])
            KNOB_FILE.write_text(json.dumps({"p": pK.tolist(), "R": RK.tolist(), "joints": fmt(q)}, indent=1))
            print("saved", KNOB_FILE)
        elif a.cmd == "door":
            k = json.loads(KNOB_FILE.read_text())
            ok = DoorKnobTask(c).run(np.array(k["p"]), np.array(k["R"]), KnobParams())
            print("door task", "OK" if ok else "aborted")
        elif a.cmd == "watch":
            while True:
                q = c.joints()
                print(c.tel.state, c.tel.faults, fmt(q) if q else "", end="\r", flush=True)
                time.sleep(0.2)
    finally:
        c.close()


if __name__ == "__main__":
    main()
