#!/usr/bin/env python3
"""WRA-1 servo setup: scan, assign IDs, calibrate zero/direction/limits, write the ESP32 limit table.

Transport (pick one):
  --adapter /dev/ttyUSB0   Waveshare Bus Servo Adapter (A) in USB mode (jumper to USB), 1 Mbps.
                           Use this to give each NEW servo its ID (connect ONE servo at a time).
  --esp32 /dev/ttyUSB0     through the ESP32 supervisor (normal wiring, adapter jumper on UART):
                           the tool puts the ESP32 in MAINT mode (all torque off) and tunnels packets.
Commands:
  scan                     ping IDs 1..20
  set-id OLD NEW           change a servo's ID (factory ID is 1)
  read ID                  position / voltage / temperature / limits
  calibrate [--only IDS]   interactive: pose each joint as instructed, writes servo EEPROM limits,
                           the ESP32 limit table (MAINT/SET_LIMIT) and config/calibration.yaml
Step-by-step instructions: docs/11_software_setup.md.
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import struct
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
DRV = HERE.parent / "wra_ros2" / "wra_driver"
sys.path.insert(0, str(DRV))
from wra_driver import protocol as pr  # noqa: E402
from wra_driver.arm_model import ArmModel  # noqa: E402

K = 4096.0 / 360.0
REG = dict(ID=5, MIN_ANGLE=9, MAX_ANGLE=11, MAX_TORQUE=16, HOMING_OFFSET=31, TORQUE_ENABLE=40, LOCK=55,
           PRESENT_POS=56, PRESENT_VOLT=62, PRESENT_TEMP=63)

# calibration pose per servo: (instruction, joint value at that pose, how to move "+")
POSES_1A = {
    1: ("J1 yaw: arm pointing straight FORWARD (use the printed pointer mark on the turntable)", 0.0,
        "turn the turntable a little to the LEFT (counter-clockwise seen from above)"),
    2: ("J2 shoulder: upper arm exactly VERTICAL (spirit level on the upper-arm tubes)", 90.0,
        "tilt the upper arm a little BACKWARD (toward the chair back)"),
    3: ("J3 forearm: forearm tubes exactly HORIZONTAL (spirit level)", 0.0,
        "lift the wrist end of the forearm a little (UP)"),
    4: ("Wrist module 1 (up-left): carrier mark aligned with the 'q0' mark on the module frame "
        "(monopole spin axis parallel to the tool axis)", 0.0, "turn the carrier a little counter-clockwise seen from its q-servo end"),
    5: ("Wrist module 1: monopole 'T0' mark pointing at the sphere centre (tool aligned with the forearm)", 0.0,
        "turn the Theta-servo gear a little so the monopole rolls its T0 mark toward the tool flange"),
    6: ("Wrist module 2 (up-right): carrier mark aligned with 'q0'", 0.0,
        "turn the carrier a little counter-clockwise seen from its q-servo end"),
    7: ("Wrist module 2: monopole 'T0' mark pointing at the sphere centre", 0.0,
        "turn the Theta-servo gear so the monopole rolls its T0 mark toward the tool flange"),
    8: ("Claw: jaws CLOSED, pads touching", 0.0, "open the jaws a few mm by hand"),
}
POSES_1B = {k: POSES_1A[k] for k in (1, 2, 3, 8)}
POSES_1B[4] = ("J4 pitch: tool flange aligned with the forearm (pitch 0)", 0.0, "tilt the tool a little UP")
POSES_1B[5] = ("J5 roll: dovetail vertical, lock screw on top (roll 0)", 0.0, "roll the tool a little counter-clockwise seen from the back")


# ------------------------------------------------------------------ transports
class AdapterBus:
    def __init__(self, port):
        import serial
        self.s = serial.Serial(port, 1000000, timeout=0.02)

    def txrx(self, pkt, expect=True):
        self.s.reset_input_buffer()
        self.s.write(pkt)
        if not expect:
            return b""
        time.sleep(0.004)
        return self.s.read(64)

    def close(self):
        self.s.close()


class Esp32Bus:
    def __init__(self, port):
        from wra_driver.client import WraClient
        import yaml
        cal = yaml.safe_load((DRV / "config" / "calibration.yaml").read_text())
        self.c = WraClient(port, calibration=cal)
        self.c.wait_state()
        code = self.c.wait_ack(pr.MAINT, self.c._send(pr.MAINT, bytes([1])))
        if code != "OK":
            raise SystemExit(f"ESP32 refused MAINT ({code}); state {self.c.tel.state if self.c.tel else '?'}: "
                             "torque must be off and the arm READY (powered) or SAFE_OFF")

    def txrx(self, pkt, expect=True):
        """Tunnel one raw STS packet through the ESP32 (MAINT mode); returns the raw reply bytes."""
        return self.c.passthru(pkt) if expect else (self.c.passthru(pkt) or b"")

    def close(self):
        self.c.wait_ack(pr.MAINT, self.c._send(pr.MAINT, bytes([0])))
        self.c.close()


# ------------------------------------------------------------------ STS helpers
def ping(bus, sid):
    return pr.sts_parse_reply(bus.txrx(pr.sts_packet(sid, 0x01))) is not None


def read(bus, sid, addr, n):
    r = pr.sts_parse_reply(bus.txrx(pr.sts_packet(sid, 0x02, bytes([addr, n]))))
    if r is None or r[0] != sid or len(r[2]) != n:
        raise IOError(f"no reply from servo {sid}")
    return r[2]


def write(bus, sid, addr, data: bytes):
    r = pr.sts_parse_reply(bus.txrx(pr.sts_packet(sid, 0x03, bytes([addr]) + data)))
    if r is None or r[1] != 0:
        raise IOError(f"write to servo {sid} reg {addr} failed: {r}")


def read16(bus, sid, addr):
    return struct.unpack("<H", read(bus, sid, addr, 2))[0]


def write16(bus, sid, addr, v):
    write(bus, sid, addr, struct.pack("<H", v & 0xFFFF))


def eeprom(bus, sid, fn):
    write(bus, sid, REG["LOCK"], b"\x00")
    try:
        fn()
    finally:
        write(bus, sid, REG["LOCK"], b"\x01")


def pos(bus, sid, n=5):
    v = [read16(bus, sid, REG["PRESENT_POS"]) & 0x0FFF for _ in range(n)]
    return sorted(v)[n // 2]


# ------------------------------------------------------------------ commands
def cmd_scan(bus):
    found = [i for i in range(1, 21) if ping(bus, i)]
    print("servos found:", found or "none (check power / jumper / baud)")


def cmd_set_id(bus, old, new):
    if ping(bus, new):
        raise SystemExit(f"ID {new} already present on the bus")
    eeprom(bus, old, lambda: write(bus, old, REG["ID"], bytes([new])))
    time.sleep(0.05)
    print("ok" if ping(bus, new) else "ID write not confirmed - power-cycle and scan")


def cmd_read(bus, sid):
    print(json.dumps({"id": sid, "pos": pos(bus, sid), "volt": read(bus, sid, REG["PRESENT_VOLT"], 1)[0] / 10,
                      "temp": read(bus, sid, REG["PRESENT_TEMP"], 1)[0],
                      "min_angle": read16(bus, sid, REG["MIN_ANGLE"]), "max_angle": read16(bus, sid, REG["MAX_ANGLE"]),
                      "homing_offset_raw": read16(bus, sid, REG["HOMING_OFFSET"])}))


def servo_ranges(m: ArmModel):
    """(soft_lo, soft_hi, hard_lo, hard_hi) servo angles (deg) per servo id."""
    L, r = m.lim, m.ratio
    out = {1: [r["j1"] * v for v in L["j1"][0] + L["j1"][1]], 2: [r["j2"] * v for v in L["j2"][0] + L["j2"][1]],
           3: [r["j3"] * v for v in L["j3"][0] + L["j3"][1]]}
    if m.abenics:
        from wra_driver import abenics_kin as AK
        q, th = AK.LIMITS["q"], AK.LIMITS["theta"]
        tr = m.g["theta_servo_per_theta"]
        out.update({4: [-q[0], q[0], -q[1], q[1]], 6: [-q[0], q[0], -q[1], q[1]],
                    5: [-tr * th[0], tr * th[0], -tr * th[1], tr * th[1]], 7: [-tr * th[0], tr * th[0], -tr * th[1], tr * th[1]]})
    else:
        out.update({4: [r["j4"] * v for v in L["j4"][0] + L["j4"][1]], 5: L["j5"][0] + L["j5"][1]})
    cmax = -math.degrees(L["claw_mm"][1] / m.g["claw_pinion_r"])
    out[8] = [cmax, 0.0, cmax - 3.0, 3.0]
    return {k: (min(v[0], v[1]), max(v[0], v[1]), min(v[2], v[3]), max(v[2], v[3])) for k, v in out.items()}


def cmd_calibrate(bus, only, esp=None):
    import yaml
    m = ArmModel()
    poses = POSES_1A if m.abenics else POSES_1B
    rng = servo_ranges(m)
    calp = DRV / "config" / "calibration.yaml"
    cal = yaml.safe_load(calp.read_text())
    for sid, (instr, jval, plus) in poses.items():
        if only and sid not in only:
            continue
        print(f"\n=== servo {sid}: {instr}")
        write(bus, sid, REG["TORQUE_ENABLE"], b"\x00")
        input("   put the joint in that pose (torque is OFF), then press Enter ")
        ratio = {1: m.ratio["j1"], 2: m.ratio["j2"], 3: m.ratio["j3"]}.get(sid, 1.0)
        if m.abenics and sid in (5, 7):
            ratio = m.g["theta_servo_per_theta"]
        if not m.abenics and sid == 4:
            ratio = m.ratio["j4"]
        ref_deg = ratio * jval
        p_ref = pos(bus, sid)
        input(f"   now {plus} (5-15 deg), hold it there and press Enter ")
        p2 = pos(bus, sid)
        d = ((p2 - p_ref + 2048) % 4096) - 2048
        if abs(d) < 20:
            raise SystemExit("   movement too small - repeat")
        sgn = 1 if d > 0 else -1
        if sid == 8:
            sgn = -sgn                   # claw: opening = negative servo angle by convention
        lo, hi, hlo, hhi = rng[sid]
        mid = 0.5 * (lo + hi)
        # shift the servo zero (homing offset) so that the middle of the range reads 2048
        want = 2048 + sgn * (ref_deg - mid) * K       # what p_ref should read after the shift
        shift = int(round(p_ref - want))
        if abs(shift) > 2:
            cur = read16(bus, sid, REG["HOMING_OFFSET"])
            cur_v = -(cur & 0x7FF) if cur & 0x800 else (cur & 0x7FF)
            for trial in (cur_v + shift, cur_v - shift):        # sign convention verified by reading back
                t = max(-2047, min(2047, trial))
                eeprom(bus, sid, lambda: write16(bus, sid, REG["HOMING_OFFSET"], (0x800 | -t) if t < 0 else t))
                time.sleep(0.05)
                if abs(pos(bus, sid) - (p2 - shift)) < 12:
                    break
            else:
                raise SystemExit("   could not verify the homing offset - check the servo firmware")
        center = int(round(2048 - sgn * mid * K))
        cnt = lambda deg: int(round(center + sgn * deg * K))   # noqa: E731
        s_lo, s_hi = sorted((cnt(lo), cnt(hi)))
        h_lo, h_hi = sorted((cnt(hlo), cnt(hhi)))
        if h_lo < 20 or h_hi > 4075:
            raise SystemExit(f"   range {h_lo}..{h_hi} does not fit one turn - check the ratio / pose")
        eeprom(bus, sid, lambda: (write16(bus, sid, REG["MIN_ANGLE"], h_lo), write16(bus, sid, REG["MAX_ANGLE"], h_hi),
                                  write16(bus, sid, REG["MAX_TORQUE"], 500)))
        cal["servos"][sid] = {"name": cal["servos"].get(sid, {}).get("name", str(sid)), "center": center, "dir": sgn}
        print(f"   center {center}  dir {sgn:+d}  soft {s_lo}..{s_hi}  hard {h_lo}..{h_hi}")
        if esp is not None:
            spd = 1500 if sid in (4, 5, 6, 7, 8) else 1200
            tc, tp = (250, 500) if sid == 8 else ((300, 300) if sid in (4, 6) else ((250, 300) if sid in (5, 7) else (250, 400)))
            code = esp.c.wait_ack(pr.SET_LIMIT, esp.c._send(pr.SET_LIMIT, pr.p_set_limit(sid, s_lo, s_hi, center, spd, tc, tp)))
            print("   ESP32 limit table:", code)
    cal["calibrated"] = all(sid in cal["servos"] for sid in poses)
    calp.write_text(yaml.safe_dump(cal, sort_keys=False))
    print(f"\nwrote {calp}.  Copy it to the Jetson's install space (or rebuild the workspace).")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--adapter")
    g.add_argument("--esp32")
    ap.add_argument("cmd", choices=["scan", "set-id", "read", "calibrate"])
    ap.add_argument("args", nargs="*", type=int)
    ap.add_argument("--only", nargs="*", type=int)
    a = ap.parse_args()
    bus = AdapterBus(a.adapter) if a.adapter else Esp32Bus(a.esp32)
    try:
        if a.cmd == "scan":
            cmd_scan(bus)
        elif a.cmd == "set-id":
            cmd_set_id(bus, *a.args)
        elif a.cmd == "read":
            cmd_read(bus, a.args[0])
        else:
            cmd_calibrate(bus, a.only, bus if a.esp32 else None)
    finally:
        bus.close()


if __name__ == "__main__":
    main()
