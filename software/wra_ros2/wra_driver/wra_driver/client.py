"""WraClient: everything the Jetson needs to drive WRA-1, without ROS.

* talks to the ESP32 supervisor (protocol.py) on a serial port, in a background thread;
* sends the heartbeat (50 Hz) - if this process dies, the ESP32 holds position and waits;
* converts joint targets <-> servo counts (arm_model.py + calibration.yaml);
* streams motion as small rate-limited steps (the ESP32 clamps every goal to its own limits).

The ROS 2 node (driver_node.py), the CLI (software/tools/wra_cli.py) and the task scripts all use it.
"""
from __future__ import annotations

import math
import pathlib
import threading
import time

import numpy as np

from . import protocol as pr
from .arm_model import ArmModel, ServoMap, CFG


def load_calibration(path=None) -> dict:
    import yaml
    p = pathlib.Path(path) if path else CFG / "calibration.yaml"
    return yaml.safe_load(p.read_text())


class WraClient:
    def __init__(self, port: str, calibration: dict | None = None, baud: int = 921600, model: ArmModel | None = None,
                 serial_obj=None, max_joint_speed_dps: float = 30.0, claw_speed_mms: float = 30.0,
                 side: str | None = "from_calibration", keepout: bool = True):
        """side: 'right' | 'left' (default: calibration.yaml 'side').  keepout=False disables the user
        keep-out zone - bench stand only, never with anyone in the chair."""
        self.cal = calibration or load_calibration()
        if side == "from_calibration":
            side = self.cal.get("side")
        self.model = model or ArmModel(side=side if keepout else None)
        self.smap = ServoMap(self.cal)
        if serial_obj is None:
            import serial
            serial_obj = serial.Serial(port, baud, timeout=0.01)
        self.ser = serial_obj
        self.dec = pr.Decoder()
        self.seq = 0
        self.lock = threading.Lock()
        self.tel: pr.Telemetry | None = None
        self.acks = []
        self.events = []
        self.config = None
        self._pt = None
        self._pt_evt = threading.Event()
        self.q_prev = (0.0, 0.0)
        self.max_dps = max_joint_speed_dps
        self.claw_mms = claw_speed_mms
        self._stop = threading.Event()
        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._hb = threading.Thread(target=self._hb_loop, daemon=True)
        self._rx.start()
        self._hb.start()

    # ------------------------------------------------------------------ link
    def _send(self, mtype, payload=b""):
        with self.lock:
            self.seq = (self.seq + 1) & 0xFF
            self.ser.write(pr.encode(mtype, self.seq, payload))
            return self.seq

    def _rx_loop(self):
        while not self._stop.is_set():
            data = self.ser.read(512)
            if not data:
                continue
            for mtype, seq, p in self.dec.feed(data):
                if mtype == pr.STATE:
                    self.tel = pr.parse_state(p)
                elif mtype == pr.ACK:
                    self.acks.append(pr.parse_ack(p))
                    self.acks = self.acks[-50:]
                elif mtype == pr.EVENT:
                    self.events.append((time.time(),) + pr.parse_event(p))
                    self.events = self.events[-200:]
                elif mtype == pr.CONFIG:
                    self.config = pr.parse_config(p)
                elif mtype == pr.PASSTHRU_RESP:
                    self._pt = p
                    self._pt_evt.set()

    def _hb_loop(self):
        while not self._stop.is_set():
            self._send(pr.HEARTBEAT)
            time.sleep(0.02)

    def close(self):
        self._stop.set()
        time.sleep(0.05)
        self.ser.close()

    def wait_ack(self, mtype, seq, timeout=0.5):
        t0 = time.time()
        while time.time() - t0 < timeout:
            for a in list(self.acks):
                if a[0] == mtype and a[1] == seq:
                    return a[2]
            time.sleep(0.005)
        return "TIMEOUT"

    def wait_state(self, timeout=1.0):
        t0 = time.time()
        while self.tel is None and time.time() - t0 < timeout:
            time.sleep(0.01)
        return self.tel

    # ------------------------------------------------------------------ high-level commands
    def arm(self):
        """SAFE_OFF -> ARMING (press START on the pendant) ; READY -> ACTIVE (torque on) ; HOLD -> ACTIVE."""
        return self.wait_ack(pr.ARM, self._send(pr.ARM, pr.p_arm(pr.ARM_ON)))

    def torque_off(self):
        return self.wait_ack(pr.ARM, self._send(pr.ARM, pr.p_arm(pr.ARM_TORQUE_OFF)))

    def power_down(self):
        return self.wait_ack(pr.ARM, self._send(pr.ARM, pr.p_arm(pr.ARM_POWER_DOWN)))

    def clear_faults(self):
        return self.wait_ack(pr.CLEAR_FAULTS, self._send(pr.CLEAR_FAULTS))

    def torque_limits(self, limits: dict):
        """{servo_id: 0..1000}; the ESP32 clamps each to that servo's configured peak (claw: continuous)."""
        return self.wait_ack(pr.TORQUE_LIMITS, self._send(pr.TORQUE_LIMITS, pr.p_torque_limits(limits.items())))

    def knob_mode(self, on: bool):
        return self.wait_ack(pr.CLAW_MODE, self._send(pr.CLAW_MODE, bytes([1 if on else 0])))

    def passthru(self, sts_packet: bytes, timeout=0.3) -> bytes:
        """Raw STS transaction through the ESP32 (MAINT state only); returns the raw reply bytes."""
        self._pt_evt.clear()
        self._send(pr.PASSTHRU, sts_packet)
        return self._pt if self._pt_evt.wait(timeout) else b""

    def get_config(self):
        self.config = None
        self._send(pr.GET_CONFIG)
        t0 = time.time()
        while self.config is None and time.time() - t0 < 1.0:
            time.sleep(0.01)
        return self.config

    # ------------------------------------------------------------------ joint space
    def joints(self):
        """Current (th1, th2, th3, wrist, claw_mm) from telemetry, or None."""
        t = self.tel
        if t is None or not all(s.online for s in t.servos.values()):
            return None
        act = {sid: self.smap.to_deg(sid, s.pos) for sid, s in t.servos.items()}
        th1, th2, th3, wrist, claw, q_now = self.model.joints_from_actuators(act)
        self.q_prev = q_now
        return th1, th2, th3, wrist, claw

    def send_joints(self, th1, th2, th3, wrist, claw_mm, acc=20, hard=False):
        """One goal set.  hard=True checks against the hard (mechanical) limits - used for the
        intermediate points of a move whose TARGET already passed the soft limits."""
        ok, why = self.model.check(th1, th2, th3, wrist, hard=hard)
        if not ok:
            raise ValueError(why)
        claw_mm = float(np.clip(claw_mm, *self.model.lim["claw_mm"]))
        act = self.model.actuators(th1, th2, th3, wrist, claw_mm, q_prev=self.q_prev)
        goals = [(sid, self.smap.to_counts(sid, deg), 0, acc) for sid, deg in act.items() if sid in self.smap.c]
        return self.wait_ack(pr.GOALS, self._send(pr.GOALS, pr.p_goals(goals)), timeout=0.2)

    def move_joints(self, target, duration=None, rate_hz=50.0, should_stop=lambda: False):
        """Smooth (minimum-jerk) joint-space move to target = (th1, th2, th3, wrist, claw_mm).
        The duration follows from max_joint_speed_dps unless given.  Returns True if completed."""
        start = self.joints()
        if start is None:
            raise RuntimeError("no telemetry / servo offline")
        ok, why = self.model.check(*target[:4])
        if not ok:
            raise ValueError(why)
        s0 = np.array(self._flat(start))
        s1 = np.array(self._flat(target))
        span = np.abs(s1 - s0)
        need = max(float(span[:-1].max()) / self.max_dps, float(span[-1]) / self.claw_mms, 0.3)
        T = max(duration or 0.0, need)
        n = int(T * rate_hz)
        for i in range(1, n + 1):
            if should_stop() or self.tel is None or self.tel.state != "ACTIVE":
                return False
            u = i / n
            s = u ** 3 * (10 - 15 * u + 6 * u * u)
            v = s0 + (s1 - s0) * s
            code = self.send_joints(*self._unflat(v), hard=i < n)
            if code not in ("OK", "CLAMPED"):
                return False
            time.sleep(1.0 / rate_hz)
        return True

    def move_tcp(self, p_tcp, R_tool, claw_mm=None, duration=None, steps_per_s=50.0, should_stop=lambda: False):
        """Straight-line TCP move (position interpolated, orientation slerped) with IK at every step."""
        from scipy.spatial.transform import Rotation, Slerp
        cur = self.joints()
        if cur is None:
            raise RuntimeError("no telemetry")
        self.model.ik(p_tcp, R_tool)                                   # target must be valid (soft limits)
        p0, R0 = self.model.tcp(*cur[:4])
        p0 = p0[:3]
        claw = cur[4] if claw_mm is None else claw_mm
        dist = float(np.linalg.norm(np.asarray(p_tcp) - p0))
        T = max(duration or 0.0, dist / 80.0, 0.5)                     # <= 80 mm/s near a person
        n = int(T * steps_per_s)
        sl = Slerp([0, 1], Rotation.from_matrix([R0, R_tool]))
        for i in range(1, n + 1):
            if should_stop() or self.tel is None or self.tel.state != "ACTIVE":
                return False
            u = i / n
            s = u ** 3 * (10 - 15 * u + 6 * u * u)
            try:
                q = self.model.ik(p0 + (np.asarray(p_tcp) - p0) * s, sl([s]).as_matrix()[0], hard=True)
            except ValueError:
                return False                                           # path leaves the workspace
            code = self.send_joints(*q, cur[4] + (claw - cur[4]) * s, hard=True)
            if code not in ("OK", "CLAMPED"):
                return False
            time.sleep(1.0 / steps_per_s)
        return True

    def _flat(self, q):
        th1, th2, th3, wrist, claw = q
        return [th1, th2, th3, *wrist, claw]

    def _unflat(self, v):
        nw = 3 if self.model.abenics else 2
        return v[0], v[1], v[2], tuple(v[3:3 + nw]), v[3 + nw]
