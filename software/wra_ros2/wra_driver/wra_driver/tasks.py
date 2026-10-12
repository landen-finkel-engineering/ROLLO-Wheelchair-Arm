"""Task sequences built on WraClient (used by the ROS 2 node and by tools/wra_cli.py).

Door knob (the headline task):  pre-grasp -> open claw -> approach along the knob axis -> close in
"knob mode" (50 % claw torque, <= 10 s) -> roll to retract the latch -> pull/push the door a few
degrees about its hinge while holding the turn -> release -> retreat.  Every step checks the
supervisor state; any fault or a stop request aborts with the arm holding position.

Knob frame K (from vision or teaching), in the arm base frame B, metres or mm (mm here):
  origin = centre of the knob's FRONT face, x_K = outward door normal (toward the chair),
  z_K = up.  Geometry params: knob_d (mm), grip_depth (mm behind the front face), hinge
  (+1 = hinge on the left of the knob as seen from the chair, -1 = right), hinge_dist (mm).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np

from .arm_model import rz


@dataclass
class KnobParams:
    knob_d: float = 56.0
    grip_depth: float = 28.0
    standoff: float = 120.0
    turn_deg: float = 70.0           # + = counter-clockwise seen from the chair
    open_deg: float = 12.0           # door opening while still holding the knob
    pull: bool = True                # True: door opens toward the chair
    hinge: int = 1
    hinge_dist: float = 760.0
    turn_tol_deg: float = 15.0       # abort if the wrist ends further than this from the turn target


class Aborted(RuntimeError):
    pass


def tool_R_for_knob(R_K):
    """Tool-flange orientation that points the claw INTO the door along the knob axis, jaws horizontal."""
    xk, zk = R_K[:, 0], R_K[:, 2]
    X = -xk
    Z = zk - X * (zk @ X)
    Z /= np.linalg.norm(Z)
    Y = np.cross(Z, X)
    return np.column_stack([X, Y, Z])


class DoorKnobTask:
    def __init__(self, client, log=print, should_stop=lambda: False):
        self.c = client
        self.log = log
        self.should_stop = should_stop

    def _ok(self):
        t = self.c.tel
        if self.should_stop():
            raise Aborted("stop requested")
        if t is None or t.state != "ACTIVE" or t.critical:
            raise Aborted(f"supervisor state {None if t is None else t.state} {[] if t is None else t.faults}")

    def _tcp(self, p, R, claw=None, duration=None):
        self._ok()
        try:
            done = self.c.move_tcp(p, R, claw_mm=claw, duration=duration, should_stop=self.should_stop)
        except ValueError as e:
            raise Aborted(f"target outside the workspace/limits: {e}")
        if not done:
            raise Aborted("motion interrupted or path leaves the workspace")

    def reachable(self, p, R):
        try:
            self.c.model.ik(p, R)
            return True
        except ValueError:
            return False

    def _claw(self, half_mm, knob=False, timeout=3.0):
        self._ok()
        if knob:
            self.c.knob_mode(True)
        cur = self.c.joints()
        self.c.send_joints(*cur[:4], half_mm)
        t0 = time.time()
        last = None
        while time.time() - t0 < timeout:          # stalls on the object at the torque limit
            time.sleep(0.1)
            q = self.c.joints()
            if q is None:
                break
            if last is not None and abs(q[4] - last) < 0.05:
                break
            last = q[4]
        q = self.c.joints()
        return None if q is None else q[4]

    def run(self, p_K, R_K, kp: KnobParams = KnobParams()):
        p_K = np.asarray(p_K, float)
        R_K = np.asarray(R_K, float)
        xk = R_K[:, 0]
        Rt = tool_R_for_knob(R_K)
        grip = p_K - xk * kp.grip_depth
        pre = p_K + xk * kp.standoff
        try:
            self.log("1/7 pre-grasp")
            self._tcp(pre, Rt, claw=min(32.5, kp.knob_d / 2 + 10.0))
            self.log("2/7 approach along the knob axis")
            self._tcp(grip, Rt, duration=2.0)
            self.log("3/7 close (knob mode)")
            held = self._claw(0.0, knob=True)
            if held is None or held < kp.knob_d / 2 - 8 or held > kp.knob_d / 2 + 6:
                raise Aborted(f"claw closed to {held} mm half-opening: knob not where expected")
            self.log(f"   gripping at {2 * held:.1f} mm")
            self.log("4/7 turn")
            R_turn = Rt @ np.array([[1, 0, 0], [0, math.cos(math.radians(kp.turn_deg)), -math.sin(math.radians(kp.turn_deg))],
                                    [0, math.sin(math.radians(kp.turn_deg)), math.cos(math.radians(kp.turn_deg))]])
            if self.c.model.abenics:                     # Theta servos up to 30 % for the turn (Theta-gear tooth limit, docs/05)
                self.c.torque_limits({5: 300, 7: 300})
            self._tcp(grip, R_turn, duration=1.5)
            time.sleep(0.3)                              # let the servos settle against the latch
            q = self.c.joints()
            if q is None:
                raise Aborted("lost servo telemetry during the turn")
            _, R_now = self.c.model.tcp(*q[:4])
            err = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R_turn.T @ R_now) - 1.0) / 2.0))))
            if err > kp.turn_tol_deg:
                self.c.send_joints(*q[:4], q[4], hard=True)   # stop pushing: hold where the knob stopped
                raise Aborted(f"knob did not turn far enough (wrist {err:.0f} deg short of the target): "
                              "latch too stiff for the 30 % wrist cap, or the pads slipped")
            self.log(f"   turned to within {err:.1f} deg of the target")
            self.log("5/7 swing the door (as far as the wrist/arm limits allow, up to %.0f deg)" % kp.open_deg)
            # y_K points to the RIGHT as seen from the chair; hinge = +1 -> hinge on the left (-y_K side).
            # Rotating about the vertical through the hinge by a moves the knob by a*hinge*d*(-x_K),
            # so pulling (toward +x_K) needs a = -hinge*|a|.
            hinge_pt = grip - R_K[:, 1] * (kp.hinge * kp.hinge_dist)
            sgn = (-1 if kp.pull else 1) * kp.hinge
            achieved = 0.0
            for a in np.linspace(0, kp.open_deg, max(2, int(kp.open_deg) + 1))[1:]:
                Rz = rz(math.radians(sgn * a))
                p = hinge_pt + Rz @ (grip - hinge_pt)
                if not self.reachable(p, Rz @ R_turn):
                    break                       # door is cracked open; the user pushes it the rest of the way
                self._tcp(p, Rz @ R_turn, duration=0.25)
                achieved = a
            self.log(f"   door swung {achieved:.1f} deg")
            if achieved < 2.0:
                self.log("   WARNING: less than 2 deg - reposition the chair (see docs/10_door_tasks.md: knob reach window)")
            self.log("6/7 release and un-roll (open claw turns freely around the knob)")
            self.c.knob_mode(False)
            if self.c.model.abenics:
                self.c.torque_limits({5: 250, 7: 250})
            self._claw(min(32.5, kp.knob_d / 2 + 4.0))
            q = self.c.joints()
            p_now, R_now = self.c.model.tcp(*q[:4])
            back = np.array([[1, 0, 0], [0, math.cos(math.radians(-kp.turn_deg)), -math.sin(math.radians(-kp.turn_deg))],
                             [0, math.sin(math.radians(-kp.turn_deg)), math.cos(math.radians(-kp.turn_deg))]])
            if self.reachable(p_now[:3], R_now @ back):
                self._tcp(p_now[:3], R_now @ back, duration=1.5)
            self.log("7/7 retreat along the knob axis, then park")
            q = self.c.joints()
            p_now, R_now = self.c.model.tcp(*q[:4])
            for d in (kp.standoff, 60.0, 40.0, 25.0):
                tgt = p_now[:3] - R_now[:, 0] * d
                if self.reachable(tgt, R_now):
                    self._tcp(tgt, R_now)
                    break
            h = self.c.model.g["home"]
            home = (h["j1"], h["j2"], h["j3"], tuple(h["wt"]) if self.c.model.abenics else (h["j4"], h["j5"]), 8.0)
            if not self.c.move_joints(home, should_stop=self.should_stop):
                raise Aborted("park move interrupted")
            self.log("done: door unlatched and opened %.1f deg - continue with the chair or the arm" % achieved)
            return True
        except Aborted as e:
            self.c.knob_mode(False)
            if self.c.model.abenics:
                self.c.torque_limits({5: 250, 7: 250})
            self.log(f"ABORTED: {e} (arm holds position; claw keeps its grip)")
            return False
