"""WRA-1 kinematics and joint <-> servo mapping.  numpy only (no ROS, no CAD).

Geometry comes from config/arm_geometry.json, which `python cad/build.py --software` writes from
params.yaml (single source of truth); calibration (servo zero counts + directions) comes from
config/calibration.yaml written by software/tools/servo_setup.py.

Joint vector (degrees / mm), internal convention = CAD convention:
  th1  J1 yaw                     th2  upper-arm elevation (absolute)
  th3  forearm elevation (ABSOLUTE, parallelogram)
  WRA-1A: wt = (pitch, yaw, roll) of the tool vs. the forearm (ABENICS, 4 actuators q1 Th1 q2 Th2)
  WRA-1B: j4 pitch (relative), j5 roll
  claw  half-opening (mm), pad-to-pad opening = 2 * claw
URDF joints: j1 = th1, j2 = th2, j3 = th3 - th2 (elbow), wrist_yaw/pitch/roll (1A) or j4/j5 (1B), claw.
"""
from __future__ import annotations

import json
import math
import pathlib

import numpy as np

from . import abenics_kin as AK

def _cfg_dir() -> pathlib.Path:
    """config/ next to the package source (colcon --symlink-install, tests, tools); otherwise the
    installed share/wra_driver/config."""
    here = pathlib.Path(__file__).resolve().parent.parent / "config"
    if here.exists():
        return here
    try:
        from ament_index_python.packages import get_package_share_directory
        return pathlib.Path(get_package_share_directory("wra_driver")) / "config"
    except Exception:          # pragma: no cover - no ROS environment
        return here


CFG = _cfg_dir()
DEG = math.pi / 180.0


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def H(R=None, p=(0, 0, 0)):
    M = np.eye(4)
    if R is not None:
        M[:3, :3] = R
    M[:3, 3] = p
    return M


class ArmModel:
    def __init__(self, geometry: dict | None = None, side: str | None = None):
        """side = 'right' | 'left' turns on the user keep-out check (docs/09 T5.1); None = bench use."""
        g = geometry or json.loads((CFG / "arm_geometry.json").read_text())
        self.g = g
        self.side = side
        self.keepout = g.get("user_keepout_C_mm", []) if side else []
        self.ko_margin = float(g.get("keepout_margin_mm", 50.0))
        if self.keepout:
            m = g["mount"][side]
            r, p_, y = m["rpy"]
            self._RC = rz(y) @ ry(p_) @ rx(r)                     # base_link -> chair frame
            self._tC = np.array(m["xyz"]) * 1000.0
        self.variant = g["variant"]
        self.abenics = self.variant == "WRA-1A"
        self.Tz, self.sh = g["turntable_origin_z"], g["shoulder_h"]
        self.L2, self.L3, self.YC = g["L2"], g["L3"], g["YC"]
        self.tcp_x = g["tcp_from_flange"]
        self.ratio = g["ratios"]
        self.lim = g["limits"]
        if self.abenics:
            self.FL1 = g["flange_from_wrist_center"]
            self.MG = np.array(g["G_in_S_rotation"])            # tool-flange axes in the sphere frame
        else:
            self.xf1 = g["x_flange1"]

    # ------------------------------------------------------------------ forward kinematics
    def frames(self, th1, th2, th3, wrist):
        """4x4 frames in B: T, U, F, A (wrist centre / pitch frame), G (tool flange)."""
        T = H(rz(th1 * DEG), (0, 0, self.Tz))
        U = T @ H(ry(-th2 * DEG), (0, 0, self.sh))
        E = (self.L2 * math.cos(th2 * DEG), 0.0, self.sh + self.L2 * math.sin(th2 * DEG))
        F = T @ H(ry(-th3 * DEG), E)
        if self.abenics:
            A = F @ H(None, (self.L3, self.YC, 0))
            R = AK.tool_rotation_pyr(*wrist)
            G = A @ H(R) @ H(self.MG, (0, 0, self.FL1))
        else:
            j4, j5 = wrist
            A = F @ H(ry(-j4 * DEG), (self.L3, 0, 0))
            Rr = A @ H(None, (0, self.YC, 0)) @ H(rx(j5 * DEG)) @ H(None, (0, -self.YC, 0))
            G = Rr @ H(None, (self.xf1, self.YC, 0))
        return dict(T=T, U=U, F=F, A=A, G=G)

    def tcp(self, th1, th2, th3, wrist):
        G = self.frames(th1, th2, th3, wrist)["G"]
        return G @ np.array([self.tcp_x, 0, 0, 1.0]), G[:3, :3]

    # ------------------------------------------------------------------ inverse kinematics
    def ik(self, p_tcp, R_tool, elbow_up=True, hard=False):
        """TCP position (mm, frame B) + tool-flange orientation (3x3, columns = X_G tool axis,
        Y_G, Z_G in B) -> (th1, th2, th3, wrist) or raises ValueError (unreachable / limits)."""
        R_tool = np.asarray(R_tool, float)
        if self.abenics:
            c = np.asarray(p_tcp, float) - R_tool @ np.array([self.tcp_x + self.FL1, 0, 0])
            y_off = self.YC
        else:
            raise NotImplementedError("WRA-1B Cartesian IK: use joint-space commands (5-DoF wrist)")
        r = math.hypot(c[0], c[1])
        if r < abs(y_off) + 1.0:
            raise ValueError("wrist centre too close to the J1 axis")
        rho = math.sqrt(r * r - y_off * y_off)
        th1 = math.degrees(math.atan2(c[1], c[0]) - math.atan2(y_off, rho))
        h = c[2] - self.Tz - self.sh
        d2 = rho * rho + h * h
        cd = (d2 - self.L2 ** 2 - self.L3 ** 2) / (2 * self.L2 * self.L3)
        if abs(cd) > 1.0:
            raise ValueError("out of reach")
        delta = math.acos(cd) * (1 if elbow_up else -1)          # th2 - th3
        th2 = math.atan2(h, rho) + math.atan2(self.L3 * math.sin(delta), self.L2 + self.L3 * math.cos(delta))
        th2 = math.degrees(th2)
        th3 = th2 - math.degrees(delta)
        fr = self.frames(th1, th2, th3, (0.0, 0.0, 0.0))
        RA = fr["A"][:3, :3]
        R_s = RA.T @ R_tool @ self.MG.T
        wrist = AK.tool_pyr(R_s)
        q = (th1, th2, th3, wrist)
        ok, why = self.check(*q, hard=hard)
        if not ok:
            raise ValueError(why)
        return q

    # ------------------------------------------------------------------ limits
    def check(self, th1, th2, th3, wrist, hard=False):
        L = self.lim
        k = 1 if hard else 0
        if not (L["j1"][k][0] <= th1 <= L["j1"][k][1]):
            return False, f"J1 {th1:.1f} outside {L['j1'][k]}"
        if not (L["j2"][k][0] <= th2 <= L["j2"][k][1]):
            return False, f"J2 {th2:.1f} outside {L['j2'][k]}"
        if not (L["j3"][k][0] <= th3 <= L["j3"][k][1]):
            return False, f"J3 {th3:.1f} outside {L['j3'][k]}"
        rel = th2 - th3
        if not (L["j3_rel"][0] <= rel <= L["j3_rel"][1]):
            return False, f"elbow (th2-th3) {rel:.1f} outside {L['j3_rel']}"
        if self.abenics:
            ok, info = AK.wrist_check(AK.tool_rotation_pyr(*wrist), hard=hard)
            if not ok:
                return False, f"wrist {tuple(round(w, 1) for w in wrist)} outside the ABENICS limits {info}"
        else:
            j4, j5 = wrist
            if not (L["j4"][k][0] <= j4 <= L["j4"][k][1]) or not (L["j5"][k][0] <= j5 <= L["j5"][k][1]):
                return False, "J4/J5 outside limits"
            if rel > 125 and j4 < 0:
                return False, "coupled limit: j4 >= 0 when th2-th3 > 125"
        if self.keepout:
            why = self.keepout_violation(th1, th2, th3, wrist)
            if why:
                return False, why
        return True, ""

    def keepout_violation(self, th1, th2, th3, wrist):
        """None, or which arm point lies inside a user keep-out box (chair frame, grown by the margin).
        Points: 5 along the upper arm, 5 along the forearm, wrist centre, tool flange, TCP."""
        fr = self.frames(th1, th2, th3, wrist)
        S, E, A, G = fr["U"][:3, 3], fr["F"][:3, 3], fr["A"][:3, 3], fr["G"][:3, 3]
        tcp = (fr["G"] @ np.array([self.tcp_x, 0.0, 0.0, 1.0]))[:3]
        pts = [("upper arm", S + (E - S) * t) for t in (0.2, 0.4, 0.6, 0.8, 1.0)]
        pts += [("forearm", E + (A - E) * t) for t in (0.2, 0.4, 0.6, 0.8)]
        pts += [("wrist", A), ("tool flange", G), ("TCP", tcp)]
        m = self.ko_margin
        for nm, pB in pts:
            pC = self._RC @ pB + self._tC
            for b in self.keepout:
                if all(b["min"][i] - m <= pC[i] <= b["max"][i] + m for i in range(3)):
                    return f"{nm} inside the user keep-out zone '{b['name']}' (+{m:.0f} mm margin)"
        return None

    # ------------------------------------------------------------------ actuators
    def actuators(self, th1, th2, th3, wrist, claw_mm, q_prev=(0.0, 0.0)):
        """Joint values -> actuator angles at each SERVO (deg), keyed by servo id."""
        r = self.ratio
        out = {1: r["j1"] * th1, 2: r["j2"] * th2, 3: r["j3"] * th3}
        if self.abenics:
            win = [(math.radians(-AK.LIMITS["q"][0]), math.radians(AK.LIMITS["q"][0]))] * 2
            q1, t1, q2, t2 = AK.ik(AK.tool_rotation_pyr(*wrist), q_prev=q_prev, windows=win)
            tr = self.g["theta_servo_per_theta"]
            out.update({4: math.degrees(q1), 5: tr * math.degrees(t1), 6: math.degrees(q2), 7: tr * math.degrees(t2)})
        else:
            out.update({4: r["j4"] * wrist[0], 5: wrist[1]})
        out[8] = -math.degrees(claw_mm / self.g["claw_pinion_r"])
        return out

    def joints_from_actuators(self, act: dict):
        """Servo angles (deg, keyed by id) -> (th1, th2, th3, wrist, claw_mm, q_now)."""
        r = self.ratio
        th1, th2, th3 = act[1] / r["j1"], act[2] / r["j2"], act[3] / r["j3"]
        if self.abenics:
            tr = self.g["theta_servo_per_theta"]
            q1, t1, q2, t2 = (math.radians(act[4]), math.radians(act[5] / tr), math.radians(act[6]),
                              math.radians(act[7] / tr))
            wrist = AK.tool_pyr(AK.fk(q1, t1, q2, t2))
            q_now = (q1, q2)
        else:
            wrist = (act[4] / r["j4"], act[5])
            q_now = (0.0, 0.0)
        claw = -math.radians(act[8]) * self.g["claw_pinion_r"]
        return th1, th2, th3, wrist, claw, q_now

    # ------------------------------------------------------------------ URDF joint vector
    def urdf_names(self):
        w = ["wrist_yaw", "wrist_pitch", "wrist_roll"] if self.abenics else ["j4", "j5"]
        return ["j1", "j2", "j3"] + w + ["claw"]

    def to_urdf(self, th1, th2, th3, wrist, claw_mm):
        v = [th1 * DEG, th2 * DEG, (th3 - th2) * DEG]
        if self.abenics:
            p, y, r = wrist
            v += [y * DEG, p * DEG, r * DEG]
        else:
            v += [wrist[0] * DEG, wrist[1] * DEG]
        return v + [claw_mm / 1000.0]

    def from_urdf(self, v):
        th1, th2 = v[0] / DEG, v[1] / DEG
        th3 = th2 + v[2] / DEG
        if self.abenics:
            wrist = (v[4] / DEG, v[3] / DEG, v[5] / DEG)
            claw = v[6] * 1000.0
        else:
            wrist = (v[3] / DEG, v[4] / DEG)
            claw = v[5] * 1000.0
        return th1, th2, th3, wrist, claw


class ServoMap:
    """Servo angle (deg) <-> position counts using calibration: counts = center + dir * deg * 4096/360."""

    def __init__(self, calib: dict):
        self.c = {int(k): v for k, v in calib["servos"].items()}

    def to_counts(self, sid, deg):
        c = self.c[sid]
        return int(round(c["center"] + c["dir"] * deg * 4096.0 / 360.0))

    def to_deg(self, sid, counts):
        c = self.c[sid]
        return (counts - c["center"]) * 360.0 / 4096.0 * c["dir"]
