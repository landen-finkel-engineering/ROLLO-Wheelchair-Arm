#!/usr/bin/env python3
"""Door-knob reach window: where a knob can be for the door task to succeed (pre-grasp, grip and a
few degrees of door swing all inside the soft limits).  Uses the same kinematics as the robot
(software/wra_ros2/wra_driver/wra_driver/arm_model.py, generated from params.yaml).

    python calcs/knob_window.py   -> calcs/output/knob_window.png / .json

Knob frame: front-face centre p_K, x_K = door normal toward the chair (here -x_B: the door faces
the arm), z_K up.  The claw approaches along the knob axis with the jaws horizontal.
"""
from __future__ import annotations

import json
import math
import pathlib
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "software" / "wra_ros2" / "wra_driver"))
from wra_driver.arm_model import ArmModel, rz  # noqa: E402
from wra_driver.tasks import tool_R_for_knob, KnobParams  # noqa: E402

OUT = HERE / "output"
OUT.mkdir(exist_ok=True)
m = ArmModel()
kp = KnobParams(standoff=80.0, grip_depth=28.0)
RK = np.column_stack([[-1.0, 0, 0], [0, -1.0, 0], [0, 0, 1.0]])
Rt = tool_R_for_knob(RK)


def ok(p, R):
    try:
        m.ik(p, R)
        return True
    except ValueError:
        return False


def swing(pK, hinge, pull=True, max_deg=15):
    """Largest door swing (deg) the arm can follow while holding the turned knob."""
    grip = pK - RK[:, 0] * kp.grip_depth
    turn = math.radians(kp.turn_deg)
    R_turn = Rt @ np.array([[1, 0, 0], [0, math.cos(turn), -math.sin(turn)], [0, math.sin(turn), math.cos(turn)]])
    hinge_pt = grip - RK[:, 1] * (hinge * kp.hinge_dist)
    sgn = (-1 if pull else 1) * hinge
    best = 0
    for a in range(1, max_deg + 1):
        Rz = rz(math.radians(sgn * a))
        if not ok(hinge_pt + Rz @ (grip - hinge_pt), Rz @ R_turn):
            break
        best = a
    return best


rows = []
for y in (-100.0, -50.0, 0.0, 50.0, 100.0, 150.0):
    for x in range(300, 651, 10):
        for z in range(200, 651, 10):
            pK = np.array([float(x), y, float(z)])
            if ok(pK + RK[:, 0] * kp.standoff, Rt) and ok(pK - RK[:, 0] * kp.grip_depth, Rt):
                rows.append({"x": x, "y": y, "z": z, "swing_hinge_left": swing(pK, 1), "swing_hinge_right": swing(pK, -1)})

zs = sorted({r["z"] for r in rows})
summary = {"standoff_mm": kp.standoff, "grip_depth_mm": kp.grip_depth, "n_points": len(rows),
           "knob_height_above_dock_mm": [zs[0], zs[-1]] if zs else None,
           "x_window_at_z_mm": {z: [min(r["x"] for r in rows if r["z"] == z and r["y"] == 0.0),
                                    max(r["x"] for r in rows if r["z"] == z and r["y"] == 0.0)]
                                for z in range(zs[0], zs[-1] + 1, 50) if any(r["z"] == z and r["y"] == 0.0 for r in rows)}
           if zs else {},
           "best_swing_deg": max((max(r["swing_hinge_left"], r["swing_hinge_right"]) for r in rows), default=0)}
(OUT / "knob_window.json").write_text(json.dumps({"summary": summary, "points": rows}, indent=1))

fig, axs = plt.subplots(1, 2, figsize=(12, 5.0))
for y, c in ((-50.0, "#9467bd"), (0.0, "#1f77b4"), (50.0, "#2ca02c"), (100.0, "#ff7f0e")):
    pts = np.array([(r["x"], r["z"]) for r in rows if r["y"] == y])
    if len(pts):
        axs[0].scatter(pts[:, 0] + (y / 25.0), pts[:, 1], s=8, color=c, alpha=0.55,
                       label=f"knob {y:+.0f} mm to the arm's left (y_B)")
axs[0].set_xlabel("knob distance in front of the J1 axis, x_B (mm)")
axs[0].set_ylabel("knob height above the dock top, z_B (mm)")
axs[0].set_title("Where the door task works (pre-grasp 80 mm + grip, soft limits)", fontsize=9)
axs[0].grid(alpha=0.3)
for mz, ls in ((400, ":"), (470, "--")):
    for h_in in (34, 36, 38):
        z = h_in * 25.4 - mz
        axs[0].axhline(z, color="r", lw=0.6, ls=ls)
        axs[0].text(axs[0].get_xlim()[1] - 2, z + 2, f"{h_in} in knob, dock {mz} mm", fontsize=6, color="r", ha="right")
axs[0].legend(fontsize=7, loc="lower left")
pts = np.array([(r["x"], r["z"], max(r["swing_hinge_left"], r["swing_hinge_right"])) for r in rows if r["y"] == 0.0])
if len(pts):
    sc = axs[1].scatter(pts[:, 0], pts[:, 1], c=pts[:, 2], cmap="viridis", s=14)
    plt.colorbar(sc, ax=axs[1], label="door swing the arm can follow (deg)")
axs[1].set_xlabel("x_B (mm)")
axs[1].set_ylabel("z_B (mm)")
axs[1].set_title("Door swing while holding the turned knob (y = 0)", fontsize=9)
axs[1].grid(alpha=0.3)
fig.tight_layout()
fig.savefig(OUT / "knob_window.png", dpi=150)
print(json.dumps(summary, indent=1))
