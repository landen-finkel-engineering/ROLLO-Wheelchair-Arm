"""LEFT/RIGHT installation checks on the PROVISIONAL reference chair.
For each side and pose: interferences between (arm + mount + electronics) and (chair + user
envelope), arm-vs-mount interferences, minimum clearances to wheels/hand rims/user, and the
overall width added to the chair.  Usage: python tools/chair_check.py [side ...]"""
from __future__ import annotations
import sys, json, itertools, pathlib, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from wra import chair as ch
from wra.config import P

ALLOW = {("strut_rod", "post_bracket"), ("strut_rod", "strut_clamp_body"),
         ("clamp_body_0", "clamp_cap_0"), ("clamp_body_1", "clamp_cap_1")}
# installation contacts with the chair that are intended (clamps/liners grip the tubes)
ALLOW_PREFIX = {("clamp_liner", "chair_front_post"), ("strut_clamp", "chair_lower_rail"),
                ("ebox_clamp", "chair_rear_post")}


def allowed(a, b):
    if (a, b) in ALLOW or (b, a) in ALLOW:
        return True
    for x, y in ALLOW_PREFIX:
        if (a.startswith(x) and b.startswith(y)) or (b.startswith(x) and a.startswith(y)):
            return True
    return False


def overlap(ba, bb, m=0.0):
    return not (ba.min.X > bb.max.X + m or bb.min.X > ba.max.X + m or ba.min.Y > bb.max.Y + m or
                bb.min.Y > ba.max.Y + m or ba.min.Z > bb.max.Z + m or bb.min.Z > ba.max.Z + m)


def run(side, q=None, tag="park"):
    t0 = time.time()
    inst = [i for i in ch.chair_instances(side, q) if i.shape is not None and not i.name.startswith(("belt", "cord", "spring"))]
    ours = [i for i in inst if i.kind in ("printed", "vitamin")]
    env = [i for i in inst if i.kind in ("reference", "envelope")]
    bbs = {i.name: i.shape.bounding_box() for i in inst}
    bad = []
    for a, b in itertools.product(ours, env):
        if not overlap(bbs[a.name], bbs[b.name]) or allowed(a.name, b.name):
            continue
        r = a.shape & b.shape
        v = r.volume if r is not None else 0.0
        if v > 0.5:
            bad.append((a.name, b.name, round(v, 1)))
    # arm vs mount/electronics
    arm = [i for i in ours if i.group not in ("mount", "electronics", "dock")]
    mnt = [i for i in ours if i.group in ("mount", "electronics")]
    for a, b in itertools.product(arm, mnt):
        if not overlap(bbs[a.name], bbs[b.name]) or allowed(a.name, b.name):
            continue
        r = a.shape & b.shape
        v = r.volume if r is not None else 0.0
        if v > 0.5:
            bad.append((a.name, b.name, round(v, 1)))
    # clearances (bounding-box prefilter 150 mm, exact distance on candidates)
    clear = {}
    for tgt in ("chair_rear_wheel", "chair_hand_rim", "chair_wheel_lock", "chair_footrest_hanger", "user_"):
        best = None
        for b in env:
            if not b.name.startswith(tgt):
                continue
            for a in ours:
                if a.group in ("electronics",) or a.name.startswith(("clamp", "strut", "stop_collar", "ebox")):
                    continue
                if not overlap(bbs[a.name], bbs[b.name], 150.0):
                    continue
                d = a.shape.distance_to(b.shape)
                if best is None or d < best[0]:
                    best = (round(d, 1), a.name, b.name)
        clear[tgt.rstrip("_")] = best
    # width added (outboard extent of our parts vs. hand rim outer face)
    s = ch.side_sign(side)
    ys = [bbs[i.name].max.Y if s > 0 else -bbs[i.name].min.Y for i in ours]
    rim = [bbs[i.name] for i in env if i.name.startswith("chair_hand_rim_" + ("L" if s > 0 else "R"))][0]
    rim_y = rim.max.Y if s > 0 else -rim.min.Y
    xs = [bbs[i.name].max.X for i in ours]
    res = {"side": side, "pose": tag, "q": q, "interferences": bad, "clearance_mm": clear,
           "outboard_extent_mm": round(max(ys), 1), "hand_rim_outer_mm": round(rim_y, 1),
           "added_width_mm": round(max(0.0, max(ys) - rim_y), 1), "front_extent_x_mm": round(max(xs), 1),
           "t_s": round(time.time() - t0, 1)}
    print(json.dumps(res), flush=True)
    return res


def knob_pose(knob_floor_mm: float = 914.0, ahead_mm: float = 450.0, claw_half_mm: float = 28.0) -> dict:
    """RIGHT-side joint pose gripping a knob (36" = 914 mm above the floor by default) straight ahead
    of J1, solved with the driver's IK (needs software/.../config/arm_geometry.json from --software)."""
    import numpy as np
    root = pathlib.Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / "software" / "wra_ros2" / "wra_driver"))
    from wra_driver.arm_model import ArmModel
    from wra_driver.tasks import tool_R_for_knob
    m = ArmModel()
    xk, zk = np.array([-1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])
    R_K = np.column_stack([xk, np.cross(zk, xk), zk])
    p_K = np.array([ahead_mm, 0.0, knob_floor_mm - P.chair.mount_z])
    th1, th2, th3, wt = m.ik(p_K - xk * 28.0, tool_R_for_knob(R_K))
    return {"j1": th1, "j2": th2, "j3": th3, "wt": tuple(wt), "j6": claw_half_mm}


def run_all(sides=("right", "left")):
    """Park + gripping a 36" knob, for each side (the knob pose is mirrored for LEFT)."""
    out = []
    try:
        qk = knob_pose()
    except Exception as e:                       # knob outside the window for this mount_z
        print("knob pose not reachable:", e)
        qk = None
    for s in sides:
        out.append(run(s))
        if qk is not None:
            out.append(run(s, qk if s == "right" else ch.mirror_q(qk), tag="knob_36in"))
    return out


if __name__ == "__main__":
    sides = sys.argv[1:] or ["right", "left"]
    out = run_all(sides)
    d = pathlib.Path(__file__).resolve().parents[2] / "output" / "reports"
    d.mkdir(parents=True, exist_ok=True)
    (d / "chair_check.json").write_text(json.dumps(out, indent=1))
