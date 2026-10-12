"""Find the coupled wrist-pitch limit j4_min(q3_rel) (and j4_max) that keeps the wrist/gripper clear
of the shoulder / upper arm / base, by bisection on full CAD intersections.
Writes cad/output/reports/coupled_limits.json (used by firmware/driver as a lookup table)."""
from __future__ import annotations
import sys, json, itertools, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from wra import assembly as asm

MOVING = {"wrist", "gripper"}
FIXED = {"shoulder", "upper_arm", "turntable", "base", "dock"}


def hits(q, margin_mm3=0.5):
    inst = [i for i in asm.arm_instances(q) if i.shape is not None and not i.name.startswith(("belt", "cord", "spring"))]
    a = [i for i in inst if i.group in MOVING]
    b = [i for i in inst if i.group in FIXED]
    bb = {i.name: i.shape.bounding_box() for i in a + b}
    for x, y in itertools.product(a, b):
        p, r = bb[x.name], bb[y.name]
        if p.min.X > r.max.X or r.min.X > p.max.X or p.min.Y > r.max.Y or r.min.Y > p.max.Y or p.min.Z > r.max.Z or r.min.Z > p.max.Z:
            continue
        s = x.shape & y.shape
        if s is not None and s.volume > margin_mm3:
            return (x.name, y.name, round(s.volume, 1))
    return None


def boundary(q, free, coll, it=6):
    """free: a j4 value without collision, coll: one with collision -> free j4 closest to coll."""
    for _ in range(it):
        mid = 0.5 * (free + coll)
        if hits(dict(q, j4=mid)):
            coll = mid
        else:
            free = mid
    return free


def main():
    out = []
    j2 = 45.0
    for rel in (60, 80, 100, 120, 140):
        row = {"q3_rel": rel}
        for sgn, key in ((-1, "j4_min"), (1, "j4_max")):
            worst = None
            for j5 in (0.0, 90.0):
                q = dict(j2=j2, j3=j2 - rel, j5=j5, j6=-45.0)
                lim = 105.0 * sgn
                if hits(dict(q, j4=lim)) is None:
                    val = lim
                else:
                    start = 30.0 if hits(dict(q, j4=30.0)) is None else 0.0
                    val = boundary(q, start, lim)
                worst = val if worst is None else (max(worst, val) if sgn < 0 else min(worst, val))
            row[key] = round(worst, 1)
        print(row, flush=True)
        out.append(row)
    d = pathlib.Path(__file__).resolve().parents[2] / "output" / "reports"
    d.mkdir(parents=True, exist_ok=True)
    (d / "coupled_limits.json").write_text(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
