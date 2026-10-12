"""ABENICS wrist + knob claw interference sweep (housing frame A).

B-rep check: every pair of wrist/claw solids (printed + vitamins) over a pose list
(wt = pitch, yaw, roll of the tool vs. the forearm; j6 = claw half-opening), except intended
contacts/press fits and the CS-gear <-> monopole pairs, which mesh by design and are checked on
the toothed meshes instead (--mesh: manifold3d, CS gear + generated monopoles in place).
Usage:  python tools/wrist_sweep.py [--mesh] [--quick] [--resume LOG]   -> output/reports/wrist_sweep.json
        (--resume LOG reuses the rows an interrupted run printed to LOG; only valid if the geometry is unchanged)
"""
from __future__ import annotations
import sys, json, time, math, itertools, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import numpy as np
from wra import assembly as asm, abenics_kin as AK
from wra.parts import abenics as ab

MESH_PAIRS = {("cs_gear_front", "monopole_1"), ("cs_gear_front", "monopole_2"),
              ("cs_gear_back", "monopole_1"), ("cs_gear_back", "monopole_2")}
ALLOW = {("abenics_housing", "abenics_post"), ("center_ball", "abenics_post")}
ALLOW_K = {("mp_fork_{k}", "mp_pin_{k}")}          # press fit in the -x arm


def allowed(a, b):
    if (a, b) in MESH_PAIRS or (b, a) in MESH_PAIRS or (a, b) in ALLOW or (b, a) in ALLOW:
        return True
    for k in (1, 2):
        for x, y in ALLOW_K:
            x, y = x.format(k=k), y.format(k=k)
            if {a, b} == {x, y}:
                return True
    return False


def poses(quick=False):
    """Boundary poses of the allowed tool set (hard limits incl. tilt toward the modules) + rolls."""
    out = [((-30, 0, 0), 8.0)]
    lim = AK.LIMITS
    dirs = range(0, 360, 45 if quick else 30)
    for d in dirs:
        # largest tilt in direction d inside the hard limits
        lo, hi = 0.0, lim["tilt"][1]
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            p, y = mid * math.sin(math.radians(d)), mid * math.cos(math.radians(d))
            ok, _ = AK.wrist_check(AK.tool_rotation_pyr(p, y, 0), hard=True)
            lo, hi = (mid, hi) if ok else (lo, mid)
        p, y = round(lo * math.sin(math.radians(d)), 2), round(lo * math.cos(math.radians(d)), 2)
        for r in ((0, 90) if quick else (-90, 0, 90)):
            for j6 in ((32.5,) if quick else (0.0, 32.5)):
                out.append(((p, y, r), j6))
    return out


def brep_check(inst):
    shapes = [(i.name, i.shape) for i in inst if i.shape is not None]
    bbs = {n: s.bounding_box() for n, s in shapes}
    bad = []
    for (a, sa), (b, sb) in itertools.combinations(shapes, 2):
        ba, bb = bbs[a], bbs[b]
        if (ba.min.X > bb.max.X or bb.min.X > ba.max.X or ba.min.Y > bb.max.Y or bb.min.Y > ba.max.Y
                or ba.min.Z > bb.max.Z or bb.min.Z > ba.max.Z) or allowed(a, b):
            continue
        r = sa & sb
        v = r.volume if r is not None else 0.0
        if v > 0.5:
            bad.append((a, b, round(v, 1)))
    return bad


def mesh_check(f):
    from wra.parts import abenics_mesh as am
    cs = am.cs_hemisphere("front") + am.cs_hemisphere("back")
    mp = am.monopole()
    csA = cs.transform(am.loc_matrix(f["S"]))
    q1, t1, q2, t2 = f["act"]
    return [round((csA ^ mp.transform(am.loc_matrix(ab.Mp_loc(k, q, th)))).volume(), 3)
            for k, q, th in ((1, q1, t1), (2, q2, t2))]


def _resume_rows(path):
    """Rows already printed by an interrupted run (one JSON object per line) -> {(wt, j6): row}."""
    done = {}
    for line in pathlib.Path(path).read_text().splitlines():
        if line.startswith("{"):
            r = json.loads(line.split(" t=")[0])
            done[(tuple(r["wt"]), r["j6"])] = r
    return done


def main():
    quick, mesh = "--quick" in sys.argv, "--mesh" in sys.argv
    done = _resume_rows(sys.argv[sys.argv.index("--resume") + 1]) if "--resume" in sys.argv else {}
    t0 = time.time()
    res, qprev = [], (0.0, 0.0)
    for wt, j6 in poses(quick):
        old = done.get((tuple(wt), j6))
        if old is not None and (not mesh or "gear_mesh_overlap_mm3" in old):
            res.append(old)
            continue
        inst, f = asm.wrist_instances(wt, j6, vitamins=True, q_prev=qprev)
        R = f["Rm"]
        ok, info = AK.wrist_check(R, f["act"], hard=True)
        bad = brep_check(inst)
        row = {"wt": wt, "j6": j6, "limits_ok": ok, "act_deg": [round(math.degrees(a), 1) for a in f["act"]],
               "interferences": bad}
        if mesh:
            row["gear_mesh_overlap_mm3"] = mesh_check(f)
        res.append(row)
        print(json.dumps(row), f"t={time.time() - t0:.0f}s", flush=True)
    d = pathlib.Path(__file__).resolve().parents[2] / "output" / "reports"
    d.mkdir(parents=True, exist_ok=True)
    (d / "wrist_sweep.json").write_text(json.dumps(res, indent=1))
    n_bad = sum(1 for r in res if r["interferences"] or any(v > 0.5 for v in r.get("gear_mesh_overlap_mm3", [])))
    print(f"poses {len(res)}  with problems {n_bad}")
    return res


if __name__ == "__main__":
    main()
