"""Self-interference sweep: builds every part once, then checks pairwise solid intersections
for a list of poses.  Usage:  python tools/sweep.py [poses.json] [--render TAG]
Reports pairs with intersection volume > 0.5 mm^3 that are not intentional press fits."""
from __future__ import annotations
import sys, json, time, itertools, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from wra import assembly as asm

ALLOW = {("ddm_bracket_j2", "ddm_pin_0"), ("ddm_bracket_j3", "ddm_pin_1"), ("stub_cheek", "stub"),
         ("elbow_block", "elbow_pin"), ("wrist_housing", "pitch_pin_0"), ("wrist_housing", "pitch_pin_1"),
         ("receiver", "dock_dowel_0"), ("receiver", "dock_dowel_1"),
         ("j1_dm_bracket", "j1_dm_pin"), ("wrist_housing", "j4_dm_pin"),
         # ABENICS: press/clamp fits; CS-gear <-> monopole pairs mesh by design (tools/wrist_sweep.py --mesh)
         ("abenics_housing", "abenics_post"), ("center_ball", "abenics_post"), ("mp_fork_1", "mp_pin_1"),
         ("mp_fork_2", "mp_pin_2"), ("cs_gear_front", "monopole_1"), ("cs_gear_front", "monopole_2"),
         ("cs_gear_back", "monopole_1"), ("cs_gear_back", "monopole_2")}

DEFAULT_POSES = [
    {}, {"j2": 40, "j3": -10, "j4": -20, "j6": -20},
    {"j2": -30, "j3": -90, "j4": 0, "j6": -45},
    {"j2": 85, "j3": 40, "j4": -60, "j5": 90, "j6": 6},
    {"j1": 105, "j2": 10, "j3": -60, "j4": 105, "j6": 0},
    {"j1": -105, "j2": 45, "j3": -95, "j4": -105, "j5": -150, "j6": 6},
    {"j2": 100, "j3": 60, "j4": -105, "j5": 150, "j6": -45},
    {"j2": 30, "j3": 5, "j4": 105, "j6": -45},
    {"j2": 30, "j3": -5, "j4": -25, "j5": 90, "j6": 0},
    {"j2": -30, "j3": -95, "j4": -105},
]


# poses inside ALL soft limits incl. the coupled ones (q3_rel = j2 - j3 in [42, 140]; j4 >= 0 when q3_rel > 125)
DEFAULT_POSES_VALID = [
    {}, {"j2": 40, "j3": -10, "j4": -20, "j6": -20},
    {"j2": -30, "j3": -90, "j4": 0, "j6": -45},
    {"j2": 85, "j3": 40, "j4": -60, "j5": 90, "j6": 6},
    {"j1": 105, "j2": 10, "j3": -60, "j4": 105, "j6": 0},
    {"j1": -105, "j2": 45, "j3": -95, "j4": 10, "j5": -150, "j6": 6},
    {"j2": 100, "j3": 58, "j4": -105, "j5": 150, "j6": -45},
    {"j2": 50, "j3": 8, "j4": 105, "j6": -45},
    {"j2": 50, "j3": 8, "j4": -105, "j5": 90, "j6": 0},
    {"j2": -30, "j3": -95, "j4": -105},
    {"j2": 100, "j3": -40, "j4": 0, "j6": -45},
]


# ABENICS (default wrist): wt = (pitch, yaw, roll) of the tool vs. the forearm, j6 = claw half-opening (mm).
# Arm poses inside the soft limits incl. q3_rel = j2 - j3 in [42, 140]; wrist poses at the hard edges.
DEFAULT_POSES_ABENICS = [
    {}, {"j2": 40, "j3": -10, "wt": (-20, 0, 0), "j6": 20},
    {"j2": -30, "j3": -90, "wt": (0, 0, 0), "j6": 32.5},
    {"j2": 85, "j3": 40, "wt": (-30, 0, 90), "j6": 6},
    {"j1": 105, "j2": 10, "j3": -60, "wt": (30, 0, 0), "j6": 0},
    {"j1": -105, "j2": 45, "j3": -95, "wt": (10, 0, -90), "j6": 6},
    {"j2": 100, "j3": 58, "wt": (-30, 0, 90), "j6": 32.5},
    {"j2": 50, "j3": 8, "wt": (33, 0, 0), "j6": 32.5},
    {"j2": 50, "j3": 8, "wt": (-33, 0, 90), "j6": 0},
    {"j2": -30, "j3": -95, "wt": (-30, 0, 0)},
    {"j2": 100, "j3": -40, "wt": (0, 0, 0), "j6": 32.5},
    {"j2": 100, "j3": -40, "wt": (0, 33, 0), "j6": 32.5},
    {"j2": 100, "j3": -40, "wt": (0, -33, 90), "j6": 32.5},
    {"j2": 100, "j3": -40, "wt": (-33, 0, 0), "j6": 32.5},
    {"j2": 45, "j3": -95, "wt": (33, 0, 0), "j6": 32.5},
    {"j2": 45, "j3": -95, "wt": (17, 17, 90), "j6": 32.5},
]


def check(inst, moving_only=True):
    shapes = [(i.name, i.shape, i.group) for i in inst if i.shape is not None
              and not i.name.startswith(("belt", "cord", "spring"))]
    bbs = {n: s.bounding_box() for n, s, _ in shapes}
    bad = []
    for (a, sa, ga), (b, sb, gb) in itertools.combinations(shapes, 2):
        ba, bb = bbs[a], bbs[b]
        if (ba.min.X > bb.max.X or bb.min.X > ba.max.X or ba.min.Y > bb.max.Y or bb.min.Y > ba.max.Y
                or ba.min.Z > bb.max.Z or bb.min.Z > ba.max.Z):
            continue
        if (a, b) in ALLOW or (b, a) in ALLOW:
            continue
        r = sa & sb
        v = r.volume if r is not None else 0.0
        if v > 0.5:
            bad.append((a, b, round(v, 1)))
    return bad


def main():
    poses = DEFAULT_POSES_ABENICS if asm.WRIST == "abenics" else DEFAULT_POSES_VALID
    if len(sys.argv) > 1 and sys.argv[1].endswith(".json"):
        poses = json.loads(pathlib.Path(sys.argv[1]).read_text())
    elif len(sys.argv) > 1 and sys.argv[1].startswith("["):
        poses = json.loads(sys.argv[1])
    t0 = time.time()
    results = []
    for q in poses:
        inst = asm.arm_instances(q)
        bad = check(inst)
        qq = dict(asm.HOME, **q)
        rel = qq["j2"] - qq["j3"]
        print(f"== {json.dumps(q)}  (q3_rel={rel:.0f})  t={time.time()-t0:.0f}s", flush=True)
        print("   INTERFERENCES:", bad, flush=True)
        results.append({"pose": q, "q3_rel": rel, "interferences": bad})
    return results


if __name__ == "__main__":
    main()
