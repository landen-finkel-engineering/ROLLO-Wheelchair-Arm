"""Chair-side assemblies: LEFT and RIGHT installations of the SAME arm on the PROVISIONAL reference
chair (see parts/mount.py).  All placements are computed from params.yaml.

Frames: C = chair (floor, under rear-axle centre; X forward, Y left, Z up), M = post bracket,
B = arm base (dock top).  side = "right" | "left".
"""
from __future__ import annotations
import math
from build123d import Pos, Rot, Location, Vector, Plane

from .config import P
from . import lib
from .assembly import Instance, arm_instances, PRINTED, VITAMIN, COL, _c, HOME
from .parts import mount as mt, base, misc

CH, MT = P.chair, P.mount
REF, ENV = "reference", "envelope"


def side_sign(side: str) -> int:
    return -1 if side == "right" else 1


def park_pose(side: str) -> dict:
    """Stowed pose: arm folded upright, facing straight forward along the chair (J1 = -receiver yaw)."""
    q = dict(HOME)
    q["j1"] = -MT.receiver_yaw_deg[side] - side_sign(side) * P.joints.park_j1_inboard_deg
    return q


def mirror_q(q: dict) -> dict:
    """Side-symmetric joint mapping: a task taught on the RIGHT is mirrored on the LEFT by negating
    the yaw (J1) and the wrist yaw/roll (ABENICS wt = (pitch, yaw, roll) -> (pitch, -yaw, -roll);
    pitch-roll wrist: J5 -> -J5).  J2, J3, J4, J6 are unchanged: the arm itself is not mirrored."""
    out = dict(q)
    for k in ("j1", "j5"):
        if k in out:
            out[k] = -out[k]
    if "wt" in out:
        p_, y_, r_ = out["wt"]
        out["wt"] = (p_, -y_, -r_)
    return out


def frames(side: str):
    s = side_sign(side)
    w2 = CH.seat_width / 2 + CH.post_from_seat_edge
    post = Vector(CH.front_post_x, s * w2, 0.0)
    psi = MT.bracket_yaw_deg[side]
    M = Pos(post.X, post.Y, CH.mount_z) * Rot(0, 0, psi)
    rec_yaw = MT.receiver_yaw_deg[side]
    # receiver centre = shelf centre (bracket +X at shelf_offset)
    c = (M * Pos(mt.shelf_center_x(), 0, 0)).position
    B = Pos(c.X, c.Y, CH.mount_z) * Rot(0, 0, rec_yaw)
    idx = (rec_yaw - psi) / (360.0 / P.dock.index_holes)
    assert abs(idx - round(idx)) < 1e-6, "receiver yaw must be a multiple of the index pitch relative to the shelf"
    return dict(M=M, B=B, psi=psi, post=post, s=s, w2=w2, index_steps=int(round(idx)))


def strut_points(side: str):
    """Torque-strut pivots in C: bracket clevis and the third clamp on the lower rail."""
    f = frames(side)
    p1 = (f["M"] * Pos(*mt.strut_pivot())).position
    p2 = Vector(CH.strut_clamp_x, f["post"].Y, CH.lower_rail_z + mt.PAD_X + 4.0)
    return p1, p2


def mount_instances(side: str, explode: float = 0.0):
    f = frames(side)
    M = f["M"]
    out = []
    zt, zb, zc1, zc2 = mt.bracket_dims()

    def add(name, key, shape, color, kind=PRINTED, loc=None, dvec=(0, 0, 0)):
        s_ = shape if loc is None else loc * shape
        if explode:
            s_ = Pos(dvec[0] * explode, dvec[1] * explode, dvec[2] * explode) * s_
        out.append(Instance(name, key, s_, color, kind, "mount"))

    add("post_bracket", "04_post_bracket", _c(mt.post_bracket), "#2f8f5b", loc=M, dvec=(60, 0, 0))
    for i, zc in enumerate((zc1, zc2)):
        add(f"clamp_body_{i}", "02_tube_clamp_body", _c(mt.clamp_body), "#3aa36b", loc=M * Pos(0, 0, zc), dvec=(30, 0, 0))
        add(f"clamp_cap_{i}", "03_tube_clamp_cap", _c(mt.clamp_cap), "#3aa36b", loc=M * Pos(0, 0, zc), dvec=(-40, 0, 0))
        add(f"clamp_liner_{i}", "V_liner", _c(mt.clamp_liner), "#202020", VITAMIN, loc=M * Pos(0, 0, zc), dvec=(-10, 0, 0))
    # stop collar directly under the lower clamp, lug toward -Y_M (away from the clevis)
    add("stop_collar", "05_stop_collar", _c(mt.stop_collar), "#2f8f5b",
        loc=M * Pos(0, 0, zc2 - mt.H / 2 - 7.5) * Rot(0, 0, 180), dvec=(0, 0, -40))
    # receiver on the shelf, oriented by the index ring
    add("receiver", "01_receiver", _c(base.receiver), "#2f8f5b", loc=f["B"], dvec=(0, 0, 30))
    # torque strut: SI5 rod ends + M5 rod, third clamp on the lower rail (pad facing up)
    p1, p2 = strut_points(side)
    add("strut_rod", "V_strut_M5", _rod(p1, p2, 6.0), COL["metal"], VITAMIN)
    rail_clamp = Pos(CH.strut_clamp_x, f["post"].Y, CH.lower_rail_z) * Rot(0, 90, 0) * Rot(0, 0, 180)
    add("strut_clamp_body", "02_tube_clamp_body", _c(mt.clamp_body), "#3aa36b", loc=rail_clamp)
    add("strut_clamp_cap", "03_tube_clamp_cap", _c(mt.clamp_cap), "#3aa36b", loc=rail_clamp)
    return out


def _rod(p0: Vector, p1: Vector, d):
    v = p1 - p0
    z = v.normalized()
    x = Vector(1, 0, 0) if abs(z.X) < 0.9 else Vector(0, 1, 0)
    return Plane(origin=p0, z_dir=z, x_dir=(x - z * x.dot(z)).normalized()).location * lib.cyl(d, v.length)


def ebox_location(side: str) -> Location:
    """Electronics box on the REAR cane of the arm side, behind the backrest (lid to the rear).
    Right side: box X -> +Z_C, Y -> +Y_C, Z -> -X_C.  Left side: flipped upside-down (same box)."""
    s = side_sign(side)
    w2 = CH.seat_width / 2 + CH.post_from_seat_edge
    zc = CH.ebox_center_z
    x_face = 40.0 - mt.PAD_X              # clamp pad face (clamp +X points to -X_C)
    if side == "right":
        pl = Plane(origin=(x_face - 6.0, s * w2 - misc.EBOX_PAD_Y, zc), x_dir=(0, 0, 1), z_dir=(-1, 0, 0))
    else:
        pl = Plane(origin=(x_face - 6.0, s * w2 + misc.EBOX_PAD_Y, zc), x_dir=(0, 0, -1), z_dir=(-1, 0, 0))
    return pl.location


def ebox_instances(side: str):
    s = side_sign(side)
    w2 = CH.seat_width / 2 + CH.post_from_seat_edge
    loc = ebox_location(side)
    out = [Instance("electronics_box", "60_electronics_box", loc * _c(misc.ebox), "#4d4d4d", PRINTED, "electronics"),
           Instance("electronics_lid", "61_electronics_lid", loc * _c(misc.ebox_lid), "#5a5a5a", PRINTED, "electronics")]
    for i, dz in enumerate(misc.ebox_pad_x()):
        cl = Pos(40.0, s * w2, CH.ebox_center_z + dz) * Rot(0, 0, 180)
        out.append(Instance(f"ebox_clamp_body_{i}", "02_tube_clamp_body", cl * _c(mt.clamp_body), "#3aa36b", PRINTED, "electronics"))
        out.append(Instance(f"ebox_clamp_cap_{i}", "03_tube_clamp_cap", cl * _c(mt.clamp_cap), "#3aa36b", PRINTED, "electronics"))
    return out


def chair_instances(side: str, q: dict | None = None, arm: bool = True, user: bool = True,
                    chair: bool = True, electronics: bool = True, vitamins: bool = True):
    """Full installation in C.  q=None -> park pose for that side."""
    out = []
    f = frames(side)
    if chair:
        for name, shp, col in mt.reference_chair():
            out.append(Instance(f"chair_{name}", "REF", shp, col, REF, "chair"))
    if user:
        for name, shp in mt.user_envelope():
            out.append(Instance(f"user_{name}", "ENV", shp, "#f0a0a0", ENV, "user"))
    out += mount_instances(side)
    if electronics:
        out += ebox_instances(side)
    if arm:
        qq = park_pose(side) if q is None else dict(HOME, **q)
        for i in arm_instances(qq, vitamins=vitamins):
            if i.name == "receiver":
                continue                       # placed by mount_instances
            out.append(Instance(i.name, i.key, f["B"] * i.shape, i.color, i.kind, i.group))
    return out
