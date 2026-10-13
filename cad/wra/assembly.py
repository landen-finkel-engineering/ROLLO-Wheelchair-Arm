"""Kinematic assembly of the arm (the chair side is in chair.py).

`arm_instances(q, explode, vitamins, wrist)` returns Instance(name, key, shape, color, kind, group)
placed in the ARM BASE frame B.  Joint dict q:
  j1  yaw (deg)                     j2  theta2 = upper-arm elevation (deg, absolute)
  j3  theta3 = forearm elevation (deg, ABSOLUTE; parallelogram)
  wrist = "abenics" (WRA-1A, default):  wt = (pitch, yaw, roll) of the tool relative to the forearm
          (deg; ABENICS IK -> carrier angles q1, q2 and monopole spins th1, th2)
  wrist = "pitch_roll" (WRA-1B):        j4 pitch (relative to forearm), j5 roll (deg)
  j6  claw half-opening (mm), 0 = closed
"""
from __future__ import annotations
import math
from dataclasses import dataclass
import numpy as np
from build123d import Pos, Rot, Location, Part, Vector

from .config import P
from . import lib, layout as L, abenics_kin as AK
from .parts import base, turntable as tt, dm, arm, wrist as wr, claw as cw, abenics as ab

A = P.arm
BSP = P.base
WRIST = P.wrist_type
HOME = dict(j1=P.joints.home_deg.j1, j2=P.joints.home_deg.j2, j3=P.joints.home_deg.j3, j4=P.joints.home_deg.j4,
            j5=P.joints.home_deg.j5, j6=8.0, wt=tuple(P.joints.home_abenics.wt))
Q_WIN = [(math.radians(P.abenics.q_window_deg[0]), math.radians(P.abenics.q_window_deg[1]))] * 2
PRINTED, VITAMIN = "printed", "vitamin"
_CACHE: dict = {}


def _c(fn, *args):
    """Build a part once per process (parts depend only on params, not on the pose)."""
    k = (fn.__module__, fn.__name__, args)
    if k not in _CACHE:
        _CACHE[k] = fn(*args)
    return _CACHE[k]

COL = dict(servo="#202020", metal="#b8b8b8", belt="#2a2a2a", bearing="#d0d0d0", spring="#c9a227",
           cord="#f2f2f2", tube="#c8ccd0")


@dataclass
class Instance:
    name: str
    key: str          # part key (PartDef name) or vitamin id
    shape: Part
    color: str
    kind: str = PRINTED
    group: str = ""


def frames(q, wrist=None):
    """Locations (in B) of T, U, K, F, the wrist frames and the tool flange G; points O, E, P, Q.
    pitch_roll: W (pitch), R (roll).  abenics: A (housing), S (CS-gear body), Rm (rotation matrix),
    act = (q1, th1, q2, th2) actuator angles (rad)."""
    wrist = wrist or WRIST
    th2, th3 = math.radians(q["j2"]), math.radians(q["j3"])
    T = Pos(0, 0, L.T_ORIGIN_B) * Rot(0, 0, q["j1"])
    O = L.O
    U = T * Pos(O.X, O.Y, O.Z) * Rot(0, -q["j2"], 0)
    K = T * Pos(O.X, O.Y, O.Z) * Rot(0, -(q["j3"] + 180.0), 0)
    E = Vector(O.X + A.L2 * math.cos(th2), 0, O.Z + A.L2 * math.sin(th2))
    F = T * Pos(E.X, E.Y, E.Z) * Rot(0, -q["j3"], 0)
    c = A.crank
    Pp = Vector(O.X + c * math.cos(th3 + math.pi), A.rod_plane_y, O.Z + c * math.sin(th3 + math.pi))
    Qp = Vector(E.X + c * math.cos(th3 + math.pi), A.rod_plane_y, E.Z + c * math.sin(th3 + math.pi))
    out = dict(T=T, U=U, K=K, F=F, O=O, E=E, P=Pp, Q=Qp)
    if wrist == "pitch_roll":
        W = F * Pos(A.L3, 0, 0) * Rot(0, -q["j4"], 0)
        R = W * Pos(0, wr.YC, 0) * Rot(q["j5"], 0, 0) * Pos(0, -wr.YC, 0)
        out.update(W=W, R=R, G=R * Pos(wr.X_FLANGE1, wr.YC, 0))
    else:
        Af = F * Pos(A.L3, wr.YC, 0)
        Rm = AK.tool_rotation_pyr(*q["wt"])
        act = AK.ik(Rm, windows=Q_WIN)
        S = Af * ab.S_loc(Rm)
        out.update(A=Af, S=S, Rm=Rm, act=act, G=S * ab.G_in_S())
    return out


def tcp_B(q, wrist=None):
    f = frames(dict(HOME, **q), wrist)
    return (f["G"] * Pos(cw.tcp_x(), 0, 0)).position


def _spring_visual(p0: Vector, p1: Vector, od=10.0):
    v = p1 - p0
    ln = v.length
    if ln < 1:
        return None
    s = lib.tube(od, od - 2.4, ln)
    # orient +Z to v
    z = v.normalized()
    from build123d import Plane
    x = Vector(1, 0, 0) if abs(z.X) < 0.9 else Vector(0, 1, 0)
    pl = Plane(origin=p0, z_dir=z, x_dir=(x - z * x.dot(z)).normalized())
    return pl.location * s


def _rod(p0: Vector, p1: Vector, d):
    v = p1 - p0
    ln = v.length
    if ln < 0.5:
        return None
    from build123d import Plane
    z = v.normalized()
    x = Vector(1, 0, 0) if abs(z.X) < 0.9 else Vector(0, 1, 0)
    pl = Plane(origin=p0, z_dir=z, x_dir=(x - z * x.dot(z)).normalized())
    return pl.location * lib.cyl(d, ln)


def arm_instances(q=None, explode: float = 0.0, vitamins: bool = True, side: str | None = None, wrist=None):
    wrist = wrist or WRIST
    q = dict(HOME, **(q or {}))
    f = frames(q, wrist)
    T, U, K, F, G = (f[k] for k in "TUKFG")
    ex = explode
    out: list[Instance] = []

    def add(name, key, shape, color, kind=PRINTED, group="", loc=None, dz=0.0, dvec=None):
        if shape is None:
            return
        s = shape if loc is None else loc * shape
        if ex and (dz or dvec):
            dv = dvec if dvec is not None else (0, 0, dz)
            s = Pos(dv[0] * ex, dv[1] * ex, dv[2] * ex) * s
        out.append(Instance(name, key, s, color, kind, group))

    # ---------------------------------------------------------------- base (B)
    add("receiver", "01_receiver", _c(base.receiver), "#2f8f5b", group="dock", dz=-60)
    add("base_housing", "10_base_housing", _c(base.base_housing), "#5b6b7b", group="base")
    add("j1_fixed_pulley", "11_j1_fixed_pulley", _c(base.fixed_pulley), "#e0a030", group="base", dz=40)
    # ---------------------------------------------------------------- turntable (T)
    tz = 120
    add("turntable", "20_turntable", _c(tt.plate), "#8a9aa9", loc=T, group="turntable", dz=tz)
    add("stub_cheek", "21_stub_cheek", _c(tt.stub_cheek), "#7f8fa0", loc=T, group="turntable", dz=tz + 40, dvec=None)
    add("ddm_bracket_j2", "22_ddm_bracket_j2", _c(tt.ddm_bracket_j2), "#4a7bd0", loc=T, group="turntable", dz=tz + 60)
    add("ddm_bracket_j3", "22_ddm_bracket_j3", _c(tt.ddm_bracket_j3), "#3d6ab8", loc=T, group="turntable", dz=tz + 60)
    for nm, dl in (("j1", L.J1_DM_LOC_T), ("j2", L.J2_DM_LOC_T), ("j3", L.J3_DM_LOC_T)):
        dzz = tz - 30 if nm == "j1" else tz + 60
        add(f"{nm}_dm_pulley", "12_dm_pulley36", _c(dm.pulley), "#e0a030", loc=T * dl, group="turntable", dz=dzz)
        add(f"{nm}_dm_cap", "12_dm_cap", _c(dm.cap), "#4a7bd0", loc=T * dl, group="turntable", dz=dzz)
        if vitamins:
            add(f"{nm}_servo", "ST3215", _c(lib.servo_model), COL["servo"], VITAMIN, loc=T * dl * dm.servo_loc(), group="turntable", dz=dzz)
    add("j1_dm_bracket", "12_dm_bracket", _c(dm.bracket), "#4a7bd0", loc=T * L.J1_DM_LOC_T, group="turntable", dz=tz - 30)
    # ---------------------------------------------------------------- shoulder
    add("ua_hub", "30_ua_hub", _c(arm.ua_hub), "#e0a030", loc=U, group="upper_arm", dz=tz + 140)
    add("j3_hub", "31_j3_hub", _c(arm.j3_hub), "#d08a20", loc=K, group="shoulder", dz=tz + 140)
    add("elbow_block", "32_elbow_block", _c(arm.elbow_block), "#7f8fa0", loc=U, group="upper_arm", dz=tz + 140)
    add("forearm_root", "34_forearm_root", _c(arm.forearm_root), "#e0a030", loc=F, group="forearm", dz=tz + 200)
    for k_, s_ in enumerate(_c(arm.stub_spacers)):
        add(f"stub_spacer_{k_}", "35_stub_spacer", s_, "#e0a030", loc=T, group="shoulder", dz=tz + 150)
    # rod ends
    th2 = q["j2"]
    Pp, Qp = f["P"], f["Q"]
    y0 = A.rod_plane_y - 4.0
    add("rod_end_P", "33_rod_end", _c(arm.rod_end), "#6a7f99",
        loc=T * Pos(Pp.X, y0, Pp.Z) * Rot(0, -th2, 0), group="shoulder", dz=tz + 140)
    add("rod_end_Q", "33_rod_end", _c(arm.rod_end), "#6a7f99",
        loc=T * Pos(Qp.X, y0, Qp.Z) * Rot(0, -(th2 + 180.0), 0), group="forearm", dz=tz + 200)
    # ---------------------------------------------------------------- wrist
    if wrist == "pitch_roll":
        W, R = f["W"], f["R"]
        add("wrist_housing", "40_wrist_housing", _c(wr.wrist_housing), "#7f8fa0", loc=F, group="forearm", dz=tz + 200)
        add("j4_dm_pulley", "12_dm_pulley36", _c(dm.pulley), "#e0a030", loc=F * wr.J4_DM_LOC_F, group="forearm", dz=tz + 200)
        add("j4_dm_cap", "12_dm_cap", _c(dm.cap), "#4a7bd0", loc=F * wr.J4_DM_LOC_F, group="forearm", dz=tz + 200)
        add("pitch_yoke", "41_pitch_yoke", _c(wr.yoke), "#e0a030", loc=W, group="wrist", dz=tz + 240)
        add("yoke_cap", "42_yoke_cap", _c(wr.yoke_cap), "#d08a20", loc=W, group="wrist", dz=tz + 240)
        add("j4_pulley54", "43_j4_pulley54", _c(wr.pulley4), "#e0a030", loc=W, group="wrist", dz=tz + 240)
        add("j5_horn_adapter", "44_j5_horn_adapter", _c(wr.horn_adapter), "#c06020", loc=R, group="wrist", dz=tz + 240)
        add("roll_spindle", "45_roll_spindle", _c(wr.spindle), "#b0b0b0", loc=R, group="wrist", dz=tz + 260)
        add("roll_spacer", "47_roll_spacer", _c(wr.roll_spacer), "#b0b0b0", loc=R, group="wrist", dz=tz + 260)
        add("tool_flange", "46_tool_flange", _c(wr.tool_flange), "#c0c0c0", loc=R, group="wrist", dz=tz + 270)
    else:
        out.extend(abenics_instances(f, add_vitamins=vitamins, explode=ex, tz=tz))
    # ---------------------------------------------------------------- claw (G)
    out.extend(claw_instances(G, q["j6"], vitamins=False, explode=ex, tz=tz))
    if not vitamins:
        return out
    # ---------------------------------------------------------------- vitamins
    # dock: 2 hardened dowels pressed into the receiver, 2 captive DIN 464 M6 thumb screws
    DK = P.dock
    for i, (x, y) in enumerate(DK.dowel_xy):
        add(f"dock_dowel_{i}", "V_dowel6x20", lib.cyl(DK.dowel_dia, DK.dowel_len, x, y, -12.5), COL["metal"], VITAMIN,
            group="dock", dz=-30)
    tp = P.base.dock_plate_t
    for i, (x, y) in enumerate(DK.thumb_xy):
        ts = lib.cyl(6.0, 25.0, x, y, tp + 7.5 - 25.0) + lib.cyl(20.0, 7.5, x, y, tp)
        add(f"thumb_screw_{i}", "V_DIN464_M6x25", ts, COL["metal"], VITAMIN, group="dock", dz=60)
    add("stub", "V_stub15", _c(tt.stub), COL["metal"], VITAMIN, loc=T, group="shoulder", dz=tz + 40)
    for i, p_ in enumerate(_c(tt.ddm_pins)):
        add(f"ddm_pin_{i}", f"V_dowel8x{P.ddm.pin_len:.0f}", p_, COL["metal"], VITAMIN, loc=T, group="turntable", dz=tz + 60)
    for i, t_ in enumerate(_c(arm.ua_tubes)):
        add(f"ua_tube_{i}", "V_tube12", t_, COL["tube"], VITAMIN, loc=U, group="upper_arm", dz=tz + 140)
    for i, t_ in enumerate(_c(wr.fa_tubes)):
        add(f"fa_tube_{i}", "V_tube12", t_, COL["tube"], VITAMIN, loc=F, group="forearm", dz=tz + 200)
    add("elbow_pin", "V_pin10", _c(arm.elbow_pin), COL["metal"], VITAMIN, loc=U, group="upper_arm", dz=tz + 140)
    add("elbow_spacer", "V_tube12_spacer", _c(arm.elbow_spacer), COL["tube"], VITAMIN, loc=U, group="upper_arm", dz=tz + 200)
    add("elbow_collar", "V_collar10", _c(arm.elbow_collar), COL["metal"], VITAMIN, loc=U, group="upper_arm", dz=tz + 200)
    add("stub_collar", "V_collar15", _c(arm.stub_collar), COL["metal"], VITAMIN, loc=T, group="shoulder", dz=tz + 160)
    add("j1_collar", "V_collar30", lib.tube(BSP.collar[1], BSP.collar[0], BSP.collar[2], z=BSP.collar_z[0]),
        COL["metal"], VITAMIN, group="base", dz=-20)
    add("j1_dm_pin", f"V_dowel8x{P.dm.pin_len:.0f}", _c(dm.pin), COL["metal"], VITAMIN, loc=T * L.J1_DM_LOC_T, group="turntable", dz=tz - 30)
    if wrist == "pitch_roll":
        add("j4_dm_pin", f"V_dowel8x{P.dm.pin_len:.0f}", _c(dm.pin), COL["metal"], VITAMIN, loc=F * wr.J4_DM_LOC_F, group="forearm", dz=tz + 200)
        for i, p_ in enumerate(_c(wr.pitch_pins)):
            add(f"pitch_pin_{i}", "V_dowel8x16", p_, COL["metal"], VITAMIN, loc=F, group="forearm", dz=tz + 200)
        add("j4_servo", "ST3215", _c(lib.servo_model), COL["servo"], VITAMIN, loc=F * wr.J4_DM_LOC_F * dm.servo_loc(), group="forearm", dz=tz + 200)
        add("j5_servo", "ST3215", _c(lib.servo_model), COL["servo"], VITAMIN, loc=f["W"] * wr.j5_servo_loc(), group="wrist", dz=tz + 240)
    out.extend(claw_vitamins(G, q["j6"], explode=ex, tz=tz))
    # rod tube
    d = (Qp - Pp).normalized()
    add("rod_tube", "V_tube10", _rod(Pp + d * arm.ROD_SOCK[0] + Vector(0, 0, 0), Qp - d * arm.ROD_SOCK[0], A.rod_od), COL["tube"],
        VITAMIN, loc=T * Pos(0, 0, 0), group="shoulder", dz=tz + 140)
    if not ex:
        out.extend(belts_and_springs(q, f))
    return out


def claw_instances(G, qo, vitamins=True, explode=0.0, tz=120.0):
    """Knob claw (printed parts; + vitamins) on the tool flange frame G at half-opening qo (mm)."""
    out = []

    def add(name, key, shape, color, dz):
        s_ = G * shape
        if explode:
            s_ = Pos(0, 0, dz * explode) * s_
        out.append(Instance(name, key, s_, color, PRINTED, "gripper"))
    add("tool_adapter", "56_tool_adapter", _c(cw.tool_adapter), "#c0c0c0", tz + 290)
    add("claw_palm", "50_claw_palm", _c(cw.palm), "#5b6b7b", tz + 310)
    add("claw_cradle_cap", "51_claw_cradle_cap", _c(cw.cradle_cap), "#5b6b7b", tz + 300)
    add("claw_pinion", "52_claw_pinion", cw.pinion_at(qo), "#e0a030", tz + 320)
    add("claw_jaw_A", "53_claw_jaw_a", cw.jaw(qo, 1), "#e0a030", tz + 330)
    add("claw_jaw_B", "53_claw_jaw_b", cw.jaw(qo, -1), "#d08a20", tz + 330)
    add("claw_pad_A", "54_claw_pad_a", cw.pad(qo, 1), "#303030", tz + 335)
    add("claw_pad_B", "54_claw_pad_b", cw.pad(qo, -1), "#303030", tz + 335)
    if vitamins:
        out.extend(claw_vitamins(G, qo, explode, tz))
    return out


def claw_vitamins(G, qo, explode=0.0, tz=120.0):
    out = []

    def add(name, key, shape, color, dz):
        s_ = G * shape
        if explode:
            s_ = Pos(0, 0, dz * explode) * s_
        out.append(Instance(name, key, s_, color, VITAMIN, "gripper"))
    add("claw_servo", "ST3215", cw.servo_loc() * _c(lib.servo_model), COL["servo"], tz + 310)
    add("claw_rail", "V_MGN9_rail", _c(cw.rail), COL["metal"], tz + 315)
    for i, b_ in enumerate(cw.blocks(qo)):
        add(f"claw_block_{i}", "V_MGN9C_block", b_, COL["tube"], tz + 318)
    return out


def wrist_instances(wt=None, j6=None, vitamins=True, q_prev=(0.0, 0.0)):
    """ABENICS wrist + claw alone, in the housing frame A (A = identity), for quick checks."""
    wt = tuple(wt if wt is not None else HOME["wt"])
    Rm = AK.tool_rotation_pyr(*wt)
    act = AK.ik(Rm, q_prev=q_prev, windows=Q_WIN)
    S = ab.S_loc(Rm)
    f = dict(A=Location(), S=S, Rm=Rm, act=act, G=S * ab.G_in_S())
    out = abenics_instances(f, add_vitamins=vitamins)
    out += claw_instances(f["G"], HOME["j6"] if j6 is None else j6, vitamins=vitamins)
    for i, t_ in enumerate(_c(wr.fa_tubes)):
        out.append(Instance(f"fa_tube_{i}", "V_tube12", Pos(-A.L3, -wr.YC, 0) * t_, COL["tube"], VITAMIN, "forearm"))
    return out, f


def belts_and_springs(q, f):
    """Visual belts, spring cords and springs (not exported as parts)."""
    out = []
    T, U, K, F = f["T"], f["U"], f["K"], f["F"]
    # J1 belt: DM pulley orbits fixed pulley -> belt plane z = J1 teeth centre (B)
    b1 = lib.belt_solid(P.pulleys.j1, P.dm.teeth, L.C1, 180.0)
    out.append(Instance("belt_j1", "V_belt240", T * Pos(0, 0, L.J1_TEETH_B - L.T_ORIGIN_B - 7.5) * b1,
                        COL["belt"], VITAMIN, "belts"))
    # J2 / J3 belts between DDM axis and O (plane XZ at y = teeth centre)
    for nm, teeth, yc in (("belt_j2", P.pulleys.j2, A.y_86t_center), ("belt_j3", P.pulleys.j3, A.y_72t_center)):
        cc = L.C2 if nm == "belt_j2" else L.C3
        b = lib.belt_solid(teeth, P.dm.teeth, cc, 180.0)
        loc = T * Pos(L.O.X, yc - 7.5, L.O.Z) * Rot(-90, 0, 0)
        out.append(Instance(nm, "V_belt", loc * b, COL["belt"], VITAMIN, "belts"))
    if "W" in f:
        b4 = lib.belt_solid(P.dm.teeth, P.pulleys.j4, L.C4, 0.0)
        loc4 = F * Pos(A.L3 - L.C4, wr.TEETH4_C - 7.5, 0) * Rot(-90, 0, 0)
        out.append(Instance("belt_j4", "V_belt276", loc4 * b4, COL["belt"], VITAMIN, "belts"))
    # J3 spring cord: B3 (on K frame) -> A3 (T frame) -> back to springs
    B3 = (K * Pos(P.springs.b3, A.j3_cord_y, 0)).position
    A3 = (T * Pos(L.A3.X, L.A3.Y, L.A3.Z)).position
    c = _rod(B3, A3, 1.5)
    if c is not None:
        out.append(Instance("cord_j3", "V_cord", c, COL["cord"], VITAMIN, "springs"))
    for dy in (-4.5, 4.5):
        p0 = (T * Pos(-14, L.A3.Y + dy, L.A3.Z - 5)).position
        p1 = (T * Pos(-14 - 60, L.A3.Y + dy, L.A3.Z - 5)).position
        s = _spring_visual(p0, p1, 9.0)
        out.append(Instance(f"spring_j3_{dy}", "V_spring", s, COL["spring"], VITAMIN, "springs"))
    # J2 cord: A2 (T) -> B2 (U rear lever) -> C2 -> along upper arm
    A2 = (T * Pos(L.A2.X, L.A2.Y, L.A2.Z)).position
    B2 = (U * Pos(-P.springs.b2, A.ua_plane_y, 0)).position
    C2 = (U * Pos(-P.springs.b2, A.ua_plane_y, P.springs.j2_offset)).position
    for a_, b_ in ((A2, B2), (B2, C2)):
        c = _rod(a_, b_, 1.5)
        if c is not None:
            out.append(Instance("cord_j2", "V_cord", c, COL["cord"], VITAMIN, "springs"))
    for dy in (-5.0, 5.0):
        p0 = (U * Pos(70, A.ua_plane_y + dy, P.springs.j2_offset + 5)).position
        p1 = (U * Pos(150, A.ua_plane_y + dy, P.springs.j2_offset + 5)).position
        out.append(Instance(f"spring_j2_{dy}", "V_spring", _spring_visual(p0, p1, 9.0), COL["spring"], VITAMIN, "springs"))
    return out


def abenics_instances(f, add_vitamins=True, explode=0.0, tz=120.0):
    """ABENICS wrist parts in B for the frames f (needs f['A'], f['S'], f['act'])."""
    Af, S = f["A"], f["S"]
    q1, t1, q2, t2 = f["act"]
    out = []

    def add(name, key, shape, color, kind=PRINTED, group="forearm", loc=None, dz=0.0):
        s_ = shape if loc is None else loc * shape
        if explode and dz:
            s_ = Pos(0, 0, dz * explode) * s_
        out.append(Instance(name, key, s_, color, kind, group))

    add("cs_gear_front", "80_cs_gear_front", _c(ab.cs_front_blank), "#e0a030", group="tool", loc=S, dz=tz + 250)
    add("cs_gear_back", "81_cs_gear_back", _c(ab.cs_back_blank), "#d08a20", group="tool", loc=S, dz=tz + 230)
    add("abenics_housing", "89_abenics_housing", _c(ab.housing), "#7f8fa0", loc=Af, dz=tz + 200)
    add("center_ball", "90_center_ball", _c(ab.center_ball), "#c0c0c0", loc=Af, dz=tz + 215)
    for k, qq, th in ((1, q1, t1), (2, q2, t2)):
        Mk = Af * ab.M_loc(k)
        Ck = Af * ab.Cq_loc(k, qq)
        g = f"wrist_m{k}"
        add(f"module_frame_{k}", "87_module_frame", _c(ab.module_frame), "#7f8fa0", group=g, loc=Mk, dz=tz + 280)
        add(f"holder_retainer_{k}", "91_holder_retainer", _c(ab.holder_retainer), "#7f8fa0", group=g, loc=Mk, dz=tz + 272)
        add(f"q_cap_{k}", "88_q_servo_cap", _c(ab.q_cap), "#7f8fa0", group=g, loc=Mk, dz=tz + 290)
        add(f"q_coupling_{k}", "86_q_coupling", _c(ab.q_coupling), "#c06020", group=g,
            loc=Mk * Rot(0, 0, math.degrees(qq)), dz=tz + 285)
        add(f"mp_fork_{k}", "83_mp_fork", _c(ab.mp_fork), "#4a7bd0", group=g, loc=Ck, dz=tz + 255)
        add(f"mp_cradle_{k}", "84_mp_cradle", _c(ab.mp_cradle), "#4a7bd0", group=g, loc=Ck, dz=tz + 265)
        add(f"cradle_strap_{k}", "92_cradle_strap", _c(ab.cradle_strap), "#4a7bd0", group=g, loc=Ck, dz=tz + 268)
        add(f"mp_spacer_{k}", "93_mp_spacer", _c(ab.mp_spacer), "#c0c0c0", group=g, loc=Ck, dz=tz + 255)
        phi_s = -math.degrees(th) * ab.TG.mp_teeth / ab.TG.servo_teeth     # servo gear counter-rotates
        add(f"theta_gear_{k}", "85_theta_gear", Pos(0, 0, ab.Z_TS) * Rot(phi_s, 0, 0) * Pos(0, 0, -ab.Z_TS) *
            _c(ab.theta_gear), "#e0a030", group=g, loc=Ck, dz=tz + 262)
        add(f"monopole_{k}", "82_monopole", _c(ab.monopole_blank), "#d08a20", group=g,
            loc=Af * ab.Mp_loc(k, qq, th), dz=tz + 255)
        if add_vitamins:
            add(f"theta_servo_{k}", "ST3215", _c(lib.servo_model), COL["servo"], VITAMIN, group=g,
                loc=Ck * ab.ts_loc(), dz=tz + 265)
            add(f"q_servo_{k}", "ST3215", _c(lib.servo_model), COL["servo"], VITAMIN, group=g,
                loc=Mk * ab.qs_loc(), dz=tz + 290)
            add(f"bearing_6808_{k}", "V_6808", lib.tube(ab.D6808, ab.d6808, ab.B6808, z=ab.BRG_Z[0]), COL["bearing"],
                VITAMIN, group=g, loc=Ck, dz=tz + 258)
            add(f"bearing_6805_{k}", "V_6805", lib.tube(ab.D6805, ab.d6805, ab.B6805, z=ab.FAR_BRG_Z[0]),
                COL["bearing"], VITAMIN, group=g, loc=Ck, dz=tz + 275)
            for j, x0 in enumerate((-ab.AB.mp_width / 2, 0.0)):
                add(f"bearing_688_{k}{j}", "V_688", lib.tube(16.0, 8.0, ab.AB.mp_width / 2, x=x0, axis="X"),
                    COL["bearing"], VITAMIN, group=g, loc=Af * ab.Mp_loc(k, qq, th), dz=tz + 255)
            add(f"mp_pin_{k}", "V_dowel8x30", lib.cyl(ab.PIN_D, ab.PIN_L, ab.PIN_X[0], 0, ab.D_MP, axis="X"),
                COL["metal"], VITAMIN, group=g, loc=Ck, dz=tz + 255)
    if add_vitamins:
        add("abenics_post", "V_tube8x1", _c(ab.post), COL["tube"], VITAMIN, loc=Af, dz=tz + 210)
    return out
