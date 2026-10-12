"""Forearm tubes, wrist housing (J4 drive + pitch fork), pitch yoke (J5 roll servo + roll
bearings), roll spindle and tool flange.

Frames:
  F : forearm frame (origin elbow axis E, X_F along forearm).
  W : wrist frame (origin on pitch axis, X_W = tool axis at q4 = 0, Y shared).
  R : roll frame (W rotated by q5 about X_W).  Tool-flange FRONT face at x_R = X_FLANGE1.
"""
from __future__ import annotations
import math
from build123d import Pos, Rot, Part, Plane, Location

from ..config import P
from .. import lib, layout as L
from ..lib import box, cyl, clr, PartDef, on_bed, prism
from . import dm
from .arm import ycyl, tube_bosses, tube_sockets, FA_Y, SOCK_D, R0, SOCK_LEN

A = P.arm
WR = P.wrist
S = P.servo
C = P.fit.servo_pocket_clear
L3 = A.L3
YC = FA_Y                                     # wrist / forearm centre plane (16.625)
d688, D688, B688 = lib.bearing(WR.pitch_bearing)
d02, D02, B02 = lib.bearing(WR.roll_bearing)

# ---- J5 servo inside the yoke (servo X -> +Z_W, servo Z -> +X_W, servo Y -> -Y_W)
J5_T = (0.0, YC, -S.out_x)                    # servo-frame origin (body centre) in W
YOKE_IN = dict(x=(S.cover_bot_z - C, S.cover_top_z + C),
               y=(YC - S.body.y[1] - C, YC + S.body.y[1] + C),
               z=(S.body.x[0] - S.out_x - C, S.body.x[1] - S.out_x + C))
WALL = WR.yoke_wall
WALL_P = WR.yoke_wall_pulley_side
YOKE_Y = (YOKE_IN["y"][0] - WALL_P, YOKE_IN["y"][1] + WALL)
YOKE_X = (YOKE_IN["x"][0] - WALL, YOKE_IN["x"][1] + WALL)
YOKE_Z = (YOKE_IN["z"][0] - WALL, YOKE_IN["z"][1] + WALL)
CAP_T = 4.0
HORN_FACE_X = S.horn.z1                       # 18.7
# roll housing (part of yoke)
RH_X = (YOKE_X[1] - 0.01, 50.0)
RB_REAR = (27.5, 27.5 + B02)
RB_FRONT = (RH_X[1] - 0.5 - B02, RH_X[1] - 0.5)
X_FLANGE0 = RH_X[1] + 0.5
X_FLANGE1 = X_FLANGE0 + WR.tool_flange.t
# 54T pulley on the -Y face of the yoke
PUL4_Y = (YOKE_Y[0] - lib.pulley_len(), YOKE_Y[0])
TEETH4_C = (PUL4_Y[0] + PUL4_Y[1]) / 2
# J4 drive module in F: x_dm -> +X_F (toward W), z_dm -> -Y_F, y_dm -> +Z_F
J4_HORN_Y = TEETH4_C + L.DM_TEETH_C
J4_DM_LOC_F = L.dm_location((L3 - L.C4, J4_HORN_Y, 0.0), (1, 0, 0), (0, -1, 0))
CHEEK_N = (J4_HORN_Y - (dm.Z_WALL0 + dm.D.pin_wall), J4_HORN_Y - dm.Z_WALL0)    # -Y cheek = J4 pin wall
CHEEK_P = (YOKE_Y[1] + 2.5, YOKE_Y[1] + 10.5)                                    # +Y cheek
X_WS = 117.0                                  # wrist-housing socket bottom (forearm tubes end before J4 DM)


def j5_servo_loc() -> Location:
    return Plane(origin=J5_T, x_dir=(0, 0, 1), z_dir=(1, 0, 0)).location


# =============================================================================== yoke (W)
def yoke() -> Part:
    body = box(YOKE_X[0] + CAP_T, YOKE_X[1], YOKE_Y[0], YOKE_Y[1], YOKE_Z[0], YOKE_Z[1])
    body -= box(YOKE_IN["x"][0] - CAP_T - 1, YOKE_IN["x"][1], YOKE_IN["y"][0], YOKE_IN["y"][1],
                YOKE_IN["z"][0], YOKE_IN["z"][1])
    # +Y pitch bearing boss
    body += ycyl(24.0, YOKE_Y[1] - 0.01, YOKE_Y[1] + 2.0, 0, 0)
    body -= ycyl(lib.seat(WR.pitch_bearing), YOKE_Y[1] + 2.0 - B688, YOKE_Y[1] + 2.1, 0, 0)
    body -= ycyl(d688 + 2.0, YOKE_Y[1] - 6, YOKE_Y[1] + 3, 0, 0)
    # roll housing
    body += cyl(34.0, RH_X[1] - RH_X[0], RH_X[0], YC, 0, axis="X")
    body -= cyl(24.5, RH_X[1] - YOKE_X[1] + 1, YOKE_X[1] - 4.5, YC, 0, axis="X")
    body -= cyl(d02 + 6.0, RH_X[1] - RH_X[0] + 2, RH_X[0] - 1, YC, 0, axis="X")
    for x0, x1 in (RB_REAR, RB_FRONT):
        body -= cyl(lib.seat(WR.roll_bearing), x1 - x0 + 0.01, x0 - 0.005, YC, 0, axis="X")
    # 54T pulley inserts (4x M3) in the thick -Y wall, BC r = 20 around the pitch axis
    for x, z in pulley4_bolt_xz():
        body -= ycyl(lib.ins_hole("M3"), YOKE_Y[0] - 0.01, YOKE_Y[0] + lib.ins_depth("M3"), x, z)
    # rear cap screws (4x M3 inserts in the rear face corners)
    for y, z in yoke_cap_yz():
        body -= cyl(lib.ins_hole("M3"), lib.ins_depth("M3"), YOKE_X[0] + CAP_T - 0.01, y, z, axis="X")
    return body


def pulley4_bolt_xz():
    return [(14.0, 9.0), (-14.0, 9.0), (14.0, -18.0), (-14.0, -18.0)]


def yoke_cap_yz():
    return [(YOKE_Y[0] + 4.0, YOKE_Z[0] + 4.0), (YOKE_Y[0] + 4.0, YOKE_Z[1] - 4.0),
            (YOKE_Y[1] - 2.5, YOKE_Z[0] + 4.0), (YOKE_Y[1] - 2.5, YOKE_Z[1] - 4.0)]


def yoke_cap() -> Part:
    """Rear frame: clamps the J5 servo forward; big window clears rear horn, block, cables."""
    body = box(YOKE_X[0], YOKE_X[0] + CAP_T, YOKE_Y[0], YOKE_Y[1], YOKE_Z[0], YOKE_Z[1])
    body -= box(YOKE_X[0] - 1, YOKE_X[0] + CAP_T + 1, YOKE_IN["y"][0] + 2.0, YOKE_IN["y"][1] - 2.0,
                YOKE_IN["z"][0] + 2.0, YOKE_IN["z"][1] + 0.01)
    for y, z in yoke_cap_yz():
        body -= cyl(clr("M3"), CAP_T + 2, YOKE_X[0] - 1, y, z, axis="X")
    return body


def pulley4() -> Part:
    body = Pos(0, PUL4_Y[0], 0) * Rot(-90, 0, 0) * lib.htd_pulley(P.pulleys.j4)
    body += ycyl(lib.od(P.pulleys.j4) - 3.0, PUL4_Y[0], PUL4_Y[1])
    body -= ycyl(lib.seat(WR.pitch_bearing), PUL4_Y[0] - 0.01, PUL4_Y[0] + B688, 0, 0)
    body -= ycyl(d688 + 2.0, PUL4_Y[0] + B688 - 0.01, PUL4_Y[1] + 1, 0, 0)
    for x, z in pulley4_bolt_xz():
        body -= ycyl(clr("M3"), PUL4_Y[0] - 1, PUL4_Y[1] + 1, x, z)
        body -= ycyl(6.0, PUL4_Y[0] - 1, PUL4_Y[1] - 4.0, x, z)
    return body


# =============================================================================== roll (R)
def horn_adapter() -> Part:
    """Bolted to the J5 horn BEFORE the servo goes into the yoke; two dogs drive the spindle
    (torque-only coupling: radial misalignment cannot load the servo shaft)."""
    x0 = HORN_FACE_X + 0.2
    body = cyl(19.0, 3.0, x0, YC, 0, axis="X")
    for a in (0.0, 180.0):
        y, z, _ = lib.polar(6.5, a)
        body += Pos(x0 + 3.0, YC + y, z) * Rot(0, 0, 0) * box(0, 4.0, -2.4, 2.4, -2.4, 2.4)
    for i in range(4):
        y, z, _ = lib.polar(S.horn.bolt_circle / 2, S.horn.bolt_angle0 + 90 * i)
        body -= cyl(clr("M3"), 10, x0 - 1, YC + y, z, axis="X")
        body -= cyl(6.0, 10, x0 + 1.8, YC + y, z, axis="X")
    body -= cyl(7.0, 2.0, x0 - 0.01, YC, 0, axis="X")
    return body


def spindle() -> Part:
    x_rear = RB_REAR[0] - 3.5
    body = cyl(20.0, RB_REAR[0] - x_rear, x_rear, YC, 0, axis="X")              # rear shoulder
    body += cyl(d02 - 0.02, RB_FRONT[1] - RB_REAR[0] + 0.02, RB_REAR[0] - 0.01, YC, 0, axis="X")
    # dog slots in the rear face
    for a in (0.0, 180.0):
        y, z, _ = lib.polar(6.5, a)
        body -= box(x_rear - 1, x_rear + 3.0, YC + y - 2.7, YC + y + 2.7, z - 2.7, z + 2.7)
    # front: M5 insert for the tool-flange screw + 2x 3 mm anti-rotation pins
    xf = RB_FRONT[1]
    body -= cyl(lib.ins_hole("M5"), lib.ins_depth("M5"), xf - lib.ins_depth("M5") + 0.01, YC, 0, axis="X")
    for a in (90.0, 270.0):
        y, z, _ = lib.polar(5.0, a)
        body -= cyl(3.0 - P.fit.pin_press_minus, 6.0, xf - 6.0 + 0.01, YC + y, z, axis="X")
    return body


def roll_spacer() -> Part:
    """Printed spacer between the two 6802 inner rings on the roll spindle (R frame)."""
    x0, x1 = RB_REAR[1] + 0.02, RB_FRONT[0] - 0.02
    return cyl(17.4, x1 - x0, x0, YC, 0, axis="X") - cyl(15.3, x1 - x0 + 2, x0 - 1, YC, 0, axis="X")


def tool_flange() -> Part:
    tf = WR.tool_flange
    x0 = RB_FRONT[1] + 0.05
    body = cyl(d02 + 3.0, X_FLANGE0 - x0, x0, YC, 0, axis="X")                   # shoulder on inner ring
    body += cyl(tf.dia, tf.t, X_FLANGE0, YC, 0, axis="X")
    body -= cyl(clr("M5"), 30, x0 - 1, YC, 0, axis="X")
    body -= cyl(9.5, 4.0, X_FLANGE1 - 4.0, YC, 0, axis="X")
    for a in (90.0, 270.0):
        y, z, _ = lib.polar(5.0, a)
        body -= cyl(3.05, 4.0, x0 - 0.01, YC + y, z, axis="X")
    for i in range(tf.n):
        y, z, _ = lib.polar(tf.bolt_circle / 2, 90 * i)
        body -= cyl(lib.ins_hole("M3"), lib.ins_depth("M3") - 1.0, X_FLANGE1 - lib.ins_depth("M3") + 1.01, YC + y, z, axis="X")
    y, z, _ = lib.polar(tf.bolt_circle / 2, 45)
    body -= cyl(tf.dowel_dia - P.fit.pin_press_minus, 4.0, X_FLANGE1 - 4.0 + 0.01, YC + y, z, axis="X")
    return body


# =============================================================================== wrist housing (F)
def wrist_housing() -> Part:
    """Tube sockets + J4 drive-module cradle + both pitch-fork cheeks in one printed part."""
    offs = (A.fa_tube_offset, -A.fa_tube_offset)
    xs0 = X_WS - SOCK_LEN
    body = tube_bosses(xs0 - 4, X_WS + 4, offs, YC)
    # J4 DM bracket (its pin wall becomes the -Y fork cheek)
    dmb = J4_DM_LOC_F * dm.bracket(mount=False, tension_lug=False)
    body += dmb
    # extend -Y cheek forward to the pitch axis, round end
    xd = L3 - L.C4
    body += box(xd + dm.WALL_X0, L3, CHEEK_N[0], CHEEK_N[1], -dm.POST_Y1, dm.POST_Y1)
    body += ycyl(2 * dm.POST_Y1, CHEEK_N[0], CHEEK_N[1], L3, 0)
    # +Y cheek
    body += box(xd + dm.WALL_X0, L3, CHEEK_P[0], CHEEK_P[1], -dm.POST_Y1, dm.POST_Y1)
    body += ycyl(2 * dm.POST_Y1, CHEEK_P[0], CHEEK_P[1], L3, 0)
    # spine joining sockets, DM cradle and both cheeks (above and below the DM, behind yoke)
    for zs in (1, -1):
        z0, z1 = (dm.POST_Y1 - 6.0, dm.POST_Y1) if zs > 0 else (-dm.POST_Y1, -dm.POST_Y1 + 6.0)
        body += box(xs0 - 4, xd + dm.WALL_X1, CHEEK_N[0], CHEEK_P[1], z0, z1)
    body += box(xs0 - 4, xs0 + 8, CHEEK_N[0], CHEEK_P[1], -dm.POST_Y1, dm.POST_Y1)
    # pitch stub-axle press bores in both cheeks (8 mm dowels)
    body -= ycyl(WR.pitch_pin_dia - P.fit.pin_press_minus, CHEEK_N[0] - 1, CHEEK_N[1] + 1, L3, 0)
    body -= ycyl(WR.pitch_pin_dia - P.fit.pin_press_minus, CHEEK_P[0] - 1, CHEEK_P[1] + 1, L3, 0)
    # keep the yoke sweep clear: carve a cylinder of radius r_sweep around the pitch axis
    r_sweep = yoke_sweep_r() + 2.0
    body -= ycyl(2 * r_sweep, CHEEK_N[1] + 0.3, CHEEK_P[0] - 0.3, L3, 0)
    body -= tube_sockets(xs0 - 4.02, X_WS, offs, YC, cross_x=xs0 + SOCK_LEN / 2)
    # J4 belt tension: DM is integral, so a back-side idler (3x 625 on M5) presses the upper
    # span from above; vertical slot in the -Y cheek sets the tension.
    xi = xd + 34.0
    body -= Pos(xi, 0, j4_idler_z()) * Rot(90, 0, 0) * lib.slot(10.0, clr("M5"), 40, 0, 0, -CHEEK_N[1] - 5, angle=90)
    return body


def j4_idler_z() -> float:
    r1, r2 = lib.pd(P.dm.teeth) / 2, lib.pd(P.pulleys.j4) / 2
    beta = math.asin((r2 - r1) / L.C4)
    ang = math.pi / 2 + beta                    # upper tangent normal
    x1, z1 = r1 * math.cos(ang), r1 * math.sin(ang)
    x2, z2 = L.C4 + r2 * math.cos(ang), r2 * math.sin(ang)
    f = (34.0 - x1) / (x2 - x1)
    return z1 + f * (z2 - z1) + 1.4 + 8.0 - 1.0


def yoke_sweep_r() -> float:
    """Max radius of the yoke (+cap, +pulley) from the pitch axis in its own XZ plane."""
    xs = (YOKE_X[0], YOKE_X[1])
    zs = (YOKE_Z[0], YOKE_Z[1])
    r_box = max(math.hypot(x, z) for x in xs for z in zs)
    return r_box


def pitch_pins():
    return [ycyl(WR.pitch_pin_dia, CHEEK_N[0], PUL4_Y[0] + B688 - 0.2, L3, 0),
            ycyl(WR.pitch_pin_dia, YOKE_Y[1] + 2.0 - B688 + 0.2, CHEEK_P[1], L3, 0)]


def fa_tubes():
    x0, x1 = R0, X_WS
    return [cyl(A.fa_tube_od, x1 - x0, x0, YC, zo, axis="X") - cyl(A.fa_tube_od - 2 * A.fa_tube_wall, x1 - x0 + 1, x0 - 0.5, YC, zo, axis="X")
            for zo in (A.fa_tube_offset, -A.fa_tube_offset)]


def parts():
    out = []
    w = wrist_housing()
    out.append(PartDef("40_wrist_housing", w, "Wrist housing: forearm sockets + J4 drive cradle + pitch fork",
                       material="PETG (PA-CF recommended)", color="#7f8fa0", print_loc=on_bed(w, (90, 0, 0)),
                       walls=5, infill="50% gyroid",
                       print_notes="Lay on the -Y cheek (J4 pin wall) face. Supports: build-plate only, "
                                   "under the +Y cheek bridge."))
    y = yoke()
    out.append(PartDef("41_pitch_yoke", y, "Pitch yoke: J5 servo pocket + roll bearing housing",
                       material="PETG (PA-CF recommended)", color="#e0a030", print_loc=on_bed(y, (0, -90, 0)),
                       walls=5, infill="50% gyroid", print_notes="Roll housing UP (front face down)."))
    yc = yoke_cap()
    out.append(PartDef("42_yoke_cap", yc, "Pitch-yoke rear cap", material="PETG", color="#d08a20",
                       print_loc=on_bed(yc, (0, 90, 0)), load_bearing=False))
    p4 = pulley4()
    out.append(PartDef("43_j4_pulley54", p4, "J4 54T pulley (bolts to yoke)", material="PETG", color="#e0a030",
                       print_loc=on_bed(p4, (90, 0, 0)), walls=6, infill="100%"))
    ha = horn_adapter()
    out.append(PartDef("44_j5_horn_adapter", ha, "J5 horn adapter with drive dogs", material="PETG (PA-CF)",
                       color="#c06020", print_loc=on_bed(ha, (0, -90, 0)), walls=6, infill="100%"))
    sp = spindle()
    out.append(PartDef("45_roll_spindle", sp, "Roll spindle", material="PA-CF or PETG", color="#b0b0b0",
                       print_loc=on_bed(sp, (0, -90, 0)), walls=8, infill="100%",
                       print_notes="Vertical (axis Z) so bearing journals are round; 100% infill."))
    rs = roll_spacer()
    out.append(PartDef("47_roll_spacer", rs, "Roll-spindle inner-ring spacer", material="PETG", color="#b0b0b0",
                       print_loc=on_bed(rs, (0, 90, 0)), walls=3, infill="100%", load_bearing=False))
    tf = tool_flange()
    out.append(PartDef("46_tool_flange", tf, "Tool flange (WRA-TF30 interface)", material="PETG (PA-CF)",
                       color="#c0c0c0", print_loc=on_bed(tf, (0, 90, 0)), walls=6, infill="100%"))
    return out
