"""Turntable (rotates on J1) and everything fixed to it: plate + spindle, left cheek with the
J2 stub axle, the Dual Drive Module (J2 + J3 servos), belt-tension idlers, spring
anchor / idler posts and the J3 spring anchor.

All geometry in the turntable frame T (origin on J1 axis, top face of plate, X = arm front).
"""
from __future__ import annotations
import math
from build123d import Pos, Rot, Part, Location

from ..config import P
from .. import lib, layout as L
from ..lib import box, cyl, clr, PartDef, on_bed, prism
from . import dm

A = P.arm
BS = P.base
PT = BS.turntable_plate_t
O = L.O
TZ = L.T_ORIGIN_B                         # base z of turntable origin

PLATE_X = (-124.0, 30.0)
PLATE_Y = (-62.0, 56.0)
SPLIT_Y = -7.0                            # DDM bracket split plane
STUB_LEN = A.stub_end_y - (A.y_stub_cheek[0] - 1.0)
STUB_Y0 = A.y_stub_cheek[0] - 1.0         # -51
CHEEK_BOSS = (40.0, 30.0)                 # dia, length (y from -60 to -30 ... see below)


def zT(zB):
    return zB - TZ


# ------------------------------------------------------------------------- plate
def plate() -> Part:
    pts = lib.rounded_rect2d(PLATE_X[1] - PLATE_X[0], PLATE_Y[1] - PLATE_Y[0], 14.0)
    cx, cy = (PLATE_X[0] + PLATE_X[1]) / 2, (PLATE_Y[0] + PLATE_Y[1]) / 2
    body = Pos(cx, cy, -PT) * prism(pts, PT)
    # spindle + bearing shoulder
    z_bot = zT(BS.collar_z[0])                               # spindle bottom end
    z_sh = zT(BS.bearing_z[1] + lib.bearing(BS.bearing)[2] + 0.05)
    body += cyl(BS.spindle_od, -PT - z_bot, z=z_bot)
    body += cyl(33.0, -PT - z_sh, z=z_sh)
    body -= cyl(BS.spindle_bore, 200, z=z_bot - 1)
    # stop pin (6 x 12 dowel) pressed into underside at front
    body -= cyl(6.0 - P.fit.pin_press_minus, 5.0, BS.stop_r, 0, -PT - 0.01)
    # J1 drive module: slots (radial) + tension lug under the plate
    for xd, yd in dm.mount_xy():
        x = -L.C1 + xd
        body -= lib.slot(10.0, clr("M4"), PT + 2, x, yd, -PT - 1)
    lug_x0 = -L.C1 + dm.WALL_X0 - 8.0
    body += box(lug_x0, lug_x0 + 6.0, -8, 8, -PT - 12.0, -PT + 0.01)
    body -= cyl(clr("M4"), 10, lug_x0 - 1, 0, -PT - 4.0, axis="X")
    # inserts: stub-cheek foot (4) and DDM feet (2 per half)
    for x, y in cheek_foot_xy() + ddm_foot_xy():
        body -= cyl(lib.ins_hole("M4"), lib.ins_depth("M4"), x, y, -lib.ins_depth("M4") + 0.01)
    # DDM tension lugs (one per half): M4 screw pulls the foot flange toward -X
    for x, y in ddm_tension_lug_xy():
        body += box(x - 4.0, x + 4.0, y - 7.0, y + 7.0, -0.01, 12.0)
        body -= cyl(clr("M4"), 20, x - 5.0, y, 3.0, axis="X")
    # A2 anchor post (cord fixed end) and A3 idler post, printed on the plate
    body += a2_post()
    body += a3_post()
    # J3 spring rear anchor block with 2 adjuster holes
    body += j3_spring_anchor()
    # zip-tie slots for cable management
    for x, y in ((12, 14), (12, -16), (-40, 30), (-40, -30)):
        body -= box(x - 2, x + 2, y - 5, y + 5, -PT - 1, 1)
    return body


def a2_post() -> Part:
    y0, y1 = -18.3, -12.3                                       # clear of the 86T flange (y <= -18.5)
    post = box(-5, 5, y0, y1, 0, L.A2.Z + 4)
    post -= cyl(clr("M4"), 20, 0, y0 - 1, L.A2.Z, axis="Y")     # M4 x 16 anchor bolt along +Y
    return post


def a3_post() -> Part:
    y0, y1 = A.j3_cord_y + 3.0, A.j3_cord_y + 10.0
    post = box(-5, 5, y0, y1, 0, L.A3.Z + 8)
    post -= cyl(clr("M3"), 20, 0, y1 + 1, L.A3.Z, axis="-Y")    # M3 axle for 623 idler
    return post


def j3_spring_anchor() -> Part:
    x0 = PLATE_X[0] + 2
    blk = box(x0, x0 + 8, A.j3_cord_y - 9, A.j3_cord_y + 9, 0, L.A3.Z + 2)
    for dy in (-4.5, 4.5):
        blk -= cyl(clr("M4"), 20, x0 - 1, A.j3_cord_y + dy, L.A3.Z - 5.0, axis="X")
    return blk


def cheek_foot_xy():
    return [(-20, -56), (20, -56), (-20, -42), (20, -42)]


def stub_cheek() -> Part:
    y0, y1 = A.y_stub_cheek
    zc = O.Z
    web = box(-26, 26, y0, y1, 0, zc)
    web += cyl(52, y1 - y0, 0, y0, zc, axis="Y")              # rounded top around boss
    boss = cyl(CHEEK_BOSS[0], CHEEK_BOSS[1], 0, y1 - CHEEK_BOSS[1], zc, axis="Y")
    foot = box(-26, 26, -62, y1, 0, 7)
    body = web + boss + foot
    # triangular gussets (YZ plane) on the outboard side
    for x in (-26.0, 21.0):
        body += lib.prism_yz([(-62.0, 6.9), (y0 + 0.01, 6.9), (y0 + 0.01, zc - 14.0)], 5.0, x)
    # stub bore + clamp slit + 2x M4 pinch bolts
    body -= cyl(A.stub_dia + 0.05, 60, 0, y0 - 10, zc, axis="Y")
    body -= box(-1.0, 1.0, y1 - CHEEK_BOSS[1] - 1, y1 + 1, zc, zc + CHEEK_BOSS[0] / 2 + 1)
    for yy in (y1 - CHEEK_BOSS[1] + 7, y1 - 7):
        body -= cyl(clr("M4"), 60, -30, yy, zc + 12.5, axis="X")
        body -= cyl(7.5, 21, -30, yy, zc + 12.5, axis="X")                  # head counterbore
        body -= Pos(9.0, yy, zc + 12.5) * Rot(0, 90, 0) * prism(lib.hexagon2d(7.4), 21)   # nut trap
    # foot bolts (heads on top of the 7 mm foot)
    for x, y in cheek_foot_xy():
        body -= cyl(clr("M4"), 20, x, y, -1)
    # cable clip hole
    body -= cyl(4, 40, -12, y0 - 10, 30, axis="Y")
    return body


# ------------------------------------------------------------------------- DDM
def _ddm_half(which: str) -> Part:
    """One half of the Dual Drive Module (J2 or J3 drive bracket), in T frame.  Each half has
    its own 8 x 25 dowel and slotted feet so each belt is tensioned independently."""
    loc = L.J2_DM_LOC_T if which == "j2" else L.J3_DM_LOC_T
    axis = L.DDM_AXIS if which == "j2" else L.DDM_AXIS3
    wall_t = (P.ddm.center_wall[1] - P.ddm.center_wall[0])
    half = loc * dm.bracket(wall_t=wall_t, mount=False, tension_lug=False)
    if which == "j2":
        half = half & box(-400, 400, -400, SPLIT_Y - 0.2, -400, 400)
        ya, yb = P.ddm.center_wall[0], SPLIT_Y - 0.2
        fya, fyb = -24.0, SPLIT_Y - 0.2
    else:
        half = half & box(-400, 400, SPLIT_Y + 0.2, 400, -400, 400)
        ya, yb = SPLIT_Y + 0.2, P.ddm.center_wall[1]
        fya, fyb = SPLIT_Y + 0.2, 16.0
    x0, x1 = axis.X + dm.WALL_X0, axis.X + dm.WALL_X1
    z_wall_bot = axis.Z - dm.POST_Y1
    half += box(x0, x1, ya, yb, 0, z_wall_bot + 0.01)               # pedestal under the pin wall
    half += box(x0 - 10.0, x1, fya, fyb, 0, 6.0)                     # foot flange
    for x, y in ddm_foot_xy(which):
        half -= lib.slot(9.0, clr("M4"), 20, x, y, -1)
    # tension screw insert in the rear face of the foot flange (pulled toward -X)
    half -= cyl(lib.ins_hole("M4"), lib.ins_depth("M4"), x0 - 10.0 - 0.01, (fya + fyb) / 2, 3.0, axis="X")
    # this half's own dowel (8 x 25), pressed from the split face
    yp0, yp1 = (SPLIT_Y - 0.2 - P.ddm.pin_len, SPLIT_Y - 0.2) if which == "j2" else (SPLIT_Y + 0.2, SPLIT_Y + 0.2 + P.ddm.pin_len)
    half -= cyl(P.dm.pin_dia - P.fit.pin_press_minus, yp1 - yp0 + 0.2, axis.X, yp0 - 0.1, axis.Z, axis="Y")
    return half


def ddm_foot_xy(which=None):
    xs = (L.DDM_AXIS.X - 24.0, L.DDM_AXIS.X + 0.0)
    ys = {"j2": (-15.5,), "j3": (4.5,)}
    keys = (which,) if which else ("j2", "j3")
    out = []
    for k in keys:
        ax = L.DDM_AXIS if k == "j2" else L.DDM_AXIS3
        xs = (ax.X - 24.0, ax.X + 0.0)
        out += [(x, y) for x in xs for y in ys[k]]
    return out


def ddm_tension_lug_xy():
    return [(L.DDM_AXIS.X + dm.WALL_X0 - 18.0, -15.5), (L.DDM_AXIS3.X + dm.WALL_X0 - 18.0, 4.5)]


def ddm_bracket_j2() -> Part:
    return _ddm_half("j2")


def ddm_bracket_j3() -> Part:
    return _ddm_half("j3")


def ddm_pins():
    out = []
    for which, axis in (("j2", L.DDM_AXIS), ("j3", L.DDM_AXIS3)):
        yp0 = SPLIT_Y - 0.2 - P.ddm.pin_len if which == "j2" else SPLIT_Y + 0.2
        out.append(cyl(P.dm.pin_dia, P.ddm.pin_len, axis.X, yp0, axis.Z, axis="Y"))
    return out


def stub() -> Part:
    return cyl(A.stub_dia, STUB_LEN, 0, STUB_Y0, O.Z, axis="Y")


def parts():
    out = []
    pl = plate()
    out.append(PartDef("20_turntable", pl, "Turntable plate + J1 spindle", material="PETG (PA-CF optional)",
                       color="#8a9aa9", print_loc=on_bed(pl, (180, 0, 0)), walls=6, infill="40% gyroid",
                       print_notes="Plate TOP face down, spindle pointing up (bearing journals print round). "
                                   "Supports from build plate under the tension lug only."))
    ck = stub_cheek()
    out.append(PartDef("21_stub_cheek", ck, "Stub cheek (clamps J2 stub axle)", material="PETG (PA-CF recommended)",
                       color="#7f8fa0", print_loc=on_bed(ck, (90, 0, 0)), walls=6, infill="60% gyroid",
                       print_notes="Lay on its INBOARD face (stub axis vertical) so the clamp boss layers "
                                   "wrap the stub; gussets then print as walls."))
    for which in ("j2", "j3"):
        h = _ddm_half(which)
        rot = (90, 0, 0) if which == "j2" else (-90, 0, 0)
        out.append(PartDef(f"22_ddm_bracket_{which}", h, f"Dual drive bracket ({which.upper()} half)",
                           material="PETG", color="#4a7bd0", print_loc=on_bed(h, rot), walls=5, infill="50% gyroid",
                           print_notes="Central-wall face (split plane) DOWN. Supports: build-plate only "
                                       "under servo-cradle lips."))
    return out
