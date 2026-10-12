"""Shoulder hubs, upper arm, parallelogram rod and forearm root.

Frames (all share the turntable Y axis):
  U : upper-arm frame. origin = J2 axis O, X_U along the upper arm (theta2), Z_U in-plane normal.
  K : J3 crank frame. origin = O, X_K along the crank (= theta3 + 180 deg).
  F : forearm frame. origin = elbow axis E, X_F along the forearm (theta3).
  R : rod-end frame. origin = pin centre, rod direction +X, pin axis +Y.
"""
from __future__ import annotations
import math
from build123d import Pos, Rot, Part

from ..config import P
from .. import lib, layout as L
from ..lib import box, cyl, clr, PartDef, on_bed, prism

A = P.arm
SP = P.springs
d6802, D6802, B6802 = lib.bearing(A.stub_bearing)
d6800, D6800, B6800 = lib.bearing(A.elbow_bearing)
d626, D626, B626 = lib.bearing(A.rod_end_bearing)
YU = A.ua_plane_y                      # -7.25 upper-arm plane
TUBE_R = A.ua_tube_od / 2
SOCK_D = A.ua_tube_od + P.fit.tube_socket_extra
R0 = A.hub_tube_r0                     # 34 : socket bottom radius from joint axis
SOCK_LEN = A.socket_depth              # 28


def ycyl(d, y0, y1, x=0.0, z=0.0) -> Part:
    """Cylinder along +Y from y0 to y1 centred at (x, z)."""
    return cyl(d, y1 - y0, x, y0, z, axis="Y")


def ypulley(teeth, y0) -> Part:
    """HTD pulley (flanges + teeth) along +Y starting at y0 (no bore)."""
    return Pos(0, y0, 0) * Rot(-90, 0, 0) * lib.htd_pulley(teeth)


def tube_bosses(x0, x1, offsets, y_c, d=18.0) -> Part:
    out = None
    for zo in offsets:
        c = cyl(d, x1 - x0, x0, y_c, zo, axis="X")
        out = c if out is None else out + c
    return out


def tube_sockets(x_open, x_bottom, offsets, y_c, cross_x=None) -> Part:
    """Socket bores (from x_open toward x_bottom) + M3 cross-bolt holes along Y."""
    out = None
    lo, hi = sorted((x_open, x_bottom))
    for zo in offsets:
        c = cyl(SOCK_D, hi - lo, lo, y_c, zo, axis="X")
        if cross_x is not None:
            c += ycyl(clr("M3"), y_c - 20, y_c + 20, cross_x, zo)
        out = c if out is None else out + c
    return out


# =============================================================================== UA hub (U)
Y86 = (-37.5, -18.5)
YH = (-18.5, 4.0)
UA_BRG_Y = ((-37.0, -32.0), (-1.0, 4.0))


def ua_hub() -> Part:
    body = ypulley(P.pulleys.j2, Y86[0])
    body += ycyl(lib.od(P.pulleys.j2) - 3.0, Y86[0], Y86[1] + 0.01)
    body += ycyl(42.0, YH[0] - 0.01, YH[1])
    offs = (A.ua_tube_offset, -A.ua_tube_offset)
    # web + socket bosses toward +X
    body += box(0, R0 + 6, YU - 4.0, YU + 4.0, -A.ua_tube_offset - 4, A.ua_tube_offset + 4)
    body += tube_bosses(R0 - 4, R0 + SOCK_LEN, offs, YU)
    # rear lever (B2 / C2 cord pulleys), 12 mm thick, with 5 mm cord slot
    lever = box(-36.0, 0.0, YU - 6.0, YU + 6.0, -7.0, SP.j2_offset + 6.0)
    lever -= box(-40.0, -21.0, YU - 2.5, YU + 2.5, -10.0, SP.j2_offset + 7.0)
    lever -= box(-40.0, 1.0, YU - 2.5, YU + 2.5, SP.j2_offset + 2.0, SP.j2_offset + 7.0)
    body += lever
    for z in (0.0, SP.j2_offset):
        body -= ycyl(clr("M3"), YU - 10, YU + 10, -SP.b2, z)
    # stub bore, bearing seats, spacer clearance
    body -= ycyl(21.0, Y86[0] - 1, YH[1] + 1)
    for y0, y1 in UA_BRG_Y:
        body -= ycyl(lib.seat(A.stub_bearing), y0 - 0.01, y1 + 0.01)
    # tube sockets with cross-bolts
    body -= tube_sockets(R0 + SOCK_LEN + 0.01, R0, offs, YU, cross_x=R0 + SOCK_LEN / 2)
    # cable pass through web
    body -= box(8.0, 24.0, YU - 5, YU + 5, -6.0, 6.0)
    return body


# =============================================================================== J3 hub (K)
YJ3 = (4.5, 30.0)
J3_BRG_Y = ((5.0, 10.0), (24.5, 29.5))
Y_CRANK = tuple(A.y_crank_arm)


def j3_hub() -> Part:
    body = ypulley(P.pulleys.j3, YJ3[0])
    body += ycyl(lib.od(P.pulleys.j3) - 3.0, YJ3[0], Y_CRANK[0] + 0.01)
    body += ycyl(34.0, Y_CRANK[0] - 0.01, Y_CRANK[1])
    # crank arm to P (r = crank) and boss
    body += box(0, A.crank, Y_CRANK[0], Y_CRANK[1], -9.0, 9.0)
    body += ycyl(22.0, Y_CRANK[0], Y_CRANK[1], A.crank, 0)
    # L-lever: axial post at r = b3 carrying the J3 spring cord (B3) out to the cord plane
    b3 = SP.b3
    body += box(b3 - 4.0, b3 + 4.0, Y_CRANK[1] - 0.01, A.j3_cord_y + 2.5, -4.0, 4.0)
    body -= cyl(2.6, 20, b3, A.j3_cord_y, -10)                     # cord hole along Z
    # bores
    body -= ycyl(21.0, YJ3[0] - 1, YJ3[1] + 1)
    for y0, y1 in J3_BRG_Y:
        body -= ycyl(lib.seat(A.stub_bearing), y0 - 0.01, y1 + 0.01)
    # rod-end shoulder screw: M5 insert + shoulder counterbore at P
    body -= ycyl(lib.ins_hole("M5"), Y_CRANK[1] - lib.ins_depth("M5"), Y_CRANK[1] + 0.01, A.crank, 0)
    body -= ycyl(6.2, Y_CRANK[1] - 1.5, Y_CRANK[1] + 0.01, A.crank, 0)
    # lightening holes in the pulley web
    for i in range(5):
        a = 36 + 72 * i
        x, z, _ = lib.polar(23.0, a)
        body -= ycyl(10.0, YJ3[0] - 1, Y_CRANK[0] - 0.5, x, z)
    return body


# =============================================================================== stub stack
SPACER_OD, SPACER_ID = 17.4, 15.3          # 6802 inner-ring face only (clear of the seals)


def stub_stack():
    """Axial stack on the fixed 15 mm stub between the cheek face and the end collar (T frame Y):
    returns [(kind, y0, y1)] with kind 'spacer' (printed) or 'shim' (DIN 988 15x21 shim set)."""
    out = []
    for y0, y1 in ((A.y_stub_cheek[1], UA_BRG_Y[0][0]), (UA_BRG_Y[0][1], UA_BRG_Y[1][0]),
                   (UA_BRG_Y[1][1], J3_BRG_Y[0][0]), (J3_BRG_Y[0][1], J3_BRG_Y[1][0]),
                   (J3_BRG_Y[1][1], J3_BRG_Y[1][1] + 0.7)):
        if y1 - y0 > 0.05:
            out.append(("spacer" if y1 - y0 >= 3.0 else "shim", y0, y1))
    return out


def stub_spacer(length: float) -> Part:
    """Printed inner-ring spacer for the stub stack (local: axis Y from 0 to length)."""
    return ycyl(SPACER_OD, 0.0, length - 0.04) - ycyl(SPACER_ID, -1, length + 1)


def stub_spacers():
    """Printed spacers placed on the stub (T frame)."""
    return [Pos(0, y0 + 0.02, L.O.Z) * stub_spacer(y1 - y0) for k, y0, y1 in stub_stack() if k == "spacer"]


def stub_collar():
    """15 mm set-screw shaft collar retaining the stub stack (vitamin, T frame)."""
    y0 = J3_BRG_Y[1][1] + 0.7
    return ycyl(28.0, y0, y0 + 12.0, 0, L.O.Z) - ycyl(A.stub_dia, y0 - 1, y0 + 13, 0, L.O.Z)


def elbow_spacer_len():
    return FR_BRG_Y[1][0] - FR_BRG_Y[0][1]


def elbow_collar():
    """10 mm set-screw collar retaining the forearm root on the elbow pin (vitamin, U frame)."""
    y0 = FR_Y[1] + 0.3
    return ycyl(20.0, y0, y0 + 8.0, A.L2, 0) - ycyl(A.elbow_pin_dia, y0 - 1, y0 + 9, A.L2, 0)


def elbow_spacer():
    """12x1 aluminium tube offcut between the two 6800 inner rings (vitamin, U frame)."""
    y0, y1 = FR_BRG_Y[0][1] + 0.02, FR_BRG_Y[1][0] - 0.02
    return ycyl(12.0, y0, y1, A.L2, 0) - ycyl(10.0, y0 - 1, y1 + 1, A.L2, 0)


# =============================================================================== elbow (U)
ELB_Y = (YU - 10.0, YU + 10.0)


def elbow_block() -> Part:
    x2 = A.L2
    offs = (A.ua_tube_offset, -A.ua_tube_offset)
    body = tube_bosses(x2 - R0 - SOCK_LEN, x2 - R0 + 4, offs, YU)
    body += box(x2 - R0 - SOCK_LEN, x2, YU - 4.0, YU + 4.0, -A.ua_tube_offset - 4, A.ua_tube_offset + 4)
    body += ycyl(32.0, ELB_Y[0], ELB_Y[1], x2, 0)
    # J2 spring anchor tab on top (2x M4 adjuster holes along X)
    zt = SP.j2_offset + 5.0
    body += box(x2 - R0 - SOCK_LEN, x2 - R0 - SOCK_LEN + 10, YU - 10, YU + 10, A.ua_tube_offset, zt + 7)
    for dy in (-5.0, 5.0):
        body -= cyl(clr("M4"), 30, x2 - R0 - SOCK_LEN - 5, YU + dy, zt, axis="X")
    # parallelogram hard-stop lugs (front of elbow). The forearm-root tab sits at +90 deg in F,
    # i.e. at (90 - q3) deg in U; lugs stop it at q3 = hard limits.
    q_lo, q_hi = P.joints.j3_rel.soft[0] - 3.0, P.joints.j3_rel.soft[1] + 3.0
    half = math.degrees(math.atan2(4.0, 22.5))
    for edge, sgn in ((90.0 - q_lo + half, 1), (90.0 - q_hi - half, -1)):
        pts = [(x2 + r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
               for r, a in ((14.5, edge), (27.5, edge), (27.5, edge + sgn * 20), (14.5, edge + sgn * 20))]
        body += lib.prism_xz(pts, ELB_Y[1] - ELB_Y[0], ELB_Y[0])
    # elbow pin bore (press 10 h6) + clamp slit + 2x M3 pinch
    body -= ycyl(A.elbow_pin_dia - P.fit.pin_press_minus, ELB_Y[0] - 1, ELB_Y[1] + 1, x2, 0)
    body -= box(x2 - 17.0, x2 - 4.0, ELB_Y[0] - 1, ELB_Y[1] + 1, -0.75, 0.75)       # clamp slit
    for yy in (ELB_Y[0] + 5.0, ELB_Y[1] - 5.0):
        body -= cyl(clr("M3"), 40, x2 - 11.0, yy, -20, axis="Z")
        body -= cyl(6.0, 10, x2 - 11.0, yy, 8.0)
        body -= Pos(x2 - 11.0, yy, -20.0) * prism(lib.hexagon2d(5.8), 12.0)
    body -= tube_sockets(x2 - R0 - SOCK_LEN - 0.01, x2 - R0, offs, YU, cross_x=x2 - R0 - SOCK_LEN / 2)
    return body


# =============================================================================== rod (R)
ROD_SOCK = (14.0, 34.0)


def rod_end() -> Part:
    w = A.rod_plane_y
    body = ycyl(2 * A.rod_end_r, 0.0, 8.0)
    body += cyl(14.0, ROD_SOCK[1] - 8.0, 8.0, 4.0, 0, axis="X")
    body += box(0, 10, 0.0, 8.0, -7.0, 7.0)
    body -= ycyl(lib.seat(A.rod_end_bearing), (8.0 - B626) / 2, (8.0 + B626) / 2)
    body -= ycyl(d626 + 2.0, -1, 9)
    body -= cyl(A.rod_od + P.fit.tube_socket_extra, ROD_SOCK[1] - ROD_SOCK[0] + 0.01, ROD_SOCK[0], 4.0, 0, axis="X")
    body -= cyl(clr("M3"), 30, (ROD_SOCK[0] + ROD_SOCK[1]) / 2, 4.0, -15, axis="Z")     # cross-bolt
    return body


def rod_tube_len():
    return A.L2 - 2 * ROD_SOCK[0]


# =============================================================================== forearm root (F)
FR_Y = (YU + 10.5, Y_CRANK[1])          # 3.25 .. 30
FA_Y = (FR_Y[0] + FR_Y[1]) / 2          # forearm plane (16.6)
FR_BRG_Y = ((FR_Y[0] + 0.5, FR_Y[0] + 0.5 + B6800), (FR_Y[1] - 0.5 - B6800, FR_Y[1] - 0.5))


def forearm_root() -> Part:
    offs = (A.fa_tube_offset, -A.fa_tube_offset)
    body = ycyl(30.0, FR_Y[0], FR_Y[1])
    body += box(0, R0 + 4, FA_Y - 6.0, FA_Y + 6.0, -A.fa_tube_offset - 4, A.fa_tube_offset + 4)
    body += tube_bosses(R0 - 4, R0 + SOCK_LEN, offs, FA_Y)
    # crank to Q (behind the elbow) in the rod plane
    body += box(-A.crank, 0, Y_CRANK[0], Y_CRANK[1], -9.0, 9.0)
    body += ycyl(22.0, Y_CRANK[0], Y_CRANK[1], -A.crank, 0)
    # hard-stop tab pointing +Z_F (hits the elbow lugs at the parallelogram hard limits):
    # radial rib on the root (y >= FR_Y[0]) + finger reaching back into the lug plane
    body += box(-4.0, 4.0, FR_Y[0] - 0.01, FR_Y[0] + 7.0, 12.0, 27.5)
    body += box(-4.0, 4.0, ELB_Y[1] - 5.5, FR_Y[0] + 0.02, 17.5, 27.5)
    # bores
    body -= ycyl(d6800 + 3.0, FR_Y[0] - 1, FR_Y[1] + 1)
    for y0, y1 in FR_BRG_Y:
        body -= ycyl(lib.seat(A.elbow_bearing), y0 - 0.01, y1 + 0.01)
    body -= ycyl(lib.ins_hole("M5"), Y_CRANK[1] - lib.ins_depth("M5"), Y_CRANK[1] + 0.01, -A.crank, 0)
    body -= ycyl(6.2, Y_CRANK[1] - 1.5, Y_CRANK[1] + 0.01, -A.crank, 0)
    body -= tube_sockets(R0 + SOCK_LEN + 0.01, R0, offs, FA_Y, cross_x=R0 + SOCK_LEN / 2)
    return body


def elbow_pin() -> Part:
    return ycyl(A.elbow_pin_dia, ELB_Y[0], A.elbow_pin_end_y, A.L2, 0)


def ua_tubes():
    x0, x1 = R0, A.L2 - R0
    return [cyl(A.ua_tube_od, x1 - x0, x0, YU, zo, axis="X") - cyl(A.ua_tube_od - 2 * A.ua_tube_wall, x1 - x0 + 1, x0 - 0.5, YU, zo, axis="X")
            for zo in (A.ua_tube_offset, -A.ua_tube_offset)]


def parts():
    out = []
    h = ua_hub()
    out.append(PartDef("30_ua_hub", h, "Upper-arm hub with integral 86T (J2) pulley", material="PETG (PA-CF recommended)",
                       color="#e0a030", print_loc=on_bed(h, (90, 0, 0)), walls=6, infill="60% gyroid",
                       print_notes="Pulley flange (-Y face) down: teeth and bearing seats vertical. "
                                   "Tube sockets horizontal: supports OFF (bores bridge as teardrops)."))
    j = j3_hub()
    out.append(PartDef("31_j3_hub", j, "J3 hub: 72T pulley + parallelogram crank + spring L-lever",
                       material="PETG (PA-CF recommended)", color="#d08a20", print_loc=on_bed(j, (90, 0, 0)),
                       walls=6, infill="60% gyroid", print_notes="Pulley flange down; L-lever prints upward."))
    e = elbow_block()
    out.append(PartDef("32_elbow_block", e, "Elbow block (clamps elbow pin, J2 spring anchor, stop lugs)",
                       material="PETG (PA-CF recommended)", color="#7f8fa0", print_loc=on_bed(e, (90, 0, 0)),
                       walls=6, infill="60% gyroid"))
    r = rod_end()
    out.append(PartDef("33_rod_end", r, "Parallelogram rod end (626 bearing)", qty=2, material="PETG",
                       color="#6a7f99", print_loc=on_bed(r, (90, 0, 0)), walls=5, infill="80%"))
    f = forearm_root()
    out.append(PartDef("34_forearm_root", f, "Forearm root: elbow bearings + crank + stop tab",
                       material="PETG (PA-CF recommended)", color="#e0a030", print_loc=on_bed(f, (-90, 0, 0)),
                       walls=6, infill="60% gyroid"))
    for k, y0, y1 in stub_stack():
        if k != "spacer":
            continue
        ln = round(y1 - y0 - 0.04, 2)
        sp = stub_spacer(y1 - y0)
        out.append(PartDef(f"35_stub_spacer_{ln:g}mm".replace(".", "p"), sp, f"Stub inner-ring spacer {ln:g} mm",
                           material="PETG", color="#e0a030", print_loc=on_bed(sp, (90, 0, 0)), walls=3,
                           infill="100%", load_bearing=False, print_notes="Print upright (axis Z), 0.12 mm layers."))
    return out
