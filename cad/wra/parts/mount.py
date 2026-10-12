"""Chair-side mounting system + PROVISIONAL reference wheelchair for visual/collision checks.

The SAME printed parts are used on the left and right sides:
  * Tube clamp (body + cap)  -> parametric in mount.tube_od (MEASURE YOUR CHAIR).
  * Post bracket (spine + shelf) is mirror-symmetric about its own mid-plane, so the left
    installation is the right one rotated 180 deg about the post axis.
  * Receiver is bolted to the shelf through its 24-position index ring -> the arm "front mark"
    yaw is set in 15 deg steps (right: -45 deg, left: +45 deg by default).
  * Torque strut = M6 threaded rod + two M6 rod-end bearings (any length 90-160 mm) to a third
    clamp on a neighbouring frame tube: positive anti-rotation without drilling the chair.

Clamp frame K_c: tube axis = Z, clamp mid-height at z = 0, mounting pad faces +X.
Bracket frame M:  post axis = Z through origin, shelf toward +X, receiver top face at z = 0.
"""
from __future__ import annotations
import math
from build123d import Pos, Rot, Part, Location

from ..config import P
from .. import lib
from ..lib import box, cyl, clr, PartDef, on_bed, prism

MT = P.mount
CH = P.chair
DK = P.dock
TUBE = MT.tube_od
ID = TUBE + 2 * MT.liner
WALL = 6.0
H = MT.clamp_h
EAR_X = ID / 2 + WALL                     # ear centre offset (+/-Y) is computed below
BOLT_Y = ID / 2 + WALL / 2 + 3.5          # clamp bolts at +/- this (Y), along X
PAD_X = ID / 2 + WALL + 2.0               # mounting face of clamp body (x)
SHELF_T = 8.0
SPINE_T = 12.0                                  # [C] 12 mm: spine bending margin >= 1.2 with the WRA-1A arm
SPINE_W = 60.0
REC_T = DK.receiver_t


# ------------------------------------------------------------------------- tube clamp
def clamp_body() -> Part:
    """Fixed half (x >= 0 side) with mounting pad (2x M5 inserts in the +X face)."""
    ring = cyl(ID + 2 * WALL, H, z=-H / 2) - cyl(ID, H + 2, z=-H / 2 - 1)
    ring &= box(0, 200, -200, 200, -200, 200)
    ears = box(-0.0, 6.0, -BOLT_Y - 6.0, BOLT_Y + 6.0, -H / 2, H / 2)
    pad = box(ID / 2 + 1.0, PAD_X, -BOLT_Y + 6.0, BOLT_Y - 6.0, -H / 2, H / 2)
    body = ring + ears + pad
    body -= cyl(ID, H + 2, z=-H / 2 - 1)
    for y in (BOLT_Y, -BOLT_Y):
        body -= cyl(clr(MT.clamp_bolt), 30, -5, y, 0, axis="X")
        body -= Pos(6.0 - 4.0, y, 0) * Rot(0, 90, 0) * prism(lib.hexagon2d(8.4), 4.01)   # nut trap
    for z in (-H / 4, H / 4):
        body -= cyl(lib.ins_hole("M5"), lib.ins_depth("M5"), PAD_X - lib.ins_depth("M5") + 0.01, 0, z, axis="X")
    return body


def clamp_cap() -> Part:
    ring = cyl(ID + 2 * WALL, H, z=-H / 2) - cyl(ID, H + 2, z=-H / 2 - 1)
    ring &= box(-200, -0.6, -200, 200, -200, 200)
    ears = box(-6.6, -0.6, -BOLT_Y - 6.0, BOLT_Y + 6.0, -H / 2, H / 2)
    body = ring + ears
    body -= cyl(ID, H + 2, z=-H / 2 - 1)
    for y in (BOLT_Y, -BOLT_Y):
        body -= cyl(clr(MT.clamp_bolt), 30, -20, y, 0, axis="X")
        body -= cyl(9.4, 10, -16.6, y, 0, axis="X")
    return body


def clamp_liner() -> Part:
    """Visual only: 1.5 mm rubber (bicycle inner tube) or TPU liner."""
    return cyl(ID, H - 2, z=-H / 2 + 1) - cyl(TUBE, H, z=-H / 2)


# ------------------------------------------------------------------------- post bracket
def bracket_dims():
    z_shelf_top = -REC_T
    z_shelf_bot = z_shelf_top - SHELF_T
    z_c1 = z_shelf_bot - MT.shelf_above_clamp - H / 2       # upper clamp centre
    z_c2 = z_c1 - MT.clamp_spacing
    return z_shelf_top, z_shelf_bot, z_c1, z_c2


def shelf_center_x():
    """Receiver centre along the bracket +X axis (from the post axis)."""
    return PAD_X + 0.3 + MT.shelf_offset


def post_bracket() -> Part:
    zt, zb, zc1, zc2 = bracket_dims()
    x_sp0 = PAD_X + 0.3
    x_sp1 = x_sp0 + SPINE_T
    off = shelf_center_x()
    shelf_w = DK.plate[0] + 10.0
    z_sb = zc2 - H / 2 - 4.0                                      # spine bottom (just below lower clamp)
    spine = box(x_sp0, x_sp1, -SPINE_W / 2, SPINE_W / 2, z_sb, zt)
    shelf = Pos(off, 0, zb) * lib.rbox(shelf_w, shelf_w, SHELF_T, 10.0)
    shelf &= box(x_sp0, 400, -300, 300, -300, 300)
    body = spine + shelf
    # two gussets (mirror-symmetric about y = 0)
    for y0 in (-SPINE_W / 2, SPINE_W / 2 - 8.0):
        body += lib.prism_xz([(x_sp1 - 0.01, zb + 0.01), (off + 30.0, zb + 0.01), (x_sp1 - 0.01, zc1 - 10.0)], 8.0, y0)
    # receiver bolts: 4x M4 inserts at 90 deg on the index BC (receiver supplies 15 deg steps)
    for i in range(4):
        x, y, _ = lib.polar(DK.index_bc / 2, 90 * i)
        body -= cyl(lib.ins_hole("M4"), lib.ins_depth("M4"), off + x, y, zt - lib.ins_depth("M4") + 0.01)
    # clamp slots (vertical, +/-12 mm height adjustment)
    for zc in (zc1, zc2):
        for dz in (-H / 4, H / 4):
            body -= Pos(x_sp0 - 1, 0, zc + dz) * Rot(0, 90, 0) * lib.slot(24.0, clr("M5"), SPINE_T + 2, angle=0)
    # torque-strut clevis under the spine: VERTICAL M5 bolt, so the SI5 rod end (and the strut)
    # can swing in the horizontal plane and react torque about the post axis (tangential force)
    cx0, cx1, cy = x_sp0, x_sp0 + STRUT_CLEVIS[0], STRUT_CLEVIS[1] / 2
    body += box(cx0, cx1, -cy, cy, z_sb - STRUT_CLEVIS[2], z_sb + 0.01)
    zl = strut_pivot()[2]
    body -= box(cx0 + 4.0, cx1 + 1, -cy - 1, cy + 1, zl - STRUT_GAP / 2, zl + STRUT_GAP / 2)
    body -= cyl(clr("M5"), 60, strut_pivot()[0], 0, zl - 30)
    body -= Pos(strut_pivot()[0], 0, z_sb - STRUT_CLEVIS[2] - 0.01) * prism(lib.hexagon2d(8.4), 4.0)  # nut trap
    # safety-tether hole and cable notch
    body -= cyl(6.0, SPINE_T + 2, x_sp0 - 1, 18.0, zc1 + 30.0, axis="X")      # safety-tether eye
    body -= box(off - 8, off + 8, shelf_w / 2 - 12, shelf_w / 2 + 1, zb - 1, zt + 1)
    return body


STRUT_CLEVIS = (30.0, 32.0, 30.0)         # clevis block x-length, y-width, height below spine
STRUT_GAP = 9.0                           # SI5 rod-end eye is 8 mm wide


def strut_pivot():
    """Strut rod-end pivot point (bracket frame M)."""
    zt, zb, zc1, zc2 = bracket_dims()
    z_sb = zc2 - H / 2 - 4.0
    return (PAD_X + 0.3 + 17.0, 0.0, z_sb - STRUT_CLEVIS[2] / 2)


def stop_collar() -> Part:
    """Printed split collar below the lower clamp: secondary stop against sliding down."""
    hc = 14.0
    ring = cyl(ID + 2 * WALL, hc, z=-hc / 2) - cyl(TUBE + 0.4, hc + 2, z=-hc / 2 - 1)
    ring += box(-4, 4, ID / 2 + 2, ID / 2 + WALL + 8, -hc / 2, hc / 2)
    ring -= box(-0.75, 0.75, 0, 100, -hc, hc)
    ring -= cyl(clr("M4"), 40, -20, ID / 2 + WALL + 2.5, 0, axis="X")
    return ring


# ------------------------------------------------------------------------- reference chair
def reference_chair():
    """Very simplified Drive Silver Sport 2-LIKE frame for visual and collision checks ONLY.
    Every dimension not marked [V] in params.yaml is PROVISIONAL.  Chair frame C: origin on the
    floor under the rear-axle centre, X forward, Y left, Z up.  Returns [(name, shape, colour)]."""
    out = []
    w2 = CH.seat_width / 2 + CH.post_from_seat_edge
    od = CH.frame_tube_od
    zs = CH.seat_height
    fx = CH.front_post_x
    f0, f1 = CH.front_post_z
    zr = CH.lower_rail_z
    R = CH.rear_wheel_dia / 2
    for sy in (1, -1):
        y = sy * w2
        sd = "L" if sy > 0 else "R"
        out.append((f"front_post_{sd}", cyl(od, f1 - f0, fx, y, f0), "#9aa3ab"))
        out.append((f"caster_housing_{sd}", cyl(42.0, f0 - (zr - 20), fx, y, zr - 20), "#7d868e"))
        out.append((f"caster_{sd}", cyl(CH.caster_dia, 28, fx + 35, y - 14, CH.caster_dia / 2, axis="Y") +
                    box(fx - 5, fx + 50, y - 18, y + 18, CH.caster_dia / 2, zr - 20), "#333333"))
        out.append((f"rear_post_{sd}", cyl(od, zs + CH.back_height + 120 - zr, 40, y, zr), "#9aa3ab"))
        out.append((f"lower_rail_{sd}", cyl(od, fx - 40, 40, y, zr, axis="X"), "#9aa3ab"))
        out.append((f"seat_rail_{sd}", cyl(od, fx - 40 + 15, 40, y, zs - 25, axis="X"), "#9aa3ab"))
        out.append((f"armrest_{sd}", box(80, 330, y - 25, y + 25, zs + 200, zs + 230) +
                    box(95, 115, y - 10, y + 10, zs - 25, zs + 200) + box(300, 320, y - 10, y + 10, zs - 25, zs + 200),
                    "#555555"))
        wy = sy * (w2 + CH.rear_wheel_camber_y)
        wheel = cyl(CH.rear_wheel_dia, 25, 0, wy - 12.5, R, axis="Y") - cyl(CH.rear_wheel_dia - 60, 30, 0, wy - 15, R, axis="Y")
        out.append((f"rear_wheel_{sd}", wheel, "#333333"))
        rim = cyl(CH.rear_wheel_dia - 40, 6, 0, wy + sy * 22 - 3, R, axis="Y") - \
            cyl(CH.rear_wheel_dia - 52, 8, 0, wy + sy * 22 - 4, R, axis="Y")
        out.append((f"hand_rim_{sd}", rim, "#aaaaaa"))
        # wheel lock (brake) on the seat rail, lever up and forward
        lx = CH.wheel_lock_x
        out.append((f"wheel_lock_{sd}", box(lx - 20, lx + 30, y + sy * 8 - 8, y + sy * 8 + 8, zs - 120, zs - 25) +
                    box(lx + 10, lx + 22, y + sy * 8 - 5, y + sy * 8 + 5, zs - 25, zs + 70), "#b03030"))
        # swing-away footrest hanger (in its normal, forward position) + footplate
        ht = CH.hanger_top_z
        hang = Pos(fx + 30, y, ht) * Rot(0, 160, 0) * cyl(22.0, (ht - 110) / math.cos(math.radians(20)))
        out.append((f"footrest_hanger_{sd}", hang, "#9aa3ab"))
        out.append((f"push_handle_{sd}", cyl(30.0, 110, 40, y, zs + CH.back_height + 120, axis="-X"), "#333333"))
    # footplates and seat/back upholstery
    for sy in (1, -1):
        xb = fx + 30 + (CH.hanger_top_z - 110) * math.tan(math.radians(20))
        y = sy * w2
        out.append((f"footplate_{'L' if sy > 0 else 'R'}", box(xb - 20, xb + 150, min(y, sy * 20), max(y, sy * 20), 100, 110), "#555555"))
    out.append(("seat", box(40, 40 + CH.seat_depth, -CH.seat_width / 2, CH.seat_width / 2, zs - 30, zs), "#3a3a3a"))
    out.append(("back", box(20, 45, -CH.seat_width / 2, CH.seat_width / 2, zs, zs + CH.back_height), "#3a3a3a"))
    return out


def user_envelope():
    """Seated-occupant keep-out volumes (simplified adult, chair frame C) - NOT anthropometric data,
    a conservative box model for collision checks.  Replace with measurements of the actual user."""
    z = CH.seat_height
    fx = CH.front_post_x
    return [
        ("torso", box(45, 290, -200, 200, z, z + 600)),
        ("head", cyl(220, 250, 160, 0, z + 600)),
        ("thighs", box(60, fx + 60, -200, 200, z, z + 170)),
        ("shins", box(fx + 20, fx + 190, -180, 180, 110, z + 40)),
        ("upper_arms_elbows", box(40, 300, -265, 265, z + 200, z + 520)),
    ]


def parts():
    out = []
    cb = clamp_body()
    out.append(PartDef("02_tube_clamp_body", cb, "Tube clamp body (parametric tube OD)", qty=5,
                       material="PA-CF recommended (PETG ok)", color="#2f8f5b", print_loc=on_bed(cb), walls=6,
                       infill="80%", print_notes="Ring end-face down (layers wrap the tube). "
                                                  "Re-torque after 24 h (creep)."))
    cc = clamp_cap()
    out.append(PartDef("03_tube_clamp_cap", cc, "Tube clamp cap", qty=5, material="PA-CF recommended (PETG ok)",
                       color="#2f8f5b", print_loc=on_bed(cc), walls=6, infill="80%"))
    pb = post_bracket()
    out.append(PartDef("04_post_bracket", pb, "Post bracket (spine + shelf), same part L/R", material="PA-CF recommended",
                       color="#2f8f5b", print_loc=on_bed(pb, (0, -90, 0)), walls=6, infill="60% gyroid",
                       print_notes="Spine back face down: gussets and shelf print as vertical walls. "
                                   "Aluminium alternative: see output/dxf/04_post_bracket_plates.dxf."))
    sc = stop_collar()
    out.append(PartDef("05_stop_collar", sc, "Post stop collar (secondary retention)", material="PETG",
                       color="#2f8f5b", print_loc=on_bed(sc), walls=6, infill="80%"))
    return out
