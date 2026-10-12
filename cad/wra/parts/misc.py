"""Electronics enclosure, E-stop pendant, print-fit test coupon and bench test stand."""
from __future__ import annotations
import math
from build123d import Pos, Rot, Part

from ..config import P
from .. import lib
from ..lib import box, cyl, clr, PartDef, on_bed, prism
from .mount import H as CLAMP_H

EL = P.electronics
WALL = 3.0


# ------------------------------------------------------------------------- electronics box
def ebox() -> Part:
    """Hangs on the two rear canes with tube clamps. Floor has M3 slot rows so the Jetson
    dev-kit, ESP32, bus adapter, DC-DC converters, relays and fuse block can be bolted
    anywhere (positions are documented in docs/07_electronics.md and docs/09_safety.md)."""
    ix, iy, iz = EL.box_inner
    b = box(-ix / 2 - WALL, ix / 2 + WALL, -iy / 2 - WALL, iy / 2 + WALL, 0, iz + WALL)
    b -= box(-ix / 2, ix / 2, -iy / 2, iy / 2, WALL, iz + WALL + 1)
    # floor slot rows (M3), 15 mm pitch
    for j in range(-4, 5):
        y = j * 15.0
        for x0 in (-80.0, -20.0, 40.0):
            b -= lib.slot(40.0, clr("M3"), WALL + 2, x0 + 20.0, y, -1)
    # lid inserts in wall tops (corner bosses)
    for sx in (1, -1):
        for sy in (1, -1):
            x, y = sx * (ix / 2 - 3.5), sy * (iy / 2 - 3.5)
            b += cyl(9.0, iz, x, y, WALL - 0.01)
            b -= cyl(lib.ins_hole("M3"), lib.ins_depth("M3"), x, y, iz + WALL - lib.ins_depth("M3") + 0.01)
    # front wall (-Y) panel cut-outs: arm GX16, pendant GX12, XT60 input, rocker switch, START, LED
    yfw = -iy / 2 - WALL - 1
    for x, z, d in ((-70.0, 32.0, 16.2), (-40.0, 32.0, 12.2), (35.0, 32.0, 22.5), (60.0, 32.0, 5.2)):
        b -= cyl(d, WALL + 2, x, yfw, z, axis="Y")
    b -= box(-18.0, -5.0, yfw, yfw + WALL + 2, 25.0, 40.0)        # XT60 panel holder window
    b -= box(5.0, 22.0, yfw, yfw + WALL + 2, 23.0, 42.0)          # rocker main switch 13 x 19 cut
    # side vents (Jetson fan + DC-DC) on +/-X walls
    for sx in (1, -1):
        for k in range(6):
            z = 14.0 + 7.0 * k
            b -= box(sx * (ix / 2 - 0.5) - (1 if sx > 0 else WALL + 1), sx * (ix / 2 - 0.5) + (WALL + 1 if sx > 0 else 1),
                     -40.0, 40.0, z, z + 3.0)
    # USB/HDMI service window in rear wall for the Jetson
    b -= box(-30.0, 30.0, iy / 2 - 0.5, iy / 2 + WALL + 1, 12.0, 30.0)
    # mounting pads on the OUTSIDE of the floor: the box hangs on ONE vertical rear cane with two
    # standard tube clamps (02/03) at the same 90 mm spacing as the arm post.  Long axis vertical,
    # floor toward the cane, lid toward the rear.  Same box both sides (flip it upside-down on the
    # left so the connector wall still faces outboard).  M5 screws from inside into the clamp pads.
    for x in ebox_pad_x():
        y = EBOX_PAD_Y
        b += box(x - 15.0, x + 15.0, y - 13.0, y + 13.0, -6.0, 0.01)
        for dx in (-CLAMP_H / 4, CLAMP_H / 4):
            b -= lib.slot(12.0, clr("M5"), WALL + 8.0, x + dx, y, -7.0, angle=90)
    return b


EBOX_PAD_Y = -55.0


def ebox_pad_x():
    return (-P.mount.clamp_spacing / 2, P.mount.clamp_spacing / 2)


def ebox_lid() -> Part:
    ix, iy, iz = EL.box_inner
    l = box(-ix / 2 - WALL, ix / 2 + WALL, -iy / 2 - WALL, iy / 2 + WALL, iz + WALL + 0.2, iz + 2 * WALL + 0.2)
    l += box(-ix / 2 + 0.3, ix / 2 - 0.3, -iy / 2 + 0.3, iy / 2 - 0.3, iz + WALL - 2.0, iz + WALL + 0.21) - \
        box(-ix / 2 + 2.3, ix / 2 - 2.3, -iy / 2 + 2.3, iy / 2 - 2.3, iz + WALL - 3, iz + WALL + 1)
    for sx in (1, -1):
        for sy in (1, -1):
            x, y = sx * (ix / 2 - 3.5), sy * (iy / 2 - 3.5)
            l -= cyl(clr("M3"), 10, x, y, iz)
            l -= box(x - 5, x + 5, y - 5, y + 5, iz + WALL - 3, iz + WALL + 0.21)
    for k in range(8):
        l -= box(-60 + 15 * k, -55 + 15 * k, -30, 30, iz + WALL - 1, iz + 2 * WALL + 1)   # top vents
    return l


# ------------------------------------------------------------------------- E-stop pendant
def pendant() -> Part:
    """Hand/armrest pendant: 22 mm E-stop mushroom (NC) + 22 mm guarded START (NO)."""
    w, d, h = 64.0, 48.0, 42.0
    b = box(-w / 2, w / 2, -d / 2, d / 2, 0, h)
    b -= box(-w / 2 + WALL, w / 2 - WALL, -d / 2 + WALL, d / 2 - WALL, WALL, h + 1)
    for sx in (1, -1):
        for sy in (1, -1):
            x, y = sx * (w / 2 - 5.5), sy * (d / 2 - 5.5)
            b += cyl(8.0, h - WALL, x, y, WALL - 0.01)
            b -= cyl(lib.ins_hole("M3"), lib.ins_depth("M3"), x, y, h - lib.ins_depth("M3") + 0.01)
    b -= cyl(12.5, WALL + 2, 0, d / 2 - 1, 14.0, axis="Y")                 # PG7 cable gland
    # strap slots for a 25 mm velcro strap around the armrest
    for sx in (1, -1):
        b -= box(sx * 22 - 2, sx * 22 + 2, -14, 14, -1, WALL + 1)
    return b


def pendant_lid() -> Part:
    w, d = 64.0, 48.0
    l = box(-w / 2, w / 2, -d / 2, d / 2, 42.0, 45.0)
    l -= cyl(P.electronics.estop_hole, 10, -14.0, 0, 40.0)                 # E-stop operator
    l -= cyl(P.electronics.estop_hole, 10, 18.0, 0, 40.0)                  # START button
    l += cyl(30.0, 9.0, 18.0, 0, 45.0) - cyl(25.0, 12.0, 18.0, 0, 44.0)    # guard ring around START
    for sx in (1, -1):
        for sy in (1, -1):
            l -= cyl(clr("M3"), 10, sx * (w / 2 - 5.5), sy * (d / 2 - 5.5), 40.0)
    return l


# ------------------------------------------------------------------------- fit-test coupon
COUPON_FEATURES = [
    # (label, kind, nominal, x, y)   kind: seat = bearing OD bore, press = pin press bore,
    #                                insert = heat-set pilot, clear = clearance, sock = tube socket
    ("A 6806 seat", "seat", "6806", 28.0, 30.0),
    ("B 608 seat", "seat", "608", 66.0, 46.0),
    ("C 6802 seat", "seat", "6802", 66.0, 16.0),
    ("D 688/625 seat", "seat", "688", 95.0, 46.0),
    ("E 626/6800 seat", "seat", "626", 95.0, 18.0),
    ("F 8 mm press", "press", 8.0, 118.0, 50.0),
    ("G 6 mm press", "press", 6.0, 118.0, 36.0),
    ("H 10 mm press", "press", 10.0, 118.0, 20.0),
    ("I M3 insert", "insert", "M3", 136.0, 52.0),
    ("J M4 insert", "insert", "M4", 136.0, 38.0),
    ("K M5 insert", "insert", "M5", 136.0, 22.0),
    ("L 12 mm tube socket", "sock", 12.0, 155.0, 30.0),
]


def fit_coupon() -> Part:
    c = lib.rbox(170.0, 64.0, 10.0, 6.0)
    c = Pos(85.0, 32.0, 0) * c
    for label, kind, nom, x, y in COUPON_FEATURES:
        if kind == "seat":
            d = lib.seat(nom)
            depth = lib.bearing(nom)[2]
            c -= cyl(d, depth + 0.01, x, y, 10.0 - depth)
            c -= cyl(lib.bearing(nom)[0] + 3.0, 11, x, y, -0.5)
        elif kind == "press":
            c -= cyl(nom - P.fit.pin_press_minus, 11, x, y, -0.5)
        elif kind == "insert":
            c -= cyl(lib.ins_hole(nom), lib.ins_depth(nom), x, y, 10.0 - lib.ins_depth(nom) + 0.01)
        elif kind == "sock":
            c -= cyl(nom + P.fit.tube_socket_extra, 11, x, y, -0.5)
    # index notches 1..12 along the front edge (count from the left)
    for i, (_, _, _, x, _) in enumerate(COUPON_FEATURES):
        for k in range(1 + i % 3):
            c -= box(x - 3 + 2.2 * k, x - 2 + 2.2 * k, -0.5, 1.5, 8.5, 10.5)
    return c


# ------------------------------------------------------------------------- bench test stand
def bench_stand() -> Part:
    """Table-top stand: clamp to a sturdy table with two C-clamps; holds a vertical tube of the
    SAME OD as the wheelchair post, so the real clamps/bracket/arm are tested away from people."""
    tube = P.mount.tube_od
    b = lib.rbox(220.0, 170.0, 12.0, 10.0)
    b += cyl(tube + 22.0, 60.0, 0, 0, 11.99)
    for a in (0, 90, 180, 270):
        rib = lib.prism_xz([(tube / 2 + 8.0, 11.99), (tube / 2 + 60.0, 11.99), (tube / 2 + 8.0, 64.0)], 8.0, -4.0)
        b += Rot(0, 0, a) * rib
    b -= cyl(tube + 0.3, 60.0, 0, 0, 14.0)
    for z in (30.0, 55.0):
        b -= cyl(clr("M5"), 40, 0, 0, z, axis="X")
        b -= Pos(tube / 2 + 3.0, 0, z) * Rot(0, 90, 0) * prism(lib.hexagon2d(8.4), 4.6)
    # C-clamp ledges (front and back edges) - flat lands, and 4x M6 holes for bolting down
    for sx in (1, -1):
        for sy in (1, -1):
            b -= cyl(clr("M6"), 20, sx * 92.0, sy * 68.0, -1)
    return b


def parts():
    out = []
    eb = ebox()
    out.append(PartDef("60_electronics_box", eb, "Electronics enclosure (Jetson, ESP32, power, E-stop relays)",
                       material="PETG", color="#4d4d4d", print_loc=on_bed(eb), walls=4, infill="20% gyroid",
                       load_bearing=False, print_notes="Open side up. 0.24 mm layers OK."))
    el = ebox_lid()
    out.append(PartDef("61_electronics_lid", el, "Electronics enclosure lid", material="PETG", color="#5a5a5a",
                       print_loc=on_bed(el, (180, 0, 0)), walls=3, infill="15%", load_bearing=False))
    pd_ = pendant()
    out.append(PartDef("62_estop_pendant", pd_, "E-stop pendant body", material="PETG (yellow recommended)",
                       color="#e8c020", print_loc=on_bed(pd_), walls=3, infill="20%", load_bearing=False))
    pl = pendant_lid()
    out.append(PartDef("63_estop_pendant_lid", pl, "E-stop pendant lid (E-stop + guarded START)",
                       material="PETG (yellow)", color="#e8c020", print_loc=on_bed(pl, (180, 0, 0)), walls=3,
                       infill="20%", load_bearing=False))
    fc = fit_coupon()
    out.append(PartDef("00_fit_test_coupon", fc, "PRINT FIRST: fit-test coupon (bearings, pins, inserts)",
                       material="same as production", color="#d0d0d0", print_loc=on_bed(fc), walls=4,
                       infill="40%", load_bearing=False))
    bs = bench_stand()
    out.append(PartDef("70_bench_test_stand", bs, "Bench test stand (holds a post tube like the chair)",
                       material="PETG", color="#7a7a7a", print_loc=on_bed(bs), walls=5, infill="40% gyroid"))
    return out
