"""Geometry helpers shared by all part modules (build123d algebra API).

Conventions: millimetres; every builder returns a build123d Part in its own LOCAL frame.
"""
from __future__ import annotations
import functools
import collections
import math
import pathlib
from dataclasses import dataclass, field

from build123d import (Align, Axis, Box, Color, Compound, Cylinder, Face, Location, Part,
                       Plane, Pos, Rot, Solid, Vector, Wire, extrude, import_step)

from .config import P

VENDOR = pathlib.Path(__file__).resolve().parents[1] / "vendor"

CENTER = (Align.CENTER, Align.CENTER, Align.CENTER)
MIN_Z = (Align.CENTER, Align.CENTER, Align.MIN)
MAX_Z = (Align.CENTER, Align.CENTER, Align.MAX)
MIN_ALL = (Align.MIN, Align.MIN, Align.MIN)


# ----------------------------------------------------------------------------- basics
def clr(size: str) -> float:
    """Printed clearance-hole diameter for a metric screw size ('M3')."""
    return P.clearance[size] + P.fit.hole_extra


def stl_clean(path) -> dict:
    """Make an exported STL print-clean: round to float32 (what the file stores), merge vertices,
    drop the zero-area triangles that rounding creates and re-write.  Returns a small report."""
    import numpy as np
    import trimesh
    m = trimesh.load(str(path))
    before = bool(m.is_watertight)
    m.vertices = np.asarray(m.vertices, dtype=np.float32).astype(np.float64)
    m.merge_vertices()
    m.update_faces(m.nondegenerate_faces())
    m.remove_unreferenced_vertices()
    m.fix_normals()
    m.export(str(path))
    m2 = trimesh.load(str(path))
    return {"watertight_before": before, "watertight": bool(m2.is_watertight), "bodies": len(m2.split(only_watertight=False))}


INSERT_LOG = collections.Counter()      # heat-set inserts modelled since the last PartDef (BOM count)


def ins_hole(size: str) -> float:
    """Pilot-hole diameter of a heat-set insert.  Every modelled insert hole calls this once, so
    the calls between two PartDef() creations are that part's insert count (used by the BOM)."""
    INSERT_LOG[size] += 1
    return P.insert[size].hole


def ins_depth(size: str) -> float:
    return P.insert[size].depth


def bearing(name: str):
    d, D, B = P.bearings[str(name)]
    return float(d), float(D), float(B)


def seat(name: str) -> float:
    """Printed housing bore for a bearing outer ring."""
    return bearing(name)[1] + P.fit.bearing_seat_extra


def box(x0, x1, y0, y1, z0, z1) -> Part:
    """Axis-aligned box from min/max coordinates."""
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def cyl(d, h, x=0, y=0, z=0, axis="Z", align="min") -> Part:
    """Cylinder of diameter d, height h, starting at (x,y,z) along +axis
    (align='min') or centred on it (align='center')."""
    al = MIN_Z if align == "min" else CENTER
    c = Cylinder(d / 2, h, align=al)
    if axis == "X":
        c = Rot(0, 90, 0) * c
    elif axis == "Y":
        c = Rot(-90, 0, 0) * c
    elif axis == "-Z":
        c = Rot(180, 0, 0) * c
    elif axis == "-X":
        c = Rot(0, -90, 0) * c
    elif axis == "-Y":
        c = Rot(90, 0, 0) * c
    return Pos(x, y, z) * c


def tube(od, idia, h, **kw) -> Part:
    return cyl(od, h, **kw) - cyl(idia, h + 0.02, **{**kw, **_shift(kw, -0.01)})


def _shift(kw, dz):
    axis = kw.get("axis", "Z")
    out = {}
    key = {"Z": "z", "-Z": "z", "X": "x", "-X": "x", "Y": "y", "-Y": "y"}[axis]
    sign = -1 if axis.startswith("-") else 1
    out[key] = kw.get(key, 0) + sign * dz
    return out


def poly_face(pts) -> Face:
    """Planar face in XY from a list of (x, y); always counter-clockwise (normal +Z) so that
    extrusions go in the +Z direction regardless of the point order supplied."""
    area2 = sum(pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1]
                for i in range(len(pts)))
    if area2 < 0:
        pts = list(reversed(pts))
    return Face(Wire.make_polygon([Vector(x, y, 0) for x, y in pts], close=True))


def prism(pts, h, z0=0.0) -> Part:
    return Pos(0, 0, z0) * extrude(poly_face(pts), h)


def prism_yz(pts, h, x0=0.0) -> Part:
    """Extrude a polygon given in (y, z) along +X from x0."""
    s = prism(pts, h)                                # (y, z) drawn as (x, y), extruded +Z
    return Pos(x0, 0, 0) * Rot(0, 90, 0) * Rot(0, 0, 90) * s


def prism_xz(pts, h, y0=0.0) -> Part:
    """Extrude a polygon given in (x, z) along +Y from y0 (built in XY, then rotated)."""
    s = prism([(x, -z) for x, z in pts], h)          # in XY, extruded +Z
    return Pos(0, y0, 0) * Rot(-90, 0, 0) * s        # XY->XZ: (x, y', z') -> (x, z', -y')


def slot2d(length, width, n=24):
    """Stadium outline centred at origin, long axis X."""
    r = width / 2
    a = length / 2 - r
    pts = []
    for i in range(n + 1):
        t = -math.pi / 2 + math.pi * i / n
        pts.append((a + r * math.cos(t), r * math.sin(t)))
    for i in range(n + 1):
        t = math.pi / 2 + math.pi * i / n
        pts.append((-a + r * math.cos(t), r * math.sin(t)))
    return pts


def slot(length, width, h, x=0, y=0, z=0, angle=0) -> Part:
    return Pos(x, y, z) * Rot(0, 0, angle) * prism(slot2d(length, width), h)


def rounded_rect2d(w, h, r, n=8):
    pts = []
    for cx, cy, a0 in ((w / 2 - r, h / 2 - r, 0), (-w / 2 + r, h / 2 - r, 90),
                       (-w / 2 + r, -h / 2 + r, 180), (w / 2 - r, -h / 2 + r, 270)):
        for i in range(n + 1):
            t = math.radians(a0 + 90 * i / n)
            pts.append((cx + r * math.cos(t), cy + r * math.sin(t)))
    return pts


def rbox(w, h, t, r, z0=0.0) -> Part:
    """Rounded-corner plate w (X) x h (Y) x t (Z), corner radius r, bottom at z0."""
    return prism(rounded_rect2d(w, h, max(r, 0.01)), t, z0)


def hexagon2d(af):
    r = af / math.sqrt(3)
    return [(r * math.cos(math.radians(30 + 60 * i)), r * math.sin(math.radians(30 + 60 * i)))
            for i in range(6)]


def nut_trap(size, depth, x=0, y=0, z=0, extra=0.25) -> Part:
    af = P.nut[size][0] + extra
    return Pos(x, y, z) * prism(hexagon2d(af), depth)


def cbore_screw(size, length, x=0, y=0, z=0, head_depth=None, axis="-Z") -> Part:
    """Counterbored clearance hole. Head seat at z (top), shank going -Z (default)."""
    hd, hh = P.shcs_head[size]
    hd += 0.6
    head_depth = hh + 0.4 if head_depth is None else head_depth
    s = cyl(clr(size), length, 0, 0, 0, axis="-Z") + cyl(hd, head_depth + 50, 0, 0, -head_depth)
    if axis == "-Z":
        r = s
    elif axis == "Z":
        r = Rot(180, 0, 0) * s
    elif axis == "X":
        r = Rot(0, -90, 0) * s
    elif axis == "-X":
        r = Rot(0, 90, 0) * s
    elif axis == "Y":
        r = Rot(90, 0, 0) * s
    elif axis == "-Y":
        r = Rot(-90, 0, 0) * s
    return Pos(x, y, z) * r


def Cone_cs(d_top, d_bot, h) -> Part:
    """Countersink cone, bottom at z=0 (dia d_bot), top at z=h (dia d_top)."""
    from build123d import Cone
    return Cone(d_bot / 2, d_top / 2, h, align=MIN_Z)


def polar(r, ang_deg, z=0.0):
    a = math.radians(ang_deg)
    return (r * math.cos(a), r * math.sin(a), z)


# ----------------------------------------------------------------------------- HTD pulley
def pd(teeth: int) -> float:
    return teeth * P.belt.pitch / math.pi


def od(teeth: int) -> float:
    return pd(teeth) - 2 * P.belt.pld


def htd_profile(teeth: int, samples_per_tooth: int = 14):
    """Polar-sampled outline of an HTD-3M pulley (printable approximation).
    Groove = circle (radius groove_r + clear) whose bottom sits groove_depth (+clear)
    below the OD."""
    b = P.belt
    r_out = od(teeth) / 2
    rg = b.groove_r + b.print_clear
    r_c = r_out - (b.groove_depth + b.print_clear) + rg
    pts = []
    n = teeth * samples_per_tooth
    pitch_ang = 2 * math.pi / teeth
    for k in range(n):
        th = 2 * math.pi * k / n
        # nearest groove centre
        i = round(th / pitch_ang)
        d = th - i * pitch_ang
        s = r_c * math.sin(d)
        r = r_out
        if abs(s) <= rg:
            rr = r_c * math.cos(d) - math.sqrt(rg * rg - s * s)
            r = min(r_out, rr)
        pts.append((r * math.cos(th), r * math.sin(th)))
    return pts


def htd_pulley(teeth: int, flanges: bool = True, length: float | None = None) -> Part:
    """Toothed pulley body (no bore), axis Z, from z=0 upward:
       [flange][teeth][flange]."""
    b = P.belt
    tl = b.tooth_len if length is None else length
    teeth_part = prism(htd_profile(teeth), tl, b.flange if flanges else 0)
    if not flanges:
        return teeth_part
    fd = od(teeth) + 2 * b.flange_h
    fl = cyl(fd, b.flange) + cyl(fd, b.flange, z=b.flange + tl)
    return teeth_part + fl


def pulley_len(flanges=True) -> float:
    b = P.belt
    return b.tooth_len + (2 * b.flange if flanges else 0)


def belt_length(t1: int, t2: int, c: float) -> float:
    """Exact open-belt pitch length for centre distance c (straight spans tangent to both pitch circles)."""
    r1, r2 = pd(t1) / 2, pd(t2) / 2
    beta = math.asin((r2 - r1) / c)
    return 2 * c * math.cos(beta) + math.pi * (r1 + r2) + 2 * beta * (r2 - r1)


def belt_center_distance(t1: int, t2: int, length: float) -> float:
    """Exact open-belt centre distance for pitch length `length`."""
    r1, r2 = pd(t1) / 2, pd(t2) / 2

    def L(c):
        if c <= abs(r2 - r1):
            return 0
        beta = math.asin((r2 - r1) / c)
        return 2 * c * math.cos(beta) + math.pi * (r1 + r2) + 2 * beta * (r2 - r1)

    lo, hi = abs(r2 - r1) + 1e-6, 2000.0
    for _ in range(200):
        mid = (lo + hi) / 2
        if L(mid) < length:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def belt_solid(t1, t2, c, ang_deg=0.0, width=None, thick=2.4) -> Part:
    """Visual belt loop between pulley t1 at origin and t2 at distance c along ang_deg,
    in XY plane, from z=0 to z=width."""
    width = P.belt.width if width is None else width
    r1, r2 = pd(t1) / 2, pd(t2) / 2
    beta = math.asin((r2 - r1) / c)
    pts_out = []
    # outer loop sampled; belt is offset outwards from pitch line by ~0.6, inwards 1.8
    def loop(off):
        pts = []
        # around pulley 1 (origin), from angle (90+beta) to (270-beta)
        a0, a1 = math.pi / 2 + beta, 3 * math.pi / 2 - beta
        for i in range(41):
            a = a0 + (a1 - a0) * i / 40
            pts.append(((r1 + off) * math.cos(a), (r1 + off) * math.sin(a)))
        # around pulley 2 at (c,0), from -(90-beta) .. (90-beta)
        a0, a1 = -math.pi / 2 + beta, math.pi / 2 - beta
        for i in range(41):
            a = a0 + (a1 - a0) * i / 40
            pts.append((c + (r2 + off) * math.cos(a), (r2 + off) * math.sin(a)))
        return pts
    outer = prism(loop(0.6), width)
    inner = prism(loop(-1.8), width + 0.2, -0.1)
    return Rot(0, 0, ang_deg) * (outer - inner)


# ----------------------------------------------------------------------------- gears
def involute_gear_profile(module, teeth, backlash=0.15, pressure_deg=20.0, n_inv=10):
    rp = module * teeth / 2
    rb = rp * math.cos(math.radians(pressure_deg))
    ra = rp + module
    rf = rp - 1.25 * module

    def inv(a):
        return math.tan(a) - a
    ap = math.radians(pressure_deg)
    half_thick_ang = (math.pi * module / 2 - backlash / 2) / (2 * rp)  # at pitch circle
    pts = []
    pitch_ang = 2 * math.pi / teeth
    r_start = max(rb, rf)
    radii = [r_start + (ra - r_start) * i / n_inv for i in range(n_inv + 1)]

    def flank_ang(r):
        ar = math.acos(min(1.0, rb / r))
        return half_thick_ang + inv(ap) - inv(ar)
    for i in range(teeth):
        c = i * pitch_ang
        # root arc to the right flank
        a_r0 = c - flank_ang(r_start)
        a_prev = c - pitch_ang + flank_ang(r_start)
        for k in range(4):
            a = a_prev + (a_r0 - a_prev) * k / 3
            pts.append((rf * math.cos(a), rf * math.sin(a)))
        if rf < rb:
            pts.append((rb * math.cos(a_r0), rb * math.sin(a_r0)))
        for r in radii:  # right flank up
            a = c - flank_ang(r)
            pts.append((r * math.cos(a), r * math.sin(a)))
        a_tip0, a_tip1 = c - flank_ang(ra), c + flank_ang(ra)
        for k in range(1, 3):
            a = a_tip0 + (a_tip1 - a_tip0) * k / 3
            pts.append((ra * math.cos(a), ra * math.sin(a)))
        for r in reversed(radii):  # left flank down
            a = c + flank_ang(r)
            pts.append((r * math.cos(a), r * math.sin(a)))
        if rf < rb:
            a = c + flank_ang(r_start)
            pts.append((rf * math.cos(a), rf * math.sin(a)))
    # dedupe consecutive
    out = []
    for p in pts:
        if not out or (abs(p[0] - out[-1][0]) > 1e-6 or abs(p[1] - out[-1][1]) > 1e-6):
            out.append(p)
    return out


def spur_gear(module, teeth, width, backlash=0.15) -> Part:
    return prism(involute_gear_profile(module, teeth, backlash), width)


# ----------------------------------------------------------------------------- vitamins
def bearing_model(name) -> Part:
    d, D, B = bearing(name)
    return tube(D, d, B)


def pin_model(d, length) -> Part:
    return cyl(d, length)


def tube_model(od_, wall, length, axis="X") -> Part:
    return tube(od_, od_ - 2 * wall, length, axis=axis)


@functools.lru_cache(maxsize=1)
def _servo_ref():
    p = VENDOR / "STS3215_03a.step"
    if p.exists():
        return import_step(str(p))
    return None


def servo_model() -> Part:
    """Reference ST3215/STS3215 solid (vendor STEP) in servo frame; falls back to envelope."""
    s = _servo_ref()
    if s is not None:
        return s
    return servo_envelope(0.0)


def servo_envelope(c: float = 0.0, horn=True, rear=True, connector=True) -> Part:
    """Conservative keep-out solid of the servo in the servo frame (+clearance c)."""
    s = P.servo
    b = s.body
    env = box(b.x[0] - c, b.x[1] + c, b.y[0] - c, b.y[1] + c, s.cover_bot_z - c, s.cover_top_z + c)
    h = s.horn
    if horn:
        env += cyl(h.dia + 2 * c, h.z1 - b.z[1] + c, s.out_x, 0, b.z[1])
        env += cyl(h.screw_head_dia + 2 * c, h.screw_head_z - b.z[1] + c, s.out_x, 0, b.z[1])
    if rear:
        r = s.rear_horn
        env += cyl(r.dia + 2 * c, s.cover_bot_z - r.z0 + c, s.out_x, 0, r.z0 - c)
        env += cyl(r.boss_dia + 2 * c, s.cover_bot_z - r.boss_z0 + c, s.out_x, 0, r.boss_z0 - c)
        k = s.rear_block
        env += box(k.x[0] - c, k.x[1] + c, k.y[0] - c, k.y[1] + c, k.z[0] - c, k.z[1] + 0.01)
    if connector:
        k = s.connector
        env += box(k.x[0] - c, k.x[1] + c, k.y[0] - c, k.y[1] + c,
                   k.z_out - k.cable_clear, s.cover_bot_z + 0.01)
    return env


def servo_body_pocket(c=None, z_top=None, z_bot=None) -> Part:
    """Rectangular body pocket (with clearance) between z_bot and z_top in servo frame."""
    s = P.servo
    c = P.fit.servo_pocket_clear if c is None else c
    b = s.body
    z_top = s.cover_top_z + c if z_top is None else z_top
    z_bot = s.cover_bot_z - c if z_bot is None else z_bot
    return box(b.x[0] - c, b.x[1] + c, b.y[0] - c, b.y[1] + c, z_bot, z_top)


def horn_bolts(z0: float, length: float, up=True):
    """4x M3 clearance holes around the output axis (servo frame), at horn bolt circle."""
    h = P.servo.horn
    out = None
    for i in range(4):
        a = math.radians(h.bolt_angle0 + 90 * i)
        x = P.servo.out_x + h.bolt_circle / 2 * math.cos(a)
        y = h.bolt_circle / 2 * math.sin(a)
        c = cyl(clr("M3"), length, x, y, z0, axis="Z" if up else "-Z")
        out = c if out is None else out + c
    return out


def horn_bolt_xy(cx=0.0, cy=0.0):
    h = P.servo.horn
    pts = []
    for i in range(4):
        a = math.radians(h.bolt_angle0 + 90 * i)
        pts.append((cx + h.bolt_circle / 2 * math.cos(a), cy + h.bolt_circle / 2 * math.sin(a)))
    return pts


# ----------------------------------------------------------------------------- part meta
@dataclass
class PartDef:
    name: str                    # file stem, e.g. "11_dm_bracket"
    shape: Part                  # geometry in MODEL (assembly-local) frame
    title: str = ""
    category: str = "printed"    # printed | vitamin | cut
    material: str = "PETG"
    qty: int = 1
    color: str = "#d9d9d9"
    print_loc: Location | None = None   # transform model frame -> bed (Z up, on z=0)
    print_notes: str = ""
    walls: int = 4
    infill: str = "40% gyroid"
    supports: str = "none"
    notes: str = ""
    lr_specific: bool = False
    load_bearing: bool = True
    extra: dict = field(default_factory=dict)
    inserts: dict = field(default_factory=dict)

    def __post_init__(self):
        if not self.inserts:
            self.inserts = {k: v for k, v in INSERT_LOG.items() if v}
        INSERT_LOG.clear()


def on_bed(shape: Part, rot=(0, 0, 0)) -> Location:
    """Location that rotates `shape` by rot (deg, XYZ) and drops it onto z=0, centred."""
    r = Rot(*rot)
    s = r * shape
    bb = s.bounding_box()
    return Pos(-(bb.min.X + bb.max.X) / 2, -(bb.min.Y + bb.max.Y) / 2, -bb.min.Z) * r


def colorize(shape, hexcol, label=None):
    shape.color = Color(hexcol)
    if label:
        shape.label = label
    return shape
