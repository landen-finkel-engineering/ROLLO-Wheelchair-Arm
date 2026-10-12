"""Knob claw: parallel rack-and-pinion gripper on a quick-change dovetail, made for door knobs.

Why this layout (docs/10_door_tasks.md, calcs/): a door knob is turned through FRICTION, so it must be
squeezed hard and stay CENTRED on the roll axis while it turns.  One pinion on the claw servo
drives rack A (above it) and rack B (below it) in opposite directions, so the jaws move
symmetrically (knob stays centred) and stay parallel (full-length pad contact).
  jaw force = servo torque / (2 * r_pinion), r = 10.5 mm
  -> 35 N per jaw at 25 % of stall (continuous), 70 N at 50 % ("knob mode", <= 10 s)
A 100 deg V-groove pad (silicone facing) centres round knobs vertically; fingertip lips hook
behind the knob so the arm (and chair) can pull the door after unlatching.

G frame (= tool flange): origin = flange FRONT face centre, X forward (tool axis), Y opening
direction, Z up at home.  Slim near the wrist (radius <= 37 for x < 50) so the ABENICS wrist
can tilt +/-33 deg.  Opening q = distance of each pad's contact plane from the X axis.
"""
from __future__ import annotations

import math

from build123d import Pos, Rot, Part, Plane, Location

from ..config import P
from .. import lib
from ..lib import box, cyl, clr, PartDef, on_bed, prism

C = P.claw
S = P.servo
M = C.pinion_module
RP = C.pinion_teeth * M / 2                       # 10.5
DOVE = P.tool_interface.dovetail
ADAPTER_T = P.wrist.tool_flange.t
CL = P.fit.servo_pocket_clear
XP0, XP1 = C.x_pinion - C.pinion_width / 2, C.x_pinion + C.pinion_width / 2     # 52.6 .. 58.6
X_HORN = XP0 - 0.6
SERVO_ORIGIN = (X_HORN - S.horn.z1, 0.0, -S.out_x)          # body centre; output axis = X_G
BODY_X = (SERVO_ORIGIN[0] + S.cover_bot_z - CL, SERVO_ORIGIN[0] + S.cover_top_z + CL)
BODY_Y = (-S.body.y[1] - CL, S.body.y[1] + CL)
BODY_Z = (-S.out_x + S.body.x[0] - CL, -S.out_x + S.body.x[1] + CL)   # hangs below the axis
RACK_ROOT = RP + 1.25 * M                         # 12.375
RACK_TOP = RACK_ROOT + C.rack_h                   # 17.375
RAIL_X = (XP0 - 5.0 - 9.0, XP0 - 5.0)             # 38.6 .. 47.6
RAIL_XC = 0.5 * (RAIL_X[0] + RAIL_X[1])
BLOCK_X = (RAIL_XC - C.block_w / 2, RAIL_XC + C.block_w / 2)
Z_RAIL0 = C.rail_z
Z_BLOCK1 = Z_RAIL0 + C.block_h                    # carriage plate bottom (28)
Z_PLATE1 = Z_BLOCK1 + C.carriage_t
FX0, FX1 = C.finger_x
PAD_X = (FX0 + 2.0, FX1 - C.lip_t - 2.0)
Q_MAX = C.open_max / 2
YC_OFF = C.block_len / 2 + 0.6                    # block centre = q + this (blocks just clear at q = 0)


def servo_loc() -> Location:
    return Plane(origin=SERVO_ORIGIN, x_dir=(0, 0, 1), z_dir=(1, 0, 0)).location


def tcp_x() -> float:
    return 0.5 * (PAD_X[0] + PAD_X[1])


def jaw_force(tau_servo_Nm: float) -> float:
    return tau_servo_Nm / (2 * RP / 1000.0)


def q_to_servo_deg(q: float) -> float:
    """Pinion rotation (deg) from fully closed (q = 0) to half-opening q (mm)."""
    return math.degrees(q / RP)


# ------------------------------------------------------------------------- gears
def pinion_at(q: float) -> Part:
    """Pinion rotated for half-opening q (top teeth move +Y with rack A)."""
    return Rot(-math.degrees(q / RP), 0, 0) * pinion()


def pinion() -> Part:
    g = Pos(0, 0, -C.pinion_width / 2) * lib.spur_gear(M, C.pinion_teeth, C.pinion_width)
    g -= cyl(9.0, C.pinion_width + 2, z=-C.pinion_width / 2 - 1)
    for x, y in lib.horn_bolt_xy():
        g -= cyl(clr("M3"), C.pinion_width + 2, x, y, -C.pinion_width / 2 - 1)
        g -= cyl(6.0, C.pinion_width, x, y, -C.pinion_width / 2 + 2.5)
    return Pos(C.x_pinion, 0, 0) * Rot(0, 90, 0) * g


def _rack_teeth(y0, y1, shift=0.0):
    """Rack outline in (y, h): pitch line at h = 0, tooth tips at h = +M (toward the pinion),
    body down to h = -(1.25 M + rack_h)."""
    p = math.pi * M
    t = math.tan(math.radians(20.0))
    hf, ha = 1.25 * M, M
    bw = p / 2 + 2 * hf * t - 0.1          # tooth width at the root line
    tw = p / 2 - 2 * ha * t - 0.1          # at the tip line
    pts = [(y0, -hf - C.rack_h)]
    k0 = math.floor((y0 - shift) / p) - 1
    k1 = math.ceil((y1 - shift) / p) + 1
    for k in range(k0, k1 + 1):
        yc = (k + 0.5) * p + shift          # rack gaps at y = shift -> pinion teeth at top/bottom when q = 0
        for y, h in ((yc - bw / 2, -hf), (yc - tw / 2, ha), (yc + tw / 2, ha), (yc + bw / 2, -hf)):
            if y0 + 0.3 < y < y1 - 0.3:
                pts.append((y, h))
    pts.append((y1, -hf - C.rack_h))
    return pts


def rack_solid(y0, y1, top: bool, shift: float = 0.0) -> Part:
    """Rack in G between y0..y1; top=True: above the pinion, teeth pointing down.  shift = rack
    displacement (teeth move with the jaw)."""
    prof = _rack_teeth(y0, y1, shift)
    if top:
        pts = [(y, RP - h) for (y, h) in prof]
    else:
        pts = [(y, -RP + h) for (y, h) in prof]
    return lib.prism_yz(pts, C.pinion_width, XP0)


# ------------------------------------------------------------------------- jaws
def jaw(q: float, side: int = 1) -> Part:
    """Jaw A (side=+1, opens toward +Y, rack on TOP of the pinion, lip on the upper half) or
    jaw B (side=-1, rack BELOW the pinion, lip on the lower half), at half-opening q."""
    s = side
    yc = s * (q + YC_OFF)
    hb = C.block_len / 2

    def yb(a, b):          # y-range helper in this jaw's side
        lo, hi = sorted((s * a, s * b))
        return lo, hi
    y_in, y_out = q + C.pad_proud, q + C.pad_proud + C.finger_t           # finger plate (unsigned)
    # carriage plate on the block, reaching forward over the pinion plane
    plate = box(BLOCK_X[0], XP1 + 12.0, *(sorted((yc - hb, yc + hb))), Z_BLOCK1, Z_PLATE1)
    # finger: from the plate down to below the pad zone, forward of the pinion plane
    fz0 = -C.pad_h / 2 - 3.0 if s > 0 else -RACK_TOP
    finger = box(XP1 + 1.4, FX1, *yb(y_in, y_out), fz0, Z_PLATE1)
    finger += box(XP1 + 1.4, FX1, *yb(y_in, y_out + 8.0), Z_BLOCK1, Z_PLATE1)   # stiff top flange
    body = plate + finger
    if s > 0:
        # web from the plate down to rack A (in the pinion plane, above the pinion)
        body += box(XP0 + 1.0, XP1, *(sorted((yc - hb, yc + hb))), RACK_TOP - 0.01, Z_BLOCK1 + 0.01)
        body += rack_solid(yc + hb - C.rack_len, yc + hb, top=True, shift=q)
    else:
        # rack B under the pinion, joined to the finger bottom by a short arm
        body += rack_solid(yc - hb, yc - hb + C.rack_len, top=False, shift=-q)
        body += box(XP0, XP1 + 4.0, *yb(y_in, y_out), -RACK_TOP, -RACK_ROOT)
    # knob lip at the tip: upper half on jaw A, lower half on jaw B; notch for the other jaw's lip
    zl = (3.0, C.pad_h / 2) if s > 0 else (-C.pad_h / 2, -3.0)
    body += box(FX1 - C.lip_t, FX1, *yb(q - C.lip_d, y_in + 0.01), *zl)
    zn = (-C.pad_h / 2 - 3.1, -2.5) if s > 0 else (2.5, C.pad_h / 2 + 0.1)
    body -= box(FX1 - C.lip_t - 0.5, FX1 + 1, *yb(y_in - 0.01, y_in + C.lip_d + 0.6), *zn)
    # pad pocket + 2 M3 inserts
    body -= box(PAD_X[0] - 0.2, PAD_X[1] + 0.2, *yb(y_in - 1.0, y_in + C.pad_pocket), -C.pad_h / 2, C.pad_h / 2)
    for x in pad_screw_x():
        # M3 through the pad and the finger, hex nut recessed in the finger's outer face
        body -= Pos(x, s * (y_in - 1.0), 0) * Rot(-90 * s, 0, 0) * cyl(clr("M3"), C.finger_t + 3.0)
        body -= Pos(x, s * (y_out - 2.4), 0) * Rot(-90 * s, 0, 0) * prism(lib.hexagon2d(5.8), 3.0)
    # 4x M3 clearance to the MGN9C block (pattern block_holes: across X x along Y)
    bx, by = C.block_holes
    for dx in (-bx / 2, bx / 2):
        for dy in (-by / 2, by / 2):
            body -= cyl(clr("M3"), C.carriage_t + 2, RAIL_XC + dx, yc + dy, Z_BLOCK1 - 1)
    return body


def pad_screw_x():
    return (PAD_X[0] + 8.0, PAD_X[1] - 8.0)


def pad(q: float, side: int = 1) -> Part:
    """TPU pad (silicone faced) with a 100 deg V-groove along X; contact plane at |y| = q."""
    s = side
    h = C.pad_h - 0.4
    v = C.v_depth
    back = q + C.pad_proud + C.pad_pocket - 0.1
    half = v * math.tan(math.radians(50.0))     # half-width of the V at the flats
    prof = [(back, -h / 2), (q, -h / 2), (q, -half), (q + v, 0.0), (q, half), (q, h / 2), (back, h / 2)]
    pts = [(s * y, z) for y, z in prof]
    if s < 0:
        pts = pts[::-1]
    pd = lib.prism_yz(pts, PAD_X[1] - PAD_X[0], PAD_X[0])
    for x in pad_screw_x():
        # screw from the V floor through the pad into the jaw insert; head recessed 3 mm
        pd -= Pos(x, s * (q - 0.5), 0) * Rot(-90 * s, 0, 0) * cyl(clr("M3"), back - q + 2)
        pd -= Pos(x, s * (q - 0.5), 0) * Rot(-90 * s, 0, 0) * cyl(6.2, v + 3.0 + 0.5)
    return pd


# ------------------------------------------------------------------------- palm
def palm() -> Part:
    rw, tw, dh, dl = DOVE
    px0 = ADAPTER_T + 0.3
    # dovetail block (slim near the flange) -> cradle around the hanging servo -> post -> rail beam
    blk = box(px0, BODY_X[0] - 3.9, -16.0, 16.0, BODY_Z[0], 14.0)
    crad = box(BODY_X[0] - 4.0, BODY_X[1] + 2.0, BODY_Y[0] - 4.0, BODY_Y[1] + 4.0, BODY_Z[0], BODY_Z[1] + 4.0)
    beam = box(RAIL_X[0] - 2.0, RAIL_X[1] + 2.0, -C.rail_len / 2, C.rail_len / 2, Z_RAIL0 - C.beam_h, Z_RAIL0)
    body = blk + crad + beam                      # beam (z 11..18) sits on the cradle roof (z <= 14.45)
    # servo pocket = the servo's own clearance envelope (body, rear horn, connector) + bottom opening
    body -= servo_loc() * lib.servo_envelope(CL)
    body -= box(BODY_X[0], BODY_X[1] + 3.0, BODY_Y[0], BODY_Y[1], BODY_Z[0] - 1, BODY_Z[0] + 8.0)
    # horn + pinion clearance through the cradle front
    body -= cyl(2 * (RP + M) + 3.0, 20, X_HORN - 8, 0, 0, axis="X")
    # servo connector / cable exit at the rear-bottom
    body -= box(BODY_X[0] - 5, BODY_X[0] + 12, -6, 6, BODY_Z[0] - 5, BODY_Z[0] + 3)
    # rail screws (M3 inserts every 20 mm)
    n = int(C.rail_len // 20)
    for i in range(n):
        y = -(n - 1) * 10 + 20 * i
        body -= cyl(lib.ins_hole("M3"), lib.ins_depth("M3"), RAIL_XC, y, Z_RAIL0 - lib.ins_depth("M3") + 0.01)
    # female dovetail (slides DOWN onto the tool adapter) + M4 lock screw with nut trap
    tri = [(-rw / 2 - 0.15, px0 - 0.01), (rw / 2 + 0.15, px0 - 0.01), (tw / 2 + 0.15, ADAPTER_T + dh + 0.3),
           (-tw / 2 - 0.15, ADAPTER_T + dh + 0.3)]
    pts = [(x_, y_) for (y_, x_) in tri]
    body -= Pos(0, 0, -40.0) * prism(pts, 80.0)
    body -= cyl(clr("M4"), 40, px0 + dh / 2 + 1.5, -30, 0.0, axis="Y")
    body -= Pos(px0 + dh / 2 + 1.5, 10.0, 0.0) * Rot(-90, 0, 0) * prism(lib.hexagon2d(7.4), 3.4)
    for x, y in cap_xy():
        body -= cyl(1.8, 8.0, x, y, BODY_Z[0] - 0.01)          # M2 thread-forming pilot, centred in the 4 mm wall
    return body


def cap_xy():
    """Cradle-cap screws: centred in the 4 mm side walls, clear of the dovetail slot and the cable exit."""
    return [(BODY_X[0] + 3.0, BODY_Y[0] - 2.0), (BODY_X[0] + 3.0, BODY_Y[1] + 2.0),
            (BODY_X[1] - 3.0, BODY_Y[0] - 2.0), (BODY_X[1] - 3.0, BODY_Y[1] + 2.0)]


def cradle_cap() -> Part:
    z1 = BODY_Z[0]
    cap = box(BODY_X[0] - 4.0, BODY_X[1] + 2.0, BODY_Y[0] - 4.0, BODY_Y[1] + 4.0, z1 - 4.0, z1)
    cap -= box(BODY_X[0] + 3.0, BODY_X[1] - 3.0, BODY_Y[0] + 3.0, BODY_Y[1] - 3.0, z1 - 5, z1 + 1)
    for x, y in cap_xy():
        cap -= cyl(clr("M2"), 6, x, y, z1 - 5)
    return cap


def tool_adapter() -> Part:
    """Quick-change adapter bolted to the tool flange (WRA-TF30 = 4x M3 on BC30 + 4 mm dowel):
    male dovetail along Z.  Same on both wrists (ABENICS and pitch-roll)."""
    tf = P.wrist.tool_flange
    rw, tw, dh, dl = DOVE
    a = cyl(tf.dia, ADAPTER_T, 0, 0, 0, axis="X")
    tri = [(ADAPTER_T - 0.01, -rw / 2), (ADAPTER_T + dh, -tw / 2), (ADAPTER_T + dh, tw / 2), (ADAPTER_T - 0.01, rw / 2)]
    a += Pos(0, 0, -dl / 2) * prism(tri, dl)
    for i in range(tf.n):
        y, z, _ = lib.polar(tf.bolt_circle / 2, 90 * i)
        a -= cyl(clr("M3"), 20, -1, y, z, axis="X")
        a -= cyl(6.0, 20, 3.0, y, z, axis="X")
    y, z, _ = lib.polar(tf.bolt_circle / 2, 45)
    a -= cyl(tf.dowel_dia + 0.1, 4, -0.01, y, z, axis="X")
    a -= cyl(8.0, 30, -1, 0, 0, axis="X")                     # J6 cable from the flange centre bore
    a -= cyl(3.0, 2.0, ADAPTER_T + dh / 2 + 1.5, -tw / 2 - 1.0, 0, axis="Y")   # lock-screw detent
    return a


# ------------------------------------------------------------------------- vitamins (visual)
def rail() -> Part:
    return box(RAIL_X[0], RAIL_X[1], -C.rail_len / 2, C.rail_len / 2, Z_RAIL0, Z_RAIL0 + C.rail_h)


def blocks(q: float):
    """MGN9C carriages (visual): U-section around the rail."""
    out = []
    for s in (1, -1):
        yc = s * (q + YC_OFF)
        b = box(BLOCK_X[0], BLOCK_X[1], yc - C.block_len / 2, yc + C.block_len / 2, Z_RAIL0 + 2.0, Z_BLOCK1)
        b -= box(RAIL_X[0] - 0.1, RAIL_X[1] + 0.1, yc - C.block_len, yc + C.block_len, Z_RAIL0 - 1, Z_RAIL0 + C.rail_h + 0.1)
        out.append(b)
    return out


def parts():
    out = []
    pm = palm()
    out.append(PartDef("50_claw_palm", pm, "Claw palm: dovetail, servo cradle, rail beam", material="PA-CF (PETG ok)",
                       color="#5b6b7b", print_loc=on_bed(pm, (180, 0, 0)), walls=5, infill="50% gyroid",
                       print_notes="Rail-beam top face DOWN on the bed (flat, precise rail seat); support "
                                   "only under the dovetail block overhang."))
    cc = cradle_cap()
    out.append(PartDef("51_claw_cradle_cap", cc, "Claw servo cradle cap", material="PETG", color="#5b6b7b",
                       print_loc=on_bed(cc), load_bearing=False))
    pn = pinion()
    out.append(PartDef("52_claw_pinion", pn, "Claw pinion 14T m1.5 (on servo horn)", material="PA-CF / PETG-CF",
                       color="#e0a030", print_loc=on_bed(pn, (0, 90, 0)), walls=6, infill="100%",
                       print_notes="0.12 mm layers, hardened nozzle for CF."))
    ja = jaw(0.0, 1)
    out.append(PartDef("53_claw_jaw_a", ja, "Claw jaw A (rack on top of pinion, upper knob lip)",
                       material="PA-CF / PETG-CF", color="#e0a030", print_loc=on_bed(ja, (90, 0, 0)), walls=6,
                       infill="60% gyroid", print_notes="Outer finger face down; rack teeth print as vertical walls."))
    jb = jaw(0.0, -1)
    out.append(PartDef("53_claw_jaw_b", jb, "Claw jaw B (rack below pinion, lower knob lip)",
                       material="PA-CF / PETG-CF", color="#e0a030", print_loc=on_bed(jb, (-90, 0, 0)), walls=6,
                       infill="60% gyroid", print_notes="Outer finger face down."))
    ta = tool_adapter()
    out.append(PartDef("56_tool_adapter", ta, "Quick-change tool adapter (dovetail, both wrists)",
                       material="PA-CF (PETG ok)", color="#c0c0c0", print_loc=on_bed(ta, (0, -90, 0)), walls=6,
                       infill="100%"))
    pa = pad(0.0, 1)
    out.append(PartDef("54_claw_pad_a", pa, "Claw pad A, 100 deg V (glue 1.5 mm silicone facing)",
                       material="TPU 95A + silicone sheet", color="#303030", print_loc=on_bed(pa, (0, 90, 0)),
                       walls=3, infill="40%", load_bearing=False))
    pb = pad(0.0, -1)
    out.append(PartDef("54_claw_pad_b", pb, "Claw pad B (mirror of A)", material="TPU 95A + silicone sheet",
                       color="#303030", print_loc=on_bed(pb, (0, 90, 0)), walls=3, infill="40%", load_bearing=False))
    return out
