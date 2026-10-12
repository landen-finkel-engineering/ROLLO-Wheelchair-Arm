"""Arm base (stationary, docks to the Receiver) and the chair-side Receiver.

Base frame B: z = 0 on the dock interface, Z = J1 axis, +X = front mark.
"""
from __future__ import annotations
import math
from build123d import Pos, Rot, Part, Text, extrude, Plane

from ..config import P
from .. import lib, layout as L
from ..lib import box, cyl, clr, PartDef, on_bed, prism

BS = P.base
DK = P.dock
d6806, D6806, B6806 = lib.bearing(BS.bearing)
SEAT = lib.seat(BS.bearing)
Z_LB, Z_UB = BS.bearing_z                       # bearing bottom faces
TEETH_C = L.J1_TEETH_B                          # 76.4 = tooth centre of J1 DM pulley
TOWER_TOP = Z_UB + B6806                        # 62.0 (flush with upper bearing)
FLANGE_T = 7.0
TEETH_Z0 = TEETH_C - P.belt.tooth_len / 2       # 68.4
BOT_FL_Z0 = TEETH_Z0 - P.belt.flange            # 66.9 (same plane as DM pulley bottom)
DISC_Z0, DISC_Z1 = TOWER_TOP + 0.05, BOT_FL_Z0 - 0.5   # base disc clears DM pulley flange
PUL_Z0 = DISC_Z0
PULLEY_BOLT_R = 31.5
STOP_BLOCK_Z = (None, None)


def _arrow(z, size=14.0, depth=0.8, x=0.0, y=0.0):
    pts = [(x + size, y), (x + size * 0.2, y + size * 0.55), (x + size * 0.2, y + size * 0.2),
           (x - size * 0.6, y + size * 0.2), (x - size * 0.6, y - size * 0.2), (x + size * 0.2, y - size * 0.2),
           (x + size * 0.2, y - size * 0.55)]
    return prism(pts, depth + 0.01, z - depth)


def base_housing() -> Part:
    w, h = DK.plate
    t = BS.dock_plate_t
    plate = lib.rbox(w, h, t, 8.0)
    # tower
    tower = cyl(BS.tower_od, TOWER_TOP - t, z=t)
    tower += cyl(BS.tower_flange_od, FLANGE_T, z=TOWER_TOP - FLANGE_T)
    # 4 gusset ribs tower -> plate (at 45 deg, clear of dock screws)
    for a in (0, 90, 180, 270):
        rib = box(BS.tower_od / 2 - 2, w / 2 - 6, -2.5, 2.5, t, t + 20)
        # taper the rib
        cut = Pos(w / 2 - 6, 0, t + 20) * Rot(0, 45, 0) * box(-30, 0, -5, 5, -30, 0)
        rib -= Pos(0, 0, 0) * cut
        tower += Rot(0, 0, a) * rib
    body = plate + tower
    # bearing bores: upper seat open to top, lower seat open to cavity, smaller middle
    body -= cyl(SEAT, TOWER_TOP - Z_UB + 1, z=Z_UB)
    body -= cyl(D6806 - 6.0, Z_UB - (Z_LB + B6806) + 0.02, z=Z_LB + B6806 - 0.01)
    body -= cyl(SEAT, Z_LB + B6806 - BS.cable_cavity[1] + 0.01, z=BS.cable_cavity[1] - 0.01)
    # cable cavity and rear exit to panel connector
    cav0, cav1 = BS.cable_cavity
    body -= cyl(BS.cavity_dia, cav1 - cav0 + 0.02, z=cav0)
    # rear connector boss (GX16 panel connector) fed by a groove in the dock-plate top
    body += box(-BS.tower_od / 2 - 12, -BS.tower_od / 2 + 4, -14, 14, t - 0.01, cav1 + 2)
    body -= cyl(BS.connector_hole, 30, -BS.tower_od / 2 - 20, 0, 15.0, axis="X")
    body -= box(-BS.tower_od / 2 - 6, -BS.tower_od / 2 + 6, -9, 9, 5.0, cav1 - 1.0)      # boss cavity
    body -= box(-BS.tower_od / 2 - 2, 0.0, -5, 5, 3.0, t + 0.01)                        # cable groove
    # fixed-pulley bolt inserts on the top flange
    for i in range(4):
        x, y, _ = lib.polar(PULLEY_BOLT_R, 45 + 90 * i)
        body -= cyl(lib.ins_hole("M3"), lib.ins_depth("M3"), x, y, TOWER_TOP - lib.ins_depth("M3") + 0.01)
    # dock features in the plate
    (x1, y1), (x2, y2) = DK.dowel_xy
    body -= cyl(DK.dowel_dia + 0.05, t + 1, x1, y1, -0.5)                        # locating hole
    ang = math.degrees(math.atan2(y2 - y1, x2 - x1))
    body -= lib.slot(DK.dowel_dia + 3.0, DK.dowel_dia + 0.05, t + 1, x2, y2, -0.5, angle=ang)  # slot
    for x, y in DK.thumb_xy:
        body -= cyl(clr("M6"), t + 1, x, y, -0.5)
        body -= cyl(13.0, 1.2, x, y, -0.01)       # O-ring recess (captive thumb screw)
    # front mark
    body -= _arrow(t, size=12, x=w / 2 - 22, y=0)
    # lightening pockets under plate between ribs
    for a in (45, 135, 225, 315):
        x, y, _ = lib.polar(30, a)
        body -= Pos(x, y, -0.01) * Rot(0, 0, a) * box(-6, 6, -6, 6, 0, 4)
    return body


def fixed_pulley() -> Part:
    """54T HTD-3M pulley fixed to the base; the J1 drive module walks around it.
    The wide base disc sits BELOW the DM pulley's flange plane so the DM can orbit freely."""
    t = P.belt.tooth_len
    n = P.pulleys.j1
    fl_d = lib.od(n) + 2 * P.belt.flange_h
    body = cyl(BS.tower_flange_od, DISC_Z1 - DISC_Z0, z=DISC_Z0)                 # base disc
    body += cyl(lib.od(n) - 3.0, BOT_FL_Z0 - DISC_Z1 + 0.02, z=DISC_Z1 - 0.01)    # neck
    body += cyl(fl_d, P.belt.flange, z=BOT_FL_Z0)                                # bottom flange
    body += prism(lib.htd_profile(n), t, TEETH_Z0)                               # teeth
    body += cyl(fl_d, P.belt.flange, z=TEETH_Z0 + t)                             # top flange
    body -= cyl(BS.spindle_od + 4.0, 100, z=DISC_Z0 - 1)                         # spindle clearance
    body -= cyl(37.0, DISC_Z1 - DISC_Z0 + 0.01, z=DISC_Z0 - 0.01)                # clears inner ring
    for i in range(4):                                                           # M3 countersunk
        x, y, _ = lib.polar(PULLEY_BOLT_R, 45 + 90 * i)
        body -= cyl(clr("M3"), 10, x, y, DISC_Z0 - 1)
        body -= Pos(x, y, DISC_Z1 - 1.75) * lib.Cone_cs(6.4, 3.3, 1.76)
    # J1 hard-stop block on top face (arc behind the arm, fills the dead zone)
    z0 = TEETH_Z0 + t + P.belt.flange
    arc = []
    r0, r1 = BS.spindle_od / 2 + 2.5, lib.od(n) / 2 - 0.5
    hw = 180 - P.joints.j1.hard[1] - math.degrees(math.atan2(3.0, BS.stop_r))
    for i in range(41):
        a = math.radians(180 - hw + 2 * hw * i / 40)
        arc.append((r1 * math.cos(a), r1 * math.sin(a)))
    for i in range(41):
        a = math.radians(180 + hw - 2 * hw * i / 40)
        arc.append((r0 * math.cos(a), r0 * math.sin(a)))
    body += prism(arc, 5.0, z0 - 0.01)
    return body


def stop_block_top_z():
    return TEETH_Z0 + P.belt.tooth_len + P.belt.flange + 5.0


def receiver() -> Part:
    """Chair-side dock receiver — IDENTICAL part for left and right installations.
    Frame: same as B (top face at z = 0)."""
    w, h = DK.plate
    t = DK.receiver_t
    body = Pos(0, 0, -t) * lib.rbox(w, h, t, 8.0)
    (x1, y1), (x2, y2) = DK.dowel_xy
    for x, y in ((x1, y1), (x2, y2)):
        body -= cyl(DK.dowel_dia - P.fit.pin_press_minus, 12.5, x, y, -12.5)   # press-fit dowel
    for x, y in DK.thumb_xy:
        body -= cyl(lib.ins_hole("M6"), lib.ins_depth("M6"), x, y, -lib.ins_depth("M6") + 0.01)
    # 24-hole yaw index ring (M4, counterbored from top -> into mount-shelf inserts)
    n = DK.index_holes
    for i in range(n):
        x, y, _ = lib.polar(DK.index_bc / 2, 360 * i / n)
        body -= cyl(clr("M4"), t + 2, x, y, -t - 1)
        body -= cyl(8.0, 4.6, x, y, -4.6 + 0.01)
    # dock-present microswitch pocket (lever pressed by the arm's dock plate)
    sw = DK.switch_pocket
    body -= box(-sw[0] / 2, sw[0] / 2, 40 - sw[1] / 2, 40 + sw[1] / 2, -sw[2] - 0.5, 0.01)
    body -= box(-sw[0] / 2 - 4, -sw[0] / 2 + 0.01, 40 - 2.5, 40 + 2.5, -sw[2] - 0.5, -1.0)  # wire exit
    body -= Pos(-sw[0] / 2 - 4, 40, -t / 2) * box(-10, 0, -2.5, 2.5, -2.5, 2.5)
    # front mark & side labels (L/R are the SAME part; the arrow shows arm front)
    body -= _arrow(0, size=12, x=w / 2 - 22, y=-18)
    # underside lightening
    body -= Pos(0, 0, -t - 0.01) * cyl(40, 6)
    return body


def parts():
    # build -> PartDef one part at a time (PartDef records the inserts modelled since the last one)
    out = []
    hb = base_housing()
    out.append(PartDef("10_base_housing", hb, "Base housing + dock plate", material="PA-CF (required: dock plate bending)",
                       color="#5b6b7b", print_loc=on_bed(hb), walls=6, infill="40% gyroid",
                       print_notes="Dock face on plate. Bearing seats print vertical = round. "
                                   "Supports: only under the connector boss (build-plate only)."))
    fp = fixed_pulley()
    out.append(PartDef("11_j1_fixed_pulley", fp, "J1 fixed pulley 54T (+ hard-stop arc)", material="PETG",
                       color="#e0a030", print_loc=on_bed(fp), walls=6, infill="100%",
                       print_notes="Base disc down. 0.16 mm layers."))
    rc = receiver()
    out.append(PartDef("01_receiver", rc, "Dock receiver (chair side, same part L/R)", material="PETG (PA-CF optional)",
                       color="#2f8f5b", print_loc=on_bed(rc, (180, 0, 0)), walls=6, infill="50% gyroid",
                       print_notes="Top face DOWN on the plate (flat dock face). Dowels pressed in after printing."))
    return out
