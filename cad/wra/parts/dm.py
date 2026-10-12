"""Standard Drive Module (DM) — used for J1, J2, J3 and J4.

DM frame:  origin on the pulley axis, in the plane of the servo-horn face.
           +Z_dm points from the servo toward the pin wall.
           +X_dm points toward the driven (large) pulley — the belt spans leave
           toward +X; the servo body extends toward -X.

Load path (the "Hub-Drive principle" used throughout the arm):
  belt tension -> 36T pulley -> 2x 608 bearings -> fixed 8 mm dowel -> pin wall -> host.
  The servo horn is bolted to the pulley but the servo BODY floats in its pocket until
  the cap is tightened, so the servo output carries torque only.
"""
from __future__ import annotations
import math
from build123d import Pos, Rot, Part

from ..config import P
from .. import lib
from ..lib import box, cyl, clr, PartDef, on_bed

S = P.servo
D = P.dm
B608 = lib.bearing(D.bearing)

# ---- key Z levels in the DM frame
Z_HORN = 0.0                                   # horn face
Z_PULLEY_TOP = 20.0
Z_PLATE_TOP = 6.0                              # top of pulley's bolted plate (= bore floor)
Z_WALL0 = Z_PULLEY_TOP + D.gap_horn            # pin wall bottom face
Z_WALL1 = Z_WALL0 + D.pin_wall                 # pin wall top face (mounting face)
SERVO_DZ = -S.horn.z1                          # servo-frame z -> DM z offset
SERVO_DX = -S.out_x                            # servo-frame x -> DM x offset
C = P.fit.servo_pocket_clear
BODY_X0 = S.body.x[0] + SERVO_DX - C           # -35.45
BODY_X1 = S.body.x[1] + SERVO_DX + C           # +10.45
BODY_Y = S.body.y[1] + C                       # 12.65
BODY_ZBOT = S.cover_bot_z + SERVO_DZ - C       # -34.85
LIP_Z = 15.9 + SERVO_DZ + 0.25                 # lip underside rests on the 15.9 cover rims
WALL = D.wall
POST_Y0, POST_Y1 = 24.0, 30.0
WALL_X0, WALL_X1 = -28.0, 6.0
CRADLE_TOP = -0.6
PULLEY_FLANGE_R = lib.od(D.teeth) / 2 + P.belt.flange_h


def servo_loc():
    """Location of the servo (servo frame) inside the DM frame."""
    return Pos(SERVO_DX, 0, SERVO_DZ)


def pulley() -> Part:
    """36T HTD-3M pulley bolted to the servo horn, spinning on 2x 608 on the fixed pin."""
    body = Pos(0, 0, 0.5) * lib.htd_pulley(D.teeth)
    body += cyl(lib.od(D.teeth) - 2.5, Z_PULLEY_TOP)          # core
    body -= cyl(lib.seat(D.bearing), Z_PULLEY_TOP - Z_PLATE_TOP + 1, z=Z_PLATE_TOP)
    body -= cyl(17.0, 0.8, z=Z_PLATE_TOP - 0.8)               # inner-ring relief at bore floor
    body -= cyl(9.0, Z_PULLEY_TOP + 1, z=-0.5)                # centre hole: pin clearance + horn screw access
    for x, y in lib.horn_bolt_xy():
        body -= cyl(clr("M3"), 10, x, y, -1)
        body -= cyl(6.2, 10, x, y, 2.6)                       # head counterbore from bore floor
    return body


def bracket(wall_t: float | None = None, mount: bool = True, tension_lug: bool = True) -> Part:
    """Cradle + posts + pin wall.  Mount face = top of pin wall (z = Z_WALL0 + wall_t).
    wall_t / mount / tension_lug are overridden by the Dual Drive Module (shared wall)."""
    W = WALL
    wall_t = D.pin_wall if wall_t is None else wall_t
    Z_WALL1 = Z_WALL0 + wall_t
    # ---- servo cradle (4 walls around the body, open top/bottom)
    cradle = box(BODY_X0 - W, BODY_X1 + W, -BODY_Y - W, BODY_Y + W, BODY_ZBOT, CRADLE_TOP)
    for x, y in cap_screw_xy():
        cradle += cyl(9.0, CRADLE_TOP - BODY_ZBOT, x, y, BODY_ZBOT)
    # top flange that ties the cradle to the two posts
    cradle += box(WALL_X0, WALL_X1, -POST_Y1, POST_Y1, CRADLE_TOP - 5, CRADLE_TOP)
    cradle -= box(BODY_X0, BODY_X1, -BODY_Y, BODY_Y, BODY_ZBOT - 1, CRADLE_TOP + 1)   # pocket
    # retaining lips along both long sides (rest on the 15.9 mm cover rims)
    lip_x1 = 6.8 + SERVO_DX - 1.0
    cradle += box(BODY_X0 - 0.01, lip_x1, 10.7, BODY_Y + 0.01, LIP_Z, CRADLE_TOP)
    cradle += box(BODY_X0 - 0.01, lip_x1, -BODY_Y - 0.01, -10.7, LIP_Z, CRADLE_TOP)
    # ---- posts and pin wall
    cradle += box(WALL_X0, WALL_X1 - 3, POST_Y0, POST_Y1, CRADLE_TOP - 5, Z_WALL0 + 0.01)
    cradle += box(WALL_X0, WALL_X1 - 3, -POST_Y1, -POST_Y0, CRADLE_TOP - 5, Z_WALL0 + 0.01)
    cradle += box(WALL_X0, WALL_X1, -POST_Y1, POST_Y1, Z_WALL0, Z_WALL1)
    cradle += cyl(11.0, Z_WALL0 - (Z_PULLEY_TOP + 0.05) + 0.01, z=Z_PULLEY_TOP + 0.05)  # inner-ring boss
    # ---- clearances for horn and pulley
    cradle -= cyl(22.0, 3.0, z=-3.0)
    cradle -= cyl(2 * PULLEY_FLANGE_R + 2.0, Z_PULLEY_TOP + 0.5, z=-0.55)
    # ---- dowel press bore (8 x 24 dowel, inserted from the mounting face)
    cradle -= cyl(D.pin_dia - P.fit.pin_press_minus, 40, z=Z_PULLEY_TOP - 1)
    # ---- mounting: 4x M4 heat-set inserts in the post tops (host part carries the slots)
    if mount:
        for x, y in mount_xy():
            cradle -= cyl(lib.ins_hole("M4"), lib.ins_depth("M4"), x, y, Z_WALL1 - lib.ins_depth("M4") + 0.01)
    # tension lug: M4 insert in the -X edge of the pin wall (pull-type tensioner)
    if tension_lug:
        cradle -= cyl(lib.ins_hole("M4"), lib.ins_depth("M4"), WALL_X0 - 0.01, 0, (Z_WALL0 + Z_WALL1) / 2, axis="X")
    # cap screw inserts (M3) in the corner bosses
    for x, y in cap_screw_xy():
        cradle -= cyl(lib.ins_hole("M3"), lib.ins_depth("M3"), x, y, BODY_ZBOT - 0.01)
    return cradle


def mount_xy():
    return [(-20.0, 27.0), (-20.0, -27.0), (0.0, 27.0), (0.0, -27.0)]


def cap_screw_xy():
    return [(BODY_X0 - 2.0, -BODY_Y - 2.0), (BODY_X0 - 2.0, BODY_Y + 2.0),
            (BODY_X1 + 2.0, -BODY_Y - 2.0), (BODY_X1 + 2.0, BODY_Y + 2.0)]


def cap() -> Part:
    """Bottom frame that clamps the servo up against the lips (tighten LAST)."""
    t = D.cap_t
    z1 = BODY_ZBOT
    plate = box(BODY_X0 - WALL, BODY_X1 + WALL, -BODY_Y - WALL, BODY_Y + WALL, z1 - t, z1)
    for x, y in cap_screw_xy():
        plate += cyl(9.0, t, x, y, z1 - t)
    # big window: rear horn, rear block and connectors protrude below the bottom cover
    plate -= box(BODY_X0 + 2.0, BODY_X1 + WALL + 1.0, -BODY_Y + 2.0, BODY_Y - 2.0, z1 - t - 1, z1 + 1)
    # pads that touch the bottom cover rim (2 mm wide) are the remaining frame
    for x, y in cap_screw_xy():
        plate -= cyl(clr("M3"), t + 2, x, y, z1 - t - 1)
    return plate


def pin() -> Part:
    return cyl(D.pin_dia, D.pin_len, z=Z_WALL1 - D.pin_len)


def bearings():
    _, _, Bw = B608
    return [Pos(0, 0, Z_PLATE_TOP) * lib.bearing_model(D.bearing),
            Pos(0, 0, Z_PLATE_TOP + Bw) * lib.bearing_model(D.bearing)]


def module_parts(prefix: str = "dm"):
    """PartDefs (DM frame) for one drive module (each shape built once: PartDef counts inserts)."""
    out = []
    b = bracket()
    out.append(PartDef(f"{prefix}_bracket", b, "Drive-module bracket", material="PETG", color="#4a7bd0",
                       print_loc=on_bed(b, (180, 0, 0)),
                       print_notes="Pin wall down on the plate (mounting face flat). Supports: none needed "
                                   "except under the cable window bridge (auto, build-plate only).",
                       walls=5, infill="50% gyroid"))
    pu = pulley()
    out.append(PartDef(f"{prefix}_pulley36", pu, "36T HTD-3M drive pulley", material="PETG",
                       color="#e0a030", print_loc=on_bed(pu, (0, 0, 0)),
                       print_notes="Horn face down. 0.16 mm layers for tooth quality; 100% infill.",
                       walls=6, infill="100%"))
    c = cap()
    out.append(PartDef(f"{prefix}_cap", c, "Drive-module servo cap", material="PETG", color="#4a7bd0",
                       print_loc=on_bed(c, (0, 0, 0)), walls=4, infill="40% gyroid", load_bearing=False))
    return out
