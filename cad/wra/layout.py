"""Derived layout: every key position computed from params.yaml (single source of truth).

Frames
  B : arm base frame, z = 0 on the dock interface (top of Receiver), Z = J1 axis.
  T : turntable frame, origin on the J1 axis at the turntable plate TOP face.
  DM frames: see parts/dm.py (origin on pulley axis at horn face, Z toward pin wall,
             X toward the driven pulley).
"""
from __future__ import annotations
import math
from build123d import Location, Plane, Vector

from .config import P
from . import lib

A = P.arm
BS = P.base

# ---------------------------------------------------------------- base frame heights
PLATE_BOT_B = BS.turntable_plate_z                  # turntable plate underside (B)
T_ORIGIN_B = PLATE_BOT_B + BS.turntable_plate_t     # turntable frame origin height (B)
DM_TEETH_C = 10.0                                   # DM pulley tooth-centre z_dm
DM_MOUNT_Z = 28.6                                   # DM pin-wall mounting face z_dm
J1_TEETH_B = PLATE_BOT_B - (DM_MOUNT_Z - DM_TEETH_C)  # fixed J1 pulley tooth centre (B)
J1_PULLEY_Z0_B = J1_TEETH_B - lib.pulley_len() / 2    # bottom of fixed pulley (B)
TOWER_TOP_B = J1_PULLEY_Z0_B - 0.4

# ---------------------------------------------------------------- belts / centres
C1 = lib.belt_center_distance(P.dm.teeth, P.pulleys.j1, P.belt_lengths.j1)
C2 = lib.belt_center_distance(P.dm.teeth, P.pulleys.j2, P.belt_lengths.j2)
C3 = lib.belt_center_distance(P.dm.teeth, P.pulleys.j3, P.belt_lengths.j3)
C4 = lib.belt_center_distance(P.dm.teeth, P.pulleys.j4, P.belt_lengths.j4)
C_DDM = C2                                          # DDM sits at the J2 belt's exact centre

# ---------------------------------------------------------------- turntable frame T
O = Vector(0, 0, A.shoulder_h)                      # J2 axis point (axis along Y)
DDM_AXIS = Vector(O.X + C_DDM * math.cos(math.radians(P.ddm.angle_deg)), 0,
                  O.Z + C_DDM * math.sin(math.radians(P.ddm.angle_deg)))


def dm_location(origin, x_dir, z_dir) -> Location:
    """Location mapping DM-frame coordinates into the parent frame."""
    return Plane(origin=origin, x_dir=x_dir, z_dir=z_dir).location


# J1 drive module hangs under the turntable plate, axis vertical, walks around the
# fixed base pulley.  Horn face at z_T = -plate_t - DM_MOUNT_Z.
J1_DM_LOC_T = dm_location((-C1, 0, -BS.turntable_plate_t - DM_MOUNT_Z), (1, 0, 0), (0, 0, 1))

# Dual drive module: J2 half (servo outboard LEFT), J3 half (servo outboard RIGHT)
_to_O = (O - DDM_AXIS).normalized()
J2_HORN_Y = A.y_86t_center - DM_TEETH_C             # -38
J3_HORN_Y = A.y_72t_center + DM_TEETH_C             # +24
J2_DM_LOC_T = dm_location((DDM_AXIS.X, J2_HORN_Y, DDM_AXIS.Z), (_to_O.X, 0, _to_O.Z), (0, 1, 0))
DDM_AXIS3 = Vector(O.X + C3 * math.cos(math.radians(P.ddm.angle_deg)), 0,
                   O.Z + C3 * math.sin(math.radians(P.ddm.angle_deg)))
J3_DM_LOC_T = dm_location((DDM_AXIS3.X, J3_HORN_Y, DDM_AXIS3.Z), (_to_O.X, 0, _to_O.Z), (0, -1, 0))

# Spring cord points
A2 = Vector(0, A.ua_plane_y, O.Z - P.springs.a)     # J2 idler (below O)
A3 = Vector(0, A.j3_cord_y, O.Z - P.springs.a)      # J3 idler (below O)


def ratio(j):
    return {1: P.pulleys.j1, 2: P.pulleys.j2, 3: P.pulleys.j3, 4: P.pulleys.j4}[j] / P.dm.teeth


def summary() -> dict:
    return {
        "C1_j1": round(C1, 3), "C2_j2": round(C2, 3), "C3_j3": round(C3, 3), "C4_j4": round(C4, 3),
        "j3_half_offset_mm": round(C3 - C2, 3),
        "turntable_origin_z_B": T_ORIGIN_B, "J2_axis_z_B": T_ORIGIN_B + A.shoulder_h,
        "fixed_pulley_z0_B": round(J1_PULLEY_Z0_B, 3),
        "ratios": {f"j{j}": round(ratio(j), 4) for j in (1, 2, 3, 4)},
    }
