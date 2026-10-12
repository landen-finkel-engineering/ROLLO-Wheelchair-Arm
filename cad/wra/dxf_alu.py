"""Optional aluminium version of the post bracket (04_post_bracket) as 2D DXF flat patterns.

Why: the post bracket carries the largest bending moment of any printed part (the whole arm
hangs off it).  If the bending check in calcs/ shows a low margin for your chair, or creep is
observed after the 24 h re-torque, cut these plates from 6 mm 6061-T6 (waterjet / laser /
hand: drill + hacksaw + file) and join them with two 40 x 40 x 4 aluminium angles.  Every
hole position comes from the same params as the printed bracket, so the printed clamps and
the receiver bolt on unchanged.

Layers: OUTLINE (cut), HOLES (drill), TAP (drill tap size, then tap), TEXT (notes, do not cut).
"""
from __future__ import annotations

import math
import pathlib

import ezdxf

from .config import P
from .parts import mount as mt
from . import lib

T_AL = 6.0


def write(path):
    doc = ezdxf.new("R2010", setup=True)
    doc.units = ezdxf.units.MM
    for name, col in (("OUTLINE", 7), ("HOLES", 1), ("TAP", 3), ("TEXT", 5)):
        doc.layers.add(name, color=col)
    msp = doc.modelspace()
    zt, zb, zc1, zc2 = mt.bracket_dims()
    H = mt.H
    # ---------------- plate 1: SPINE (lies against the two clamp pads), drawn with z -> y
    w = mt.SPINE_W
    z_sb = zc2 - H / 2 - 4.0
    y0, y1 = z_sb, zt
    x0 = 0.0
    msp.add_lwpolyline([(x0 - w / 2, y0), (x0 + w / 2, y0), (x0 + w / 2, y1), (x0 - w / 2, y1)], close=True,
                       dxfattribs={"layer": "OUTLINE"})
    for zc in (zc1, zc2):
        for dz in (-H / 4, H / 4):
            # vertical slot 24 long, 5.5 wide (M5) -> draw as two arcs + lines
            _slot(msp, x0, zc + dz, 24.0 - 5.5, 5.5)
    # angle-bracket holes near the top (2 x M5 clearance)
    for x in (-18.0, 18.0):
        msp.add_circle((x, zt - 12.0), 2.75, dxfattribs={"layer": "HOLES"})
    # strut clevis: an M5 hole near the bottom for a standard clevis/rod end (bolt through plate)
    msp.add_circle((0.0, z_sb + 10.0), 2.75, dxfattribs={"layer": "HOLES"})
    msp.add_circle((0.0, zc1 + 30.0), 3.0, dxfattribs={"layer": "HOLES"})         # tether eye
    msp.add_text("PLATE 1 SPINE  6 mm 6061-T6  qty 1   (slots: clamp pads M5)", height=3.5,
                 dxfattribs={"layer": "TEXT"}).set_placement((x0 - w / 2, y0 - 10))
    # ---------------- plate 2: SHELF (receiver sits on it), placed to the right
    ox = 120.0
    s = P.dock.plate[0] + 10.0
    r = 10.0
    _rrect(msp, ox, 0.0, s, s, r)
    for i in range(4):
        x, y, _ = lib.polar(P.dock.index_bc / 2, 90 * i)
        msp.add_circle((ox + x, y), 3.3 / 2, dxfattribs={"layer": "TAP"})    # M4 tap drill 3.3
    # angle-bracket holes along the spine edge
    for y in (-18.0, 18.0):
        msp.add_circle((ox - s / 2 + 12.0, y), 2.75, dxfattribs={"layer": "HOLES"})
    msp.add_circle((ox, 0.0), 20.0, dxfattribs={"layer": "HOLES"})               # cable / lightening hole
    msp.add_text("PLATE 2 SHELF  6 mm 6061-T6  qty 1   (4x M4 tapped on BC %.0f for the receiver)" % P.dock.index_bc,
                 height=3.5, dxfattribs={"layer": "TEXT"}).set_placement((ox - s / 2, -s / 2 - 10))
    msp.add_text("Join plates 1+2 with two 40x40x4 Al angles, 4x M5x12 + nyloc. Shelf edge flush with spine "
                 "face; receiver centre %.1f mm from post axis." % mt.shelf_center_x(),
                 height=3.0, dxfattribs={"layer": "TEXT"}).set_placement((x0 - w / 2, y0 - 22))
    doc.saveas(str(path))
    return path


def _slot(msp, cx, cy, straight, width):
    r = width / 2
    a = straight / 2
    msp.add_line((cx - r, cy - a), (cx - r, cy + a), dxfattribs={"layer": "HOLES"})
    msp.add_line((cx + r, cy - a), (cx + r, cy + a), dxfattribs={"layer": "HOLES"})
    msp.add_arc((cx, cy + a), r, 0, 180, dxfattribs={"layer": "HOLES"})
    msp.add_arc((cx, cy - a), r, 180, 360, dxfattribs={"layer": "HOLES"})


def _rrect(msp, cx, cy, w, h, r):
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    msp.add_line((x0 + r, y0), (x1 - r, y0), dxfattribs={"layer": "OUTLINE"})
    msp.add_line((x1, y0 + r), (x1, y1 - r), dxfattribs={"layer": "OUTLINE"})
    msp.add_line((x1 - r, y1), (x0 + r, y1), dxfattribs={"layer": "OUTLINE"})
    msp.add_line((x0, y1 - r), (x0, y0 + r), dxfattribs={"layer": "OUTLINE"})
    for (x, y, a0) in ((x1 - r, y0 + r, 270), (x1 - r, y1 - r, 0), (x0 + r, y1 - r, 90), (x0 + r, y0 + r, 180)):
        msp.add_arc((x, y), r, a0, a0 + 90, dxfattribs={"layer": "OUTLINE"})
