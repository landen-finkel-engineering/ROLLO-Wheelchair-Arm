"""Dimensioned engineering drawings for every printed part (third-angle projection, A3).

Pure matplotlib output (PDF + SVG) so it regenerates on any machine.  Each sheet has:
  * front / top / right views computed by hidden-line removal on the B-rep (build123d/OCCT),
  * overall dimensions on every view (true size, mm) and the model-frame origin,
  * an automatically extracted HOLE TABLE (every cylindrical bore grouped by diameter and
    classified: heat-set insert pilot, screw clearance, bearing seat, press-fit pin, ...),
  * title block (part, material, qty, estimated mass, scale, params hash).
Model-frame orientation is the ASSEMBLY frame of the part (see the part's docstring), not the
print orientation; the print orientation is in docs/13_print_guide.md and the STL files.
"""
from __future__ import annotations

import datetime
import hashlib
import math
import pathlib
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402

from OCP.BRepAdaptor import BRepAdaptor_Surface  # noqa: E402
from OCP.GeomAbs import GeomAbs_Cylinder  # noqa: E402

from .config import P  # noqa: E402
from . import lib  # noqa: E402

SHEET = (420.0, 297.0)           # A3 landscape, mm
FRAME = 10.0
TB = (180.0, 46.0)               # title block size
SCALES = [(2, 1), (1, 1), (1, 1.5), (1, 2), (1, 2.5), (1, 4), (1, 5)]
VIEWS = {   # name: (viewport origin direction, up)
    "FRONT (from -Y)": ((0, -1, 0), (0, 0, 1)),
    "TOP (from +Z)": ((0, 0, 1), (0, 1, 0)),
    "RIGHT (from +X)": ((1, 0, 0), (0, 0, 1)),
}


# --------------------------------------------------------------------------- features
def _known_diameters():
    """diameter -> list of possible functions (a bore table entry can be ambiguous by size alone)."""
    k = defaultdict(list)

    def put(d, txt):
        k[round(d, 2)].append(txt)
    for s in ("M3", "M4", "M5", "M6"):
        put(lib.ins_hole(s), f"{s} insert pilot (depth {lib.ins_depth(s):.1f})")
        put(lib.clr(s), f"{s} clearance")
    for b in ("6806", "608", "6802", "688", "626", "6800", "623", "625"):
        try:
            put(lib.seat(b), f"bearing seat {b}")
            put(lib.bearing(b)[0] + 2.0, f"{b} inner-ring clearance")
        except Exception:
            pass
    pm = P.fit.pin_press_minus
    for d in (3.0, 4.0, 6.0, 8.0, 10.0, 15.0):
        put(d - pm, f"press fit {d:.0f} mm pin")
    put(12.0 + P.fit.tube_socket_extra, "12 mm tube socket")
    put(10.0 + P.fit.tube_socket_extra, "10 mm tube socket")
    return k


def hole_table(shape):
    """Cylindrical concave faces grouped by diameter -> [(dia, count, note)]."""
    known = _known_diameters()
    groups = defaultdict(set)
    for f in shape.faces():
        try:
            ad = BRepAdaptor_Surface(f.wrapped)
            if ad.GetType() != GeomAbs_Cylinder:
                continue
            cy = ad.Cylinder()
            r = cy.Radius()
            ax = cy.Axis()
            loc, dr = ax.Location(), ax.Direction()
            c = f.center()
            n = f.normal_at(c)
            # vector from axis to centre point
            v = (c.X - loc.X(), c.Y - loc.Y(), c.Z - loc.Z())
            dd = (dr.X(), dr.Y(), dr.Z())
            t = sum(a * b for a, b in zip(v, dd))
            radial = tuple(a - t * b for a, b in zip(v, dd))
            if sum(a * b for a, b in zip(radial, (n.X, n.Y, n.Z))) >= 0:
                continue                                   # convex (a boss/shaft), not a hole
            d = round(2 * r, 2)
            if d < 1.5 or d > 120:
                continue
            # identify the bore by its axis line (projected point closest to origin) to merge split faces
            p0 = (loc.X(), loc.Y(), loc.Z())
            s = sum(a * b for a, b in zip(p0, dd))
            foot = tuple(round(a - s * b, 1) for a, b in zip(p0, dd))
            dkey = tuple(round(abs(x), 3) for x in dd)
            groups[d].add((foot, dkey))
        except Exception:
            continue
    rows = []
    for d in sorted(groups):
        note = ""
        for kd, txts in known.items():
            if abs(kd - d) < 0.03:
                note = " / ".join(dict.fromkeys(txts))
                break
        rows.append((d, len(groups[d]), note))
    return rows


# --------------------------------------------------------------------------- projection
def _edges_2d(shape, vdir, up):
    bb = shape.bounding_box()
    c = bb.center()
    dist = bb.diagonal * 3 + 100
    origin = (c.X + vdir[0] * dist, c.Y + vdir[1] * dist, c.Z + vdir[2] * dist)
    vis, hid = shape.project_to_viewport(origin, up, (c.X, c.Y, c.Z))

    def pts(edges):
        out = []
        for e in edges:
            try:
                ln = e.length
                n = 2 if e.geom_type.name == "LINE" else max(6, min(60, int(ln / 1.5)))
                out.append([(p.X, p.Y) for p in (e.position_at(i / (n - 1)) for i in range(n))])
            except Exception:
                continue
        return out
    return pts(vis), pts(hid)


def _extent(polys):
    xs = [x for pl in polys for x, _ in pl]
    ys = [y for pl in polys for _, y in pl]
    if not xs:
        return (0, 0, 1, 1)
    return min(xs), min(ys), max(xs), max(ys)


def _dim(ax, p0, p1, offset, text, horiz=True, fs=6.5):
    """Draw a linear dimension between paper points p0, p1 (mm on sheet)."""
    (x0, y0), (x1, y1) = p0, p1
    kw = dict(color="#1a3b8a", lw=0.3)
    if horiz:
        y = y0 + offset
        ax.plot([x0, x0], [y0, y + (1.5 if offset > 0 else -1.5)], **kw)
        ax.plot([x1, x1], [y1, y + (1.5 if offset > 0 else -1.5)], **kw)
        ax.annotate("", (x0, y), (x1, y), arrowprops=dict(arrowstyle="<->", color="#1a3b8a", lw=0.35,
                                                          shrinkA=0, shrinkB=0, mutation_scale=4))
        ax.text((x0 + x1) / 2, y + (0.8 if offset > 0 else -0.8), text, ha="center",
                va="bottom" if offset > 0 else "top", fontsize=fs, color="#1a3b8a")
    else:
        x = x0 + offset
        ax.plot([x0, x + (1.5 if offset > 0 else -1.5)], [y0, y0], **kw)
        ax.plot([x1, x + (1.5 if offset > 0 else -1.5)], [y1, y1], **kw)
        ax.annotate("", (x, y0), (x, y1), arrowprops=dict(arrowstyle="<->", color="#1a3b8a", lw=0.35,
                                                          shrinkA=0, shrinkB=0, mutation_scale=4))
        ax.text(x + (0.8 if offset > 0 else -0.8), (y0 + y1) / 2, text, ha="left" if offset > 0 else "right",
                va="center", rotation=90, fontsize=fs, color="#1a3b8a")


def _choose_scale(w_tot, h_tot, aw, ah):
    for num, den in SCALES:
        s = num / den
        if w_tot * s <= aw and h_tot * s <= ah:
            return num, den, s
    num, den = SCALES[-1]
    return num, den, num / den


def draw_part(pd, row=None, sheet_no=1, sheets=1, params_hash=""):
    shp = pd.shape
    views = {k: _edges_2d(shp, *v) for k, v in VIEWS.items()}
    ext = {k: _extent(v[0] + v[1]) for k, v in views.items()}
    w = {k: e[2] - e[0] for k, e in ext.items()}
    h = {k: e[3] - e[1] for k, e in ext.items()}
    gap = 26.0
    fr, tp, rt = list(VIEWS)
    w_tot = w[fr] + w[rt] + 3 * gap
    h_tot = h[fr] + h[tp] + 3 * gap
    aw = SHEET[0] - 2 * FRAME - 20
    ah = SHEET[1] - 2 * FRAME - TB[1] - 6
    x0 = FRAME + 16 + gap
    y0 = FRAME + TB[1] + 8 + gap * 0.6
    # the hole table sits on the right, above the title block: keep the views clear of it, either by
    # narrowing the view area or by lifting the views above the table - whichever keeps the larger scale
    rows = hole_table(shp)
    tx, ty = SHEET[0] - FRAME - TB[0], FRAME + TB[1] + 6
    n_lines = (2 + min(len(rows), 22) + (1 if len(rows) > 22 else 0)) if rows else 0
    tab_top = ty + n_lines * 2.37 + 2.5 if rows else ty
    if rows:
        cand_w = _choose_scale(w_tot, h_tot, tx - 4 - x0, ah)
        lift = max(0.0, tab_top + 4 - (FRAME + TB[1] + 8))
        cand_h = _choose_scale(w_tot, h_tot, aw, ah - lift)
        if cand_h[2] > cand_w[2]:
            num, den, s = cand_h
            y0 += lift
        else:
            num, den, s = cand_w
    else:
        num, den, s = _choose_scale(w_tot, h_tot, aw, ah)
    fig = plt.figure(figsize=(SHEET[0] / 25.4, SHEET[1] / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, SHEET[0])
    ax.set_ylim(0, SHEET[1])
    ax.set_aspect("equal")
    ax.axis("off")
    ax.add_patch(plt.Rectangle((FRAME, FRAME), SHEET[0] - 2 * FRAME, SHEET[1] - 2 * FRAME, fill=False, lw=0.8))
    # view placement (third angle: top above front, right to the right of front)
    place = {fr: (x0, y0), tp: (x0, y0 + h[fr] * s + gap), rt: (x0 + w[fr] * s + gap, y0)}
    for name, (vis, hid) in views.items():
        ex = ext[name]
        ox, oy = place[name]

        def tr(p):
            return (ox + (p[0] - ex[0]) * s, oy + (p[1] - ex[1]) * s)
        for pl in hid:
            q = [tr(p) for p in pl]
            ax.plot([a for a, _ in q], [b for _, b in q], color="#9a9a9a", lw=0.15, ls=(0, (2, 1.5)))
        for pl in vis:
            q = [tr(p) for p in pl]
            ax.plot([a for a, _ in q], [b for _, b in q], color="black", lw=0.3)
        pw, ph = w[name] * s, h[name] * s
        _dim(ax, (ox, oy), (ox + pw, oy), -7, f"{w[name]:.1f}", True)
        _dim(ax, (ox, oy), (ox, oy + ph), -7, f"{h[name]:.1f}", False)
        ax.text(ox, oy + ph + 4, name, fontsize=7, fontweight="bold")
    # model origin marker on the front view (X right, Z up)
    bb = shp.bounding_box()
    ex = ext[fr]
    ox, oy = place[fr]
    # front view: screen x = model X, screen y = model Z  (camera from -Y, up +Z)
    mx, my = ox + (0 - (bb.min.X - 0) - 0) * 0, 0
    _ = (mx, my)
    # hole table (rows, tx, ty computed above with the layout)
    if rows:
        lines = ["HOLE / BORE TABLE (auto-extracted, model nominal)", "dia mm   qty   function"]
        for d, n, note in rows[:22]:
            lines.append(f"{d:6.2f}  x{n:<3d}  {note}")
        if len(rows) > 22:
            lines.append(f"... {len(rows) - 22} more sizes (see STEP)")
        ax.text(tx, ty, "\n".join(lines), fontsize=5.6, family="monospace", va="bottom",
                bbox=dict(boxstyle="square,pad=0.6", fc="white", ec="#888888", lw=0.4))
    # title block
    bx, by = SHEET[0] - FRAME - TB[0], FRAME
    ax.add_patch(plt.Rectangle((bx, by), TB[0], TB[1], fill=False, lw=0.6))
    mass = f"{row['mass_g']:.0f} g est." if row else "-"
    info = [
        (f"WRA-1  {pd.name}", 9, "bold"),
        (pd.title[:70], 6.5, "normal"),
        (f"Material: {pd.material}   Qty: {pd.qty}   Mass: {mass}", 6.2, "normal"),
        (f"Print: {pd.walls} walls, {pd.infill}   L/R-specific: {'YES' if pd.lr_specific else 'no (same part both sides)'}", 6.2, "normal"),
        (f"Scale {num}:{den:g}   Units mm   Third-angle projection   Model (assembly) frame", 6.2, "normal"),
        (f"Fits: see print guide (bearing seats/pins/inserts from params.fit). Sheet {sheet_no}/{sheets}", 6.0, "normal"),
        (f"Generated {datetime.date.today().isoformat()} from params.yaml sha256:{params_hash}  - DO NOT EDIT, re-run build.py", 5.4, "normal"),
    ]
    yy = by + TB[1] - 6
    for txt, fs, fw in info:
        ax.text(bx + 3, yy, txt, fontsize=fs, fontweight=fw, va="top")
        yy -= 6.0
    ax.text(FRAME + 3, SHEET[1] - FRAME - 4, "WHEELCHAIR ROBOTIC ARM WRA-1  |  research prototype - not validated for "
            "human-facing use", fontsize=6, va="top", color="#a01010")
    return fig


def make_all(parts, out_dir: pathlib.Path, rows: dict | None = None):
    out_dir = pathlib.Path(out_dir)
    ph = hashlib.sha256((pathlib.Path(__file__).parent / "params.yaml").read_bytes()).hexdigest()[:16]
    rows = rows or {}
    pdf_path = out_dir / "WRA1_drawings.pdf"
    with PdfPages(pdf_path) as pdf:
        # cover / index
        fig = plt.figure(figsize=(SHEET[0] / 25.4, SHEET[1] / 25.4))
        ax = fig.add_axes([0, 0, 1, 1])
        ax.set_xlim(0, SHEET[0])
        ax.set_ylim(0, SHEET[1])
        ax.axis("off")
        ax.text(20, 280, "WRA-1 wheelchair robotic arm - part drawings", fontsize=16, fontweight="bold", va="top")
        ax.text(20, 268, "Generated from the parametric source (cad/wra). Units mm. Third-angle projection.\n"
                "Dimensions shown are overall sizes and auto-extracted bore tables; the STEP files are the master geometry.",
                fontsize=8, va="top")
        lines = [f"{i + 1:3d}  {p.name:28s} x{p.qty:<2d} {p.material[:28]:28s} {p.title[:60]}" for i, p in enumerate(parts)]
        half = (len(lines) + 1) // 2
        ax.text(20, 250, "\n".join(lines[:half]), fontsize=5.8, family="monospace", va="top")
        ax.text(215, 250, "\n".join(lines[half:]), fontsize=5.8, family="monospace", va="top")
        pdf.savefig(fig)
        plt.close(fig)
        for i, pd in enumerate(parts):
            fig = draw_part(pd, rows.get(pd.name), i + 1, len(parts), ph)
            fig.savefig(out_dir / f"{pd.name}.svg")
            pdf.savefig(fig)
            plt.close(fig)
            print("drawing", pd.name, flush=True)
    return pdf_path
