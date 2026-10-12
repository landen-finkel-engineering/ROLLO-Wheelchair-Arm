#!/usr/bin/env python3
"""WRA-1 build pipeline: regenerates EVERY CAD output from wra/params.yaml.

    python build.py --all          everything below (20-40 min on a laptop)
    python build.py --parts        STL (print orientation) + STEP (model frame) for every part,
                                   mass properties, BOM / fastener schedule
    python build.py --assemblies   STEP assemblies: arm (park + work), exploded, wrist, RIGHT and LEFT installs
    python build.py --renders      PNG renders (optional: needs Node.js + playwright, see README)
    python build.py --drawings     dimensioned 3-view drawings (PDF + SVG) and the aluminium DXF
    python build.py --verify       verification report (validity, build volume, mesh, interfaces)
    python build.py --software     arm_geometry.json + abenics_kin.py + URDF/meshes for the ROS 2 packages

Units: millimetres, degrees.  Outputs go to ../output (relative to this file).
"""
from __future__ import annotations

import argparse
import csv
import datetime
import json
import math
import pathlib
import shutil
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = ROOT / "output"
sys.path.insert(0, str(HERE))

from build123d import Color, Compound, export_step, export_stl, import_step  # noqa: E402

from wra.config import P  # noqa: E402
from wra import lib, layout, assembly as asm  # noqa: E402
from wra.parts import base, turntable as tt, dm, arm, wrist as wr, claw as cw, abenics as ab, mount as mt, misc  # noqa: E402
from wra import abenics_kin as AK  # noqa: E402

BUILD_VOLUME = (256.0, 256.0, 256.0)     # Bambu Lab P1S [V]
BED_MARGIN = 5.0
DENSITY = {"PETG-CF": 1.25, "PA-CF": 1.20, "PETG": 1.27, "TPU": 1.22}   # g/cm^3 [V: typical TDS values]
STL_TOL, STL_ANG = 0.02, 0.1             # mm, rad  (fine enough for 0.4 mm nozzle)


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


# ============================================================================ registry
VARIANT = "WRA-1A" if P.wrist_type == "abenics" else "WRA-1B"


def registry():
    """Every printed part.  extra['variant']: WRA-1A = ABENICS wrist (default), WRA-1B = pitch-roll
    fallback wrist; parts without it are common.  All parts are exported; the BOM / print list
    count the ACTIVE variant (params.yaml wrist_type) and list the other variant separately."""
    lib.INSERT_LOG.clear()
    parts = []
    parts += base.parts()
    n_dm = 3 if VARIANT == "WRA-1A" else 4
    for pd in dm.module_parts("12_dm"):
        pd.qty = {"12_dm_bracket": 1, "12_dm_pulley36": n_dm, "12_dm_cap": n_dm}[pd.name]
        pd.notes = {"12_dm_bracket": "J1 only (J2/J3 use 22_ddm_bracket_*; WRA-1B J4 is integral in 40_wrist_housing)",
                    "12_dm_pulley36": "J1, J2, J3 (+ J4 on WRA-1B)", "12_dm_cap": "J1, J2, J3 (+ J4 on WRA-1B)"}[pd.name]
        parts.append(pd)
    parts += tt.parts() + arm.parts() + cw.parts() + mt.parts() + misc.parts()
    for pd in ab.parts():
        pd.extra["variant"] = "WRA-1A"
        parts.append(pd)
    for pd in wr.parts():
        pd.extra["variant"] = "WRA-1B"
        parts.append(pd)
    names = [p.name for p in parts]
    assert len(names) == len(set(names)), "duplicate part names"
    return sorted(parts, key=lambda p: p.name)


def active(pd) -> bool:
    return pd.extra.get("variant", VARIANT) == VARIANT


def material_key(material: str) -> str:
    m = material.upper()
    for k in ("PETG-CF", "PA-CF", "TPU", "PETG"):
        if m.startswith(k) or m.startswith("SAME") and k == "PETG":
            return k
    for k in ("PETG-CF", "PA-CF", "TPU", "PETG"):
        if k in m:
            return k
    return "PETG"


def infill_frac(s: str) -> float:
    try:
        return float(s.split("%")[0]) / 100.0
    except ValueError:
        return 0.4


def mass_estimate(pd) -> dict:
    """Printed mass estimate: perimeter shell (walls x 0.45 mm) solid + infill fraction inside."""
    v = pd.shape.volume                       # mm^3
    a = pd.shape.area                         # mm^2
    t_sh = max(pd.walls * 0.45, 1.0)
    shell = min(v, a * t_sh * 0.5)            # half the surface-normal shell avoids double counting thin walls
    fill = infill_frac(pd.infill)
    eff = (shell + max(0.0, v - shell) * fill) / v if v > 0 else 1.0
    mk = material_key(pd.material)
    rho = DENSITY[mk]
    return {"volume_cm3": round(v / 1000, 2), "area_cm2": round(a / 100, 1), "fill_effective": round(eff, 3),
            "material_key": mk, "mass_g": round(v / 1000 * rho * eff, 1),
            "mass_solid_g": round(v / 1000 * rho, 1)}


# ============================================================================ parts
def export_parts(parts, stl=True, step=True):
    (OUT / "stl").mkdir(parents=True, exist_ok=True)
    (OUT / "step" / "parts").mkdir(parents=True, exist_ok=True)
    mesh_stats = {}
    mesh_parts = {pd.name: pd for pd in parts if pd.extra.get("mesh_teeth")}
    if stl and mesh_parts:
        from wra.parts import abenics_mesh as am
        t0 = time.time()
        mesh_stats = am.export(OUT / "stl", {n: pd.print_loc for n, pd in mesh_parts.items()})
        log(f"toothed ABENICS meshes (manifold3d): {mesh_stats}  ({time.time() - t0:.0f}s)")
    rows = []
    for pd in parts:
        t0 = time.time()
        shp = pd.shape
        bed = pd.print_loc * shp if pd.print_loc is not None else shp
        bb = bed.bounding_box()
        mb = shp.bounding_box()
        if stl and pd.name not in mesh_parts:
            export_stl(bed, str(OUT / "stl" / f"{pd.name}.stl"), tolerance=STL_TOL, angular_tolerance=STL_ANG)
            lib.stl_clean(OUT / "stl" / f"{pd.name}.stl")
        if step:
            s2 = shp
            s2.label = pd.name
            s2.color = Color(pd.color)
            export_step(s2, str(OUT / "step" / "parts" / f"{pd.name}.step"))
        com = shp.center()
        me = mass_estimate(pd)
        if pd.name in mesh_stats:          # toothed volume (the STEP is the tooth-less blank)
            k = mesh_stats[pd.name]["volume_mm3"] / shp.volume
            me["mass_g"] = round(me["mass_g"] * k, 1)
            me["mass_solid_g"] = round(me["mass_solid_g"] * k, 1)
            me["volume_cm3"] = round(mesh_stats[pd.name]["volume_mm3"] / 1000, 2)
        row = {"part": pd.name, "title": pd.title, "qty": pd.qty, "material": pd.material,
               "variant": pd.extra.get("variant", "common"), "active": active(pd),
               "stl_source": "toothed mesh (manifold3d)" if pd.name in mesh_parts else "B-rep",
               "walls": pd.walls, "infill": pd.infill, "supports": pd.supports, "load_bearing": pd.load_bearing,
               "lr_specific": pd.lr_specific, "print_notes": pd.print_notes, "notes": pd.notes,
               "inserts": pd.inserts,
               "model_bbox_mm": [round(mb.size.X, 1), round(mb.size.Y, 1), round(mb.size.Z, 1)],
               "print_bbox_mm": [round(bb.size.X, 1), round(bb.size.Y, 1), round(bb.size.Z, 1)],
               "print_zmin_mm": round(bb.min.Z, 3), "com_model_mm": [round(com.X, 1), round(com.Y, 1), round(com.Z, 1)],
               **me}
        if pd.name in mesh_stats:
            row["mesh"] = mesh_stats[pd.name]
        rows.append(row)
        log(f"part {pd.name:28s} {row['mass_g']:7.1f} g  bed {row['print_bbox_mm']}  ({time.time() - t0:.1f}s)")
    (OUT / "reports").mkdir(parents=True, exist_ok=True)
    (OUT / "reports" / "parts.json").write_text(json.dumps(rows, indent=1))
    with open(OUT / "reports" / "printed_parts.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["part", "title", "variant", "qty", "material", "walls", "infill", "mass_each_g", "mass_total_g",
                    "print_bbox_x", "print_bbox_y", "print_bbox_z", "supports", "load_bearing", "inserts", "print_notes"])
        for r in rows:
            w.writerow([r["part"], r["title"], r["variant"], r["qty"], r["material"], r["walls"], r["infill"],
                        r["mass_g"], round(r["mass_g"] * r["qty"], 1), *r["print_bbox_mm"], r["supports"],
                        r["load_bearing"], " ".join(f"{k}x{v}" for k, v in r["inserts"].items()), r["print_notes"]])
    return rows


# ============================================================================ assemblies
def _compound(instances, label):
    kids = []
    for i in instances:
        if i.shape is None:
            continue
        s = i.shape
        s.label = i.name
        try:
            s.color = Color(i.color)
        except Exception:
            pass
        kids.append(s)
    c = Compound(children=kids)
    c.label = label
    return c


def _zip_step(path: pathlib.Path):
    import zipfile
    with zipfile.ZipFile(str(path) + ".zip", "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        z.write(path, arcname=path.name)


def export_assemblies():
    from wra import chair as ch
    d = OUT / "step" / "assemblies"
    d.mkdir(parents=True, exist_ok=True)
    jobs = {
        "WRA1_arm_park": lambda: asm.arm_instances(asm.HOME),
        "WRA1_arm_work_pose": lambda: asm.arm_instances(WORK_POSE),
        "WRA1_arm_exploded": lambda: asm.arm_instances(asm.HOME, explode=1.0),
        "WRA1_RIGHT_install_park": lambda: [i for i in ch.chair_instances("right", user=False)
                                            if not i.name.startswith(("belt", "cord", "spring"))],
        "WRA1_LEFT_install_park": lambda: [i for i in ch.chair_instances("left", user=False)
                                           if not i.name.startswith(("belt", "cord", "spring"))],
    }
    if asm.WRIST == "abenics":
        jobs["WRA1A_wrist_and_claw"] = lambda: asm.wrist_instances((0, 0, 0), 20.0)[0]
    for name, fn in jobs.items():
        t0 = time.time()
        inst = fn()
        export_step(_compound(inst, name), str(d / f"{name}.step"))
        _zip_step(d / f"{name}.step")       # assemblies are 10-45 MB of text: the .zip (~1/6) is what git tracks
        log(f"assembly {name}: {len(inst)} bodies ({time.time() - t0:.0f}s)")


WORK_POSE = ({"j2": 40, "j3": -10, "wt": (-20, 0, 0), "j6": 20.0} if asm.WRIST == "abenics"
             else {"j2": 40, "j3": -10, "j4": -20, "j6": 20.0})


# ============================================================================ renders
def renders():
    sys.path.insert(0, str(HERE / "tools" / "render"))
    import render as rnd
    if not rnd.available():
        log("renderer not available (needs `pip install playwright` + `npm install` in tools/render) - skipped")
        return
    from wra import chair as ch
    d = OUT / "renders"
    tmp = OUT / "_render_tmp"
    d.mkdir(parents=True, exist_ok=True)

    def scene(instances, tag, views, title, opacity=None, edges=False, zoom=1.0, size=(1600, 1150)):
        if tmp.exists():
            shutil.rmtree(tmp)
        tmp.mkdir(parents=True)
        files = []
        for k, i in enumerate(instances):
            if i.shape is None:
                continue
            f = tmp / f"{k:03d}.stl"
            export_stl(i.shape, str(f), tolerance=0.15, angular_tolerance=0.35)
            op = 1.0
            if opacity:
                for pre, o in opacity.items():
                    if i.name.startswith(pre):
                        op = o
            files.append({"file": str(f), "color": i.color, "opacity": op, "edges": edges,
                          "metal": i.kind == "vitamin" and i.color in (asm.COL["metal"], asm.COL["tube"])})
        for v in views:
            vv = v if isinstance(v, str) else v[0]
            vec = None if isinstance(v, str) else v[1]
            rnd.render_scene(files, str(d / f"{tag}_{vv}.png"), view=vv if vec is None else "iso", view_vec=vec,
                             size=size, title=title, zoom=zoom,
                             subtitle="WRA-1 | generated by cad/build.py from params.yaml | mm")
            log(f"render {tag}_{vv}")

    scene(asm.arm_instances(asm.HOME), "arm_park", ["iso", "iso_right", "left"], "WRA-1 arm - park pose")
    scene(asm.arm_instances(WORK_POSE), "arm_work", ["iso", "iso_back_right"], "WRA-1 arm - work pose")
    scene(asm.arm_instances(asm.HOME, explode=1.0, vitamins=True), "arm_exploded", ["iso"],
          "WRA-1 arm - exploded (park pose)", zoom=1.0, size=(1600, 2000))
    env_op = {"user_": 0.18, "chair_": 1.0}
    for side in ("right", "left"):
        inst = [i for i in ch.chair_instances(side) if not i.name.startswith(("belt", "cord", "spring"))]
        scene(inst, f"install_{side}", ["iso", "front", "top"],
              f"{side.upper()} installation on the PROVISIONAL reference chair (park pose)", opacity=env_op)
    # both sides at once (illustration only: same parts)
    inst = [i for i in ch.chair_instances("right", user=True) if not i.name.startswith(("belt", "cord", "spring"))]
    inst += [i for i in ch.chair_instances("left", chair=False, user=False)
             if not i.name.startswith(("belt", "cord", "spring"))]
    scene(inst, "install_both_sides", ["top", "front"], "Identical arm on RIGHT and LEFT (illustration)",
          opacity=env_op)
    # the headline task: RIGHT installation gripping a 36" (914 mm) knob 450 mm ahead of J1
    try:
        sys.path.insert(0, str(HERE / "tools"))
        import chair_check
        from build123d import Pos as _Pos, Box as _Box, Sphere as _Sphere, Cylinder as _Cyl, Rot as _Rot
        qk = chair_check.knob_pose()
        inst = [i for i in ch.chair_instances("right", qk) if not i.name.startswith(("belt", "cord", "spring"))]
        B = ch.frames("right")["B"]
        zk, xk = 914.0 - P.chair.mount_z, 450.0               # knob face centre in B
        door_x = xk + 56.0 + 10.0                              # knob Ø56 + 10 mm stem to the door face
        door = B * _Pos(door_x + 20.0, 400.0 - 65.0, 1000.0 - P.chair.mount_z) * _Box(40.0, 800.0, 2000.0)
        knob = B * _Pos(xk + 28.0, 0.0, zk) * _Sphere(28.0)
        stem = B * _Pos(xk + 28.0 + 20.0, 0.0, zk) * _Rot(0, 90, 0) * _Cyl(10.0, 40.0)
        inst += [asm.Instance("ref_door", "door", door, "#c8b89a", "reference", "env"),
                 asm.Instance("ref_knob", "knob", knob, "#c9a227", "reference", "env"),
                 asm.Instance("ref_knob_stem", "knob", stem, "#c9a227", "reference", "env")]
        scene(inst, "door_knob_right", ["iso", "top", "iso_right"],
              'Gripping a 36" door knob (RIGHT installation, dock 460 mm, PROVISIONAL chair)',
              opacity=dict(env_op, ref_door=0.35))
    except Exception as e:                                     # knob outside the window for this mount_z
        log(f"door-knob render skipped: {e}")
    for nm, grp in (("claw", ("gripper",)), ("base", ("dock", "base", "turntable"))):
        sel = [i for i in asm.arm_instances({"j6": 20.0}) if i.group in grp]
        scene(sel, f"detail_{nm}", ["iso", "iso_back_right"], f"Detail - {nm}", edges=True)
    if asm.WRIST == "abenics":
        for tag, wt, j6 in (("home", (0, 0, 0), 20.0), ("pitch_down", (-30, 0, 0), 32.5),
                            ("yaw_left_roll", (0, 30, 60), 10.0)):
            inst, _ = asm.wrist_instances(wt, j6)
            scene(inst, f"detail_abenics_{tag}", ["iso", "iso_back_right", "front"],
                  f"ABENICS wrist + knob claw, (pitch, yaw, roll) = {wt}", edges=True)
    if tmp.exists():
        shutil.rmtree(tmp)


# ============================================================================ drawings
def drawings(parts):
    from wra import drawing
    d = OUT / "drawings"
    d.mkdir(parents=True, exist_ok=True)
    rows = {r["part"]: r for r in json.loads((OUT / "reports" / "parts.json").read_text())} \
        if (OUT / "reports" / "parts.json").exists() else {}
    drawing.make_all(parts, d, rows)
    from wra import dxf_alu
    (OUT / "dxf").mkdir(parents=True, exist_ok=True)
    dxf_alu.write(OUT / "dxf" / "04_post_bracket_plates.dxf")
    log("drawings + dxf done")


# ============================================================================ BOM
def bom(rows=None):
    from wra import bom as bm
    rows = rows or json.loads((OUT / "reports" / "parts.json").read_text())
    bm.write(rows, OUT / "reports")
    print_guide(rows)
    log("BOM + print guide written")


def print_guide(rows):
    """docs/13_print_guide.md, generated from the part registry (orientation = the STL as exported)."""
    act = [r for r in rows if r.get("variant", "common") in ("common", VARIANT)]
    oth = [r for r in rows if r.get("variant", "common") not in ("common", VARIANT)]
    g = sum(r["mass_g"] * r["qty"] for r in act)
    L = ["# 13 - Print guide (GENERATED by `python cad/build.py --parts`; do not edit)", "",
         f"Active variant **{VARIANT}**: {len(act)} part files, {sum(r['qty'] for r in act)} prints, "
         f"about **{g / 1000:.1f} kg** of filament and roughly **{g / 25:.0f} h** of printing at ~25 g/h [E].", "",
         "## Printer and slicer (Bambu Lab P1S, 256 x 256 x 256 mm)", "",
         "* Every STL in `output/stl/` is already in its PRINT ORIENTATION, sitting on z = 0: import, do not rotate.",
         "* Nozzle: 0.4 mm **hardened steel** for any CF filament (PA-CF / PETG-CF are abrasive).",
         "* PA-CF: dry 8 h at 80 C before printing, print from a dry box, enclosure closed, textured PEI plate with glue stick, "
         "bed 80-100 C.  PETG: 0.2 mm layers, bed 70-80 C.  TPU 95A (pads): 25 mm/s.",
         "* Gears / pulleys / racks: 0.12 mm layers, 100 % infill, no ironing; check the first tooth with calipers.",
         "* Structural parts: walls and infill below are minimums; 0.2 mm layers; no 'thin wall' detection shortcuts.",
         "* Print `00_fit_test_coupon` first and tune `fit.*` in params.yaml (docs/04) before the real parts.",
         "* ABENICS `80/81/82`: print the **STL** (it has the teeth); the STEP files are tooth-less blanks.", "",
         "## Parts (active variant)", "",
         "| part | qty | material | walls / infill | supports | bed footprint (mm) | g each | orientation / notes |",
         "|---|---|---|---|---|---|---|---|"]
    for r in act:
        bb = r["print_bbox_mm"]
        L.append(f"| {r['part']} | {r['qty']} | {r['material']} | {r['walls']} / {r['infill']} | {r['supports']} | "
                 f"{bb[0]:.0f} x {bb[1]:.0f} x {bb[2]:.0f} | {r['mass_g']:.0f} | "
                 f"{(r['print_notes'] or '-').replace('|', '/')} |")
    if oth:
        L += ["", f"## Only for the other wrist variant ({'WRA-1B pitch-roll' if VARIANT == 'WRA-1A' else 'WRA-1A ABENICS'})",
              "", "| part | qty | material | g each |", "|---|---|---|---|"]
        for r in oth:
            L.append(f"| {r['part']} | {r['qty']} | {r['material']} | {r['mass_g']:.0f} |")
    L += ["", "Suggested order: fit coupon -> base / dock -> drive modules -> turntable + shoulder -> arm nodes -> "
          "wrist -> claw -> mount -> electronics box.  Inspect every load-bearing print for layer splits before use."]
    (ROOT / "docs" / "13_print_guide.md").write_text("\n".join(L) + "\n")


# ============================================================================ verification
def verify(parts):
    import trimesh
    # The driver's chair -> arm transform and user keep-out boxes live in arm_geometry.json, and the chair
    # checks below solve the knob pose with the driver's model: refresh it from THIS params.yaml first.
    software_export(meshes=False)
    rep = {"generated": datetime.datetime.now().isoformat(timespec="seconds"), "params_sha": _params_hash(),
           "build_volume_mm": BUILD_VOLUME, "margin_mm": BED_MARGIN, "parts": [], "summary": {},
           "arm_geometry_params_sha": json.loads((SW / "wra_driver" / "config" / "arm_geometry.json").read_text())["params_sha"]}
    ok_all = True
    for pd in parts:
        shp = pd.shape
        bed = pd.print_loc * shp if pd.print_loc is not None else shp
        bb = bed.bounding_box()
        r = {"part": pd.name, "variant": pd.extra.get("variant", "common"), "valid": bool(shp.is_valid),
             "solids": len(shp.solids()),
             "fits_bed": all(s <= lim - 2 * BED_MARGIN for s, lim in zip((bb.size.X, bb.size.Y, bb.size.Z), BUILD_VOLUME)),
             "on_bed": abs(bb.min.Z) < 1e-3}
        stl = OUT / "stl" / f"{pd.name}.stl"
        if stl.exists():
            m = trimesh.load(str(stl))
            r["stl_watertight"] = bool(m.is_watertight)
            r["stl_winding_ok"] = bool(m.is_winding_consistent)
            if pd.extra.get("mesh_teeth"):        # toothed print mesh: grooves cut from the B-rep blank
                r["stl_source"] = "toothed mesh"
                r["stl_volume_vs_blank_pct"] = round(100 * (abs(m.volume) - shp.volume) / shp.volume, 2)
            else:
                r["stl_volume_err_pct"] = round(100 * (abs(m.volume) - shp.volume) / shp.volume, 3)
            r["stl_triangles"] = int(len(m.faces))
        stp = OUT / "step" / "parts" / f"{pd.name}.step"
        if stp.exists():
            try:
                s2 = import_step(str(stp))
                r["step_volume_err_pct"] = round(100 * (s2.volume - shp.volume) / shp.volume, 4)
            except Exception as e:      # pragma: no cover
                r["step_volume_err_pct"] = f"import failed: {e}"
        r["pass"] = (r["valid"] and r["solids"] == 1 and r["fits_bed"] and r["on_bed"]
                     and r.get("stl_watertight", True) and abs(r.get("stl_volume_err_pct", 0)) < 1.0
                     and (isinstance(r.get("step_volume_err_pct", 0), float) and abs(r.get("step_volume_err_pct", 0)) < 0.1
                          or "step_volume_err_pct" not in r))
        ok_all &= r["pass"]
        rep["parts"].append(r)
        log(f"verify {pd.name:28s} {'PASS' if r['pass'] else 'FAIL'} {r}")
    # interface checks
    rep["interfaces"] = interface_checks()
    for k, v in rep["interfaces"].items():
        ok_all &= v["pass"]
    # interference sweep + chair checks
    sys.path.insert(0, str(HERE / "tools"))
    import sweep
    res = []
    poses = sweep.DEFAULT_POSES_ABENICS if asm.WRIST == "abenics" else sweep.DEFAULT_POSES_VALID
    for q in poses:
        inst = asm.arm_instances(q)
        bad = sweep.check(inst)
        res.append({"pose": q, "interferences": bad, "pass": not bad})
        ok_all &= not bad
        log(f"sweep {q}: {bad}")
    rep["self_interference_sweep"] = res
    if asm.WRIST == "abenics":
        wsj = OUT / "reports" / "wrist_sweep.json"
        if not wsj.exists():
            import wrist_sweep
            sys.argv = [sys.argv[0], "--quick"]
            wrist_sweep.main()
        wsr = json.loads(wsj.read_text())
        bad = [r for r in wsr if r["interferences"] or any(v > 0.5 for v in r.get("gear_mesh_overlap_mm3", []))]
        rep["wrist_sweep"] = {"poses": len(wsr), "with_mesh_check": all("gear_mesh_overlap_mm3" in r for r in wsr),
                              "problems": bad, "pass": not bad}
        ok_all &= not bad
    import chair_check
    cc = chair_check.run_all()                  # park + gripping a 36" knob, RIGHT and LEFT
    rep["installation_checks"] = cc
    (OUT / "reports").mkdir(parents=True, exist_ok=True)
    (OUT / "reports" / "chair_check.json").write_text(json.dumps(cc, indent=1))
    for c in cc:
        ok_all &= not c["interferences"]
    rep["summary"] = {"all_pass": ok_all, "parts": len(parts),
                      "parts_pass": sum(1 for r in rep["parts"] if r["pass"])}
    (OUT / "reports").mkdir(parents=True, exist_ok=True)
    (OUT / "reports" / "verification.json").write_text(json.dumps(rep, indent=1))
    _verification_md(rep)
    log("VERIFICATION", "PASS" if ok_all else "FAIL", rep["summary"])
    return rep


def interface_checks():
    """Numerical checks that mating features come from the same parameters and actually align."""
    out = {}
    inst = {i.name: i for i in asm.arm_instances(asm.HOME)}

    def vol(a, b):
        r = inst[a].shape & inst[b].shape
        return round(r.volume, 2) if r is not None else 0.0

    for k in (0, 1):
        v = vol(f"dock_dowel_{k}", "base_housing")
        out[f"dock_dowel_{k}_clears_base_hole"] = {"value_mm3": v, "pass": v < 0.5}
        v = vol(f"thumb_screw_{k}", "base_housing")
        out[f"thumb_screw_{k}_clears_base_hole"] = {"value_mm3": v, "pass": v < 0.5}
        v = vol(f"thumb_screw_{k}", "receiver")
        out[f"thumb_screw_{k}_in_receiver_insert_bore"] = {"value_mm3": v, "pass": v < 0.5}
    # belt centre distances reproduce the catalogue belt lengths
    for j, (t2, ln) in {1: (P.pulleys.j1, P.belt_lengths.j1), 2: (P.pulleys.j2, P.belt_lengths.j2),
                        3: (P.pulleys.j3, P.belt_lengths.j3), 4: (P.pulleys.j4, P.belt_lengths.j4)}.items():
        c = getattr(layout, f"C{j}")
        L = lib.belt_length(P.dm.teeth, t2, c)                  # exact (the textbook approximation is 0.06 mm off on J2)
        out[f"belt_j{j}_length"] = {"centre_mm": round(c, 3), "pitch_length_mm": round(L, 3), "nominal": ln,
                                    "pass": abs(L - ln) < 0.05}
    # claw: pads touch at q = 0, carriages stay on the rail at full opening
    out.update(_claw_checks())
    # ABENICS: kinematics round trip, gear centre distances, toothed-mesh overlap at home
    if asm.WRIST == "abenics":
        out["abenics_fk_ik_roundtrip"] = {"max_err": float(AK.self_test()), "pass": AK.self_test() < 1e-9}
        cd_mp = (P.abenics.cs_teeth + P.abenics.mp_teeth) * P.abenics.module / 2
        out["abenics_cs_monopole_centre_distance"] = {"D_MP": ab.D_MP, "from_teeth": cd_mp,
                                                      "pass": abs(ab.D_MP - cd_mp) < 1e-9}
        tg = P.abenics.theta_gear
        out["abenics_theta_pair_centre_distance"] = {"Z_TS - D_MP": round(ab.Z_TS - ab.D_MP, 4),
                                                     "from_teeth": (tg.servo_teeth + tg.mp_teeth) * tg.module / 2,
                                                     "pass": abs(ab.Z_TS - ab.D_MP - (tg.servo_teeth + tg.mp_teeth) * tg.module / 2) < 1e-9}
        sys.path.insert(0, str(HERE / "tools"))
        import wrist_sweep as ws
        _, f = asm.wrist_instances((0, 0, 0), 0.0)
        ov = ws.mesh_check(f)
        out["abenics_toothed_mesh_overlap_home"] = {"overlap_mm3": ov, "pass": max(ov) < 0.5}
    return out


def _claw_checks():
    a, b = cw.pad(0.0, 1), cw.pad(0.0, -1)
    r = a & b
    v = r.volume if r is not None else 0.0
    gap = a.distance_to(b)
    edge = cw.Q_MAX + cw.YC_OFF + P.claw.block_len / 2
    return {"claw_closed_pads_touch": {"overlap_mm3": round(v, 2), "gap_mm": round(gap, 3), "pass": v < 0.5 and gap < 0.2},
            "claw_carriage_on_rail_at_full_open": {"carriage_edge_mm": round(edge, 2), "rail_half_mm": P.claw.rail_len / 2,
                                                   "pass": edge <= P.claw.rail_len / 2 + 1e-6},
            "claw_jaw_force_N_per_jaw": {"cont": round(cw.jaw_force(P.claw.torque_cont_Nm), 1),
                                         "knob": round(cw.jaw_force(P.claw.torque_knob_Nm), 1), "pass": True}}


def _params_hash():
    import hashlib
    return hashlib.sha256((HERE / "wra" / "params.yaml").read_bytes()).hexdigest()[:16]


def _verification_md(rep):
    L = ["# Verification report (generated)", "",
         f"Generated {rep['generated']} from params.yaml sha256:{rep['params_sha']}.", "",
         f"**Overall: {'PASS' if rep['summary']['all_pass'] else 'FAIL'}** "
         f"({rep['summary']['parts_pass']}/{rep['summary']['parts']} parts pass).", "",
         "## Printed parts", "",
         "| part | valid B-rep | 1 solid | fits P1S (246 mm) | on bed | STL watertight | STL vol err % | STEP vol err % | result |",
         "|---|---|---|---|---|---|---|---|---|"]
    for r in rep["parts"]:
        L.append(f"| {r['part']} | {r['valid']} | {r['solids'] == 1} | {r['fits_bed']} | {r['on_bed']} | "
                 f"{r.get('stl_watertight', '-')} | {r.get('stl_volume_err_pct', '-')} | {r.get('step_volume_err_pct', '-')} | "
                 f"{'PASS' if r['pass'] else '**FAIL**'} |")
    L += ["", "## Interface checks", "", "| check | value | result |", "|---|---|---|"]
    for k, v in rep["interfaces"].items():
        vals = {a: b for a, b in v.items() if a != "pass"}
        L.append(f"| {k} | {vals} | {'PASS' if v['pass'] else '**FAIL**'} |")
    L += ["", "## Self-interference sweep (arm, full CAD solids)", "", "| pose (deg) | interferences | result |",
          "|---|---|---|"]
    for r in rep["self_interference_sweep"]:
        L.append(f"| `{json.dumps(r['pose'])}` | {r['interferences'] or 'none'} | {'PASS' if r['pass'] else '**FAIL**'} |")
    if "wrist_sweep" in rep:
        w = rep["wrist_sweep"]
        L += ["", "## ABENICS wrist + claw sweep (tools/wrist_sweep.py)", "",
              f"{w['poses']} poses at the hard limits (tilt 33 deg, 24 deg toward a module axis, roll -90/0/+90, "
              f"claw closed/open); toothed gear-mesh check: {w['with_mesh_check']}.  Problems: "
              f"{w['problems'] or 'none'} -> **{'PASS' if w['pass'] else 'FAIL'}**"]
    L += ["", "## Installation checks on the PROVISIONAL reference chair (park, and gripping a 36-inch knob)", ""]
    for c in rep["installation_checks"]:
        L.append(f"* **{c['side'].upper()}, {c.get('pose', 'park')}**: interferences {c['interferences'] or 'none'}; "
                 f"added width beyond hand rim {c['added_width_mm']} mm; minimum clearances {c['clearance_mm']}")
    if "arm_geometry_params_sha" in rep:
        L += ["", f"`software/wra_ros2/wra_driver/config/arm_geometry.json` (chair -> arm transform, user keep-out boxes) "
                  f"was refreshed from this params.yaml (params_sha {rep['arm_geometry_params_sha']}). The driver logs "
                  f"this value at start-up and `wra_cli.py status` prints it: it must match before the arm is used "
                  f"on the chair (docs/09 T4.0)."]
    L += ["", "Clearances to the provisional chair are only as good as the provisional chair: re-run "
          "`python build.py --parts --verify` after entering YOUR measurements in params.yaml, then update the "
          "Jetson (docs/11 section G)."]
    (OUT / "reports" / "verification.md").write_text("\n".join(L) + "\n")


# ============================================================================ software export
SW = ROOT / "software" / "wra_ros2"


def software_export(meshes=True):  # noqa: C901
    """Geometry for the ROS 2 driver (numbers straight from params.yaml / the CAD frames), a copy
    of the ABENICS kinematics, and a URDF with per-link meshes for RViz."""
    from wra.parts import wrist as wr_
    A = P.arm
    lim = P.joints
    g = {"variant": VARIANT, "generated_from_params_sha": _params_hash(),
         "turntable_origin_z": layout.T_ORIGIN_B, "shoulder_h": A.shoulder_h, "L2": A.L2, "L3": A.L3,
         "YC": wr_.YC, "tcp_from_flange": cw.tcp_x(),
         "ratios": {f"j{j}": layout.ratio(j) for j in (1, 2, 3, 4)},
         "limits": {"j1": [lim.j1.soft, lim.j1.hard], "j2": [lim.j2.soft, lim.j2.hard],
                    "j3": [lim.j3.soft, lim.j3.hard], "j3_rel": lim.j3_rel.soft,
                    "j4": [lim.j4.soft, lim.j4.hard], "j5": [lim.j5.soft, lim.j5.hard],
                    "claw_mm": lim.j6_half_open_mm},
         "home": {"j1": lim.home_deg.j1, "j2": lim.home_deg.j2, "j3": lim.home_deg.j3,
                  "wt": list(lim.home_abenics.wt), "j4": lim.home_deg.j4, "j5": lim.home_deg.j5},
         "claw_pinion_r": cw.RP, "claw_jaw_force_N_per_Nm": cw.jaw_force(1.0),
         "theta_servo_per_theta": P.abenics.theta_gear.mp_teeth / P.abenics.theta_gear.servo_teeth,
         "servo_ids": ({"j1": 1, "j2": 2, "j3": 3, "q1": 4, "theta1": 5, "q2": 6, "theta2": 7, "claw": 8}
                       if VARIANT == "WRA-1A" else {"j1": 1, "j2": 2, "j3": 3, "j4": 4, "j5": 5, "claw": 8}),
         "servo": {"stall_Nm": P.servo.stall_torque_Nm, "continuous_fraction": P.servo.continuous_fraction,
                   "peak_fraction": P.servo.peak_fraction, "no_load_rpm": P.servo.no_load_rpm}}
    if VARIANT == "WRA-1A":
        M = ab.G_in_S()
        t = M.wrapped.Transformation()
        g["flange_from_wrist_center"] = ab.FL1
        g["G_in_S_rotation"] = [[round(t.Value(i, j), 12) for j in range(1, 4)] for i in range(1, 4)]
    else:
        g["x_flange1"] = wr_.X_FLANGE1
    from wra import chair as ch
    g["mount"] = {}
    for side in ("right", "left"):
        t = ch.frames(side)["B"].wrapped.Transformation()
        Rm = [[t.Value(i, j) for j in range(1, 4)] for i in range(1, 4)]
        g["mount"][side] = {"xyz": [round(t.Value(i, 4) / 1000.0, 5) for i in range(1, 4)],
                            "rpy": [round(math.atan2(Rm[2][1], Rm[2][2]), 6),
                                    round(math.atan2(-Rm[2][0], math.hypot(Rm[0][0], Rm[1][0])), 6),
                                    round(math.atan2(Rm[1][0], Rm[0][0]), 6)],
                            "note": "chair frame C: floor under the rear-axle centre, x forward, y left (PROVISIONAL chair)"}
    # user keep-out (docs/09 T5.1): head + torso boxes of the CAD occupant envelope, chair frame, mm.
    # The driver refuses any target or path point that puts the elbow, wrist, flange or TCP inside
    # these boxes grown by keepout_margin_mm.  The envelope is a conservative box model, not anthropometry.
    g["user_keepout_C_mm"] = []
    for nm, shp in mt.user_envelope():
        if nm in ("torso", "head"):
            bb = shp.bounding_box()
            g["user_keepout_C_mm"].append({"name": nm, "min": [round(v, 1) for v in (bb.min.X, bb.min.Y, bb.min.Z)],
                                           "max": [round(v, 1) for v in (bb.max.X, bb.max.Y, bb.max.Z)]})
    g["keepout_margin_mm"] = 50.0
    # which params.yaml this file came from: the driver logs it at start-up and `wra_cli.py status` prints it;
    # it must equal params_sha in output/reports/verification.json (docs/09 T4.0)
    g["params_sha"] = _params_hash()
    d = SW / "wra_driver" / "config"
    d.mkdir(parents=True, exist_ok=True)
    (d / "arm_geometry.json").write_text(json.dumps(g, indent=1))
    shutil.copy(HERE / "wra" / "abenics_kin.py", SW / "wra_driver" / "wra_driver" / "abenics_kin.py")
    log("software export: arm_geometry.json + abenics_kin.py")
    if meshes:
        urdf_export()


def urdf_export():
    """Per-link meshes (link frames = URDF frames at the zero pose) + wra.urdf.xacro."""
    import numpy as np
    from build123d import Location, Pos
    d = SW / "wra_description"
    (d / "meshes").mkdir(parents=True, exist_ok=True)
    (d / "urdf").mkdir(parents=True, exist_ok=True)
    z = dict(asm.HOME, j1=0.0, j2=0.0, j3=0.0, j6=0.0)
    if VARIANT == "WRA-1A":
        z["wt"] = (0.0, 0.0, 0.0)
    else:
        z.update(j4=0.0, j5=0.0)
    f = asm.frames(z)
    links = {"base_link": Location(), "turntable": f["T"], "upper_arm": f["U"], "forearm": f["F"],
             "tool_flange": f["G"]}
    groups = {"base_link": ("dock", "base"), "turntable": ("turntable", "shoulder"), "upper_arm": ("upper_arm",),
              "forearm": ("forearm", "wrist_m1", "wrist_m2", "wrist_base", "wrist"), "tool_flange": ("tool", "gripper")}
    inst = [i for i in asm.arm_instances(z) if not i.name.startswith(("belt", "cord", "spring"))]
    for ln, grp in groups.items():
        sel = [i for i in inst if i.group in grp and not i.name.startswith("claw_jaw") and not i.name.startswith("claw_pad")]
        if not sel:
            continue
        comp = _compound(sel, ln)
        export_stl(links[ln].inverse() * comp, str(d / "meshes" / f"{ln}.stl"), tolerance=0.3, angular_tolerance=0.5)
    for side in (1, -1):
        jaw = [i for i in inst if i.name in (f"claw_jaw_{'A' if side > 0 else 'B'}", f"claw_pad_{'A' if side > 0 else 'B'}")]
        comp = _compound(jaw, "jaw")
        export_stl(f["G"].inverse() * comp, str(d / "meshes" / f"jaw_{'a' if side > 0 else 'b'}.stl"),
                   tolerance=0.3, angular_tolerance=0.5)

    def rel(a, b):
        M = a.inverse() * b
        t = M.wrapped.Transformation()
        R = np.array([[t.Value(i, j) for j in range(1, 4)] for i in range(1, 4)])
        p = [t.Value(i, 4) / 1000.0 for i in range(1, 4)]
        sy = math.hypot(R[0, 0], R[1, 0])
        rpy = (math.atan2(R[2, 1], R[2, 2]), math.atan2(-R[2, 0], sy), math.atan2(R[1, 0], R[0, 0]))
        return " ".join(f"{v:.6f}" for v in p), " ".join(f"{v:.6f}" for v in rpy)

    L = ['<?xml version="1.0"?>', "<!-- GENERATED by cad/build.py --software from params.yaml. Do not edit. -->",
         f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="wra1">',
         '  <xacro:arg name="side" default="right"/>', "  <!-- side only changes the base mounting (see launch); the arm is identical L/R -->",
         '  <material name="grey"><color rgba="0.55 0.6 0.65 1"/></material>',
         '  <material name="orange"><color rgba="0.88 0.63 0.19 1"/></material>']

    def link(name, mesh, mat="grey"):
        L.extend([f'  <link name="{name}">', f'    <visual><geometry><mesh filename="package://wra_description/meshes/{mesh}.stl" '
                  f'scale="0.001 0.001 0.001"/></geometry><material name="{mat}"/></visual>', "  </link>"])

    def joint(name, typ, parent, child, xyz, rpy, axis=None, lo=None, hi=None, vel=1.0, mimic=None):
        L.append(f'  <joint name="{name}" type="{typ}">')
        L.append(f'    <parent link="{parent}"/><child link="{child}"/><origin xyz="{xyz}" rpy="{rpy}"/>')
        if axis:
            L.append(f'    <axis xyz="{axis}"/>')
        if lo is not None:
            L.append(f'    <limit lower="{lo:.5f}" upper="{hi:.5f}" effort="3.0" velocity="{vel}"/>')
        if mimic:
            L.append(f'    <mimic joint="{mimic[0]}" multiplier="{mimic[1]}"/>')
        L.append("  </joint>")

    r = math.radians
    link("base_link", "base_link")
    link("turntable", "turntable")
    link("upper_arm", "upper_arm")
    link("forearm", "forearm")
    xyz, rpy = rel(links["base_link"], links["turntable"])
    joint("j1", "revolute", "base_link", "turntable", xyz, rpy, "0 0 1", r(P.joints.j1.soft[0]), r(P.joints.j1.soft[1]))
    xyz, rpy = rel(links["turntable"], links["upper_arm"])
    joint("j2", "revolute", "turntable", "upper_arm", xyz, rpy, "0 -1 0", r(P.joints.j2.soft[0]), r(P.joints.j2.soft[1]))
    xyz, rpy = rel(links["upper_arm"], links["forearm"])
    rl = P.joints.j3_rel.soft
    joint("j3", "revolute", "upper_arm", "forearm", xyz, rpy, "0 -1 0", -r(rl[1]), -r(rl[0]))
    if VARIANT == "WRA-1A":
        xyz, rpy = rel(links["forearm"], f["A"])
        for nm in ("wrist_yaw_link", "wrist_pitch_link", "wrist_roll_link"):
            L.append(f'  <link name="{nm}"/>')
        tl = math.radians(P.abenics.tilt_hard_deg)
        joint("wrist_yaw", "revolute", "forearm", "wrist_yaw_link", xyz, rpy, "0 0 1", -tl, tl, 3.0)
        joint("wrist_pitch", "revolute", "wrist_yaw_link", "wrist_pitch_link", "0 0 0", "0 0 0", "0 -1 0", -tl, tl, 3.0)
        joint("wrist_roll", "revolute", "wrist_pitch_link", "wrist_roll_link", "0 0 0", "0 0 0", "1 0 0", -r(90), r(90), 3.0)
        G0 = Location(Pos(0, 0, 0)) * ab.S_loc(AK.R_HOME) * ab.G_in_S()
        xyz, rpy = rel(Location(), G0)
        link("tool_flange", "tool_flange", "orange")
        joint("wrist_tool", "fixed", "wrist_roll_link", "tool_flange", xyz, rpy)
    else:
        xyz, rpy = rel(links["forearm"], f["W"])
        L.append('  <link name="pitch_link"/>')
        joint("j4", "revolute", "forearm", "pitch_link", xyz, rpy, "0 -1 0", r(P.joints.j4.soft[0]), r(P.joints.j4.soft[1]))
        xyz, rpy = rel(f["W"], f["G"])
        link("tool_flange", "tool_flange", "orange")
        joint("j5", "revolute", "pitch_link", "tool_flange", xyz, rpy, "1 0 0", r(P.joints.j5.soft[0]), r(P.joints.j5.soft[1]))
    link("jaw_a", "jaw_a", "orange")
    link("jaw_b", "jaw_b", "orange")
    joint("claw", "prismatic", "tool_flange", "jaw_a", "0 0 0", "0 0 0", "0 1 0", 0.0, cw.Q_MAX / 1000, 0.05)
    joint("claw_b", "prismatic", "tool_flange", "jaw_b", "0 0 0", "0 0 0", "0 -1 0", 0.0, cw.Q_MAX / 1000, 0.05,
          mimic=("claw", 1.0))
    L.append('  <link name="tcp"/>')
    joint("tcp_joint", "fixed", "tool_flange", "tcp", f"{cw.tcp_x() / 1000:.6f} 0 0", "0 0 0")
    L.append("</robot>")
    (d / "urdf" / "wra.urdf.xacro").write_text("\n".join(L) + "\n")
    log("URDF + link meshes written")


# ============================================================================ main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    for k in ("all", "parts", "assemblies", "renders", "drawings", "verify", "bom", "software"):
        ap.add_argument(f"--{k}", action="store_true")
    ap.add_argument("--only", nargs="*", help="restrict --parts/--drawings to these part names")
    a = ap.parse_args()
    if not any(vars(a)[k] for k in ("all", "parts", "assemblies", "renders", "drawings", "verify", "bom", "software")):
        ap.print_help()
        return
    t0 = time.time()
    parts = registry()
    if a.only:
        parts = [p for p in parts if p.name in a.only]
    rows = None
    if a.all or a.parts:
        rows = export_parts(parts)
    if a.all or a.parts or a.bom:
        bom(rows)
    if a.all or a.software:                     # first: the knob-pose render and the checks use the driver model
        software_export()
    if a.all or a.assemblies:
        export_assemblies()
    if a.all or a.drawings:
        drawings(parts)
    if a.all or a.renders:
        renders()
    if a.all or a.verify:
        verify(parts)
    log(f"done in {time.time() - t0:.0f}s")


if __name__ == "__main__":
    main()
