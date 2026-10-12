# WRA-1 — 3D-printable wheelchair robotic arm (ABENICS wrist, knob claw)

A modular, fully 3D-printable assistive arm for a **manual (self-propelled) wheelchair**: no motors, joystick or battery on the chair, so the arm brings its own battery, and it comes off the dock (two thumb screws) when you don't need it. It mounts on the **LEFT or RIGHT** front post with the same parts. Its main job is **opening doors**, especially gripping and turning **door knobs**; it also fetches and moves light objects.

Hardware:
- 8× Waveshare **ST3215** serial-bus servos
- an **ESP32** safety supervisor
- an NVIDIA **Jetson Orin Nano 8GB** running ROS 2 Humble
- a **ReSpeaker** USB mic for voice commands
- every printed part fits a **Bambu Lab P1S** (256 mm bed)

> **Status: research prototype — designed and checked in CAD and simulation, not built or tested yet.**
> It is **not** safe for human-facing use until the staged structural, electrical and operational tests in `docs/09_safety.md` have passed. Bench-test first; never run a loaded arm near a person before then.

## What is in here

| folder | contents |
|---|---|
| `docs/` | the engineering report (`00_engineering_report.md`) and 15 topic documents: research, trade study, L/R guide, design rules, calculations, ABENICS wrist, electronics, control interface, safety + FMEA + test plan, door tasks, software setup, measurement worksheet, print guide, assembly, maintenance |
| `cad/` | **editable parametric CAD source** (Python, build123d + manifold3d). `cad/wra/params.yaml` is the single source of truth. `cad/build.py` regenerates every output |
| `output/stl/` | one STL per printable part, **already in print orientation** (ABENICS gears with real teeth) |
| `output/step/` | STEP per part (`parts/`) and assemblies (`assemblies/*.step.zip`, unzip first: 10–40 MB each): park, work pose, exploded, wrist + claw, RIGHT and LEFT installations |
| `output/drawings/`, `output/dxf/` | A3 3-view drawing per part (SVG) and all of them in `WRA1_drawings.pdf`: overall dimensions, auto-extracted hole/bore table, material, print settings. The STEP files are the master geometry. Plus the aluminium bracket plate DXF |
| `output/renders/` | PNG renders |
| `output/reports/` | BOM (`BOM.md`, CSV), fastener schedule, parts list with masses, verification report, sweep and chair-check results |
| `calcs/` | engineering calculations (`arm_calcs.py`, `knob_window.py`) → `calcs/output/*.png`, `results.json` |
| `firmware/esp32_safety_controller/` | ESP32 supervisor: servo bus master, limits, watchdog, SAFE_OK, CRC link to the Jetson. Compiles and links for the ESP32 (arduino-esp32 2.0.17); not yet run on a board |
| `software/wra_ros2/` | ROS 2 packages: `wra_driver` (driver node, kinematics incl. ABENICS IK, door-knob task, voice node, simulated ESP32, tests) and `wra_description` (generated URDF + meshes) |
| `software/tools/` | `servo_setup.py` (IDs, calibration, limits), `wra_cli.py` (bench control, teaching, door demo, `--fake` simulation) |

## Regenerate everything

```bash
pip install -r requirements.txt
cd cad
python build.py --parts         # STL + STEP per part, masses, BOM, fastener schedule, print guide   (~5 min)
python build.py --software      # arm_geometry.json, ABENICS kinematics copy, URDF + link meshes      (~3 min)
python build.py --assemblies    # STEP assemblies (+ .step.zip)                                      (~7 min)
python build.py --drawings      # dimensioned drawings                                               (~40 min)
python build.py --renders       # needs Node.js: (cd tools/render && npm install) and
                                #   python -m playwright install chromium                           (~15 min)
python tools/wrist_sweep.py --mesh   # ABENICS + claw sweep with the toothed gear meshes            (~40 min)
                                     #   (interrupted? add --resume <its log> to keep the poses already done)
python build.py --verify        # validity, bed fit, meshes, interfaces, arm sweep, L/R chair checks; reads
                                #   the wrist sweep above                                            (~45 min)
cd ../calcs && python arm_calcs.py && python knob_window.py   # calculations -> docs/05
cd ../software/wra_ros2/wra_driver && python -m pytest -q test  # driver tests (no hardware, ~1 min)
```

Change a dimension, a fit, a chair measurement or the wrist type in `cad/wra/params.yaml` and re-run. Never edit the generated files by hand. `--verify` also refreshes `software/wra_ros2/wra_driver/config/arm_geometry.json` (the driver's chair → arm transform and user keep-out boxes): after a change, copy it to the Jetson (docs/11 §G).

## Start here

1. `docs/00_engineering_report.md` — the whole design in 13 sections.
2. **Materials list:** [`output/reports/BOM.md`](output/reports/BOM.md) — every part to print, every part to buy (with estimated prices, about $577 in all) and every screw. The buy list is also a spreadsheet: [`bom_purchased.csv`](output/reports/bom_purchased.csv). Where to buy each part (mostly Amazon links): [`docs/16_shopping_list.md`](docs/16_shopping_list.md).
3. `docs/12_measurement_worksheet.md` — measure your chair; the CAD chair is provisional.
4. `docs/13_print_guide.md` → `docs/14_assembly_manual.md` → `docs/11_software_setup.md` → `docs/09_safety.md` test stages.
5. `docs/test_log.md` — what has been verified in CAD/simulation so far, and the table for your physical test results.

## Repository

https://github.com/landen-finkel-engineering/ROLLO-Wheelchair-Arm — this repository. On a Jetson: `git clone https://github.com/landen-finkel-engineering/ROLLO-Wheelchair-Arm.git` (docs/11, section B).

## Licence

MIT for code and CAD source, CC BY 4.0 for documentation. Change it if your school or fair needs something else.
