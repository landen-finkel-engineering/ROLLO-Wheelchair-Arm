# 03 — LEFT / RIGHT interchangeability

**Design rule:** the arm (every link, joint, the ABENICS wrist and the claw) is **one design** that is never mirrored. Only the *mounting* changes side.

**Side-specific printed parts: none.** The post bracket, receiver, tube clamps, electronics box and pendant are the same parts on both sides; the bracket and the box are only *placed* differently.

## 1. What is different per side

| item | RIGHT | LEFT | where it is set |
|---|---|---|---|
| post bracket `04` | on the right front post, shelf pointing **outboard** (yaw −90°) | on the left front post, shelf pointing outboard (yaw +90°) — same part, rotated 180° about the post | `mount.bracket_yaw_deg` |
| receiver `01` on the shelf | index 0 → arm front mark points forward | index 0 (same) | `mount.receiver_yaw_deg` (24-hole index ring, 15° steps) |
| torque strut | rod end in the bracket clevis → 3rd clamp on the right lower rail, behind the post | same parts on the left rail. The clevis bolt is vertical, so the strut can swing rearward on either side | `chair.strut_clamp_x` |
| electronics box `60/61` | right rear cane, lid to the rear | left rear cane, box turned upside-down (symmetric clamp pads) | `chair.ebox_center_z` |
| E-stop pendant | where the user (or the attendant) reaches it | same | — |
| software | `side: right` | `side: left` | `config/calibration.yaml` + launch arg `side:=` |

What `side` changes in software:
- the static transform chair → base_link (from `arm_geometry.json`);
- the direction "inboard" for the park pose (J1 parks 20° inboard so the drive modules clear the wheel lock);
- mirroring of taught tasks: `mirror_q` maps J1 → −J1 and wrist (pitch, yaw, roll) → (pitch, −yaw, −roll). Positions taught in the base frame are mirrored about its x–z plane.

Joint calibration lives **in the arm** (servo EEPROM, the ESP32's NVS and `calibration.yaml`). It does not change when you switch sides.

## 2. Switching sides (10 steps, no reprinting, ~30 min)

1. Park the arm, power down (`wra_cli power-down`), and switch off the battery.
2. Undo the two DIN 464 thumb screws and lift the arm off the dock (dowels + slot). Put it on the bench stand `70`.
3. Remove the torque-strut bolt at the rail clamp, then the two post clamps (M5). Keep the liners with the clamps.
4. Fit the bracket on the other front post, shelf **outboard**, at the same height. Use your measured `chair.mount_z`; transfer the witness mark. Tighten the M5 clamp bolts evenly to the torque in docs/14.
5. Fit the stop collar directly under the lower clamp, and the safety tether to the chair frame.
6. Fit the 3rd clamp on that side's lower rail at `chair.strut_clamp_x`. Connect the strut and adjust its length so it is unstressed with the bracket square. Then tighten the jam nuts. The strut is the **positive** anti-rotation lock; clamp friction is only the backup.
7. Check the receiver index (0). Move the e-box to the other rear cane (box upside-down; same clamps) and re-route the arm cable along the frame.
8. Set `side: left` (or `right`) in `config/calibration.yaml` and use `side:=left` in the launch command.
9. Dock the arm and fit the thumb screws. The dock switch must read **docked** (`wra_cli status`).
10. Run installation test **T4.2** (docs/09) on that side. Re-teach or mirror any taught door poses.

## 3. Verification in the CAD

`cad/tools/chair_check.py` builds the RIGHT and LEFT installations on the provisional chair. It checks for interference between the arm, mount and e-box and the chair and user envelope, and reports the minimum clearances to the wheels, hand rims, wheel locks and footrest hangers, plus the added width. Results are in `output/reports/chair_check.json` and in the verification report.

The STEP assemblies `WRA1_RIGHT_install_park.step` and `WRA1_LEFT_install_park.step` show both installations. The renders `install_right_*`, `install_left_*` and `install_both_sides_*` show the same arm on both sides.

**Caveat:** the chair model is provisional. Only the published Drive Silver Sport 2 data is real (seat width/depth/height, 8" casters). Every clearance has to be re-checked with your measurements (docs/12).
