# 14 — Assembly manual (numbered)

**Tools:**
- metric hex keys 2–4 mm, 8 mm wrench, small torque driver (0.5–5 N·m)
- soldering iron with heat-set insert tips
- 8 mm reamer (or 7.9 + 8.0 drills), deburring tool, tube cutter
- spirit level, calipers, threadlocker (medium, *not* on plastic threads), 5-minute + structural epoxy, PTFE dry lube

**Torques (into brass inserts):**

| screw | torque |
|---|---|
| M3 | 0.6 N·m |
| M4 | 1.2 N·m |
| M5 | 2.0 N·m |
| M5 tube-clamp bolts | 2.5 N·m, tightened evenly |

**Never** tighten screws that thread directly into plastic beyond snug.

Part numbers refer to `output/stl/<number>_*.stl`; quantities and fasteners are in `output/reports/BOM.md`.

## A. Print and prepare (≈ 60–80 h of printing)

1. Print `00_fit_test_coupon` first. Check the bearing seat, pin press/slip and insert pilots. Adjust `fit.*` in `params.yaml` and re-run `python cad/build.py --parts` until all of them fit.
2. Print everything else as described in docs/13 (orientation, material, walls, infill). Use PA-CF or PETG-CF for every part docs/13 marks load-bearing. Dry the filament.
3. Install every heat-set insert: the per-part count is in the BOM table. Ream the 8 mm pin bores: press fits by hand-reaming, slip fits by drill.
4. Cut the stock:
   - aluminium tubes 12×1 — 2 × 182.0 (upper arm), 2 × 83.0 (forearm), 1 × 15.71 (elbow spacer)
   - rod tube 10×1 — 222.0
   - 15 mm h6 shaft — 94.0
   - 10 mm h6 shaft — 56.25
   - stainless 8×1 tube — 53.5 (ABENICS post)
   - MGN9 rail — 124

   Deburr and check each against the BOM (`cut_lengths_mm`).

## B. Base, dock and J1

5. Press the 2× 6×20 dowels into `01_receiver` and fit the dock microswitch (2× M2 self-tapping). This part goes on the chair later.
6. Press the 2× 6806 face-to-face into `10_base_housing`. Fit `11_j1_fixed_pulley` (4× M3 countersunk).
7. Insert the turntable spindle (`20_turntable`) through the bearings from above. Fit the 30 mm clamp collar from below, snug, with no end play. Fit the J1 hard-stop pin (6×12).
8. Assemble the J1 drive module:
   - 2× 608 in `12_dm_pulley36`, on the 8×24 dowel pressed into `12_dm_bracket`
   - servo ID 1 in the pocket, horn bolted to the pulley (4× M3×8)
   - cap `12_dm_cap`
   - hang it under the turntable on its slotted M4 screws with the 240 mm belt round the fixed pulley
   - tension with the pull screw until the long span deflects ≈ 3 mm under 5 N, then lock

## C. Shoulder (J2, J3) and upper arm

9. Assemble the two halves of the dual drive module (`22_ddm_bracket_j2/j3`, servos ID 2 and 3, pulleys, 8×24 pins). Bolt them behind the shoulder with slotted feet; do not tension yet.
10. Clamp the 15 mm stub axle in `21_stub_cheek` (2× M4 pinch). Stack on it: `30_ua_hub` (2× 6802) → stub spacers `35_*` → `31_j3_hub` (2× 6802) → shims → 15 mm collar. End play 0.05–0.15 mm.
11. Fit the 345 mm (J2) and 321 mm (J3) belts and tension both as in step 8.
12. Bond (structural epoxy) and cross-bolt (M3×20 through 3.4 mm drilled holes) the two 182 mm tubes into `30_ua_hub` and `32_elbow_block`. Check the elbow pin bore is parallel to the stub (≤ 0.3 mm over the length) before the epoxy cures.

## D. Elbow, forearm, parallelogram

13. Press 2× 6800 into `34_forearm_root`. Slide it onto the 10 mm elbow pin with the 15.71 mm tube spacer and the 10 mm collar. Clamp the pin in the elbow block (2× M3 pinch).
14. Rod ends `33_rod_end` (626 bearings) on 6×10 shoulder screws: crank P on `31_j3_hub`, crank Q on `34_forearm_root`. Bond and cross-bolt the 222 mm rod tube. Set the length so the forearm stays parallel to the J3 crank in two poses 90° apart (± 0.5°).
15. Forearm tubes (2 × 83 mm) into `34_forearm_root` and the wrist housing `89_abenics_housing` (WRA-1B: `40_wrist_housing`). Bond and cross-bolt.

## E. ABENICS wrist (WRA-1A) — see docs/06 §6 for details

16. Monopoles: 2× 688 each, spacer `93` on the 8×30 pin, between the fork arms of `83_mp_fork`. Press the pin into the −x arm. Check free spin and 0.1–0.3 mm end play.
17. 6808 onto the fork ring (far side → near shoulder). Fork + bearing into `87_module_frame` from the far side. Retainer `91` (3× M3 countersunk).
18. Cradle `84_mp_cradle` sleeve over the fork ring, 2× radial M3 countersunk. Θ-servo (ID 5 / 7) with the 20T gear `85` on its horn at servo **2048**, into the cradle, meshed. Strap `92`.
19. 6805 on the cradle hub, into the frame boss. q-coupling `86` on the q-servo horn (ID 4 / 6) at **2048**. Engage the dogs and fit the q-servo, then the cap `88` with 4× M3×12 thread-forming screws into the bosses on `87`.
20. Module frames onto the housing pads (2× M4 low-head each). Post (8×1 stainless) clamped in the housing (M3 slit bolt). Centre ball `90` bonded and pinned (2 mm) on the post.
21. Close the CS gear halves `80`/`81` round the ball (2× 3×12 dowels, 4× M3×16). Before tightening, mesh both monopoles with the tool on the home marks. Lubricate (PTFE).

## F. Claw

22. MGN9 rail on `50_claw_palm` (M3×8 every 20 mm, reference edge toward the pinion). Carriages on the rail.
23. Claw servo (ID 8) into the palm cradle, then the `51` cap with 4× M2×10 thread-forming screws in the side walls (snug only: the walls around the pilots are about 1 mm thick). Pinion `52` on the horn at **servo 2048 = jaws half open** (16 mm each). Engage racks A (top) and B (bottom) symmetrically, then bolt the jaws `53_a/b` to the carriages (4× M3×6 each — check the carriage thread depth).
24. Glue 1.5 mm silicone onto the TPU pads `54_a/b` and screw the pads in (M3×16 + nut).
25. `56_tool_adapter` onto the tool flange (4× M3×10, 4 mm dowel). Slide the claw down the dovetail and fit the M4 lock.

## G. Springs and wiring

26. Spring cords (Dyneema) and springs as specified in `calcs/output/springs.json` (J2: 2× 1.12 N/mm, J3: 2× 1.55 N/mm). Set the cord length so that with torque OFF the arm floats and drifts slowly from any pose (test T2.2).
27. Wiring per docs/07: power star on the turntable, data daisy-chain, Θ-servo service loops (one turn of 26 AWG silicone lead around each module), claw lead through the post.

## H. Chair installation (docs/03, docs/12 measurements first)

28. Post bracket `04` + 2 clamps `02/03` (+ liners) on the front post, shelf outboard, at `mount_z`. Stop collar `05` under the lower clamp. Steel tether to the frame.
29. Third clamp on the lower rail, torque strut (SI5 rod ends + M5 rod), adjusted unstressed. Receiver `01` on the shelf (index 0, 4× M4).
30. E-box `60/61` on the rear cane (2 clamps). Pendant `62/63` within reach. Route the arm cable along the frame, clear of the wheels and brakes.
31. Dock the arm (dowels, 2 thumb screws). The dock switch must read *docked*.

## I. Bring-up

32. Software setup and calibration: docs/11.
33. Run the test stages T0–T4 (docs/09) and log the results before anyone sits in the chair with the arm powered.
