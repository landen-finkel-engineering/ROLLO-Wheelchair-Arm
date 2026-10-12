# 10 — Door tasks (the headline requirement)

## 1. What a knob needs and why the claw looks the way it does

A round knob is turned by **friction**: the grip force times the friction coefficient times the knob radius. The knob also has to stay **centred on the wrist roll axis** while it turns, otherwise the arm drags it sideways. Hence:

- **Parallel rack-and-pinion jaws.** One 14T pinion (r = 10.5 mm) on the claw servo drives rack A above it and rack B below it in opposite directions. The jaws stay parallel and symmetric, so the knob stays centred. The grip force does not depend on the knob size:
  - jaw force = τ / (2 r) = **35 N per jaw** at the continuous limit (25 % stall, 0.735 N·m);
  - **70 N** in *knob mode* (50 % stall, firmware-limited to 10 s, then a 20 s cool-down).
- **100° V-groove pads** (TPU body + 1.5 mm silicone facing) centre a round knob vertically. The normal force is 1/cos 40° = **1.31× the jaw force** per jaw.
- **Fingertip lips** (6 mm): jaw A's lip on the upper half, jaw B's on the lower half, with a notch for the other jaw's lip. They hook behind the knob rose so the arm can **pull** the door after unlatching.
- **Opening** 0–65 mm pad to pad (typical knobs are 50–60 mm; the stem behind is 20–30 mm). Pads are 41 mm long. The TCP is the pad centre, 88.5 mm in front of the tool flange.

Slip torque at the knob = 2 × F_jaw × 1.31 × μ × r_knob. Wrist roll capacity (ABENICS, both modules) = 2 × 1.5 × Θ-servo torque.

| case | jaw force | Ø56 knob, μ 0.5 (TPU) | μ 0.8 (silicone) | wrist roll capacity |
|---|---|---|---|---|
| continuous grip | 35 N | 1.28 N·m | 2.05 N·m | 2.2 N·m (Θ at 25 %) |
| knob mode (≤ 10 s) | 70 N | 2.57 N·m | 4.10 N·m | 2.65 N·m (Θ capped at 30 %) |

The demand side:
- Residential latches typically need roughly 0.3–1.5 N·m. This is an estimate — measure your doors with a luggage scale on a lever arm.
- BHMA A156.2 caps lever-handle torque at 28 lbf·in (3.2 N·m).
- ADA 2010 §309.4 does not allow hardware that needs twisting of the wrist on accessible routes, which is why public buildings use levers. Knobs remain common in homes.
- **Knob mode covers the typical residential range with margin if the silicone facing is in good condition.** The wrist, not the grip, is the limit: the 20T/15T Θ-gear teeth allow 30 % of stall for knob turning (docs/05), so the firmware caps Θ torque there. That gives ≈ 2.65 N·m of roll — about 1.8× the 1.5 N·m upper estimate, but below the 3.2 N·m BHMA lever maximum. A stiffer knob stalls the wrist at the cap and the task stops instead of overloading the teeth.

**Levers** are easier: the claw closes on the lever bar, about 15–20 mm thick, and the arm pushes it down. The force needed is 22 N max (ADA) at about 70 mm from the spindle. The arm's push-down capability vs. distance is in `calcs/output/door_force_capability.png`.

## 2. The knob reach window (read this before mounting)

With a ±30° wrist cone the forearm must stay within about 30° of horizontal to hold the tool level. That puts the knob in a **window** relative to the arm base. From `calcs/knob_window.py` (pre-grasp 80 mm, grip 28 mm behind the knob face, soft limits):

| knob height above the **dock top** (z_B) | knob distance in front of the J1 axis (x_B) |
|---|---|
| 360 mm | 410–520 mm |
| 410 mm | 420–500 mm |
| 460 mm | 420–470 mm |
| 510 mm | 400–420 mm |
| overall | 310–530 mm |

- Laterally, the knob works from about 50 mm to the arm's right to 100 mm to its left (+y_B). The 16.6 mm offset of the wrist centre makes the right side slightly worse.
- Door swing the arm can follow while holding the turned knob: 5–15°, best around z_B ≈ 400–450 and x_B ≈ 450–490 (right panel of `knob_window.png`).

**Mounting consequence.** Knob height above the floor = dock height + z_B.

| dock (`chair.mount_z`) | knobs that fit |
|---|---|
| 400 mm | up to about 36" (914 mm), right at the top edge |
| **460 mm (default in `params.yaml`)** | 34–38" knobs in the middle of the window ✓ — a 36" knob sits at z_B = 454 |

→ **Mount the dock as high as your post allows, ideally ≈ 460 mm.** Measure your post (docs/12), set `chair.mount_z`, and re-run `python cad/build.py --verify` to re-check clearances.

The CAD check at 460 mm (`output/reports/chair_check.json`) is clean on both sides. That covers the park pose and the arm gripping a 36" knob 450 mm ahead. The closest approaches on that check are:
- footrest hanger 11.5 mm (post bracket)
- wheel lock 16 mm (turntable, knob pose)
- the user's elbows 44 mm

These use the **provisional** chair model.

In that pose the chair is square to the door, and the footplates stop ≈ 215 mm short of the door face (`output/renders/door_knob_right_*.png`). That leaves room for a pull door to swing the 5–15° the arm can follow.

![Gripping a 36-inch knob](../output/renders/door_knob_right_iso.png)

**Positioning the chair:**
- Stop with the arm base 0.42–0.48 m from the door face, measured to the knob.
- Keep the knob within ±50 mm of the arm's straight-ahead line, or up to 100 mm to its left.
- The ROS driver refuses unreachable targets *before* moving, and `/wra/status` reports why.

## 3. The sequence (`wra_driver/tasks.py → DoorKnobTask`)

1. **Pre-grasp:** TCP 80 mm in front of the knob on its axis; claw opened to knob Ø + 20 mm.
2. **Approach** along the knob axis to 28 mm behind the knob face (the pads straddle the widest part).
3. **Close in knob mode.** The claw stalls on the knob at the torque limit. **Grip-width check:** the task aborts unless the measured opening is within −16/+12 mm of the expected knob diameter (no knob, wrong knob, or slipped).
4. **Turn** about the tool axis (default 70°, sign by door handing).
5. **Swing:** follow the door's arc about the hinge in 1° steps while holding the turn, as far as the limits allow (up to 12–15°). This cracks the door open.
6. **Release** knob mode, open the claw slightly and **un-roll**; the open claw turns freely around the knob.
7. **Retreat** along the knob axis, then **park**.

The door is then pushed or pulled further by the chair or by the arm on the door edge. That avoids holding a sprung door with a 0.3 kg-class arm.

Any fault, stop request, E-stop, or a target outside the limits aborts the task. The arm **holds position and keeps its grip**, and the user decides: release with "robot open hand", or reverse.

**Simulation result** (fake ESP32 with the claw blocked at the knob width, knob 460 mm ahead at z_B 400): all four layouts (knob 60 mm left/right × hinge left/right) complete. The door swings 4–12° before the wrist limits stop the swing.

Two cases are automated tests in `software/wra_ros2/wra_driver/test/test_core.py`:
- the full sequence on a normal knob;
- a **stiff latch**, simulated by freezing the Θ servos. Here the task stops after the turn with "knob did not turn far enough", instead of trying to swing a latched door. The check aborts if the wrist ends more than 15° from the turn target (`KnobParams.turn_tol_deg`).

## 4. Inputs for the knob pose

- **Teach:** `wra_cli.py teach-knob`. With torque off, guide the claw onto the knob by hand; the pose is saved and reused for that door and chair position.
- **Vision (future):** publish `/wra/knob_pose`. Your stereo doorknob grasp project can do this; no camera mount is part of this design. Remember that the knob pose is only valid while the chair stays where it was when the pose was measured.

## 5. Limits you should expect

- Heavy or closer-equipped doors: exterior doors and fire doors (15 lbf / 67 N or more) are beyond what this arm should pull. Unlatch, then use the chair.
- Knob heights outside 34–38" or very small knobs (< 40 mm): adjust `mount_z` or use the lever strategy.
- Glazed or dirty pads: friction drops to around μ 0.3 → re-face (docs/15).
