# 06 — ABENICS 3-DoF wrist (WRA-1A, default)

The wrist is an **ABENICS** active ball joint (Abe, Tadakuma & Tadakuma, *IEEE Transactions on Robotics* 37(5):1806–1825, 2021, DOI 10.1109/TRO.2021.3070124). The CAD for it is in `cad/wra/parts/abenics.py` (solid-model parts) and `cad/wra/parts/abenics_mesh.py` (toothed gears). The kinematics are in `cad/wra/abenics_kin.py`, which is copied unchanged into the ROS 2 driver.
The pitch-roll wrist (WRA-1B) is kept as a fallback (`wrist_type: pitch_roll` in `params.yaml`). It has the same forearm tube sockets and the same tool-flange interface (WRA-TF30).

## 1. How it works

| element | WRA-1A value | role |
|---|---|---|
| Cross-spherical gear (CS) | m 1.5, 34 teeth on a great circle → pitch radius ρ = 25.5 mm, tip 27 mm | output sphere; carries the tool flange. Involute **ring teeth about body axes g and h** (two orthogonal families), cut only in the band 45°–135° from the tool axis |
| Monopole gears (MP) ×2 | m 1.5, 17 T → r = 12.75 mm, centre distance 38.25 mm | each sits on a fixed module axis u₁ = (0, ½√2, ½√2), u₂ = (0, −½√2, ½√2) in the housing frame (45° either side of vertical, normal to the tool axis) |
| Module k actuators | q-servo (carrier angle q about u_k) + Θ-servo (monopole spin Θ, 20T→15T m 1.5 spur pair, 4.5 mm wide: servo angle = 0.75·Θ) | 2 servos per module, 4 in total → 3 DoF + 1 redundant |
| Centre ball | Ø19 mm, fixed on an Ø8×1 post through the back of the sphere | takes all **force** on the sphere so the teeth only transmit **torque** |
| Tool flange | WRA-TF30: Ø40, 4×M3 on BC30, Ø4 clocking dowel, 56 mm from the sphere centre | same interface as WRA-1B |

The monopole is an S-gear: a 2-D involute profile revolved about its pole axis, then cut to a 10 mm disc.
- **Spinning** the monopole by Θ about its axis I_m rolls the sphere's ring teeth. That tilts the sphere's body axis by **−Θ/2** about I_m (2:1 because ρ_CS = 2·r_MP).
- **Rotating the carrier** by q swings I_m around u_k, and with it the tilt direction.
- Rotation of the sphere **about** the ring axis is free for that module. So module 1 positions body axis g, module 2 positions body axis h, and together they fix the orientation.

## 2. Kinematics (`abenics_kin.py`)

Forward: `g = Rot(I_m1(q1), −Θ1/2)·u1`, `h = Rot(I_m2(q2), −Θ2/2)·u2`, `i = g × h`.
Here I_m(q) = cos q·x_M + sin q·y_M, and the module frame M_k has z = u_k and x = tool axis at home.

Inverse, per module, for the required body axis `col` (g or h):
- α = acos(u·col) and n = u × col.
- Branch 1: q = atan2(n·y, n·x), Θ = −2α.
- Branch 2: q + 180°, Θ = +2α.
- The solver picks the branch inside the carrier window **q ∈ [−120°, +120°]** that is closest to the previous q. FK(IK(R)) − R < 1e-14 (self-test).

At α = 0 (col = u) q is undefined, and the solver keeps the previous q.
A path that circles that singular axis needs a full 360° of q. The window then forces a **branch flip** (q → q ± 180°, Θ → −Θ), which can only be done where Θ ≈ 0. The ROS driver warns when the window is reached. Plan wrist motions so they pass through the home orientation instead of circling it.

User-facing wrist coordinates are **(pitch, yaw, roll) of the tool relative to the forearm**: R = Rz(yaw)·Ry(−pitch)·Rx(roll)·R_home. Pitch is + up, yaw is + toward module 1, roll is about the tool axis.
For a task mirrored from RIGHT to LEFT, (p, y, r) maps to (p, −y, −r); that is `chair.mirror_q()`.

**Statics** (`actuator_torques`):
- Each module transmits torque about its actuated axis I_m (Θ-servo) and about its passive axis u × I_m, which the carrier bearings react.
- No module transmits torque about u_k.
- The q-servos therefore carry **no static load from the tool**: the mesh forces act on the module axis. They only see carrier gravity, under 0.03 N·m.
- Sphere torque per module = 2 × monopole torque = 2 × 0.75 × Θ-servo torque = **1.5 × Θ-servo torque**.

## 3. Limits (soft / hard)

| limit | soft | hard | origin |
|---|---|---|---|
| tilt of the tool axis from the forearm axis | 30° | 33° | tooth band (45° = 33° + 12° contact margin), post opening in the back hemisphere |
| tilt **toward a module axis** (up-left / up-right diagonals) | 21° | 24° | module stacks vs. the claw's sweep cone (§5) |
| roll | ±90° | — | Θ-servo travel: \|Θ\| ≤ 220° soft / 236° hard (servo = 0.75·Θ, within one turn) |
| carrier q | ±120° | ±125° | the Θ-servo cable service loop; the module strut sits in the unswept sector |

Pure pitch and pure yaw reach 33° hard. The up-diagonals (toward a module) stop at 24°; the down-diagonals reach 33°. See `calcs/output/abenics_direction_workspace.png`.

## 4. Making the teeth (why there are meshes)

The CS gear is the **intersection** of two S-gears whose ring teeth cross. A B-rep boolean of that in OCCT grew past 4 GB and 30 min, so `abenics_mesh.py` builds the teeth with manifold3d (exact mesh booleans) on top of the B-rep blanks. The blanks have the joint screws, the flange and the socket.

The **monopole is generated, not drawn**. A plain S-gear monopole was tried first: off the rolling plane, at body-axis tilts above about 60°, the equatorial ridges of the other tooth family hit it. So the monopole blank is cut by sweeping the CS ring teeth through the real relative motion, T(Θ) = Rot_x(−Θ)·Trans(0,0,−D)·Rot_x(−Θ/2), in 1° steps over Θ ∈ [−180°, 180°]. This is a hobbing simulation, as for the original MP gear.

| mesh | triangles | volume |
|---|---|---|
| CS front half | ~207 k | ~53.6 cm³ |
| CS back half | ~184 k | ~18.7 cm³ |
| monopole (incl. the 15T Θ gear with tip relief) | ~372 k | ~4.5 cm³ |

All three are watertight. The STEP files contain the tooth-less blanks (labelled as such). **Print from the STL.**

**Θ-gear tip relief.** Each monopole also carries the 15T Θ spur gear, 6–10.5 mm off its mid-plane. The inner tip corner of that gear (r = 12.75 at x = −6) sits 26.2 mm from the sphere centre, which is inside the CS tooth tips (27.0 mm). At the wrist limits it clipped the CS teeth by up to 0.76 mm³. The fix is a revolved tip relief: the gear is cut by the torus that the CS tip sphere (+0.4 mm) sweeps as the monopole spins (`TG_RELIEF_CLR` in `abenics.py`).
- At the inner face, 0.27 mm of addendum remains, with a contact ratio of about 1.0 in that slice.
- The outer face keeps the full tooth, with a contact ratio of 1.5.
- The tooth roots are untouched, so the Lewis check (docs/05) is unchanged.

Mesh check (`tools/wrist_sweep.py --mesh`): CS gear vs. both monopoles (each with its Θ gear) at 73 poses on the hard-limit boundary.
- The largest overlap is 0.20 mm³. It is made of slivers no thicker than 0.27 mm on the monopole tooth flanks, which is the 1° step of the generating sweep; the design backlash is 0.15 mm.
- Before the tip relief, the same check found the Θ gear clipping the CS teeth (0.76 mm³).
- A deliberate half-tooth phase error produces 15 mm³ of overlap, which confirms the check detects real interference.

## 5. Module packaging (the redesign)

Each module stacks along u_k:
- monopole at u = 38.25;
- 6808 thin-section bearing at u = 43–50, around the monopole;
- Θ-servo cradle at u = 54–104;
- 6805 floating bearing at u = 104.5–111.5;
- dog coupling;
- q-servo, out to u ≈ 157 mm.

The first layout put the 6808 at u = 26.5–33.5, right beside the sphere. It reached 50° from the module axis and hit the claw at 17° of tilt.
The fix was driven by a **cone-envelope analysis**: the tool envelope φ_t(ρ) is the largest angle from the tool axis of any claw or CS point at radius ρ. Every module point must stay clear of φ_t at every allowed tool direction.

| part | requirement | result |
|---|---|---|
| carrier part 1 `83_mp_fork` | stays inside r = 21.75 so the fork with its bearing drops into the holder | two arms carry the 8×30 monopole pin; ring takes the 6808 inner race on a near-side shoulder |
| carrier part 2 `84_mp_cradle` | clamps the inner race | a sleeve does this, with 2× radial countersunk M3 |
| Θ-servo (in the cradle) | must not sweep into the claw's cone | flipped so its body sits over the module axis: radius ≤ 37 mm instead of 51 mm |
| far end | — | hub on the floating 6805; two dog slots driven by `86_q_coupling` on the q-servo horn (torque only) |
| `87_module_frame` | — | 6808 holder with near lip, retainer `91`, lug to the housing, strut at r ≥ 42 behind the carrier sweep, 6805 boss, q-servo cradle |

Minimum angular margins at the hard limits:

| region | margin |
|---|---|
| carrier | 4.3° |
| fixed module parts | 5.8° |
| monopole beyond the sphere | 13.7° |

Then the full solid-model sweep: 73 poses (home + 12 tilt directions at their hard boundary × roll −90/0/+90 × claw closed/open) with all printed parts and vitamins. Result: **no interferences** (`output/reports/wrist_sweep.json`).

## 6. Assembly notes (wrist)

1. Press the 2× 688 into each monopole, drive-gear side first. Slide the 10.5/8.2 spacer `93` onto the 8×30 pin.
2. Fit the monopole between the fork arms. Press the pin into the −x arm (it slips through the +x arm). Check that the monopole spins freely with 0.1–0.3 mm end play; shim with DIN 988 8×14.
3. Slide the 6808 onto the fork ring from the far side, against the near shoulder. Insert the fork + bearing into the module-frame holder from the far side. Fit the retainer `91` (3× M3 countersunk).
4. Slide the cradle sleeve over the fork ring, against the inner race. Fit 2× radial M3 countersunk screws (thread-forming).
5. Θ-servo: bolt the 20T gear `85` to the horn with the servo at **2048**. Set the monopole to **Θ = 0** (alignment mark up). The 15T monopole gear has an odd tooth count, so at Θ = 0 a gap faces the servo and a tooth of the 20T gear drops into it. Drop the servo into the cradle from the open side, mesh the gear and fit the strap `92`. Any residual offset (less than half a tooth) is removed by calibration.
6. Push the 6805 onto the cradle hub and into the frame boss. Bolt the coupling `86` to the q-servo horn at **q = 0 / servo 2048**. Engage the dogs and fit the q-servo, then the cap `88` with 4× M3×12 thread-forming screws into the frame bosses.
7. Bolt both module frames to the housing pads (2× M4 low-head each). Bond and pin the centre ball on the post.
8. Close the CS hemispheres around the ball: 2× 3×12 dowels, 4× M3×16. Mesh both monopoles with the tool at the **home alignment marks**.
9. Run `servo_setup.py calibrate --only 4 5 6 7`. Lubricate the teeth and the socket with PTFE dry film.

## 7. Known limitations

- Four servos plus structure make the wrist **245 mm wide × 149 mm tall** (CAD bounding box). The forearm group with the wrist weighs 909 g (`docs/05_calculations.md`), which costs payload at reach. Rated 0.3 kg still holds at the full 593 mm reach.
- The 24° limit toward each module and the ±30° pitch cone keep the forearm near horizontal for knob work. That gives a **knob reach window** (`docs/10_door_tasks.md`).
- Backlash in printed gears (20/15 Θ mesh + CS mesh, 0.15 mm each) gives roughly 1° of tool-orientation play. Measure it on your build (test plan T2.4).
- Teeth stress: the 4.5 mm-wide 20T/15T Θ pair is the weakest link. Its Lewis margin is ≥ 1 up to **30 % of stall**, so the firmware caps Θ torque at 30 % for knob turning (roll ≈ 2.65 N·m) and 25 % otherwise (roll ≈ 2.2 N·m). See `docs/05_calculations.md`. The pair cannot be wider without moving the fork arms (the ring/fork-bore clearance sets the width).
- F. Thomas (ARK 2022, DOI 10.1007/978-3-031-08140-8_35) shows a three-actuator ABENICS variant without the over-actuation. That is a possible WRA-2 simplification that would save one servo.
