# 02 — Architecture trade study and joint specification

## 1. Constraints that drove the architecture

- **Actuator:** Waveshare ST3215 only. 2.94 N·m stall, used at ≤ 0.74 N·m continuous (docs/04), 89 g, 45 rpm. One servo cannot lift a 600 mm arm, so gravity has to be cancelled and the heavy servos kept near the base.
- **Printer:** 256 mm bed, so every part is ≤ 246 mm. Long members are aluminium tubes joined by printed nodes.
- **Tasks:** doors first (knob turn = roll torque + centring; lever = push-down force; pull = arm force), then reaching and fetching light objects.
- **Chair:** one design for the LEFT or RIGHT front post, a dock for transfers, nothing in the wheel, hand-rim or footrest envelopes.
- **No camera hardware** on the arm (software interface only).

## 2. Candidates

| | A — WRA-1A (chosen) | B — WRA-1B (fallback) | C — direct-drive serial 6-DoF | D — SCARA + lift column |
|---|---|---|---|---|
| positioning | J1 yaw (belt 1.5:1), J2/J3 from a dual drive module behind the shoulder (belts 2.39:1, 2:1), **parallelogram** so J3 is absolute and its servo stays on the turntable; zero-free-length springs on J2/J3 | same as A | servos inside each joint (SO-ARM100-style, scaled) | vertical lead-screw column + 2 horizontal links |
| wrist | **ABENICS** 3-DoF spherical gear: pitch/yaw ±30°, roll ±90°, 4 servos | pitch (belt) + roll, 2 servos | 3 servos in series | roll only |
| gripper | knob claw (parallel rack & pinion, V-pads, lips) | same claw | — | — |
| servos | 8 | 6 | 6–7 | 4–5 |
| mass beyond J2 | ≈ 1.7 kg (calc) | ≈ 1.2 kg | high: elbow/wrist servos ride on the arm | low |
| knob turning | ✓ roll 2.2 N·m continuous (2.6 at the 30 % knob limit), centred on the roll axis | ✓ J5 direct 0.74 N·m (1.47 peak): low | ✓ | ✓ but reach to 36" knobs is awkward |
| lever / pull | ✓ (springs give J2/J3 headroom) | ✓ | ✗ weak at reach | ✓ |
| tool orientation | full 3-DoF (yaw independent of J1) | 2-DoF (yaw only through J1) | 3-DoF | 1-DoF |
| printability / risk | high complexity: spherical gear teeth, 2-part carriers | proven style: belts, bearings | simple | lead screw + column ≥ 256 mm (split) |
| width at the chair side | wrist 245 mm across the modules (CAD); at park the drive modules, not the wrist, set the outboard extent | slim | slim | bulky column |

**Weighted scoring** (1–5, weights in brackets):

| criterion (weight) | A | B | C | D |
|---|---|---|---|---|
| door-knob capability (0.25) | 5 | 3 | 3 | 2 |
| payload at reach (0.15) | 3 | 4 | 1 | 4 |
| workspace / dexterity (0.15) | 5 | 3 | 4 | 2 |
| printability / build risk (0.15) | 2 | 4 | 5 | 3 |
| user safety: moving mass, pinch points (0.15) | 3 | 4 | 2 | 3 |
| L/R interchangeability (0.10) | 5 | 5 | 5 | 3 |
| cost (0.05) | 3 | 4 | 4 | 3 |
| **total** | **3.85** | **3.70** | 3.25 | 2.75 |

**Decision.** A is the default, as requested: it is the only candidate with independent wrist yaw and a centred roll strong enough for knobs. B is kept as a parameter switch (`wrist_type: pitch_roll`). It uses the same forearm sockets and the same tool-flange interface, so if ABENICS testing fails (tooth wear, backlash, packaging on your chair) the arm still works without reprinting anything else. C was rejected on payload: a 2.94 N·m servo at the elbow carrying the wrist servos leaves almost nothing for the object. D was rejected on knob height and chair-side bulk.

## 3. Joint specification (WRA-1A)

| joint | drive | ratio | range (soft) | torque at the joint, continuous / peak | speed (servo-limited) |
|---|---|---|---|---|---|
| J1 yaw | DM walks around a fixed 54T, HTD-3M-15, 240 mm belt | 1.5 | ±105° | 1.10 / 2.21 N·m | 180°/s (software 30°/s) |
| J2 shoulder | DDM → 86T, 345 mm belt; springs 2×1.12 N/mm | 2.389 | −30…100° | 1.76 / 3.51 N·m + springs | 113°/s |
| J3 forearm (absolute) | DDM → 72T, 321 mm belt, parallelogram; springs 2×1.55 N/mm | 2.0 | −95…60°, th2−th3 ∈ [42, 140] | 1.47 / 2.94 N·m + springs | 135°/s |
| wrist pitch/yaw | ABENICS (Θ-servos via 20T/15T m1.5, ×1.33) | sphere = 1.5 × servo torque per module | tilt 30° (21° toward a module) | 1.10 per module (2.2 roll, both) | 180°/s |
| wrist roll | ABENICS, both modules | — | ±90° | 2.2 / 2.6 (knob limit 30 %) | 180°/s |
| claw | pinion 14T m1.5 on the servo, 2 racks, MGN9 | r = 10.5 mm | 0–65 mm | 35 N / 70 N per jaw | 49 mm/s per jaw |

**Link lengths.**

| quantity | value |
|---|---|
| upper arm L2 | 250 mm |
| forearm L3 | 230 mm (to the sphere centre) |
| sphere centre → tool flange | 56 mm |
| flange → TCP (pad centre) | 88.5 mm |
| shoulder height | 185 mm above the dock |
| max horizontal TCP reach | ≈ 593 mm (docs/05) |

## 4. Why belts, springs and a parallelogram

- **Belts** move the J2/J3 servos onto the turntable. The elbow and forearm then carry no shoulder servos: the moving mass beyond J2 is the tubes, the hubs, the wrist and the claw.
- **Zero-free-length springs** (an extension spring on a cord over an idler, with the anchor geometry set so tension ∝ length) cancel ~90 % of the gravity torque in every pose. That leaves the servo torque for the payload and the door.
- **The parallelogram** makes θ3 absolute, so the J3 spring balances the forearm independently of J2.
- **Every servo carries torque only.** Drive pulleys spin on their own bearings on fixed pins; the horn just turns the pulley.
