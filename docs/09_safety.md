# 09 — Safety concept, FMEA and staged test plan

> **Status: research prototype, NOT validated for human-facing use.**
> No structural, electrical or operational testing has been done yet. Treat every claim here as a design intent until the tests in §4 pass and are documented.
> The ESP32 supervisor is not a certified safety controller. The design borrows the *structure* of ISO 13849-1 and IEC 60204-1 (Category 0 stop, manual reset, monitored contacts) without claiming a Performance Level.
> Never run a loaded arm over or near a person before the bench stages (T0–T3) pass.

## 1. Hazards considered

| # | hazard | main mitigations (design) |
|---|---|---|
| H1 | Impact / crush (face, hands, lap) | Low force by design: 30 kg·cm servos derated to 25 % (50 % peak) by firmware **and** by servo EEPROM (Max_Torque 50 %). Belt reductions slip-free but compliant. Joint speed ≤ 30°/s, TCP ≤ 80 mm/s in software. Soft limits in 3 places (Jetson, ESP32, servo EEPROM). Park pose folded upright and outboard. Hard-wired E-stop within reach. |
| H2 | Pinch points | **Fingers:** claw closing force ≤ 35 N (70 N for ≤ 10 s in knob mode only). **Open gears:** printed covers recommended over the open belts and gears (TODO in the CAD). **Elbow:** the parallelogram can shear at the elbow. **Wrist:** ABENICS teeth are exposed — keep hair, fingers and clothing away and fit a cover sleeve before user trials. |
| H3 | Arm falls when power is cut | Springs balance about 90 % of gravity, so the arm sags slowly. Belts and gears give friction. The park pose is near the gravity-neutral position. |
| H4 | Unexpected start | Restart always needs START, which is not latched. The ESP32 needs an explicit ARM from the host. Torque turns on with goal = present position (no jump). HOLD after a host timeout needs an explicit resume. |
| H5 | Software runaway on the Jetson | The ESP32 clamps every goal to the calibrated window and limits speed. Host heartbeat 250 ms → HOLD. E-stop is independent of all software. |
| H6 | ESP32 hang/crash | Task watchdog (2 s) → reset → GPIO25 pull-down → K0 opens → K1, K2 drop. |
| H7 | Relay weld / wiring fault | `BUS_POWER_UNEXPECTED` check. Use a 2-NC E-stop: the second contact goes directly in the K2 coil path. |
| H8 | Electrical: battery fire, shorts | Fused branches. The arm's own BMS pack (a manual chair has no battery). 14 AWG power wire. XT60 connectors. TVS clamp. Don't charge it on the chair unattended. |
| H9 | Servo overheating | Firmware warns at 60 °C and faults at 70 °C. Continuous torque limit 25 %. |
| H10 | Interference with wheelchair functions | Arm and mount checked against wheels, hand rims, wheel locks, footrest hanger and user envelope. On a manual chair also check the push stroke on the hand rims, swinging the footrests away, and folding the chair (T4.2). That check used a **provisional** chair — re-run with your measurements (docs/12). The dock releases for transfers. A tether catches an accidental release. |
| H11 | Chair stability (tipping, kerbs) | Mount close to the frame. Arm ≈ 3 kg. Park the arm before kerbs and thresholds. The arm sits over the front caster on its side, which makes pushing harder and can pull the chair toward that side: undock it when it isn't needed (T5.3). Do not lean on the arm or use it for support. |
| H12 | Door tasks: a door that won't release, or closes on the arm | Claw torque time-limited. The task aborts on any fault or stop; the arm holds and the claw keeps its grip until the user decides. Only crack doors a few degrees; the user pushes the door the rest of the way. |
| H13 | Voice false trigger | Motion commands need a wake word. "Stop" is always accepted. Voice is never the only stop. |
| H14 | Dropped object | Payload ≤ 0.3 kg at reach. Parallel claw with lips. No liquids above the lap during tests. |
| H15 | Chair rolls during a task | A manual chair has no motor brakes, and pulling a door can roll an unlocked chair toward it. Set **both wheel locks** before any arm task; `wra_cli.py door` asks for it. Nothing senses the wheel locks (open item), so this depends on the user. |

## 2. Stop functions

| stop | trigger | effect | category (IEC 60204-1 sense) |
|---|---|---|---|
| E-stop | pendant mushroom | K1 → K2 drop, servo power off | 0 (uncontrolled) |
| Undock | dock microswitch | same as E-stop | 0 |
| Supervisor fault | critical fault bits (docs/08) | SAFE_OK low → relays drop; ESP32 also disables torque | 0 |
| Host silent | no frame for 250 ms | HOLD: servos keep the present position, goals rejected | 2 (controlled, power on) |
| User stop | `/wra/stop`, voice "stop", CLI Ctrl-C | motion/task cancelled; arm holds | 2 |

## 3. FMEA (design + process)

S = severity, O = occurrence, D = detection (1 best … 10 worst). RPN = S·O·D, before → after the listed action.

| item / failure mode | effect | cause | S | O | D | RPN | action (status) | RPN after |
|---|---|---|---|---|---|---|---|---|
| Servo seizes / gears strip (J2, J3) | arm drops / stops | overload, wear | 6 | 3 | 4 | 72 | springs carry ~90 %; overload status → fault; load telemetry trend in the maintenance log | 36 |
| Belt tooth jump or break (J1–J3) | joint uncontrolled within the spring balance | low tension, wear | 6 | 3 | 5 | 90 | tension check each session (docs/15); position-deviation check in the driver (TODO firmware) | 45 |
| Printed part cracks (bracket, hubs, CS gear) | arm detaches / drops | layer adhesion, creep | 8 | 3 | 5 | 120 | SF 4 on printed structure; PA-CF for load parts; tether; inspect each session; T3 proof load ×2 | 48 |
| Dock thumb screw loosens | arm tilts / falls | vibration | 8 | 3 | 3 | 72 | dowels carry shear; dock switch cuts power if lifted; tether; captive O-ring | 32 |
| Tube clamp slips on the chair post | arm rotates on the post | low clamp force | 7 | 3 | 4 | 84 | torque strut (positive retention); stop collar; witness marks | 28 |
| ESP32 firmware bug: goal outside limits | collision | coding error | 7 | 3 | 4 | 84 | servo EEPROM angle limits independent; T1 limit tests; unit tests | 42 |
| Jetson freezes mid-motion | arm keeps the last goal | software | 5 | 4 | 2 | 40 | 250 ms heartbeat → HOLD | 20 |
| ESP32 freezes | no supervision | firmware / EMI | 7 | 2 | 3 | 42 | task watchdog 2 s → reset → SAFE_OK low (pull-down) | 21 |
| K0/K1/K2 contact welds | E-stop ineffective | inrush, wear | 9 | 2 | 6 | 108 | 2-NC E-stop with the second NC in the K2 coil path; `BUS_POWER_UNEXPECTED`; monthly E-stop function test (T0.4) | 36 |
| E-stop cable cut | E-stop ineffective? | damage | 9 | 2 | 2 | 36 | NC chain: an open circuit = stop (fail-safe by design) | 36 |
| Calibration wrong (dir / center) | joint moves the wrong way / to the wrong place | user error | 7 | 4 | 3 | 84 | `servo_setup` verifies by reading back; T1.3 jog test at 10 % speed with a hand on the E-stop | 42 |
| ABENICS branch flip mid-motion | wrist whips round 180° in q | planner hitting the q-window | 6 | 3 | 5 | 90 | IK keeps q continuous, refuses moves that need a flip outside Θ ≈ 0; q speed limit | 45 |
| ABENICS tooth skip / wear | tool orientation jumps | overload, wear, under-lubrication | 5 | 3 | 5 | 75 | Θ torque ≤ 30 % for knob turning (firmware cap); inspect teeth; PTFE lube | 45 |
| Claw slips on the knob | door not unlatched | low friction (glazed pad) | 3 | 5 | 2 | 30 | task checks the grip width; replace silicone facing (docs/15) | 15 |
| Claw pinches the user's finger | injury | user reaches into the claw | 6 | 3 | 4 | 72 | 35 N cap; knob mode only inside the door task; voice/E-stop; label | 48 |
| Voice misrecognition | unwanted motion | noise | 5 | 4 | 3 | 60 | wake word + fixed grammar; "stop" always active | 30 |
| Battery short | fire | damage | 10 | 1 | 4 | 40 | fuses at the source; BMS; enclosure | 30 |
| Arm hits the chair / wheel / hand rim | damage, entrapment | pose outside the checked set | 6 | 3 | 4 | 72 | chair check on YOUR measurements; park pose; J1 soft limits per side (TODO: side-specific J1 window) | 36 |
| Chair rolls during the door task | chair creeps toward the door; footplates hit it; user jolted | wheel locks not set (a manual chair has no motor brakes) | 5 | 4 | 6 | 120 | wheel-lock step in docs/10 and the `wra_cli.py door` prompt; torque limits make the arm stall rather than pull hard; the task aborts on a fault. Nothing senses the wheel locks yet | 60 |

## 4. Staged test plan (entry → test → pass criteria)

**T0 — electronics on the desk, no servos**

| step | test | pass criteria |
|---|---|---|
| T0.1 | power-up | ESP32 LED blips (SAFE_OFF); STATE frames arrive (`wra_cli status`) |
| T0.2 | E-stop chain with a lamp in place of the servo bus | ARM → START → K2 closes; E-stop opens K1, K2 within 50 ms |
| T0.3 | fault injection | unplug USB → (in ACTIVE) HOLD; reset the ESP32 → K0 opens |
| T0.4 | welded-relay simulation | jumper K2 closed → `BUS_POWER_UNEXPECTED` |

**T1 — single servo on the bench** (bracket in a vice, no load)

| step | test | pass criteria |
|---|---|---|
| T1.1 | assign IDs | `servo_setup.py scan / set-id` |
| T1.2 | telemetry | voltage 11.5–12.6 V; temperature stable over 10 min holding |
| T1.3 | limits | goals outside the window are clamped (ACK 3); EEPROM angle limits stop the servo even with the firmware limits widened |
| T1.4 | torque limit | stall the horn by hand: current ≤ ~0.8 A at 25 % |

**T2 — complete arm on `70_bench_test_stand`** (no chair, no person within 1 m)

| step | test | pass criteria |
|---|---|---|
| T2.1 | calibrate all joints | `servo_setup.py calibrate`; jog each joint ±10° at 10 % speed; directions correct |
| T2.2 | spring balance | with torque off, the arm drifts slowly (< 10°/s) from any pose; adjust cord lengths |
| T2.3 | workspace walk | 20 random reachable poses; no collisions (compare with `tools/sweep.py`) |
| T2.4 | accuracy / backlash | dial indicator at the TCP, ±1 N reversal; record vs. the budget in docs/05 |
| T2.5 | ABENICS | pitch/yaw/roll sweeps to the soft limits, both q-branches; listen for tooth skip |

**T3 — loads and faults on the bench**

| step | test | pass criteria |
|---|---|---|
| T3.1 | rated payload | 0.3 kg at max reach, 10 min hold: servo temperature < 60 °C |
| T3.2 | proof load (unpowered) | 2 × rated payload hung at the TCP with the arm locked: no cracking or permanent deflection > 1 mm |
| T3.3 | E-stop during motion at full speed | stop within 0.5 s; no damage |
| T3.4 | knob task on a door fixture on the bench | 50 cycles; grip width, roll torque and abort paths all exercised (pull the USB mid-task, freeze the latch to trigger the "did not turn" abort). Afterwards inspect the Θ-pair and monopole teeth: no white stress marks, chips or new play. The Θ pair runs at margin 1.0 at the 30 % knob cap |

**T4 — on the chair, nobody seated** (wheel locks on)

| step | test | pass criteria |
|---|---|---|
| T4.0 | build match | `wra_cli.py status` prints `geometry_params_sha` (the driver also logs it at start-up). It must equal `params_sha` in `output/reports/verification.json` of the build you checked with YOUR chair measurements. If it differs, the chair → arm transform and the user keep-out boxes on the Jetson belong to other measurements: update the Jetson (docs/11 §G) |
| T4.1 | mount | clamp + strut torque check; mark witness lines; tether fitted |
| T4.2 | installation clearance | park and move through the workspace: clearance to wheels, rims, wheel locks and footrests ≥ 10 mm; LEFT and RIGHT if both are used. With the arm parked, a full push stroke on both hand rims touches nothing (hands and knuckles ≥ 25 mm clear); the footrest on the arm side still swings away; with the arm undocked, the chair still folds |
| T4.3 | chair stability | arm at max reach with payload; tilt the chair 6° (ramp): no tipping tendency |

**T5 — seated user, supervised** (only after T0–T4 pass; a second person at the E-stop)

| step | test | pass criteria |
|---|---|---|
| T5.1 | user zone | check the software keep-out: with the side set (`side:=right/left`, calibration.yaml), the driver refuses any target or path point that puts the arm (upper arm, forearm, wrist, flange, TCP) inside the head/torso boxes of `user_envelope` + 50 mm. Command a pose toward the torso and confirm the refusal; re-measure the boxes for the real user. Speed 50 %; the user practises the E-stop |
| T5.2 | tasks | both wheel locks on: pick a light object from a table, then the door task on the fixture, then a real door with the attendant holding it |
| T5.3 | pushing the chair | arm docked and parked: push 10 m in a straight line on a level floor and turn both ways. Note the extra effort and any drift toward the arm side; if it is a problem, undock the arm for long pushes |
| T5.4 | record | completion time, interventions, discomfort; stop criteria: any unplanned contact with the user |

Record results in `docs/test_log.md` (template: date, build version / `params_sha`, test, pass/fail, notes). Re-run the relevant stage after any hardware, firmware or parameter change.
