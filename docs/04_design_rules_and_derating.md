# 04 — Design rules, derating and fits

These rules are applied in `params.yaml` and the calcs. Change them there, never in a part file.

## Servos (Waveshare ST3215, SKU 22414, 12 V) [V]

| datasheet value | figure |
|---|---|
| stall torque | 30 kg·cm = 2.94 N·m at 12 V |
| no-load speed | 0.222 s/60° ≈ 45 rpm |
| current | 0.2 A no-load, 2.7 A stall; kt ≈ 1.08 N·m/A |
| encoder | 12-bit magnetic, 360° / 4096 |
| mass | 89 g |
| supply range | 6–12.6 V |

| design rule | value | why |
|---|---|---|
| continuous torque | ≤ **25 % of stall** (0.735 N·m) | hobby servos overheat holding near stall. The springs carry gravity, so 25 % is the *working* torque |
| peak torque | ≤ **50 %** for seconds | firmware Torque_Limit ceiling and servo EEPROM Max_Torque = 500 |
| ABENICS Θ servos | ≤ 25 % continuous, ≤ **30 %** for knob turning (≤ 10 s) | Θ-pair (20T/15T m1.5, 4.5 mm) tooth stress (docs/05) |
| loads on the output shaft | **torque only** | every drive pulley spins on its own two bearings and pin, and the horn drives it through bolts and dogs. The ABENICS carriers have their own 6808 + 6805; the q-coupling is a dog drive with radial float. The claw pinion sits on the horn, but the racks are guided by an MGN9 rail, so the pinion carries only the tooth reaction |

## Printed structure

| item | rule |
|---|---|
| materials | PA-CF (or PETG-CF) for load paths, PETG for covers |
| strengths used [E, typical TDS] | PETG 40/25 MPa; PETG-CF 35/29; PA-CF 60/30 (XY / Z) |
| static safety factor | **4** on printed structure (voids, creep, impact) |
| dynamic factor | 3 g on chair-mounted loads (kerbs, door thresholds) |
| printed gears | SF 2 on the XY strength, Lewis bending |
| layer orientation | layers never carry the main tension across Z; see each part's print notes (docs/13) |
| walls / infill | ≥ 5 walls, 40–60 % gyroid for structure; 100 % for gears and pulleys |

## Fits (tune with `00_fit_test_coupon` FIRST)

| fit | value |
|---|---|
| bearing seat (outer ring, light press) | OD + 0.10 |
| bearing on a printed seat (inner ring) | d − 0.05, bonded |
| steel pin, press | d − 0.05 |
| steel pin, slip | d + 0.15 (+0.05 on the monopole +x arm) |
| clearance holes | ISO 273 medium + 0.15 |
| heat-set inserts | CNC-Kitchen standard length; pilots M3 4.0, M4 5.6, M5 6.4, M6 8.0 |
| servo pockets | +0.25 per side |
| sliding printed parts | +0.30 per side |
| tube sockets | tube OD + 0.20; plus a cross-bolt and structural epoxy |

## Belts (HTD-3M, 15 mm)

| item | value |
|---|---|
| allowable working tension | 168 N (Gates) [V] |
| centre distances | computed so the catalogue pitch length matches within 0.05 mm (verification report) |
| tension | screw tensioners on the drive modules; 0.5 × peak effective tension + 10 N static (docs/05) |

## Kinematic limits

| item | rule |
|---|---|
| soft limits (firmware) | inside the hard limits (servo EEPROM and mechanical stops) |
| coupled elbow limit | th2 − th3 ∈ [42°, 140°] |
| ABENICS limits | docs/06 |
| speed (Jetson software) | ≤ 30°/s per joint; TCP ≤ 80 mm/s near the user |

## Clearances

| item | rule |
|---|---|
| moving parts | ≥ 0.5 mm in the CAD (sweeps report interferences above 0.5 mm³) |
| arm/mount vs. chair and user envelope | 0 interferences at park, then measured clearances |
