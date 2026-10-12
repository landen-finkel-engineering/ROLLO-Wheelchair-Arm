# 15 — Maintenance

| when | check | action if not OK |
|---|---|---|
| **every session** | E-stop: press → servo power off, LED rapid flicker; twist-release + START needed | do not use the arm until fixed |
| every session | dock thumb screws tight; tether attached; strut jam nuts tight | re-tighten; replace a cracked clamp |
| every session | `wra_cli status`: no faults, bus 11.5–12.6 V, all 8 servos online, temperatures < 45 °C at rest | find the cause before arming |
| weekly | belt tension J1–J3 (≈ 3 mm deflection at 5 N mid-span); tooth wear | re-tension; replace a belt with damaged teeth |
| weekly | spring cords: fraying at idlers and anchors; arm still floats with torque off | replace the cord (Dyneema); re-set the length |
| weekly | claw pad facing: glazed, dirty, peeling | clean with isopropyl alcohol; re-face with silicone sheet |
| monthly | ABENICS teeth (CS gear and monopoles): wear, white stress marks, chips; play at the tool flange (rock it by hand: ≤ ~1°) | re-lube (PTFE dry film); replace a worn monopole (≈ 6 g, 1 h print) |
| monthly | printed load parts (post bracket, hubs, elbow block, housing, module frames): cracks, especially around inserts and tube sockets | replace; never glue a cracked load part |
| monthly | bearings: rough or noisy (J1 6806 pair, 6808/6805 carriers, 688 monopoles) | replace |
| monthly | wiring: chafing at the J1 spindle and at the Θ-servo service loops | re-route; replace the lead |
| monthly | relays: E-stop function test with a multimeter on the bus (T0.2/T0.4) | replace a relay that shows any welding |
| every 3 months | servo logs: load and temperature trend (`/wra/status` recorded); servos running hotter for the same task | inspect that joint's friction and balance |
| battery | per its BMS / maker instructions; store at ~50 % charge; never charge it on the chair unattended | — |

**After any repair or parameter change:**
1. Re-run `python cad/build.py --verify` if parameters changed.
2. Re-calibrate the affected servos (`servo_setup.py calibrate --only <ids>`).
3. Repeat the relevant test stage (docs/09).

Keep a log in `docs/test_log.md`.

**Spares worth printing ahead:** monopoles `82` ×2, Θ gears `85` ×2, claw pinion `52`, claw pads `54`, one tube clamp set `02/03`, the q-coupling `86`. Keep one spare ST3215 with its ID unassigned.
