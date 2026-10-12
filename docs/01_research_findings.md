# 01 — Research findings (with sources)

## 1. The task: doors and reaching from a manual wheelchair

- **Where door hardware is.** ADA 2010 §404.2.7 places door hardware 34–48 in (864–1219 mm) above the floor. §309.4 limits operable parts to 5 lbf (22.2 N) with no tight grasping, pinching or wrist twisting. §404.2.9 limits interior hinged doors to 5 lbf opening force; that figure excludes latch retraction and closers on fire doors. [1]
- **Latch torque.** BHMA A156.2 limits lever-handle torque to 28 lbf·in (3.2 N·m). ICC A117.1-2017 allows 28 in·lb for rotational hardware. [1] Knobs are not "accessible" hardware, but they are common in homes, which is exactly where a personal arm is needed.
- **Strategy from robotics.** Robots that open doors reliably usually *unlatch* first, then follow the door's arc with compliance rather than a pre-programmed path. An example is equilibrium-point control on the Cody robot. [2] WRA-1 follows that pattern at a small scale: grip, turn, follow the hinge arc for a few degrees under the arm's torque limits, then release; the user pushes the door the rest of the way while rolling through.

## 2. Wheelchair-mounted robotic arms (prior art)

- **Commercial devices.** Kinova JACO and the Assistive Innovations iARM are the established wheelchair-mounted arms. Insurers still treat them as specialised equipment. [3]
- **Clinical evidence.**
  - JACO reduced the assistance time needed by powered-wheelchair users with upper-limb impairments. [4]
  - Long-term users kept using it for eating, drinking, picking up objects and opening doors. They also reported the cost of fine manipulation and set-up time. [5]
  - A literature review of assistive manipulators found that tasks like door opening are among those most valued, and among the hardest to do reliably. [6]
- **What WRA-1 keeps from them:** low-speed, low-force motion; a stowed pose beside the chair; a dock for transfers; user-driven tasks plus semi-automatic routines (door, pick).
- **What it does differently:** fully 3D-printable on a 256 mm bed; hobby serial-bus servos; one design for LEFT or RIGHT mounting; and an ABENICS spherical wrist.
- **Built for a manual chair.** The JACO studies above were with powered-wheelchair users [4]. A manual chair has no battery to draw from, and its user's hands are busy on the hand rims. So WRA-1 carries its own battery, parks clear of the hand rims, and comes off its dock when it isn't needed. A manual chair also has no motor brakes, so the wheel locks must be on before the arm pulls on a door (docs/10).

## 3. ABENICS (the wrist)

- **Origin.** Abe, Tadakuma and Tadakuma, *ABENICS: Active Ball Joint Mechanism With Three-DoF Based on Spherical Gear Meshings*, IEEE T-RO 37(5):1806–1825, 2021. [7] A cross-spherical (CS) gear with two orthogonal families of ring teeth is driven by two monopole (MP) gears. Each MP gear module has two actuators (MP spin + holder rotation), giving 3 DoF with one redundant actuator. Overviews: [8][9].
- **Kinematics.** F. Thomas derived closed-form kinematics and proposed a **three-actuator** variant without the over-actuation. [10] WRA-1A uses the original four-actuator form with an explicit q-window branch choice (docs/06). The three-actuator form is a documented option for a later version.
- **Hobby replications** exist as 3D-printed educational models. [11] None of them is a load-bearing wrist with a gripper, so WRA-1A's tooth generation and packaging were developed here and checked by CAD sweeps. They are *not* yet tested in hardware.

## 4. Actuators and electronics (verified data)

- **Waveshare ST3215 (SKU 22414)** [12]:

  | spec | value |
  |---|---|
  | supply | 6–12.6 V |
  | stall torque | 30 kg·cm at 12 V |
  | no-load speed | 0.222 s/60° |
  | current | 0.2 A no-load, 2.7 A stall; kt 11 kg·cm/A |
  | encoder | 12-bit magnetic, 360° |
  | mass | 89 g |
  | bus | UART TTL serial bus, 1 Mbps default, IDs 0–253 |
  | feedback | position, speed, load, voltage, temperature, current |

- **Register map** (STS series, little-endian), as used by the firmware and tools [13]:
  - 0x28 torque switch; writing 128 sets the present position to 2048
  - 0x29 acceleration (×100 step/s²)
  - 0x2A goal position
  - 0x2E goal speed (0–3400 step/s)
  - 0x30 torque limit (0–1000)
  - 0x37 EEPROM lock
  - 0x38–0x45 telemetry; status bits are voltage / sensor / temperature / current / angle / overload; current unit 6.5 mA
  - 0x09/0x0B angle limits
  - 0x1F homing offset (±2047, bit 11 = sign)
- **Waveshare Bus Servo Adapter (A)** [14]: jumper **A = UART** (for ESP32 / MCU hosts), **B = USB** (for PC / Jetson). Power input 9–12.6 V, which must match the servos. The wiki's wiring note "RX-RX, TX-TX" is ambiguous, so docs/07 says to swap if there is no reply.

## 5. Safety references (structure, not certification)

- IEC 60204-1 stop categories (Cat 0 = removal of power), ISO 13850 emergency-stop function, ISO 13849-1 safety-related control architecture. WRA-1 uses the *pattern* — a hard-wired E-stop, monitored relays and manual reset — without claiming a Performance Level.
- ISO 7176 (wheelchairs) static stability: re-check stability with the arm fitted (test T4.3).

## Sources

1. L. Greene, "Decoded: Operational Force for Door Hardware," *Doors & Hardware*, May 2014 — https://idighardware.com/2014/03/decoded-operational-force-for-door-hardware (summarises ADA 2010 §309.4/§404.2.7/§404.2.9, ICC A117.1-2017, BHMA A156.2).
2. A. Jain, C. C. Kemp, "Pulling Open Novel Doors and Drawers with Equilibrium Point Control," Humanoids 2009 — https://repository.gatech.edu/entities/publication/4fb8cf1b-1091-4554-bc55-ff5ff668ba76
3. Molina Clinical Policy, "Wheelchair Mounted Robotic Arm Devices" — https://www.molinahealthcare.com/molinaclinicalpolicy/-/media/Molina/PublicWebsite/PDF/Common/Molina-Clinical-Policy/Wheelchair-Mounted-Robotic-Arm-Devices_R.pdf
4. V. Maheu et al., "Evaluation of the JACO robotic arm: Clinico-economic study for powered wheelchair users with upper-extremity disabilities," IEEE ICORR 2011, DOI 10.1109/ICORR.2011.5975397.
5. M. Beaudoin et al., "Long-term use of the JACO robotic arm: a case series," *Disabil. Rehabil. Assist. Technol.* 14(3):267–275, 2019, DOI 10.1080/17483107.2018.1428692.
6. C.-S. Chung, H. Wang, R. A. Cooper, "Functional assessment and performance evaluation for assistive robotic manipulators: Literature review," *J. Spinal Cord Med.* 36(4):273–289, 2013 — https://pmc.ncbi.nlm.nih.gov/articles/PMC3758524
7. K. Abe, K. Tadakuma, R. Tadakuma, "ABENICS: Active Ball Joint Mechanism With Three-DoF Based on Spherical Gear Meshings," *IEEE Trans. Robotics* 37(5):1806–1825, 2021, DOI 10.1109/TRO.2021.3070124.
8. "Three Degrees of Freedom" (ABENICS feature), *Gear Technology*, 2023 — https://www.geartechnology.com/three-degrees-of-freedom
9. "ABENICS," Wikipedia — https://en.wikipedia.org/wiki/ABENICS
10. F. Thomas, "Kinematics of a gear-based spherical mechanism," ARK 2022, Springer Proc. Adv. Robotics 24, pp. 323–331, DOI 10.1007/978-3-031-08140-8_35 — https://www.iri.upc.edu/publications/show/2601
11. Spherical gear educational model (MakerWorld) — https://makerworld.com/ja/models/3064336-spherical-gear-educational-model
12. Waveshare, ST3215 Servo product page — https://www.waveshare.com/st3215-servo.htm?sku=22414
13. python-st3215 (register table and protocol notes) — https://github.com/Mickael-Roger/python-st3215
14. Waveshare wiki, Bus Servo Adapter (A) — https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A)
