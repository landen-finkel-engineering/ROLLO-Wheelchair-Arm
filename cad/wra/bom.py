"""Bill of materials: printed parts (from the part registry) + purchased parts ("vitamins"),
fastener / bearing schedule.  Lengths of cut stock and heat-set insert counts are COMPUTED from
the CAD so they cannot drift from the model.  Prices are rough 2025-2026 hobby-market estimates
in USD [E] - check your supplier.  Run via `python build.py --bom` (or --parts / --all).

Variants: WRA-1A = ABENICS 3-DoF wrist (default, params.yaml wrist_type: abenics),
          WRA-1B = pitch-roll 2-DoF fallback wrist.  Lines tagged "common" are needed by both.
"""
from __future__ import annotations

import csv
import json
import math
import pathlib

from .config import P
from . import layout as L
from .parts import arm, wrist as wr, turntable as tt, claw as cw, abenics as ab

A = P.arm
ACTIVE = "WRA-1A" if P.wrist_type == "abenics" else "WRA-1B"
OTHER = "WRA-1B" if ACTIVE == "WRA-1A" else "WRA-1A"


def cut_lengths():
    fa = None
    for t in wr.fa_tubes():
        fa = t.bounding_box().size.X
    return {"ua_tube": A.L2 - 2 * arm.R0, "fa_tube": fa, "stub": tt.STUB_LEN,
            "elbow_pin": arm.elbow_pin().bounding_box().size.Y, "rod_tube": arm.rod_tube_len(),
            "elbow_spacer": arm.elbow_spacer_len() - 0.04,
            "pitch_pins_1B": [p.bounding_box().size.Y for p in wr.pitch_pins()],
            "abenics_post_1A": ab.POST_X[1] - ab.POST_X[0], "claw_rail": P.claw.rail_len}


def vitamins():
    c = cut_lengths()
    std8 = [8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 35, 40]

    def dowel(length):
        return min(s for s in std8 if s >= length - 0.3)

    V = []

    def add(cat, item, qty, spec, where, src="[V] catalogue part", price=None, var="common"):
        V.append({"variant": var, "category": cat, "item": item, "qty": qty, "spec": spec, "where": where,
                  "source": src, "unit_price_usd_est": price})

    # ---------------------------------------------------------------- actuators / electronics
    st = "Waveshare ST3215 serial bus servo (30 kg.cm @12 V)"
    sts = "SKU 22414; 12 V; 4096-step magnetic encoder; 25T spline; supplied metal disc horn + screws"
    add("actuator", st, 4, sts, "J1, J2, J3, claw", "[V] owned / Waveshare", 22)
    add("actuator", st, 4, sts, "ABENICS wrist: q1, Theta1, q2, Theta2", "[V] Waveshare", 22, "WRA-1A")
    add("actuator", st, 2, sts, "J4 pitch, J5 roll", "[V] Waveshare", 22, "WRA-1B")
    add("actuator", st + " - SPARE", 1, "recommended: swap-in while a servo is re-calibrated", "spares", "[V]", 22)
    add("electronics", "NVIDIA Jetson Orin Nano 8GB developer kit", 1, "owned", "e-box", "[V] owned", None)
    add("electronics", "ESP32-WROOM-32 dev board (Freenove or DevKitC)", 1, "safety supervisor / servo bridge",
        "e-box", "[V] owned", 8)
    add("electronics", "Waveshare Bus Servo Adapter (A)", 1, "jumper A = UART mode (ESP32 drives the bus)", "e-box",
        "[V] Waveshare", 6)
    add("electronics", "ReSpeaker USB mic array", 1, "owned; USB to Jetson (voice commands)", "user side",
        "[V] owned", None)
    add("power", "DC-DC buck 12 V / 20 A (input >= 15-24 V)", 1, "servo bus supply; output must stay <= 12.6 V",
        "e-box", "[E] generic", 18)
    add("power", "DC-DC converter for the Jetson (voltage per YOUR carrier spec, >= 45 W)", 1,
        "do NOT feed raw 5S battery (21 V full) unless the carrier datasheet allows it", "e-box", "[E] generic", 15)
    add("power", "18-20 V power-tool battery (5S Li-ion, >= 4 Ah) + adapter with fuse/switch", 1,
        "or any 15-24 V pack with BMS; separate from any chair battery", "e-box / chair", "[E]", 60)
    add("power", "ATO blade fuse holder + fuses 25 A, 5 A, 2 A", 1, "servo bus / Jetson / logic", "e-box", "[E]", 8)
    add("power", "TVS diode SMBJ15A (or 1.5KE15A) + 2200 uF 25 V low-ESR capacitor", 1, "servo bus clamp/bulk",
        "e-box", "[E]", 3)
    add("power", "XT60 connector pair", 2, "battery input", "e-box", "[E]", 2)
    add("safety", "22 mm E-stop, red mushroom, latching twist-release, 1 NC (2 NC preferred)", 1,
        "IEC 60947-5-5 marked", "pendant", "[E]", 12)
    add("safety", "22 mm momentary push button, NO, flush (START/RESET)", 1, "guarded by the printed ring",
        "pendant", "[E]", 5)
    add("safety", "Relay K1: 12 V coil DPDT (e.g. Omron G2R-2) + socket", 1, "self-hold latch", "e-box", "[E]", 8)
    add("safety", "Relay K2: 12 V coil automotive SPST-NO 40 A + socket", 1, "servo-bus power contactor", "e-box",
        "[E]", 6)
    add("safety", "Roller-lever microswitch (SS-5GL size, 20 x 10 x 6.5)", 1, "dock-present interlock",
        "01_receiver pocket", "[E]", 1)
    add("wiring", "GX16-5 panel + cable connector pair", 1, "arm cable: 12 V, GND, DATA, dock-switch pair",
        "e-box <-> 10_base_housing", "[E]", 4)
    add("wiring", "GX12-4 panel + cable connector pair", 1, "pendant: E-stop NC loop + START", "e-box <-> pendant",
        "[E]", 3)
    add("wiring", "Silicone wire 14 AWG red/black (2 m each), 22 AWG 3-colour (5 m), 5264 3-pin servo leads", 1,
        "star power from the turntable distribution block; DATA daisy-chained", "arm", "[E]", 15)
    add("wiring", "Extra-flexible 3-core servo lead 26 AWG silicone, 0.5 m", 2,
        "Theta-servo service loops (carrier turns +/-120 deg)", "ABENICS modules", "[E]", 2, "WRA-1A")
    add("wiring", "WAGO 221 lever connectors or 4-way terminal block (turntable power star)", 1, "", "20_turntable",
        "[E]", 4)
    add("wiring", "Spiral wrap 6 mm, cable ties 2.5 mm, PG7 cable gland", 1, "", "arm / pendant", "[E]", 5)
    # ---------------------------------------------------------------- bearings
    add("bearing", "6806-2RS deep-groove ball bearing 30x42x7", 2, "J1 tower (face-to-face pair)", "10_base_housing",
        price=3)
    add("bearing", "608-2RS 8x22x7", 6, "2 per drive-module pulley (J1, J2, J3)", "12_dm_pulley36 x3", price=0.6)
    add("bearing", "6802-2RS 15x24x5", 4, "UA hub 2, J3 hub 2", "30/31", price=1.5)
    add("bearing", "6800-2RS 10x19x5", 2, "elbow", "34_forearm_root", price=1.5)
    add("bearing", "626-2RS 6x19x6", 2, "parallelogram rod ends", "33_rod_end x2", price=1)
    add("bearing", "623-2RS 3x10x4", 3, "spring-cord idlers: A3 post 1, UA-hub lever B2/C2 2", "20/30", price=0.5)
    add("bearing", "6808-2RS (61808) thin-section 40x52x7", 2, "carrier locating bearing, 1 per module",
        "83_mp_fork / 87_module_frame", price=4, var="WRA-1A")
    add("bearing", "6805-2RS (61805) 25x37x7", 2, "carrier floating bearing, 1 per module",
        "84_mp_cradle / 87_module_frame", price=2.5, var="WRA-1A")
    add("bearing", "688-2RS 8x16x5", 4, "monopoles, 2 each", "82_monopole", price=1, var="WRA-1A")
    add("bearing", "608-2RS 8x22x7", 2, "J4 drive-module pulley", "12_dm_pulley36", price=0.6, var="WRA-1B")
    add("bearing", "6802-2RS 15x24x5", 2, "roll spindle", "41_pitch_yoke", price=1.5, var="WRA-1B")
    add("bearing", "688-2RS 8x16x5", 2, "pitch (yoke + 54T pulley)", "41/43", price=1, var="WRA-1B")
    add("bearing", "625-2RS 5x16x5", 3, "J4 back-side belt idler (stacked on M5)", "40_wrist_housing", price=0.6,
        var="WRA-1B")
    add("linear", f"MGN9 rail, cut {c['claw_rail']:.0f} mm (min. 2 mounting holes per 20 mm pitch)", 1,
        "claw jaw guide; cut + deburr, keep the reference edge", "50_claw_palm", "[V] Hiwin-pattern MGN9", 8)
    add("linear", "MGN9C carriage (standard length) for MGN9 rail", 2, "one per jaw; preloaded/Z0 preferred",
        "53_claw_jaw_a/b", "[V] check hole pattern 15 x 10 (params claw.block_holes)", 6)
    # ---------------------------------------------------------------- belts
    for j, (t2, ln) in {1: (P.pulleys.j1, P.belt_lengths.j1), 2: (P.pulleys.j2, P.belt_lengths.j2),
                        3: (P.pulleys.j3, P.belt_lengths.j3), 4: (P.pulleys.j4, P.belt_lengths.j4)}.items():
        add("belt", f"HTD-3M closed belt {ln:.0f}-3M-15 ({ln / 3:.0f} teeth)", 1,
            f"36T -> {t2}T, centre {getattr(L, f'C{j}'):.2f} mm; if unavailable pick the nearest length and re-run build.py",
            f"J{j}", "[V] standard HTD pitch; check stock", 6, "WRA-1B" if j == 4 else "common")
    # ---------------------------------------------------------------- metal stock / pins
    add("stock", f"Aluminium tube 12 x 1 mm (6060/6063), cut {c['ua_tube']:.1f} mm", 2, "upper-arm tubes",
        "30_ua_hub <-> 32_elbow_block", "[V] stock size; [C] length", 4)
    add("stock", f"Aluminium tube 12 x 1 mm, cut {c['fa_tube']:.1f} mm", 2, "forearm tubes",
        "34_forearm_root <-> wrist housing (89 or 40: same sockets)", "[C] length", 4)
    add("stock", f"Aluminium tube 12 x 1 mm, cut {c['elbow_spacer']:.2f} mm (deburr square)", 1,
        "elbow inner-ring spacer", "elbow pin", "[C]", 0)
    add("stock", f"Aluminium tube 10 x 1 mm, cut {c['rod_tube']:.1f} mm", 1, "parallelogram rod", "33_rod_end x2",
        "[C]", 3)
    add("stock", f"15 mm h6 linear shaft (hardened), cut {c['stub']:.1f} mm", 1, "J2/J3 stub axle (cantilever)",
        "21_stub_cheek", "[C] length", 6)
    add("stock", f"10 mm h6 linear shaft, cut {c['elbow_pin']:.1f} mm", 1, "elbow pin", "32_elbow_block", "[C]", 4)
    add("stock", f"Aluminium tube 8 x 1 mm, cut {c['abenics_post_1A']:.1f} mm", 1,
        "ABENICS centre-ball post (claw cable inside)", "89_abenics_housing -> 90_center_ball", "[C]", 2, "WRA-1A")
    add("pin", "ISO 8734 hardened dowel 8 x 24", 3, "drive-module pulley axles (J1, J2, J3)", "12/22", price=0.5)
    add("pin", "ISO 8734 hardened dowel 8 x 24", 1, "J4 drive-module pulley axle", "40", price=0.5, var="WRA-1B")
    add("pin", f"ISO 8734 hardened dowel 8 x {P.abenics.mp_pin[1]:.0f}", 2, "monopole axles (press in the -x arm)",
        "83_mp_fork", price=0.5, var="WRA-1A")
    add("pin", "ISO 8734 dowel 3 x 12", 2, "CS-gear hemisphere alignment", "80/81", price=0.2, var="WRA-1A")
    add("pin", "ISO 8734 dowel 2 x 20 (or M2 x 20 screw + nut)", 1, "centre ball cross pin (plus epoxy)",
        "90_center_ball", price=0.2, var="WRA-1A")
    for L_ in c["pitch_pins_1B"]:
        add("pin", f"ISO 8734 dowel 8 x {dowel(L_)} (needs {L_:.1f})", 1, "wrist pitch stub axle", "40_wrist_housing",
            price=0.5, var="WRA-1B")
    add("pin", "ISO 8734 dowel 3 x 8", 2, "spindle -> tool flange anti-rotation", "45_roll_spindle", price=0.2,
        var="WRA-1B")
    add("pin", "ISO 8734 hardened dowel 6 x 20", 2, "dock locating dowels (pressed into receiver)", "01_receiver",
        price=0.4)
    add("pin", "ISO 8734 dowel 6 x 12", 1, "J1 hard-stop pin", "20_turntable", price=0.3)
    add("pin", "ISO 8734 dowel 4 x 8", 1, "tool-flange clocking dowel", "80_cs_gear_front (1A) / 46 (1B)", price=0.2)
    add("pin", "ISO 7379 shoulder screw 6 x 10, M5 thread", 2, "parallelogram crank pins P and Q (626 bearings)",
        "31_j3_hub, 34_forearm_root", price=1.5)
    add("collar", "Shaft collar 30 mm bore, OD <= 45, width 13 (clamp type)", 1, "J1 spindle axial retention",
        "base cavity", price=5)
    add("collar", "Shaft collar 15 mm bore, OD <= 28, width <= 12 (set-screw)", 1, "stub stack retention", "stub", price=2)
    add("collar", "Shaft collar 10 mm bore, OD <= 20, width <= 8 (set-screw)", 1, "elbow pin retention", "elbow pin", price=2)
    add("shim", "DIN 988 shim rings 15 x 21 assorted (0.1 / 0.3 / 0.5 / 1.0)", 1, "stub stack end play", "stub", price=4)
    add("shim", "DIN 988 shim rings 10 x 16 x 0.5", 2, "elbow end play", "elbow", price=1)
    add("shim", "DIN 988 shim rings 8 x 14 x 0.1 / 0.3", 4, "monopole axial play (between fork arm and 688)",
        "83_mp_fork", price=0.3, var="WRA-1A")
    # ---------------------------------------------------------------- springs / cord / consumables
    add("spring", "Extension springs for gravity balance (rate/preload from calcs/report)", 4,
        "2 per joint (J2, J3); see calcs/output/springs.json", "upper arm, turntable", "[C] spec from calcs", 3)
    add("spring", "UHMWPE (Dyneema) braided cord 1.5-2 mm, >= 100 kg break", 1, "1 m", "spring cords", price=5)
    add("spring", "M4 eye bolts 40 mm + nuts (spring tension adjusters)", 4, "", "32_elbow_block, 20_turntable", price=0.5)
    add("consumable", "Silicone rubber sheet 1.5 mm, Shore A 40-60, + silicone-compatible adhesive", 1,
        "claw pad facing (friction for knobs); replace when glazed", "54_claw_pad_a/b", "[E]", 8)
    add("consumable", "PTFE dry-film lubricant or PTFE grease (plastic-safe)", 1,
        "ABENICS teeth, centre-ball socket, claw rack", "wrist", "[E]", 8, "WRA-1A")
    # ---------------------------------------------------------------- dock / mount
    add("fastener", "DIN 464 knurled thumb screw M6 x 25 (+ 13 x 1.5 O-ring to make it captive)", 2,
        "arm dock quick release", "10_base_housing -> 01_receiver", price=1.5)
    add("mount", "Rod end SI5 (M5 female, 5 mm bore)", 2, "torque strut", "04_post_bracket <-> strut clamp", price=3)
    add("mount", "M5 threaded rod, 150 mm, + 2 M5 jam nuts", 1, "torque strut (cut to fit 90-160 mm)", "strut", price=1)
    add("mount", "Rubber liner 1.5 mm (bicycle inner tube) or printed TPU sleeve", 5, "inside every tube clamp",
        "02/03", price=0)
    add("mount", "Steel safety tether, 1.5 mm wire rope loop with ferrule + snap hook", 1,
        "bracket eye -> chair frame (catches the arm if the dock is opened by mistake)", "04_post_bracket", price=4)
    return V


# Fastener schedule: (variant, step/location, fastener, qty, into)
FASTENERS = [
    ("common", "Servo horns J1-J3 (36T DM pulleys)", "ISO 7380 M3 x 8 button head", 12, "servo horn threads"),
    ("common", "Drive-module caps (J1, J2, J3)", "ISO 4762 M3 x 10", 12, "M3 inserts in 12/22"),
    ("common", "J1 drive module -> turntable (slotted)", "ISO 4762 M4 x 16 + washer", 4, "M4 inserts in 12_dm_bracket"),
    ("common", "Belt tensioners J1, J2, J3 (pull screws)", "ISO 4762 M4 x 30 + washer", 3, "tension inserts in 12/22"),
    ("common", "DDM halves -> turntable (slotted feet)", "ISO 4762 M4 x 16 + washer", 4, "M4 inserts in 20_turntable"),
    ("common", "Stub cheek -> turntable", "ISO 4762 M4 x 16", 4, "M4 inserts in 20_turntable"),
    ("common", "Stub cheek pinch (stub clamp)", "ISO 4762 M4 x 25 + hex nut", 2, "nut traps in 21"),
    ("common", "Fixed 54T pulley -> base housing", "ISO 10642 countersunk M3 x 12", 4, "M3 inserts in 10_base_housing"),
    ("common", "Tube cross-bolts (UA hub, elbow block, forearm root)", "ISO 4762 M3 x 20 + nyloc nut", 6,
     "drill the Al tubes 3.4 mm through the printed guide holes"),
    ("common", "Rod-end cross-bolts", "ISO 4762 M3 x 20 + nyloc nut", 2, "drill rod tube 3.4 mm"),
    ("common", "Elbow pin pinch", "ISO 4762 M3 x 20 + hex nut", 2, "nut traps in 32"),
    ("common", "Crank pins P, Q", "ISO 7379 shoulder screw 6x10 / M5", 2, "M5 inserts in 31, 34"),
    ("common", "J2 cord anchor (A2 post)", "ISO 4762 M4 x 16 + nut", 1, "20_turntable post"),
    ("common", "Cord idler axles (A3, B2, C2)", "ISO 4762 M3 x 20 + nyloc nut + 623 bearing", 3, "20, 30"),
    # ---- WRA-1A ABENICS wrist
    ("WRA-1A", "CS-gear hemispheres (front -> back)", "ISO 4762 M3 x 16", 4, "M3 inserts in 81"),
    ("WRA-1A", "Tool adapter 56 -> CS-gear flange (BC 30)", "ISO 4762 M3 x 10", 4, "M3 inserts in 80"),
    ("WRA-1A", "Theta gear 85 -> Theta-servo horn", "ISO 7380 M3 x 8", 8, "servo horn threads"),
    ("WRA-1A", "q coupling 86 -> q-servo horn", "ISO 7380 M3 x 8", 8, "servo horn threads"),
    ("WRA-1A", "Cradle 84 -> fork 83 (radial)", "ISO 10642 countersunk M3 x 8 (thread-forming in 2.5 mm pilot)", 4,
     "fork ring"),
    ("WRA-1A", "Holder retainer 91 -> module frame", "ISO 10642 countersunk M3 x 8", 6, "M3 inserts in 87"),
    ("WRA-1A", "Theta-servo strap 92", "ISO 4762 M3 x 10", 4, "M3 inserts in 84"),
    ("WRA-1A", "q-servo cap 88", "ISO 4762 M3 x 12 (thread-forming in 2.5 mm pilot)", 8, "bosses on 87"),
    ("WRA-1A", "Module frame lugs -> housing pads", "DIN 7984 low-head M4 x 16", 4, "M4 inserts in 89"),
    ("WRA-1A", "Post clamp (housing slit)", "ISO 4762 M3 x 20 + hex nut", 1, "89"),
    ("WRA-1A", "Housing -> forearm tubes (cross-bolts)", "ISO 4762 M3 x 20 + nyloc nut", 2, "drill tubes 3.4 mm"),
    # ---- WRA-1B pitch-roll wrist
    ("WRA-1B", "Servo horns J4, J5 (pulley 12, horn adapter 44)", "ISO 7380 M3 x 8 button head", 8, "servo horn threads"),
    ("WRA-1B", "Drive-module cap J4", "ISO 4762 M3 x 10", 4, "M3 inserts in 40"),
    ("WRA-1B", "Wrist housing 40 -> forearm tubes (cross-bolts)", "ISO 4762 M3 x 20 + nyloc nut", 2, "drill tubes 3.4 mm"),
    ("WRA-1B", "J4 back-side idler", "ISO 4762 M5 x 40 + 3x 625 + washers + nyloc", 1, "slot in 40"),
    ("WRA-1B", "Yoke cap", "ISO 4762 M3 x 10", 4, "M3 inserts in 41"),
    ("WRA-1B", "54T J4 pulley -> yoke", "ISO 4762 M3 x 25", 4, "M3 inserts in 41"),
    ("WRA-1B", "Tool flange -> roll spindle", "ISO 4762 M5 x 16 + washer", 1, "M5 insert in 45"),
    ("WRA-1B", "Tool adapter -> tool flange (BC 30)", "ISO 4762 M3 x 10", 4, "M3 inserts in 46"),
    # ---- knob claw (both)
    ("common", "Claw pinion 52 -> claw-servo horn", "ISO 7380 M3 x 6", 4, "servo horn threads"),
    ("common", "Claw cradle cap 51", "ISO 7380 M2 x 10 (thread-forming in 1.8 mm pilot)", 4, "side walls of 50"),
    ("common", "MGN9 rail -> palm 50", "ISO 4762 M3 x 8", int(P.claw.rail_len // 20), "M3 inserts in 50"),
    ("common", "Jaws 53 -> MGN9C carriages", "ISO 4762 M3 x 6", 8, "carriage threads (check length!)"),
    ("common", "Pads 54 -> jaws 53", "ISO 4762 M3 x 16 + hex nut", 4, "nut recess in the finger"),
    ("common", "Dovetail lock (palm on tool adapter)", "M4 x 20 thumb screw (or SHCS) + hex nut", 1, "nut trap in 50"),
    # ---- dock / mount / electronics
    ("common", "Dock thumb screws", "DIN 464 M6 x 25", 2, "M6 inserts in 01_receiver"),
    ("common", "Receiver -> post bracket shelf (index ring)", "ISO 4762 M4 x 16 + washer", 4, "M4 inserts in 04"),
    ("common", "Tube clamps (body + cap), 5 clamps", "ISO 4762 M5 x 30 + hex nut", 10, "nut traps in 02"),
    ("common", "Post bracket -> 2 post clamps", "ISO 4762 M5 x 16 + washer", 4, "M5 inserts in 02 pads"),
    ("common", "Strut rod end -> bracket clevis", "ISO 4762 M5 x 35 + nut (nut trap)", 1, "04 clevis"),
    ("common", "Strut rod end -> strut clamp pad", "ISO 4762 M5 x 16 + washer", 1, "M5 insert in 02"),
    ("common", "Stop collar", "ISO 4762 M4 x 20 + nut", 1, "05"),
    ("common", "Electronics box -> 2 cane clamps", "ISO 4762 M5 x 12 + washer", 4, "M5 inserts in 02 pads"),
    ("common", "Electronics box lid", "ISO 4762 M3 x 10", 4, "M3 inserts in 60"),
    ("common", "Pendant lid", "ISO 4762 M3 x 10", 4, "M3 inserts in 62"),
    ("common", "Electronics boards", "M3 nylon standoffs + screws (assorted)", 1, "floor slots in 60"),
    ("common", "Dock microswitch", "M2 x 10 self-tapping", 2, "01_receiver pocket"),
]

INSERT_SPEC = {"M3": ("Brass heat-set insert M3 x 5.7 (pilot 4.0)", 0.06), "M4": ("Brass heat-set insert M4 x 8.1 (pilot 5.6)", 0.1),
               "M5": ("Brass heat-set insert M5 x 9.5 (pilot 6.4)", 0.12), "M6": ("Brass heat-set insert M6 x 12.7 (pilot 8.0)", 0.2)}


def insert_lines(rows):
    """Heat-set inserts counted from the CAD (rows from parts.json), per variant."""
    out = []
    for var in ("common", "WRA-1A", "WRA-1B"):
        tot = {}
        where = {}
        for r in rows:
            if r.get("variant", "common") != var:
                continue
            for k, n in (r.get("inserts") or {}).items():
                if r["part"].startswith("00_"):
                    continue                      # the fit coupon's test inserts are not consumed
                tot[k] = tot.get(k, 0) + n * r["qty"]
                where.setdefault(k, []).append(f"{r['part']} {n * r['qty']}")
        for k in sorted(tot):
            item, price = INSERT_SPEC[k]
            out.append({"variant": var, "category": "insert", "item": item, "qty": tot[k] + (4 if var == "common" else 0),
                        "spec": "counted from the CAD" + (" (+4 for the fit coupon / spares)" if var == "common" else ""),
                        "where": ", ".join(where[k]), "source": "[C]", "unit_price_usd_est": price})
    return out


def write(rows, out_dir: pathlib.Path):
    out_dir = pathlib.Path(out_dir)
    V = vitamins() + insert_lines(rows)
    act = [v for v in V if v["variant"] in ("common", ACTIVE)]
    oth = [v for v in V if v["variant"] == OTHER]
    with open(out_dir / "bom_purchased.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(V[0].keys()))
        w.writeheader()
        w.writerows(V)
    with open(out_dir / "fastener_schedule.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["variant", "location / step", "fastener", "qty", "into"])
        w.writerows(FASTENERS)
    totals = {}
    for var, loc, fas, q, _ in FASTENERS:
        if var in ("common", ACTIVE):
            key = fas.split(" + ")[0]
            totals[key] = totals.get(key, 0) + q
    prow = [r for r in rows if r.get("variant", "common") in ("common", ACTIVE)]
    orow = [r for r in rows if r.get("variant", "common") == OTHER]
    printed_g = sum(r["mass_g"] * r["qty"] for r in prow)
    price = sum((v["unit_price_usd_est"] or 0) * v["qty"] for v in act)
    n_servo = sum(v["qty"] for v in act if v["category"] == "actuator" and "SPARE" not in v["item"])
    summary = {"active_variant": ACTIVE, "printed_parts": len(prow), "printed_instances": sum(r["qty"] for r in prow),
               "printed_mass_total_g_est": round(printed_g, 0), "purchased_lines": len(act), "servos": n_servo,
               "purchased_cost_usd_est_excl_owned": round(price, 0), "fastener_totals": totals,
               "cut_lengths_mm": {k: (round(v, 2) if isinstance(v, float) else v) for k, v in cut_lengths().items()},
               "other_variant": {"name": OTHER, "printed_parts": [r["part"] for r in orow],
                                 "purchased_lines": len(oth)}}
    (out_dir / "bom_summary.json").write_text(json.dumps(summary, indent=1, default=str))
    # markdown BOM
    Lm = ["# Bill of materials (generated by `python build.py --bom`)", "",
          f"Active variant: **{ACTIVE}** ({'ABENICS 3-DoF wrist' if ACTIVE == 'WRA-1A' else 'pitch-roll wrist'}, "
          f"set by `wrist_type` in params.yaml).  {n_servo} servos + 1 spare.  The other variant's extra parts are "
          f"listed at the end.", "",
          "Status tags: [V] verified catalogue/vendor data, [C] computed from the CAD/params, [E] estimate.", "",
          "## Printed parts", "", "| part | title | qty | material | mass each (g, est.) | bed size (mm) | inserts |",
          "|---|---|---|---|---|---|---|"]
    for r in prow:
        ins = " ".join(f"{k}x{v}" for k, v in (r.get("inserts") or {}).items()) or "-"
        Lm.append(f"| {r['part']} | {r['title']} | {r['qty']} | {r['material']} | {r['mass_g']:.0f} | "
                  f"{r['print_bbox_mm'][0]:.0f} x {r['print_bbox_mm'][1]:.0f} x {r['print_bbox_mm'][2]:.0f} | {ins} |")
    Lm += ["", f"Total printed mass (all instances, est.): **{printed_g:.0f} g**.", "",
           "## Purchased parts", "", "| category | item | qty | spec | where | source | ~USD each |", "|---|---|---|---|---|---|---|"]
    for v in act:
        Lm.append(f"| {v['category']} | {v['item']} | {v['qty']} | {v['spec']} | {v['where']} | {v['source']} | "
                  f"{v['unit_price_usd_est'] if v['unit_price_usd_est'] is not None else 'owned'} |")
    Lm += ["", f"Purchased cost estimate excluding owned items: **~${price:.0f}** [E].", "",
           "## Fastener schedule", "", "| location / step | fastener | qty | into |", "|---|---|---|---|"]
    for var, loc, fas, q, into in FASTENERS:
        if var in ("common", ACTIVE):
            Lm.append(f"| {loc} | {fas} | {q} | {into} |")
    Lm += ["", "### Fastener totals", "", "| fastener | qty |", "|---|---|"]
    for k, v in sorted(totals.items()):
        Lm.append(f"| {k} | {v} |")
    Lm += ["", f"## {OTHER} only (not needed for {ACTIVE})", "", "Printed: " +
           (", ".join(f"{r['part']} x{r['qty']}" for r in orow) or "-"), "",
           "| category | item | qty | where |", "|---|---|---|---|"]
    for v in oth:
        Lm.append(f"| {v['category']} | {v['item']} | {v['qty']} | {v['where']} |")
    for var, loc, fas, q, into in FASTENERS:
        if var == OTHER:
            Lm.append(f"| fastener | {fas} | {q} | {loc} |")
    (out_dir / "BOM.md").write_text("\n".join(Lm) + "\n")
    return summary
