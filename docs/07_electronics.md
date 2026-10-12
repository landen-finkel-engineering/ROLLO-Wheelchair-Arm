# 07 — Electronics and wiring

**Rule 1:** servos never connect to the Jetson's GPIO. The ESP32 is the only master on the servo bus, through a Waveshare Bus Servo Adapter (A).

**Rule 2:** the E-stop is **hard-wired**. It removes servo power through relays without any software in the loop. The ESP32 can only *withdraw* permission (SAFE_OK), never bypass the chain.

## 1. Block diagram

```
 18-20 V battery (5S Li-ion + BMS) --F1 30 A--+-- buck 12 V / 20 A --+-- F2 25 A -- K2 (40 A NO) --+-- TVS SMBJ15A + 2200 uF
   (separate from the chair battery)           |                       |                              +-- Bus Servo Adapter (A) 12 V in
                                               |                       |                              +-- arm cable (GX16-5) -> turntable star
                                               |                       |                                   -> 8x ST3215 (daisy-chained DATA)
                                               |                       +-- F3 2 A -- E-stop chain (12 V logic, see 2)
                                               +-- F4 5 A -- Jetson DC-DC (per YOUR carrier spec) -- Jetson Orin Nano 8GB
                                                                                                      |  USB  -> ESP32 (921 600 Bd, CRC frames)
                                                                                                      |  USB  -> ReSpeaker mic array
                                               ESP32 UART2 (GPIO16 RX / GPIO17 TX, 1 Mbps) -- 1 k series -- adapter UART header (jumper A)
```

## 2. Hard-wired stop chain (Category 0 stop, manual reset)

```
 +12V(F3) --[E-stop NC]--[dock switch, closed when docked]--[K0 NO: ESP32 SAFE_OK]--+--[START NO, momentary]--+--( K1 coil )-- 0V
                                                                                   |                         |
                                                                                   +------[K1a NO, self-hold]+
 K1b NO switches the K2 coil:   +12V --[K1b]--( K2 coil )-- 0V        K2 main contact switches the servo bus +12 V.
 ESTOP_SENSE (PC817 opto) taps the chain after the dock switch; BUS_VSENSE (100k/22k) taps the bus after K2.
```

- Pressing the E-stop, undocking the arm, or losing SAFE_OK drops K1 → K2 → all servo power. The springs hold most of the arm's weight, so it sags slowly instead of falling.
- **Restart always needs a person.** Release the E-stop by twisting it, send `clear` / `arm` from the Jetson, then press **START** within 30 s. Nothing restarts by itself after power returns or a fault clears.
- **K0** is a 5 V relay (or opto-MOSFET) driven from GPIO25 through an NPN with a flyback diode. GPIO25 has a 10 kΩ pull-down, so a reset, crash or brown-out of the ESP32 opens K0.
- **Welded-contact check:** if the bus has power while SAFE_OK is low, the ESP32 latches `BUS_POWER_UNEXPECTED` and refuses to arm until it is fixed.
- Use an E-stop with **two NC contacts** if you can get one. Wire the second contact in series with K2's coil so one stuck relay cannot keep power on.

## 3. ESP32 pin map (`firmware/esp32_safety_controller/src/config.h`)

| GPIO | function | notes |
|---|---|---|
| 16 / 17 | UART2 RX / TX → bus adapter | 1 kΩ series in both lines (adapter may be unpowered while K2 is open). If the servos never answer, swap TX/RX: Waveshare's labelling ("RX-RX, TX-TX") is ambiguous. |
| 25 | SAFE_OK out → K0 driver | HIGH = permission; 10 kΩ pull-down |
| 34 | ESTOP_SENSE in | PC817 from the chain; HIGH = chain closed |
| 35 | DOCK_SENSE in | 2nd contact of the dock microswitch via PC817 |
| 32 | BUS_VSENSE (ADC) | 100 k / 22 k divider → 12.6 V reads 2.27 V |
| 2 | status LED | on-board |
| 26 | buzzer (optional) | NPN-driven active piezo |
| USB | Jetson link | CH340/CP2102 → `/dev/ttyUSB0`; udev rule in docs/11 |

LED codes:

| LED | state |
|---|---|
| slow blip | SAFE_OFF |
| fast blink | ARMING — press START |
| steady | READY |
| steady with a short gap | ACTIVE |
| double blink | HOLD (Jetson silent) |
| rapid flicker | FAULT |
| 1 Hz | MAINT |

## 4. Servo bus

| item | value |
|---|---|
| IDs (fixed by function) | 1 J1, 2 J2, 3 J3, 4 q1, 5 Θ1, 6 q2, 7 Θ2, 8 claw (WRA-1B: 4 J4, 5 J5) |
| baud | 1 Mbps (factory) |
| protocol | Feetech STS: little-endian, `FF FF ID LEN INSTR … CHK` |
| registers used | 40 torque enable (128 = set present position to 2048), 41 acc, 42 goal, 46 speed, 48 torque limit, 55 lock, 56–70 telemetry, 9/11 angle limits, 16 max torque, 31 homing offset |
| ESP32 polling | 50 Hz SYNC_READ of regs 56–70 (all servos ≈ 2 ms) |
| goals | one SYNC_WRITE per Jetson goal frame |
| power distribution | star from a WAGO / terminal block on the turntable, not through the servos' own pass-through connectors (each servo can draw 2.7 A at stall) |
| wire | 14 AWG to the turntable, 20 AWG to each servo |
| data | through the J1 spindle bore (Ø20) and along the arm tubes in spiral wrap |
| ABENICS Θ-servo leads | 0.5 m extra-flexible service loop around each module (carrier turns ±120°) |
| claw servo lead | through the centre post (Ø6 bore) |

## 5. Power budget

These come from `calcs/output/results.json`: system ≈ 43 W typical and ≈ 177 W worst case, so a 72 Wh pack gives ≈ 1.4 h of typical use.

| load | typical | worst case |
|---|---|---|
| 8 servos (idle + 2 moving) | ≈ 31 W | all at the 50 % torque limit ≈ 150 W, 12.5 A at 12 V (not a normal state) |
| Jetson Orin Nano 8GB | 7–15 W (25 W "MAXN SUPER") | 25 W |
| ESP32, relays, mic | ≈ 2 W | 3 W |

| part | rating | why |
|---|---|---|
| fuse F2 | 25 A | above the servo worst case |
| buck converter | 20 A with its own current limit | the converter limits current before the fuse is ever needed |
| servo supply | ≤ 12.6 V | the ST3215's maximum input is 12.6 V (Waveshare datasheet). **Never feed raw pack voltage to the bus.** |

## 6. Enclosure (`60_electronics_box`, `61_lid`) and pendant (`62/63`)

The e-box hangs on the rear cane below the seat, using two tube clamps. Layout: Jetson carrier on standoffs, ESP32 + adapter + relay board on the floor slots, and buck converters on the side wall with an air gap.
The pendant holds the 22 mm E-stop and the guarded START button. It hangs within the user's reach; if the user cannot reach it, mount it where the **attendant** can. The cable is 4-core with a GX12-4 connector.
