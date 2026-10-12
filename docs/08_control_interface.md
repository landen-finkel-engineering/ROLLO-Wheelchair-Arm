# 08 — Control interface specification

Three layers:
1. **Jetson software** (ROS 2 Humble or the plain-Python CLI) plans motion and runs tasks.
2. The **ESP32 supervisor** owns the servo bus, the limits, the watchdog and SAFE_OK.
3. The **hard-wired stop chain** removes power.

Each layer can stop the arm without the layers above it.

## 1. Joint conventions

| joint | symbol | unit | convention | soft limit | servo ratio (servo° per joint°) |
|---|---|---|---|---|---|
| J1 yaw | th1 | deg | + = counter-clockwise from above | ±105 | 1.5 (DM walks round a fixed 54T) |
| J2 shoulder | th2 | deg | absolute elevation of the upper arm | −30 … 100 | 86/36 = 2.389 |
| J3 forearm | th3 | deg | **absolute** elevation (parallelogram) | −95 … 60, with th2 − th3 ∈ [42, 140] | 72/36 = 2.0 |
| wrist (1A) | (pitch, yaw, roll) | deg | tool vs. forearm, R = Rz(yaw)·Ry(−pitch)·Rx(roll)·R_home | tilt ≤ 30, toward a module ≤ 21, roll ±90 | ABENICS IK → q1 Θ1 q2 Θ2; q: 1.0, Θ: 0.75 |
| wrist (1B) | j4, j5 | deg | pitch relative to the forearm, roll | ±105, ±150 | 1.5, 1.0 |
| claw | claw | mm | half-opening; pad-to-pad = 2 × claw | 0 … 32.5 | servo° = −deg(claw / 10.5 mm) |

URDF joints (`software/wra_ros2/wra_description/urdf/wra.urdf.xacro`, generated):

| URDF joint | equals |
|---|---|
| `j1` | th1 |
| `j2` | th2 |
| `j3` | th3 − th2 |
| `wrist_yaw`, `wrist_pitch`, `wrist_roll` | virtual serial joints that reproduce the ABENICS orientation exactly |
| `claw` (prismatic, m) | half-opening |
| `claw_b` | mimic of `claw` |

`tcp` is a fixed frame 88.5 mm in front of the tool flange (pad centre).

Servo counts: `counts = center + dir × servo_deg × 4096/360`. `center` and `dir` come from `config/calibration.yaml`, written by `software/tools/servo_setup.py calibrate`. The calibration shifts each servo's homing offset so that the **middle** of the joint range reads 2048, so no range wraps through 0/4095.

## 2. Jetson ↔ ESP32 link

USB serial, 921 600 Bd, 8N1. Code: `firmware/.../src/host_link.*` and `software/.../wra_driver/protocol.py` (identical framing).

```
A5 5A | type u8 | seq u8 | len u16 LE | payload[len] | crc16 u16 LE      (CRC-16/CCITT-FALSE over type..payload)
```

**Host → ESP32**

| type | name | payload | allowed in |
|---|---|---|---|
| 0x01 | HEARTBEAT | – (the host sends it at 50 Hz; any valid frame resets the 250 ms watchdog) | all |
| 0x02 | ARM | u8: 1 = arm (SAFE_OFF→ARMING, READY→ACTIVE, HOLD→ACTIVE); 0 = torque off (→READY); 2 = power down (→SAFE_OFF) | see states |
| 0x10 | GOALS | u8 n, n × {u8 id, u16 counts, u16 speed steps/s (0 = joint max), u8 acc ×100 steps/s²} | ACTIVE |
| 0x11 | TORQUE | u8 n, n × {u8 id, u8 on} (goal := present position before enabling) | READY, ACTIVE |
| 0x12 | CLAW_MODE | u8: 1 = knob mode (claw torque limit 50 %, ≤ 10 s, 20 s cool-down), 0 = normal 25 % | ACTIVE |
| 0x13 | TORQUE_LIMITS | u8 n, n × {u8 id, u16 0..1000}. Each limit is clamped to the servo's table value (claw ≤ continuous). ABENICS Θ servos (5, 7) are clamped to ≤ 30 % (`THETA_TORQUE_CEILING`) and only while knob mode is on; otherwise to their continuous 25 %. Leaving knob mode, or its 10 s timeout, puts them back to continuous. Answers CLAMPED if any value was reduced | READY, ACTIVE |
| 0x20 | GET_STATE | – | all |
| 0x21 | GET_CONFIG | – | all |
| 0x30 | MAINT | u8 1/0 (enter only with all torque off) | READY, SAFE_OFF |
| 0x31 | PASSTHRU | raw STS packet → reply PASSTHRU_RESP | MAINT |
| 0x32 | SET_LIMIT | u8 id, u16 min, u16 max, u16 center, u16 max_speed, u16 torque_cont, u16 torque_peak, u8 calibrated (persisted in NVS) | MAINT |
| 0x33 | CLEAR_FAULTS | – (critical faults clear only when the cause is gone) | all |

**ESP32 → host**

| type | name | payload |
|---|---|---|
| 0x80 | STATE (50 Hz) | u8 state, u32 faults, u8 inputs (b0 E-stop chain closed, b1 docked, b2 bus powered, b3 SAFE_OK, b4 knob mode, b5 all calibrated), u16 bus mV, u32 uptime ms, u8 n, n × 13 B {u8 id, u8 flags (online, torque, moving), u16 pos, i16 speed, i16 load, u8 0.1 V, u8 °C, u16 mA, u8 servo status} |
| 0x81 | ACK | u8 type, u8 seq, u8 code (0 OK, 1 BAD_STATE, 2 BAD_ARG, 3 CLAMPED, 4 UNKNOWN) |
| 0x82 | EVENT | u32 fault bit, u8 servo id, ASCII text |
| 0x83 | CONFIG | u8 n, n × 14 B {id, min, max, center, max_speed, torque_cont, torque_peak, calibrated} |
| 0x84 | PASSTHRU_RESP | raw reply bytes |

**States**

| # | state | meaning |
|---|---|---|
| 0 | BOOT | |
| 1 | SAFE_OFF | SAFE_OK low |
| 2 | ARMING | SAFE_OK high, waiting for START, 30 s |
| 3 | READY | powered, torque off |
| 4 | ACTIVE | |
| 5 | HOLD | host silent > 250 ms: servos hold position, goals rejected until ARM |
| 6 | FAULT | latched, SAFE_OK low |
| 7 | MAINT | |

**Fault bits**

| bit | fault | critical (Cat 0 stop + latched)? |
|---|---|---|
| 0 | ESTOP_OPEN | yes |
| 1 | HOST_TIMEOUT | no |
| 2 | SERVO_OFFLINE (3 missed polls) | yes |
| 3 | SERVO_OVERTEMP (≥ 70 °C) | yes |
| 4 | SERVO_ERROR (status bits voltage / sensor / temperature / current / overload) | yes |
| 5 | BUS_UNDERVOLT (< 10.0 V) | yes |
| 6 | BUS_OVERVOLT (> 12.9 V) | yes |
| 7 | LIMIT_CLAMPED | no |
| 8 | NOT_CALIBRATED | no |
| 9 | BUS_POWER_UNEXPECTED | yes |
| 10 | ARMING_TIMEOUT | no |
| 11 | KNOB_TIMEOUT | no |
| 12 | WDT_RESET | no |
| 13 | TEMP_WARN (≥ 60 °C) | no |

The ESP32 **always** clamps goals to the calibrated per-servo window and caps speed at the joint maximum. It refuses torque until every servo is calibrated, and the servo EEPROM carries its own angle limits plus a 50 % torque ceiling.

## 3. ROS 2 interface (`wra_driver`)

| kind | name | type | notes |
|---|---|---|---|
| pub | `/joint_states` | sensor_msgs/JointState | URDF joints, 50 Hz |
| pub | `/wra/status` | std_msgs/String (JSON) | state, faults, inputs, bus V, servo temperatures, busy |
| pub | `/wra/tcp_pose` | geometry_msgs/PoseStamped | TCP in `base_link` |
| sub | `/wra/joint_goal` | sensor_msgs/JointState | any subset of URDF joints; minimum-jerk move |
| sub | `/wra/tcp_goal` | geometry_msgs/PoseStamped | straight-line TCP move, ≤ 80 mm/s, IK at every step |
| sub | `/wra/claw` | std_msgs/Float32 | pad-to-pad opening, mm |
| sub | `/wra/knob_pose` | geometry_msgs/PoseStamped | knob frame (x = out of the door, z = up), from vision or `wra_cli teach-knob` |
| sub | `/wra/stop` | std_msgs/Empty | cancel the motion/task; arm holds position |
| srv | `/wra/arm` | std_srvs/SetBool | true: arm/resume (press START when ARMING); false: torque off |
| srv | `/wra/power_down`, `/wra/clear_faults`, `/wra/home`, `/wra/open_door` | std_srvs/Trigger | |
| TF | `chair → base_link` | static | from `arm_geometry.json` for `side:=right|left`; the arm itself is identical on both sides |

**User keep-out.** `side` (node parameter, or `side:` in calibration.yaml) also turns on the keep-out check. Every joint or TCP target, and every point on a planned path, is refused if the upper arm, forearm, wrist centre, tool flange or TCP would enter the head/torso boxes of the CAD occupant envelope (`user_keepout_C_mm` in `arm_geometry.json`) grown by `keepout_margin_mm` (50 mm). The refusal reason is reported on `/wra/status`. Measure the real user and adjust `mount.user_envelope()` in the CAD, then re-run `build.py --software`.

**Vision hook.** No camera hardware is part of this design. Any perception node, such as your stereo doorknob detector, only needs to publish `/wra/knob_pose` in `base_link`. It can be calibrated by touching the TCP to known points.

**Voice** (`voice_node`, ReSpeaker + offline Vosk):
- "stop" / "freeze" always works and publishes `/wra/stop`.
- Every other command needs the wake word, e.g. "robot open the door", "robot go home", "robot open hand", "robot close hand", "robot arm on", "robot arm off".
