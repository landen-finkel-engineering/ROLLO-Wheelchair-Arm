// WRA-1 ESP32 safety supervisor + ST3215 bus master -- build-time configuration.
// Pin numbers are for an ESP32-WROOM-32 dev board (Freenove ESP32-WROOM / Espressif DevKitC).
// Wiring: docs/07_electronics.md.  Every value here is also listed in that document.
#pragma once
#include <stdint.h>

// ---------------------------------------------------------------- pins
#define PIN_BUS_RX        16   // UART2 RX  <- Waveshare Bus Servo Adapter (A) TX  (jumper in UART position)
#define PIN_BUS_TX        17   // UART2 TX  -> adapter RX
#define PIN_SAFE_OK       25   // HIGH = supervisor permits power. Drives K0 (opto/MOSFET); 10k pull-down on the board
#define PIN_ESTOP_SENSE   34   // input-only, HIGH = E-stop chain + dock switch closed (PC817 opto from the 12 V chain)
#define PIN_DOCK_SENSE    35   // input-only, HIGH = arm docked (2nd contact of the dock microswitch via opto)
#define PIN_BUS_VSENSE    32   // ADC1_CH4: servo-bus voltage after K2 via 100k / 22k divider
#define PIN_LED           2    // on-board LED (strapping pin: only driven after boot)
#define PIN_BUZZER        26   // optional piezo (active) via NPN

// ---------------------------------------------------------------- links
#define HOST_BAUD         921600UL     // USB serial to the Jetson
#define BUS_BAUD          1000000UL    // ST3215 factory default (1 Mbps)
#define BUS_ECHO          0            // 1 if YOUR adapter echoes TX bytes back to RX
#define BUS_TIMEOUT_US    3000         // per servo reply

// ---------------------------------------------------------------- timing (ms)
#define HOST_TIMEOUT_MS   250          // no valid host frame -> HOLD (position hold, goals ignored)
#define ARMING_TIMEOUT_MS 30000        // START must be pressed within this after ARM
#define STATE_PERIOD_MS   20           // 50 Hz telemetry + servo polling
#define KNOB_MODE_MAX_MS  10000        // claw "knob" torque limit allowed for at most 10 s
#define KNOB_COOLDOWN_MS  20000
#define WDT_TIMEOUT_S     2            // task watchdog -> reset -> SAFE_OK low (relays drop)
#define OFFLINE_STRIKES   3            // consecutive missed replies before SERVO_OFFLINE

// ---------------------------------------------------------------- limits
#define TEMP_WARN_C       60
#define TEMP_FAULT_C      70
#define BUS_MIN_MV        10000        // while powered
#define BUS_MAX_MV        12900
#define BUS_OFF_MV        2000         // below this the bus counts as unpowered
#define TORQUE_CEILING    500          // firmware never sets Torque_Limit above 50 % (0..1000)
#define VSENSE_RATIO      (122.0f / 22.0f)   // divider 100k + 22k

// ---------------------------------------------------------------- servos
// Fixed ID per function (same IDs on both wrists so firmware and tools stay generic):
//   1 J1 yaw   2 J2 shoulder   3 J3 elbow (parallelogram, absolute)
//   WRA-1A (ABENICS): 4 q1 carrier  5 Theta1  6 q2 carrier  7 Theta2
//   WRA-1B (pitch-roll): 4 J4 pitch  5 J5 roll
//   8 claw
#define MAX_SERVOS        8
#define CLAW_ID           8
// ABENICS Theta servos: the 20T/15T Theta gear pair reaches its tooth-stress limit at 30 % of stall
// (docs/05).  Above torque_cont they are allowed ONLY while knob mode is on (<= KNOB_MODE_MAX_MS),
// and never above THETA_TORQUE_CEILING, whatever the host or the NVS table asks for.
#define THETA_TORQUE_CEILING 300
#ifndef WRA_VARIANT_1B
static inline bool isTheta(uint8_t id) { return id == 5 || id == 7; }
#else
static inline bool isTheta(uint8_t id) { (void)id; return false; }
#endif

#ifndef WRA_VARIANT_1B
static const uint8_t SERVO_IDS[] = {1, 2, 3, 4, 5, 6, 7, 8};
#else
static const uint8_t SERVO_IDS[] = {1, 2, 3, 4, 5, 8};
#endif
static const uint8_t N_SERVOS = sizeof(SERVO_IDS);

// STS memory map (Feetech STS3215 / Waveshare ST3215)
namespace reg {
static const uint8_t ID = 5, MIN_ANGLE = 9, MAX_ANGLE = 11, MAX_TEMP = 13, MAX_VOLT = 14, MIN_VOLT = 15,
                     MAX_TORQUE = 16, PROT_CURRENT = 28, HOMING_OFFSET = 31, MODE = 33,
                     TORQUE_ENABLE = 40, ACC = 41, GOAL_POS = 42, GOAL_TIME = 44, GOAL_SPEED = 46,
                     TORQUE_LIMIT = 48, LOCK = 55, PRESENT_POS = 56, PRESENT_SPEED = 58, PRESENT_LOAD = 60,
                     PRESENT_VOLT = 62, PRESENT_TEMP = 63, STATUS = 65, MOVING = 66, PRESENT_CURRENT = 69;
}
