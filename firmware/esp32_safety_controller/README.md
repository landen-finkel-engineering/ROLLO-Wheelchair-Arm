# ESP32 safety supervisor + ST3215 bus master (WRA-1)

* `src/main.cpp`   state machine (SAFE_OFF / ARMING / READY / ACTIVE / HOLD / FAULT / MAINT), watchdog, SAFE_OK
* `src/sts_bus.*`  Feetech STS (ST3215) protocol, SYNC_READ telemetry / SYNC_WRITE goals
* `src/host_link.*` CRC-framed link to the Jetson (spec: docs/08_control_interface.md)
* `src/joints.*`   per-servo limit table in NVS (written by software/tools/servo_setup.py)
* `src/config.h`   pins, IDs, timing, thresholds

Build + upload: docs/11_software_setup.md section A (PlatformIO `pio run -t upload`, or Arduino IDE with
this folder as the sketch).  Wiring: docs/07_electronics.md.

Verification status
* Compiled and linked for the ESP32 with the same core and compiler PlatformIO uses for
  `espressif32@6.9.0`: arduino-esp32 2.0.17 (ESP-IDF 4.4.7) and xtensa-esp32-elf GCC 8.4.0
  (esp-2021r2-patch5), board "ESP32 Dev Module" defaults.  Result: no errors;
  flash 285 kB (21.8 % of the 1.25 MB app partition), static RAM 22.6 kB (6.9 %).
  `tools/offline_build.py` reproduces this without PlatformIO.
* That real compile caught one bug the earlier stub check missed: the host-link object was named
  `link`, which collides with POSIX `link()` from <unistd.h>.  It is now `hostLink`.
* The firmware itself caps the ABENICS Θ servos (IDs 5, 7) at 25 % of stall, and at 30 % only while knob mode is on
  (`THETA_TORQUE_CEILING`, the Θ-gear tooth limit from docs/05), whatever the host requests.
* The Arduino-ESP32 3.x watchdog API is handled by an `#if` branch, but 3.x was not compiled.
* NOT yet run on a board, and NOT a certified safety device.  The E-stop chain is hard-wired
  (docs/07) and must be tested on the bench first (docs/09, stage T0).
