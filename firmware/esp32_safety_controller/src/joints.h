// Per-servo limits, persisted in NVS by the calibration tool (tools/servo_setup.py via MAINT/SET_LIMIT).
// Until a servo is calibrated the firmware refuses to enable its torque.
#pragma once
#include <Arduino.h>
#include "config.h"

struct JointCfg {
  uint8_t id;
  uint16_t min, max, center;   // servo counts 0..4095 (center = calibrated zero of the joint)
  uint16_t max_speed;          // steps/s at the servo
  uint16_t torque_cont;        // Torque_Limit in normal operation (0..1000 = 0..100 % stall)
  uint16_t torque_peak;        // short-time ceiling (claw knob mode), <= TORQUE_CEILING
  uint8_t calibrated;          // 1 after the calibration tool stored real limits
};

class JointTable {
 public:
  void begin();                                  // load from NVS (or conservative defaults)
  JointCfg *find(uint8_t id);
  bool store(const JointCfg &c);                 // validate + persist
  bool allCalibrated() const;
  JointCfg j[MAX_SERVOS];
  uint8_t n = 0;
};
