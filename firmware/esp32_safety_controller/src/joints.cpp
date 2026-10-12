#include "joints.h"
#include <Preferences.h>

static Preferences prefs;

static JointCfg defaults(uint8_t id) {
  JointCfg c;
  c.id = id;
  c.center = 2048;
  c.min = 2048 - 228;          // +/- 20 deg at the servo until calibrated
  c.max = 2048 + 228;
  c.max_speed = 800;
  c.torque_cont = 250;         // 25 % of stall
  c.torque_peak = 400;
  if (id == CLAW_ID) c.torque_peak = 500;     // knob mode (<= KNOB_MODE_MAX_MS)
  if (isTheta(id)) c.torque_peak = THETA_TORQUE_CEILING;
  c.calibrated = 0;
  return c;
}

void JointTable::begin() {
  n = N_SERVOS;
  prefs.begin("wra-joints", false);
  for (uint8_t i = 0; i < n; i++) {
    char key[6];
    snprintf(key, sizeof(key), "j%u", SERVO_IDS[i]);
    JointCfg c;
    if (prefs.getBytes(key, &c, sizeof(c)) == sizeof(c) && c.id == SERVO_IDS[i] && c.min < c.max && c.max <= 4095 &&
        c.torque_peak <= TORQUE_CEILING && c.torque_cont <= c.torque_peak) {
      j[i] = c;
    } else {
      j[i] = defaults(SERVO_IDS[i]);
    }
  }
}

JointCfg *JointTable::find(uint8_t id) {
  for (uint8_t i = 0; i < n; i++)
    if (j[i].id == id) return &j[i];
  return nullptr;
}

bool JointTable::store(const JointCfg &c) {
  JointCfg *dst = find(c.id);
  if (!dst || c.min >= c.max || c.max > 4095 || c.center < c.min || c.center > c.max ||
      c.torque_peak > TORQUE_CEILING || c.torque_cont > c.torque_peak || c.max_speed == 0 || c.max_speed > 3400)
    return false;
  *dst = c;
  char key[6];
  snprintf(key, sizeof(key), "j%u", c.id);
  return prefs.putBytes(key, &c, sizeof(c)) == sizeof(c);
}

bool JointTable::allCalibrated() const {
  for (uint8_t i = 0; i < n; i++)
    if (!j[i].calibrated) return false;
  return true;
}
