// WRA-1 ESP32 safety supervisor + ST3215 bus master.
//
// The ESP32 is the ONLY device on the servo bus.  The Jetson sends joint goals over USB; the ESP32
// clamps them to the calibrated limits, writes them to the servos, polls the servos at 50 Hz and
// streams telemetry back.  It owns the SAFE_OK output that sits IN SERIES with the hard-wired
// E-stop chain (E-stop NC + dock switch -> K1 self-hold via START -> K2 servo-power contactor):
//   * E-stop or undocking cuts servo power in hardware, independent of this firmware (Cat 0);
//   * this firmware can only REMOVE permission (SAFE_OK low), never bypass the chain;
//   * a crash/hang -> task watchdog reset -> GPIO25 low (pull-down) -> K0/K1 drop -> power off.
// States: BOOT, SAFE_OFF, ARMING, READY (powered, torque off), ACTIVE, HOLD (host lost), FAULT, MAINT.
// Protocol: docs/08_control_interface.md.  Not a certified safety device (see docs/09_safety.md).
#include <Arduino.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include "config.h"
#include "sts_bus.h"
#include "host_link.h"
#include "joints.h"

enum State : uint8_t { BOOT = 0, SAFE_OFF, ARMING, READY, ACTIVE, HOLD, FAULT, MAINT };
enum FaultBit : uint32_t {
  F_ESTOP_OPEN = 1u << 0, F_HOST_TIMEOUT = 1u << 1, F_SERVO_OFFLINE = 1u << 2, F_SERVO_OVERTEMP = 1u << 3,
  F_SERVO_ERROR = 1u << 4, F_BUS_UNDERVOLT = 1u << 5, F_BUS_OVERVOLT = 1u << 6, F_LIMIT_CLAMPED = 1u << 7,
  F_NOT_CALIBRATED = 1u << 8, F_BUS_POWER_UNEXPECTED = 1u << 9, F_ARMING_TIMEOUT = 1u << 10,
  F_KNOB_TIMEOUT = 1u << 11, F_WDT_RESET = 1u << 12, F_TEMP_WARN = 1u << 13
};
static const uint32_t CRITICAL = F_ESTOP_OPEN | F_SERVO_OFFLINE | F_SERVO_OVERTEMP | F_SERVO_ERROR | F_BUS_UNDERVOLT |
                                 F_BUS_OVERVOLT | F_BUS_POWER_UNEXPECTED;

static sts::Bus bus;
static host::Link hostLink;            // (not "link": that name is the POSIX link() in <unistd.h>)
static JointTable joints;
static sts::Telemetry tel[MAX_SERVOS];
static bool torque_on[MAX_SERVOS];

static State state = BOOT;
static uint32_t faults = 0;            // latched until CLEAR_FAULTS (critical) or condition gone (warnings)
static uint32_t t_last_host = 0, t_state = 0, t_poll = 0, t_arming = 0, t_knob = 0, t_knob_end = 0, t_bus_on = 0;
static bool knob_mode = false, estop_ok = false, docked = false, bus_on = false;
static uint16_t bus_mV = 0;

// ------------------------------------------------------------------ helpers
static int idx(uint8_t id) {
  for (uint8_t i = 0; i < N_SERVOS; i++)
    if (SERVO_IDS[i] == id) return i;
  return -1;
}

static void event(uint32_t bit, uint8_t id, const char *msg) {
  uint8_t p[72];
  memcpy(p, &bit, 4);
  p[4] = id;
  size_t n = strnlen(msg, 64);
  memcpy(p + 5, msg, n);
  hostLink.send(host::EVENT, p, uint16_t(5 + n));
}

static void setState(State s) {
  if (s == state) return;
  state = s;
  t_state = millis();
}

static bool safeOkAllowed() {
  return (state == ARMING || state == READY || state == ACTIVE || state == HOLD || state == MAINT) && !(faults & CRITICAL);
}

static void allTorque(bool on) {
  for (uint8_t i = 0; i < N_SERVOS; i++) {
    if (on) {
      JointCfg *c = joints.find(SERVO_IDS[i]);
      bus.write16(SERVO_IDS[i], reg::GOAL_POS, tel[i].pos);          // no jump: goal = present position
      bus.write16(SERVO_IDS[i], reg::TORQUE_LIMIT, c ? c->torque_cont : 250);
    }
    bus.write8(SERVO_IDS[i], reg::TORQUE_ENABLE, on ? 1 : 0);
    torque_on[i] = on;
  }
}

static void holdPosition() {
  uint8_t data[MAX_SERVOS * 2];
  for (uint8_t i = 0; i < N_SERVOS; i++) {
    data[2 * i] = uint8_t(tel[i].pos & 0xFF);
    data[2 * i + 1] = uint8_t(tel[i].pos >> 8);
  }
  bus.syncWrite(reg::GOAL_POS, 2, SERVO_IDS, N_SERVOS, data);
}

static void fault(uint32_t bit, uint8_t id, const char *msg) {
  bool is_new = !(faults & bit);
  faults |= bit;
  if (bit & CRITICAL) {
    if (bus_on) allTorque(false);      // best effort; the relays remove power anyway
    setState(FAULT);
  }
  if (is_new) event(bit, id, msg);
}

// ------------------------------------------------------------------ inputs / servos
static void readInputs() {
  estop_ok = digitalRead(PIN_ESTOP_SENSE) == HIGH;
  docked = digitalRead(PIN_DOCK_SENSE) == HIGH;
  bus_mV = uint16_t(analogReadMilliVolts(PIN_BUS_VSENSE) * VSENSE_RATIO);
  bool was = bus_on;
  bus_on = bus_mV > BUS_OFF_MV;
  if (bus_on && !was) t_bus_on = millis();
}

static void pollServos() {
  if (!bus_on || millis() - t_bus_on < 300) {          // let the servos boot after power-up
    for (uint8_t i = 0; i < N_SERVOS; i++) tel[i].online = false;
    return;
  }
  static uint8_t raw[15 * MAX_SERVOS];
  bool ok[MAX_SERVOS];
  bus.syncRead(reg::PRESENT_POS, 15, SERVO_IDS, N_SERVOS, raw, ok);
  for (uint8_t i = 0; i < N_SERVOS; i++) {
    if (ok[i]) {
      sts::Bus::decodeTelemetry(raw + 15 * i, tel[i]);
      tel[i].online = true;
      tel[i].misses = 0;
    } else {
      tel[i].online = false;
      if (tel[i].misses < 255) tel[i].misses++;
    }
  }
}

static void thetaToContinuous() {
  for (uint8_t i = 0; i < joints.n; i++)
    if (isTheta(joints.j[i].id)) bus.write16(joints.j[i].id, reg::TORQUE_LIMIT, joints.j[i].torque_cont);
}

static void checkFaults() {
  uint32_t now = millis();
  if (!estop_ok && (state == ARMING || state == READY || state == ACTIVE || state == HOLD || state == MAINT))
    fault(F_ESTOP_OPEN, 0, "E-stop chain open or arm undocked");
  // power present although permission is withdrawn -> welded contact / wiring error
  static uint32_t t_unexp = 0;
  if (bus_on && !safeOkAllowed() && state != BOOT) {
    if (!t_unexp) t_unexp = now;
    if (now - t_unexp > 500) fault(F_BUS_POWER_UNEXPECTED, 0, "servo power present without SAFE_OK: check K1/K2");
  } else {
    t_unexp = 0;
  }
  if (state == READY || state == ACTIVE || state == HOLD) {
    if (bus_on && now - t_bus_on > 500) {
      if (bus_mV < BUS_MIN_MV) fault(F_BUS_UNDERVOLT, 0, "servo bus undervoltage");
      if (bus_mV > BUS_MAX_MV) fault(F_BUS_OVERVOLT, 0, "servo bus overvoltage");
    }
    for (uint8_t i = 0; i < N_SERVOS; i++) {
      if (tel[i].misses >= OFFLINE_STRIKES) fault(F_SERVO_OFFLINE, SERVO_IDS[i], "servo not answering");
      if (!tel[i].online) continue;
      if (tel[i].temp_C >= TEMP_FAULT_C) fault(F_SERVO_OVERTEMP, SERVO_IDS[i], "servo over temperature");
      else if (tel[i].temp_C >= TEMP_WARN_C && !(faults & F_TEMP_WARN)) {
        faults |= F_TEMP_WARN;
        event(F_TEMP_WARN, SERVO_IDS[i], "servo warm: reduce duty");
      }
      // STS status bits: 0 voltage, 1 sensor, 2 temperature, 3 current, 5 overload
      if (tel[i].status & 0x2F) fault(F_SERVO_ERROR, SERVO_IDS[i], "servo reports an error");
    }
  }
  if (state == ACTIVE && now - t_last_host > HOST_TIMEOUT_MS) {
    holdPosition();
    faults |= F_HOST_TIMEOUT;
    event(F_HOST_TIMEOUT, 0, "host silent: holding position, send ARM to resume");
    setState(HOLD);
  }
  if (state == ARMING && now - t_arming > ARMING_TIMEOUT_MS) {
    faults |= F_ARMING_TIMEOUT;
    event(F_ARMING_TIMEOUT, 0, "START not pressed in time");
    setState(SAFE_OFF);
  }
  if (knob_mode && now - t_knob > KNOB_MODE_MAX_MS) {
    JointCfg *c = joints.find(CLAW_ID);
    bus.write16(CLAW_ID, reg::TORQUE_LIMIT, c ? c->torque_cont : 250);
    thetaToContinuous();
    knob_mode = false;
    t_knob_end = now;
    faults |= F_KNOB_TIMEOUT;
    event(F_KNOB_TIMEOUT, CLAW_ID, "knob mode ended (time limit)");
  }
}

// ------------------------------------------------------------------ telemetry
static void sendState() {
  uint8_t p[16 + 14 * MAX_SERVOS];
  size_t k = 0;
  p[k++] = state;
  memcpy(p + k, &faults, 4); k += 4;
  p[k++] = uint8_t((estop_ok ? 1 : 0) | (docked ? 2 : 0) | (bus_on ? 4 : 0) | (digitalRead(PIN_SAFE_OK) ? 8 : 0) |
                   (knob_mode ? 16 : 0) | (joints.allCalibrated() ? 32 : 0));
  memcpy(p + k, &bus_mV, 2); k += 2;
  uint32_t up = millis();
  memcpy(p + k, &up, 4); k += 4;
  p[k++] = N_SERVOS;
  for (uint8_t i = 0; i < N_SERVOS; i++) {
    const sts::Telemetry &t = tel[i];
    p[k++] = SERVO_IDS[i];
    p[k++] = uint8_t((t.online ? 1 : 0) | (torque_on[i] ? 2 : 0) | (t.moving ? 4 : 0));
    memcpy(p + k, &t.pos, 2); k += 2;
    memcpy(p + k, &t.speed, 2); k += 2;
    memcpy(p + k, &t.load, 2); k += 2;
    p[k++] = t.volt_dV;
    p[k++] = t.temp_C;
    memcpy(p + k, &t.current_mA, 2); k += 2;
    p[k++] = t.status;
  }
  hostLink.send(host::STATE, p, uint16_t(k));
}

static void sendConfig() {
  uint8_t p[2 + 14 * MAX_SERVOS];
  size_t k = 0;
  p[k++] = joints.n;
  for (uint8_t i = 0; i < joints.n; i++) {
    const JointCfg &c = joints.j[i];
    p[k++] = c.id;
    memcpy(p + k, &c.min, 2); k += 2;
    memcpy(p + k, &c.max, 2); k += 2;
    memcpy(p + k, &c.center, 2); k += 2;
    memcpy(p + k, &c.max_speed, 2); k += 2;
    memcpy(p + k, &c.torque_cont, 2); k += 2;
    memcpy(p + k, &c.torque_peak, 2); k += 2;
    p[k++] = c.calibrated;
  }
  hostLink.send(host::CONFIG, p, uint16_t(k));
}

// ------------------------------------------------------------------ host commands
static uint16_t rd16(const uint8_t *p) { return uint16_t(p[0] | (p[1] << 8)); }

static void handle(const host::Frame &f) {
  t_last_host = millis();
  const uint8_t *p = f.payload;
  bool needs_payload = f.type == host::ARM || f.type == host::GOALS || f.type == host::TORQUE ||
                       f.type == host::CLAW_MODE || f.type == host::TORQUE_LIMITS || f.type == host::MAINT ||
                       f.type == host::PASSTHRU || f.type == host::SET_LIMIT;
  if (needs_payload && f.len < 1) { hostLink.ack(f.type, f.seq, host::BAD_ARG); return; }
  switch (f.type) {
    case host::HEARTBEAT:
      return;                                            // no ACK: keeps the link quiet at 50 Hz
    case host::GET_STATE: sendState(); return;
    case host::GET_CONFIG: sendConfig(); return;
    case host::ARM: {
      uint8_t cmd = p[0];
      if (cmd == 1) {
        if (state == SAFE_OFF) {
          if (faults & CRITICAL) { hostLink.ack(f.type, f.seq, host::BAD_STATE); return; }
          if (!joints.allCalibrated()) { faults |= F_NOT_CALIBRATED; hostLink.ack(f.type, f.seq, host::BAD_STATE); return; }
          if (!estop_ok) { hostLink.ack(f.type, f.seq, host::BAD_STATE); return; }
          t_arming = millis();
          setState(ARMING);                              // SAFE_OK high: user presses START now
        } else if (state == READY) {
          allTorque(true);
          setState(ACTIVE);
        } else if (state == HOLD) {
          faults &= ~F_HOST_TIMEOUT;
          setState(ACTIVE);
        } else if (state != ACTIVE) { hostLink.ack(f.type, f.seq, host::BAD_STATE); return; }
      } else if (cmd == 0) {                             // torque off, stay powered
        if (bus_on) allTorque(false);
        if (state == ACTIVE || state == HOLD) setState(READY);
      } else if (cmd == 2) {                             // power down
        if (bus_on) allTorque(false);
        if (state != FAULT) setState(SAFE_OFF);
      }
      hostLink.ack(f.type, f.seq, host::OK);
      return;
    }
    case host::GOALS: {
      if (state != ACTIVE) { hostLink.ack(f.type, f.seq, host::BAD_STATE); return; }
      uint8_t n = p[0];
      if (f.len < 1 + 6 * n || n > N_SERVOS) { hostLink.ack(f.type, f.seq, host::BAD_ARG); return; }
      uint8_t ids[MAX_SERVOS], data[MAX_SERVOS * 7];
      uint8_t m = 0, code = host::OK;
      for (uint8_t i = 0; i < n; i++) {
        const uint8_t *b = p + 1 + 6 * i;
        JointCfg *c = joints.find(b[0]);
        int ix = idx(b[0]);
        if (!c || ix < 0 || !torque_on[ix]) { code = host::BAD_ARG; continue; }
        uint16_t pos = rd16(b + 1), spd = rd16(b + 3);
        if (pos < c->min) { pos = c->min; code = host::CLAMPED; }
        if (pos > c->max) { pos = c->max; code = host::CLAMPED; }
        if (spd == 0 || spd > c->max_speed) spd = c->max_speed;
        uint8_t *d = data + 7 * m;
        d[0] = b[5];                                     // acceleration (x100 steps/s^2)
        d[1] = uint8_t(pos & 0xFF); d[2] = uint8_t(pos >> 8);
        d[3] = 0; d[4] = 0;                              // goal time unused
        d[5] = uint8_t(spd & 0xFF); d[6] = uint8_t(spd >> 8);
        ids[m++] = b[0];
      }
      if (m) bus.syncWrite(reg::ACC, 7, ids, m, data);
      if (code == host::CLAMPED) faults |= F_LIMIT_CLAMPED;
      hostLink.ack(f.type, f.seq, code);
      return;
    }
    case host::TORQUE: {
      if (!(state == READY || state == ACTIVE || state == MAINT)) { hostLink.ack(f.type, f.seq, host::BAD_STATE); return; }
      uint8_t n = p[0];
      for (uint8_t i = 0; i < n && f.len >= 1 + 2 * (i + 1); i++) {
        int ix = idx(p[1 + 2 * i]);
        if (ix < 0) continue;
        bool on = p[2 + 2 * i] != 0;
        JointCfg *c = joints.find(SERVO_IDS[ix]);
        if (on && (state == MAINT || !c || !c->calibrated)) continue;
        if (on) {
          bus.write16(SERVO_IDS[ix], reg::GOAL_POS, tel[ix].pos);
          bus.write16(SERVO_IDS[ix], reg::TORQUE_LIMIT, c->torque_cont);
        }
        bus.write8(SERVO_IDS[ix], reg::TORQUE_ENABLE, on ? 1 : 0);
        torque_on[ix] = on;
      }
      bool any = false;
      for (uint8_t i = 0; i < N_SERVOS; i++) any |= torque_on[i];
      if (any && state == READY) setState(ACTIVE);
      if (!any && state == ACTIVE) setState(READY);
      hostLink.ack(f.type, f.seq, host::OK);
      return;
    }
    case host::CLAW_MODE: {
      JointCfg *c = joints.find(CLAW_ID);
      if (!c || state != ACTIVE) { hostLink.ack(f.type, f.seq, host::BAD_STATE); return; }
      if (p[0] == 1) {
        if (millis() - t_knob_end < KNOB_COOLDOWN_MS && t_knob_end) { hostLink.ack(f.type, f.seq, host::BAD_STATE); return; }
        bus.write16(CLAW_ID, reg::TORQUE_LIMIT, c->torque_peak);
        knob_mode = true;
        t_knob = millis();
      } else {
        bus.write16(CLAW_ID, reg::TORQUE_LIMIT, c->torque_cont);
        thetaToContinuous();
        if (knob_mode) t_knob_end = millis();
        knob_mode = false;
      }
      hostLink.ack(f.type, f.seq, host::OK);
      return;
    }
    case host::TORQUE_LIMITS: {
      if (state != ACTIVE && state != READY) { hostLink.ack(f.type, f.seq, host::BAD_STATE); return; }
      uint8_t n = p[0], code = host::OK;
      for (uint8_t i = 0; i < n && f.len >= 1 + 3 * (i + 1); i++) {
        JointCfg *c = joints.find(p[1 + 3 * i]);
        if (!c) continue;
        uint16_t lim = rd16(p + 2 + 3 * i);
        uint16_t cap = (c->id == CLAW_ID) ? c->torque_cont : c->torque_peak;   // claw above cont only via knob mode
        if (isTheta(c->id)) cap = knob_mode ? min<uint16_t>(c->torque_peak, THETA_TORQUE_CEILING) : c->torque_cont;
        if (lim > cap) { lim = cap; code = host::CLAMPED; }
        bus.write16(c->id, reg::TORQUE_LIMIT, lim);
      }
      hostLink.ack(f.type, f.seq, code);
      return;
    }
    case host::MAINT: {
      bool any = false;
      for (uint8_t i = 0; i < N_SERVOS; i++) any |= torque_on[i];
      if (p[0] == 1 && (state == READY || state == SAFE_OFF) && !any) setState(MAINT);
      else if (p[0] == 0 && state == MAINT) setState(bus_on ? READY : SAFE_OFF);
      else { hostLink.ack(f.type, f.seq, host::BAD_STATE); return; }
      hostLink.ack(f.type, f.seq, host::OK);
      return;
    }
    case host::PASSTHRU: {
      if (state != MAINT) { hostLink.ack(f.type, f.seq, host::BAD_STATE); return; }
      static uint8_t rep[128];
      size_t n = bus.raw(p, f.len, rep, sizeof(rep), 4000);
      hostLink.send(host::PASSTHRU_RESP, rep, uint16_t(n));
      return;
    }
    case host::SET_LIMIT: {
      if (state != MAINT || f.len < 14) { hostLink.ack(f.type, f.seq, host::BAD_STATE); return; }
      JointCfg c;
      c.id = p[0];
      c.min = rd16(p + 1); c.max = rd16(p + 3); c.center = rd16(p + 5);
      c.max_speed = rd16(p + 7); c.torque_cont = rd16(p + 9); c.torque_peak = rd16(p + 11);
      c.calibrated = p[13] ? 1 : 0;
      hostLink.ack(f.type, f.seq, joints.store(c) ? host::OK : host::BAD_ARG);
      if (joints.allCalibrated()) faults &= ~F_NOT_CALIBRATED;
      return;
    }
    case host::CLEAR_FAULTS: {
      // critical faults clear only when their cause is gone; the arm is never re-powered automatically
      if (estop_ok) faults &= ~F_ESTOP_OPEN;
      if (!bus_on || safeOkAllowed()) faults &= ~F_BUS_POWER_UNEXPECTED;
      faults &= ~(F_SERVO_OFFLINE | F_SERVO_ERROR | F_BUS_UNDERVOLT | F_BUS_OVERVOLT | F_LIMIT_CLAMPED |
                  F_ARMING_TIMEOUT | F_KNOB_TIMEOUT | F_WDT_RESET | F_TEMP_WARN);
      bool hot = false;
      for (uint8_t i = 0; i < N_SERVOS; i++) hot |= tel[i].online && tel[i].temp_C >= TEMP_WARN_C;
      if (!hot) faults &= ~F_SERVO_OVERTEMP;
      if (state == FAULT && !(faults & CRITICAL)) setState(SAFE_OFF);
      hostLink.ack(f.type, f.seq, (faults & CRITICAL) ? host::BAD_STATE : host::OK);
      return;
    }
  }
  hostLink.ack(f.type, f.seq, host::UNKNOWN);
}

// ------------------------------------------------------------------ LED
static void led() {
  uint32_t t = millis();
  bool on;
  switch (state) {
    case SAFE_OFF: on = (t % 2000) < 100; break;
    case ARMING: on = (t % 300) < 150; break;
    case READY: on = true; break;
    case ACTIVE: on = (t % 1000) > 60; break;
    case HOLD: on = (t % 1000) < 100 || ((t % 1000) > 200 && (t % 1000) < 300); break;
    case FAULT: on = (t % 120) < 60; break;
    case MAINT: on = (t % 1000) < 500; break;
    default: on = false;
  }
  digitalWrite(PIN_LED, on);
  digitalWrite(PIN_BUZZER, state == FAULT && (t % 1000) < 80);
}

// ------------------------------------------------------------------ Arduino entry points
void setup() {
  pinMode(PIN_SAFE_OK, OUTPUT);
  digitalWrite(PIN_SAFE_OK, LOW);                        // first thing: no permission
  pinMode(PIN_ESTOP_SENSE, INPUT);
  pinMode(PIN_DOCK_SENSE, INPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  analogSetPinAttenuation(PIN_BUS_VSENSE, ADC_11db);
  Serial.setRxBufferSize(1024);
  Serial.begin(HOST_BAUD);
  hostLink.begin(Serial);
  bus.begin(Serial2, BUS_BAUD, PIN_BUS_RX, PIN_BUS_TX, BUS_ECHO);
  joints.begin();
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  // Arduino-ESP32 3.x (ESP-IDF 5): the core may already have started the task watchdog
  esp_task_wdt_config_t wdt_cfg = {};
  wdt_cfg.timeout_ms = WDT_TIMEOUT_S * 1000;
  wdt_cfg.idle_core_mask = 0;
  wdt_cfg.trigger_panic = true;
  if (esp_task_wdt_reconfigure(&wdt_cfg) != ESP_OK) esp_task_wdt_init(&wdt_cfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);                // Arduino-ESP32 2.0.x (ESP-IDF 4.4), as pinned in platformio.ini
#endif
  esp_task_wdt_add(NULL);
  esp_reset_reason_t rr = esp_reset_reason();
  if (rr == ESP_RST_TASK_WDT || rr == ESP_RST_INT_WDT || rr == ESP_RST_WDT || rr == ESP_RST_PANIC)
    faults |= F_WDT_RESET;
  if (!joints.allCalibrated()) faults |= F_NOT_CALIBRATED;
  setState(SAFE_OFF);
}

void loop() {
  esp_task_wdt_reset();
  readInputs();
  host::Frame f;
  while (hostLink.poll(f)) handle(f);
  uint32_t now = millis();
  if (now - t_poll >= STATE_PERIOD_MS) {
    t_poll = now;
    if (state != MAINT) pollServos();
    checkFaults();
    if (state == ARMING && bus_on && now - t_bus_on > 300) setState(READY);   // START pressed, K2 closed
    if ((state == READY || state == ACTIVE || state == HOLD) && !bus_on) {
      for (uint8_t i = 0; i < N_SERVOS; i++) torque_on[i] = false;
      if (!(faults & CRITICAL)) setState(SAFE_OFF);
    }
    sendState();
  }
  digitalWrite(PIN_SAFE_OK, safeOkAllowed() ? HIGH : LOW);
  led();
}
