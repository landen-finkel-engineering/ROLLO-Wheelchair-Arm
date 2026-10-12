#include "sts_bus.h"
#include "config.h"

namespace sts {

void Bus::begin(HardwareSerial &port, uint32_t baud, int rx, int tx, bool echo) {
  port_ = &port;
  echo_ = echo;
  port_->begin(baud, SERIAL_8N1, rx, tx);
  port_->setTimeout(2);
}

void Bus::flushInput() {
  while (port_->available()) port_->read();
}

void Bus::send(uint8_t id, uint8_t instr, const uint8_t *params, uint8_t n) {
  size_t k = 0;
  tx_[k++] = 0xFF;
  tx_[k++] = 0xFF;
  tx_[k++] = id;
  tx_[k++] = uint8_t(n + 2);
  tx_[k++] = instr;
  uint8_t sum = id + uint8_t(n + 2) + instr;
  for (uint8_t i = 0; i < n; i++) {
    tx_[k++] = params[i];
    sum += params[i];
  }
  tx_[k++] = uint8_t(~sum);
  txn_ = k;
  flushInput();
  port_->write(tx_, k);
  port_->flush();               // wait until the last stop bit left the UART (adapter switches direction)
  if (echo_) {                  // swallow our own bytes if the adapter echoes them
    uint32_t t0 = micros();
    size_t got = 0;
    while (got < k && micros() - t0 < 1000) {
      if (port_->available()) { port_->read(); got++; }
    }
  }
}

bool Bus::receive(uint8_t id, uint8_t *params, uint8_t nmax, uint8_t *nout, uint8_t *err, uint32_t timeout_us) {
  // state machine: FF FF ID LEN ERR params CHK
  uint32_t t0 = micros();
  uint8_t st = 0, rid = 0, len = 0, e = 0, sum = 0, cnt = 0;
  while (micros() - t0 < timeout_us) {
    if (!port_->available()) continue;
    uint8_t b = port_->read();
    switch (st) {
      case 0: st = (b == 0xFF) ? 1 : 0; break;
      case 1: st = (b == 0xFF) ? 2 : 0; break;
      case 2:
        if (b == 0xFF) break;     // tolerate an extra 0xFF
        rid = b; sum = b; st = 3; break;
      case 3:
        len = b; sum += b;
        if (len < 2 || len - 2 > nmax) { st = 0; break; }
        st = 4; break;
      case 4:
        e = b; sum += b; cnt = 0;
        st = (len == 2) ? 6 : 5; break;
      case 5:
        params[cnt++] = b; sum += b;
        if (cnt >= len - 2) st = 6;
        break;
      case 6:
        if (uint8_t(~sum) == b && rid == id) {
          if (nout) *nout = cnt;
          if (err) *err = e;
          return true;
        }
        st = 0; break;
    }
  }
  return false;
}

bool Bus::ping(uint8_t id) {
  send(id, PING, nullptr, 0);
  uint8_t n, e;
  return receive(id, nullptr, 0, &n, &e, BUS_TIMEOUT_US);
}

bool Bus::read(uint8_t id, uint8_t addr, uint8_t n, uint8_t *out, uint8_t *err) {
  uint8_t p[2] = {addr, n};
  send(id, READ, p, 2);
  uint8_t got = 0, e = 0;
  if (!receive(id, out, n, &got, &e, BUS_TIMEOUT_US)) return false;
  if (err) *err = e;
  return got == n;
}

bool Bus::write(uint8_t id, uint8_t addr, const uint8_t *data, uint8_t n, bool wait_reply) {
  uint8_t p[64];
  if (n > 62) return false;
  p[0] = addr;
  memcpy(p + 1, data, n);
  send(id, WRITE, p, uint8_t(n + 1));
  if (!wait_reply || id == 0xFE) return true;
  uint8_t got, e;
  return receive(id, nullptr, 0, &got, &e, BUS_TIMEOUT_US) && e == 0;
}

void Bus::syncWrite(uint8_t addr, uint8_t len, const uint8_t *ids, uint8_t n_ids, const uint8_t *data) {
  uint8_t p[150];
  size_t k = 0;
  p[k++] = addr;
  p[k++] = len;
  for (uint8_t i = 0; i < n_ids && k + 1 + len < sizeof(p); i++) {
    p[k++] = ids[i];
    memcpy(p + k, data + i * len, len);
    k += len;
  }
  send(0xFE, SYNC_WRITE, p, uint8_t(k));   // broadcast: no reply
}

uint8_t Bus::syncRead(uint8_t addr, uint8_t len, const uint8_t *ids, uint8_t n_ids, uint8_t *out, bool *ok) {
  uint8_t p[2 + MAX_SERVOS];
  p[0] = addr;
  p[1] = len;
  memcpy(p + 2, ids, n_ids);
  send(0xFE, SYNC_READ, p, uint8_t(2 + n_ids));
  uint8_t cnt = 0;
  for (uint8_t i = 0; i < n_ids; i++) {     // servos answer in the order requested
    uint8_t got = 0, e = 0;
    ok[i] = receive(ids[i], out + i * len, len, &got, &e, BUS_TIMEOUT_US) && got == len;
    if (ok[i]) cnt++;
  }
  return cnt;
}

size_t Bus::raw(const uint8_t *pkt, size_t n, uint8_t *reply, size_t cap, uint32_t timeout_us) {
  flushInput();
  port_->write(pkt, n);
  port_->flush();
  uint32_t t0 = micros();
  size_t k = 0;
  while (micros() - t0 < timeout_us && k < cap) {
    if (port_->available()) {
      reply[k++] = port_->read();
      t0 = micros() - timeout_us / 2;      // after the first byte keep reading until a 0.5 timeout gap
    }
  }
  return k;
}

static int16_t signMag(uint16_t v, uint8_t bit) {
  int16_t m = int16_t(v & ((1u << bit) - 1));
  return (v & (1u << bit)) ? -m : m;
}

void Bus::decodeTelemetry(const uint8_t *r, Telemetry &t) {
  t.pos = uint16_t(r[0] | (r[1] << 8)) & 0x0FFF;
  t.speed = signMag(uint16_t(r[2] | (r[3] << 8)), 15);
  t.load = signMag(uint16_t(r[4] | (r[5] << 8)), 10);
  t.volt_dV = r[6];
  t.temp_C = r[7];
  t.status = r[9];
  t.moving = r[10];
  t.current_mA = uint16_t(uint16_t(r[13] | (r[14] << 8)) * 65 / 10);
}

}  // namespace sts
