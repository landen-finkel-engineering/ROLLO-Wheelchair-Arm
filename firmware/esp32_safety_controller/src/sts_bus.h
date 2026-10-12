// Minimal Feetech STS (ST3215) half-duplex bus driver.
// Packet: FF FF ID LEN INSTR PARAM... CHK   LEN = nparams + 2, CHK = ~(ID+LEN+INSTR+params) & 0xFF
// Reply:  FF FF ID LEN ERR   PARAM... CHK.  Multi-byte registers are little-endian (STS series).
#pragma once
#include <Arduino.h>

namespace sts {

enum Instr : uint8_t { PING = 0x01, READ = 0x02, WRITE = 0x03, REG_WRITE = 0x04, ACTION = 0x05,
                       SYNC_READ = 0x82, SYNC_WRITE = 0x83 };

struct Telemetry {          // registers 56..70 of one servo
  bool online = false;
  uint16_t pos = 0;         // 0..4095
  int16_t speed = 0;        // steps/s (sign from bit 15)
  int16_t load = 0;         // -1000..1000 (0.1 % of max; sign from bit 10)
  uint8_t volt_dV = 0;      // 0.1 V
  uint8_t temp_C = 0;
  uint8_t status = 0;       // error bits reported by the servo
  uint8_t moving = 0;
  uint16_t current_mA = 0;  // 6.5 mA per unit
  uint8_t misses = 0;       // consecutive missed replies
};

class Bus {
 public:
  void begin(HardwareSerial &port, uint32_t baud, int rx, int tx, bool echo);
  bool ping(uint8_t id);
  bool read(uint8_t id, uint8_t addr, uint8_t n, uint8_t *out, uint8_t *err = nullptr);
  bool write(uint8_t id, uint8_t addr, const uint8_t *data, uint8_t n, bool wait_reply = true);
  bool write8(uint8_t id, uint8_t addr, uint8_t v) { return write(id, addr, &v, 1); }
  bool write16(uint8_t id, uint8_t addr, uint16_t v) {
    uint8_t b[2] = {uint8_t(v & 0xFF), uint8_t(v >> 8)};
    return write(id, addr, b, 2);
  }
  // one packet for many servos; data = n_ids blocks of `len` bytes
  void syncWrite(uint8_t addr, uint8_t len, const uint8_t *ids, uint8_t n_ids, const uint8_t *data);
  // reads `len` bytes from `addr` of each id; returns number of replies received
  uint8_t syncRead(uint8_t addr, uint8_t len, const uint8_t *ids, uint8_t n_ids, uint8_t *out, bool *ok);
  // raw transaction for maintenance passthrough; returns reply length (0 = none)
  size_t raw(const uint8_t *pkt, size_t n, uint8_t *reply, size_t cap, uint32_t timeout_us);
  static void decodeTelemetry(const uint8_t *r, Telemetry &t);   // r = 15 bytes from reg 56

 private:
  HardwareSerial *port_ = nullptr;
  bool echo_ = false;
  uint8_t tx_[160];
  size_t txn_ = 0;
  void send(uint8_t id, uint8_t instr, const uint8_t *params, uint8_t n);
  bool receive(uint8_t id, uint8_t *params, uint8_t nmax, uint8_t *nout, uint8_t *err, uint32_t timeout_us);
  void flushInput();
};

}  // namespace sts
