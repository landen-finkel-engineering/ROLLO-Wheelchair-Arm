// Framed binary link to the Jetson (USB serial).  Identical framing in software/wra_ros2/wra_driver/protocol.py
//   A5 5A | type u8 | seq u8 | len u16 LE | payload[len] | crc16 u16 LE
//   crc16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type..payload
#pragma once
#include <Arduino.h>

namespace host {

static const uint16_t MAX_PAYLOAD = 300;

enum Type : uint8_t {
  // host -> ESP32
  HEARTBEAT = 0x01, ARM = 0x02, GOALS = 0x10, TORQUE = 0x11, CLAW_MODE = 0x12, TORQUE_LIMITS = 0x13,
  GET_STATE = 0x20, GET_CONFIG = 0x21, MAINT = 0x30, PASSTHRU = 0x31, SET_LIMIT = 0x32, CLEAR_FAULTS = 0x33,
  // ESP32 -> host
  STATE = 0x80, ACK = 0x81, EVENT = 0x82, CONFIG = 0x83, PASSTHRU_RESP = 0x84
};

enum Ack : uint8_t { OK = 0, BAD_STATE = 1, BAD_ARG = 2, CLAMPED = 3, UNKNOWN = 4, BAD_CRC = 5 };

struct Frame {
  uint8_t type = 0, seq = 0;
  uint16_t len = 0;
  uint8_t payload[MAX_PAYLOAD];
};

uint16_t crc16(const uint8_t *d, size_t n, uint16_t crc = 0xFFFF);

class Link {
 public:
  void begin(Stream &s) { s_ = &s; }
  bool poll(Frame &out);                                   // true when a complete, valid frame arrived
  void send(uint8_t type, const uint8_t *payload, uint16_t len);
  void ack(uint8_t type, uint8_t seq, uint8_t code);
  uint32_t crc_errors = 0;

 private:
  Stream *s_ = nullptr;
  uint8_t st_ = 0, hdr_[4];
  uint16_t idx_ = 0;
  Frame f_;
  uint8_t crc_[2];
  uint8_t tx_seq_ = 0;
};

}  // namespace host
