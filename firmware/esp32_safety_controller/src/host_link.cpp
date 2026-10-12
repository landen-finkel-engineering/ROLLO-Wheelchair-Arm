#include "host_link.h"

namespace host {

uint16_t crc16(const uint8_t *d, size_t n, uint16_t crc) {
  for (size_t i = 0; i < n; i++) {
    crc ^= uint16_t(d[i]) << 8;
    for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
  }
  return crc;
}

bool Link::poll(Frame &out) {
  while (s_->available()) {
    uint8_t b = s_->read();
    switch (st_) {
      case 0: st_ = (b == 0xA5) ? 1 : 0; break;
      case 1: st_ = (b == 0x5A) ? 2 : (b == 0xA5 ? 1 : 0); break;
      case 2: case 3: case 4: case 5:
        hdr_[st_ - 2] = b;
        st_++;
        if (st_ == 6) {
          f_.type = hdr_[0];
          f_.seq = hdr_[1];
          f_.len = uint16_t(hdr_[2] | (hdr_[3] << 8));
          idx_ = 0;
          if (f_.len > MAX_PAYLOAD) { st_ = 0; break; }
          st_ = (f_.len == 0) ? 7 : 6;
        }
        break;
      case 6:
        f_.payload[idx_++] = b;
        if (idx_ >= f_.len) st_ = 7;
        break;
      case 7: crc_[0] = b; st_ = 8; break;
      case 8: {
        crc_[1] = b;
        st_ = 0;
        uint16_t c = crc16(hdr_, 4);
        c = crc16(f_.payload, f_.len, c);
        if (c == uint16_t(crc_[0] | (crc_[1] << 8))) {
          out.type = f_.type;
          out.seq = f_.seq;
          out.len = f_.len;
          memcpy(out.payload, f_.payload, f_.len);
          return true;
        }
        crc_errors++;
        break;
      }
    }
  }
  return false;
}

void Link::send(uint8_t type, const uint8_t *payload, uint16_t len) {
  uint8_t h[6] = {0xA5, 0x5A, type, tx_seq_++, uint8_t(len & 0xFF), uint8_t(len >> 8)};
  uint16_t c = crc16(h + 2, 4);
  c = crc16(payload, len, c);
  uint8_t t[2] = {uint8_t(c & 0xFF), uint8_t(c >> 8)};
  s_->write(h, 6);
  if (len) s_->write(payload, len);
  s_->write(t, 2);
}

void Link::ack(uint8_t type, uint8_t seq, uint8_t code) {
  uint8_t p[3] = {type, seq, code};
  send(ACK, p, 3);
}

}  // namespace host
