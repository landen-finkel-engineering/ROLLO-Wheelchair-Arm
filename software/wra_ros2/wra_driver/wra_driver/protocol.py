"""Jetson <-> ESP32 link: framing, CRC and message (de)serialisation.  Pure Python, no ROS.

Frame (both directions):  A5 5A | type u8 | seq u8 | len u16 LE | payload | crc16 u16 LE
crc16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type..payload.
Identical to firmware/esp32_safety_controller/src/host_link.*; message layouts: docs/08_control_interface.md.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field

SOF = b"\xA5\x5A"

# host -> ESP32
HEARTBEAT, ARM, GOALS, TORQUE, CLAW_MODE, TORQUE_LIMITS = 0x01, 0x02, 0x10, 0x11, 0x12, 0x13
GET_STATE, GET_CONFIG, MAINT, PASSTHRU, SET_LIMIT, CLEAR_FAULTS = 0x20, 0x21, 0x30, 0x31, 0x32, 0x33
# ESP32 -> host
STATE, ACK, EVENT, CONFIG, PASSTHRU_RESP = 0x80, 0x81, 0x82, 0x83, 0x84

ARM_TORQUE_OFF, ARM_ON, ARM_POWER_DOWN = 0, 1, 2
ACK_CODES = {0: "OK", 1: "BAD_STATE", 2: "BAD_ARG", 3: "CLAMPED", 4: "UNKNOWN", 5: "BAD_CRC"}
STATES = {0: "BOOT", 1: "SAFE_OFF", 2: "ARMING", 3: "READY", 4: "ACTIVE", 5: "HOLD", 6: "FAULT", 7: "MAINT"}
FAULTS = {0: "ESTOP_OPEN", 1: "HOST_TIMEOUT", 2: "SERVO_OFFLINE", 3: "SERVO_OVERTEMP", 4: "SERVO_ERROR",
          5: "BUS_UNDERVOLT", 6: "BUS_OVERVOLT", 7: "LIMIT_CLAMPED", 8: "NOT_CALIBRATED",
          9: "BUS_POWER_UNEXPECTED", 10: "ARMING_TIMEOUT", 11: "KNOB_TIMEOUT", 12: "WDT_RESET", 13: "TEMP_WARN"}
CRITICAL = {"ESTOP_OPEN", "SERVO_OFFLINE", "SERVO_OVERTEMP", "SERVO_ERROR", "BUS_UNDERVOLT", "BUS_OVERVOLT",
            "BUS_POWER_UNEXPECTED"}


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(mtype: int, seq: int, payload: bytes = b"") -> bytes:
    hdr = struct.pack("<BBH", mtype, seq & 0xFF, len(payload))
    return SOF + hdr + payload + struct.pack("<H", crc16(hdr + payload))


class Decoder:
    """Incremental frame decoder (feed arbitrary byte chunks)."""

    def __init__(self, max_payload: int = 1024):
        self.buf = bytearray()
        self.max_payload = max_payload
        self.crc_errors = 0

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(SOF)
            if i < 0:
                del self.buf[:-1]
                return out
            if i:
                del self.buf[:i]
            if len(self.buf) < 6:
                return out
            mtype, seq, n = struct.unpack_from("<BBH", self.buf, 2)
            if n > self.max_payload:
                del self.buf[:2]
                continue
            if len(self.buf) < 8 + n:
                return out
            body = bytes(self.buf[2:6 + n])
            (c,) = struct.unpack_from("<H", self.buf, 6 + n)
            if crc16(body) == c:
                out.append((mtype, seq, bytes(self.buf[6:6 + n])))
                del self.buf[:8 + n]
            else:
                self.crc_errors += 1
                del self.buf[:2]


# ---------------------------------------------------------------- payload builders
def p_arm(cmd: int) -> bytes:
    return bytes([cmd])


def p_goals(goals) -> bytes:
    """goals: iterable of (servo_id, counts, speed_steps_s, acc)"""
    goals = list(goals)
    b = bytes([len(goals)])
    for sid, pos, spd, acc in goals:
        b += struct.pack("<BHHB", sid, int(max(0, min(4095, pos))), int(max(0, min(65535, spd))), int(max(0, min(254, acc))))
    return b


def p_torque(items) -> bytes:
    items = list(items)
    return bytes([len(items)]) + b"".join(struct.pack("<BB", sid, 1 if on else 0) for sid, on in items)


def p_torque_limits(items) -> bytes:
    items = list(items)
    return bytes([len(items)]) + b"".join(struct.pack("<BH", sid, int(lim)) for sid, lim in items)


def p_set_limit(sid, lo, hi, center, max_speed, t_cont, t_peak, calibrated=True) -> bytes:
    return struct.pack("<BHHHHHHB", sid, lo, hi, center, max_speed, t_cont, t_peak, 1 if calibrated else 0)


# ---------------------------------------------------------------- payload parsers
@dataclass
class ServoState:
    id: int
    online: bool
    torque_on: bool
    moving: bool
    pos: int
    speed: int
    load: int
    volt: float
    temp: int
    current_mA: int
    status: int


@dataclass
class Telemetry:
    state: str
    faults: list
    estop_ok: bool
    docked: bool
    bus_on: bool
    safe_ok: bool
    knob_mode: bool
    calibrated: bool
    bus_V: float
    uptime_ms: int
    servos: dict = field(default_factory=dict)

    @property
    def critical(self):
        return [f for f in self.faults if f in CRITICAL]


def parse_state(p: bytes) -> Telemetry:
    st, faults, inp, mv, up, n = struct.unpack_from("<BIBHIB", p, 0)
    k = 13
    servos = {}
    for _ in range(n):
        sid, fl, pos, spd, load, vd, temp, cur, status = struct.unpack_from("<BBHhhBBHB", p, k)
        k += 13                                     # 13 bytes per servo record
        servos[sid] = ServoState(sid, bool(fl & 1), bool(fl & 2), bool(fl & 4), pos, spd, load, vd / 10.0, temp, cur, status)
    return Telemetry(STATES.get(st, str(st)), [FAULTS[b] for b in FAULTS if faults & (1 << b)], bool(inp & 1),
                     bool(inp & 2), bool(inp & 4), bool(inp & 8), bool(inp & 16), bool(inp & 32), mv / 1000.0, up, servos)


def parse_ack(p: bytes):
    t, s, c = struct.unpack_from("<BBB", p, 0)
    return t, s, ACK_CODES.get(c, str(c))


def parse_event(p: bytes):
    bit, sid = struct.unpack_from("<IB", p, 0)
    name = [FAULTS[b] for b in FAULTS if bit & (1 << b)]
    return (name[0] if name else hex(bit)), sid, p[5:].decode("ascii", "replace")


def parse_config(p: bytes):
    n = p[0]
    out = {}
    for i in range(n):
        sid, lo, hi, c, ms, tc, tp, cal = struct.unpack_from("<BHHHHHHB", p, 1 + 14 * i)
        out[sid] = dict(min=lo, max=hi, center=c, max_speed=ms, torque_cont=tc, torque_peak=tp, calibrated=bool(cal))
    return out


# ---------------------------------------------------------------- STS packets (maintenance passthrough)
def sts_packet(sid: int, instr: int, params: bytes = b"") -> bytes:
    ln = len(params) + 2
    body = bytes([sid, ln, instr]) + params
    return b"\xFF\xFF" + body + bytes([(~sum(body)) & 0xFF])


def sts_parse_reply(raw: bytes):
    """FF FF ID LEN ERR params CHK -> (id, error, params) or None (first valid reply in raw)."""
    i = raw.find(b"\xFF\xFF")
    while i >= 0:
        if len(raw) < i + 6:
            return None
        sid, ln = raw[i + 2], raw[i + 3]
        end = i + 3 + ln                                   # index of the checksum byte
        if ln >= 2 and len(raw) > end and ((~sum(raw[i + 2:end])) & 0xFF) == raw[end]:
            return sid, raw[i + 4], bytes(raw[i + 5:end])
        i = raw.find(b"\xFF\xFF", i + 1)
    return None
