"""Software stand-in for the ESP32 supervisor + servos (same protocol), for testing the Jetson
software without hardware:  `python -m wra_driver.fake_esp32 --pty` prints a /dev/pts/N to use
as the serial port, or use FakeSerial() in-process (tests).  The START button is pressed
automatically 0.5 s after ARM; call press_estop() to simulate the E-stop."""
from __future__ import annotations

import queue
import struct
import threading
import time

from . import protocol as pr


class FakeEsp32:
    def __init__(self, ids=(1, 2, 3, 4, 5, 6, 7, 8), centers=None, calibrated=True):
        self.ids = list(ids)
        self.pos = {i: (centers or {}).get(i, 2048) for i in ids}
        self.goal = dict(self.pos)
        self.speed = {i: 1000 for i in ids}
        self.torque = {i: False for i in ids}
        self.limits = {i: (0, 4095) for i in ids}
        self.state = 1                    # SAFE_OFF
        self.faults = 0
        self.estop_ok = True
        self.bus_on = False
        self.knob = False
        self.tlim = {}                      # sid -> Torque_Limit last accepted (0..1000)
        self.calibrated = calibrated
        self.block = {}                   # sid -> (lo, hi) counts the servo cannot leave (simulated object)
        self.t_host = time.time()
        self.t_arm = 0.0
        self.out = queue.Queue()
        self.dec = pr.Decoder()
        self.seq = 0
        self._stop = threading.Event()
        self.th = threading.Thread(target=self._loop, daemon=True)
        self.th.start()

    # ------------------------------------------------------------------ host side I/O
    def host_write(self, data: bytes):
        for mtype, seq, p in self.dec.feed(data):
            self.t_host = time.time()
            self._handle(mtype, seq, p)

    def _send(self, mtype, payload=b""):
        self.seq = (self.seq + 1) & 0xFF
        self.out.put(pr.encode(mtype, self.seq, payload))

    def _ack(self, mtype, seq, code=0):
        self._send(pr.ACK, bytes([mtype, seq, code]))

    def press_estop(self):
        self.estop_ok = False
        self.bus_on = False
        self.torque = {i: False for i in self.ids}
        self.faults |= 1
        self.state = 6
        self._send(pr.EVENT, struct.pack("<IB", 1, 0) + b"E-stop chain open or arm undocked")

    def release_estop(self):
        self.estop_ok = True

    # ------------------------------------------------------------------ protocol
    def _handle(self, t, seq, p):
        if t == pr.HEARTBEAT:
            return
        if t == pr.ARM:
            c = p[0]
            if c == 1:
                if self.state == 1 and self.calibrated and self.estop_ok and not (self.faults & 1):
                    self.state, self.t_arm = 2, time.time()
                elif self.state == 3:
                    for i in self.ids:
                        self.goal[i] = self.pos[i]
                        self.torque[i] = True
                    self.state = 4
                elif self.state == 5:
                    self.state = 4
                elif self.state != 4:
                    return self._ack(t, seq, 1)
            elif c == 0:
                self.torque = {i: False for i in self.ids}
                if self.state in (4, 5):
                    self.state = 3
            elif c == 2:
                self.torque = {i: False for i in self.ids}
                self.bus_on = False
                if self.state != 6:
                    self.state = 1
            return self._ack(t, seq, 0)
        if t == pr.GOALS:
            if self.state != 4:
                return self._ack(t, seq, 1)
            n, code = p[0], 0
            for k in range(n):
                sid, pos, spd, acc = struct.unpack_from("<BHHB", p, 1 + 6 * k)
                lo, hi = self.limits[sid]
                if not lo <= pos <= hi:
                    pos, code = min(max(pos, lo), hi), 3
                self.goal[sid] = pos
                self.speed[sid] = spd or 1500
            return self._ack(t, seq, code)
        if t == pr.CLAW_MODE:
            self.knob = bool(p[0])
            if not self.knob:                       # firmware drops the Theta servos back to continuous
                for sid in self._theta_ids():
                    self.tlim[sid] = min(self.tlim.get(sid, 250), 250)
            return self._ack(t, seq, 0 if self.state == 4 else 1)
        if t == pr.TORQUE_LIMITS:                   # same caps as the firmware (main.cpp TORQUE_LIMITS)
            if self.state not in (3, 4):
                return self._ack(t, seq, 1)
            code = 0
            for k in range(p[0]):
                sid, lim = struct.unpack_from("<BH", p, 1 + 3 * k)
                if sid not in self.ids:
                    continue
                cap = 250 if sid == max(self.ids) else 400            # claw: above 250 only via knob mode
                if sid in self._theta_ids():
                    cap = 300 if self.knob else 250                   # Theta pair tooth limit (docs/05)
                if lim > cap:
                    lim, code = cap, 3
                self.tlim[sid] = lim
            return self._ack(t, seq, code)
        if t == pr.CLEAR_FAULTS:
            if self.estop_ok:
                self.faults = 0
                if self.state == 6:
                    self.state = 1
            return self._ack(t, seq, 0 if not self.faults else 1)
        if t == pr.GET_CONFIG:
            b = bytes([len(self.ids)])
            for i in self.ids:
                b += struct.pack("<BHHHHHHB", i, self.limits[i][0], self.limits[i][1], 2048, 1500, 250, 500,
                                 1 if self.calibrated else 0)
            return self._send(pr.CONFIG, b)
        return self._ack(t, seq, 4 if t not in (pr.TORQUE, pr.TORQUE_LIMITS, pr.MAINT, pr.PASSTHRU, pr.SET_LIMIT) else 0)

    def _theta_ids(self):
        return (5, 7) if len(self.ids) == 8 else ()

    # ------------------------------------------------------------------ simulation loop (50 Hz)
    def _loop(self):
        dt = 0.02
        while not self._stop.is_set():
            now = time.time()
            if self.state == 2 and now - self.t_arm > 0.5:           # user pressed START
                self.bus_on, self.state = True, 3
            if self.state == 4 and now - self.t_host > 0.25:
                for i in self.ids:
                    self.goal[i] = self.pos[i]
                self.state = 5
                self.faults |= 2
                self._send(pr.EVENT, struct.pack("<IB", 2, 0) + b"host silent: holding position")
            for i in self.ids:
                if self.torque[i]:
                    step = self.speed[i] * dt
                    d = self.goal[i] - self.pos[i]
                    self.pos[i] += max(-step, min(step, d))
                    if i in self.block:                               # e.g. the claw stalls on a knob
                        lo, hi = self.block[i]
                        self.pos[i] = min(max(self.pos[i], lo), hi)
            self._send(pr.STATE, self._state_payload())
            time.sleep(dt)

    def _state_payload(self):
        inp = (1 if self.estop_ok else 0) | 2 | (4 if self.bus_on else 0) | (8 if self.state in (2, 3, 4, 5, 7) else 0) | \
            (16 if self.knob else 0) | (32 if self.calibrated else 0)
        b = struct.pack("<BIBHIB", self.state, self.faults, inp, 12000 if self.bus_on else 0, int(time.time() * 1000) & 0xFFFFFFFF,
                        len(self.ids))
        for i in self.ids:
            fl = (1 if self.bus_on else 0) | (2 if self.torque[i] else 0) | (4 if abs(self.goal[i] - self.pos[i]) > 1 else 0)
            b += struct.pack("<BBHhhBBHB", i, fl, int(round(self.pos[i])) & 0xFFF, 0, 0, 120, 35, 150, 0)
        return b

    def stop(self):
        self._stop.set()


class FakeSerial:
    """serial.Serial look-alike connected to a FakeEsp32."""

    def __init__(self, esp: FakeEsp32):
        self.esp = esp
        self.buf = b""

    def write(self, data):
        self.esp.host_write(bytes(data))
        return len(data)

    def read(self, n=1):
        try:
            while len(self.buf) < n:
                self.buf += self.esp.out.get(timeout=0.01)
        except queue.Empty:
            pass
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def close(self):
        self.esp.stop()


def main():
    import argparse
    import os
    import pty
    ap = argparse.ArgumentParser()
    ap.add_argument("--pty", action="store_true", help="expose the fake on a pseudo-terminal")
    a = ap.parse_args()
    esp = FakeEsp32()
    if not a.pty:
        print("use --pty")
        return
    master, slave = pty.openpty()
    print("fake ESP32 on", os.ttyname(slave), flush=True)

    def pump_out():
        while True:
            os.write(master, esp.out.get())
    threading.Thread(target=pump_out, daemon=True).start()
    while True:
        esp.host_write(os.read(master, 1024))


if __name__ == "__main__":
    main()
