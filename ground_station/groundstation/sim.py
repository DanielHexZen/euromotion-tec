"""Simulated vehicle on a pseudo-terminal: develop and test the PC side without hardware.

Speaks the real protocol through the real codec. Not a model of the firmware, only of what
the PC layer can observe: HELLO, 10 Hz telemetry, ACKs, E-stop, dead-man timeout.
"""

import math
import os
import random
import select
import threading
import time
import tty

from .protocol.codec import Codec
from .protocol.schema import Schema

FW_HASH = 0xC0FFEE01


class SimVehicle(threading.Thread):
    def __init__(self, schema: Schema, vehicle_id: int = 1, proto_hash: int | None = None,
                 drop: float = 0.0, seed: int = 0):
        super().__init__(name="sim-vehicle", daemon=True)
        self.schema = schema
        self.codec = Codec(schema, vehicle_id)
        self.proto_hash = schema.hash if proto_hash is None else proto_hash
        self.drop = drop                 # probability that a frame to the PC is lost
        self.silent = False              # True: stop ACKing (command-failure tests)
        self.received: list = []         # every decoded PC->vehicle message
        self._rng = random.Random(seed)
        self._master, self._slave = os.openpty()
        tty.setraw(self._slave)          # no echo, no CR/LF translation
        self.port_name = os.ttyname(self._slave)
        self._seq = 0
        self._mode, self._faults = 0, 0
        self._v = self._w = 0
        self._x = self._y = self._yaw = 0.0
        self._last_cmd = 0.0
        self._running = True

    def send(self, name: str, **fields) -> None:
        self._seq = (self._seq + 1) & 0xFF
        frame = self.codec.encode(name, self._seq, **fields)
        if self._rng.random() >= self.drop:
            os.write(self._master, frame)

    def inject(self, data: bytes) -> None:
        """Raw bytes onto the line: foreign transmitters, corruption."""
        os.write(self._master, data)

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        mode = self.schema.enums["Mode"]
        t0 = time.monotonic()
        last_tick = last_hello = t0 - 10
        while self._running:
            now = time.monotonic()
            if now - last_hello >= 1.0:
                last_hello = now
                self.send("HELLO", fw_hash=FW_HASH, proto_hash=self.proto_hash)
            if now - last_tick >= 0.1:
                dt, last_tick = now - last_tick, now
                if self._mode == mode["MANUAL"] and now - self._last_cmd > 0.5:
                    self._v = self._w = 0       # dead-man
                self._yaw += self._w / 1000 * dt
                self._x += self._v * math.cos(self._yaw) * dt
                self._y += self._v * math.sin(self._yaw) * dt
                known = dict(t_ms=int((now - t0) * 1000) & 0xFFFFFFFF, mode=self._mode,
                             faults=self._faults, vbat_mv=7400, x_mm=int(self._x),
                             y_mm=int(self._y), yaw_mrad=int(self._yaw * 1000))
                self.send("TELEM_FAST", **{f.name: known.get(f.name, 0)
                                           for f in self.schema.by_name["TELEM_FAST"].fields})
            if select.select([self._master], [], [], 0.01)[0]:
                for msg in self.codec.feed(os.read(self._master, 4096)):
                    self.received.append(msg)
                    self._handle(msg, now, mode)
        os.close(self._master)
        os.close(self._slave)

    def _handle(self, msg, now: float, mode: dict) -> None:
        f = msg.fields
        if msg.name == "ESTOP":
            self._mode, self._faults, self._v, self._w = mode["FAULT"], 0x10, 0, 0
        elif msg.name == "SET_MODE":
            ok = f["mode"] == mode["IDLE"] or not self._faults
            if ok:
                self._mode = f["mode"]
                self._faults = 0 if f["mode"] == mode["IDLE"] else self._faults
                self._v = self._w = 0
            if not self.silent:
                self.send("ACK", acked_seq=msg.seq, status=0 if ok else 1)
        elif msg.name == "PING":
            if not self.silent:
                self.send("ACK", acked_seq=msg.seq, status=0)
        elif msg.name == "DRIVE" and self._mode == mode["MANUAL"]:
            self._v, self._w, self._last_cmd = f["v_mmps"], f["w_mradps"], now
        elif msg.name == "HEARTBEAT":
            self._last_cmd = now
