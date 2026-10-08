"""Wires port, recorder, codec, stats and command manager into one running link.

Receive chain: port -> recorder (raw bytes, first!) -> codec -> stats / commands / listeners.
"""

import threading
import time

from ..protocol.codec import Codec
from ..protocol.schema import Schema
from ..record.recorder import RX
from ..record.session import Session
from ..transport.serial_port import PortClosed, SerialPort
from .command_manager import CommandManager
from .link_stats import LinkStats
from .writer import Writer


class Link:
    def __init__(self, port: SerialPort, session: Session, schema: Schema, vehicle_id: int):
        self.port, self.session = port, session
        self.codec = Codec(schema, vehicle_id)
        self.stats = LinkStats()
        self.writer = Writer(port, session.recorder, on_error=self._port_lost)
        self.commands = CommandManager(self.codec, self.writer, self.stats)
        self.latest: dict[str, tuple[object, float]] = {}   # name -> (message, monotonic rx time)
        self.listeners: list = []                            # callables taking a Decoded message
        self.hello: dict | None = None
        self.closed = threading.Event()
        self.close_reason = ""
        self._reader = threading.Thread(target=self._read_loop, name="reader", daemon=True)

    def start(self) -> None:
        self.writer.start()
        self.commands.start()
        self._reader.start()

    def wait_hello(self, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while self.hello is None and time.monotonic() < end and not self.closed.is_set():
            time.sleep(0.02)
        return self.hello is not None

    def close(self) -> None:
        self.commands.stop()
        self.writer.stop()
        self.closed.set()
        self._reader.join(timeout=1.0)
        self.port.close()
        self.close_reason = self.close_reason or "closed by operator"
        c = self.codec
        self.session.close(
            close_reason=self.close_reason,
            link={"frames_ok": c.frames_ok, "crc_errors": c.crc_errors, "malformed": c.malformed,
                  "unknown_type": c.unknown_type, "foreign": c.foreign,
                  "lost": self.stats.lost, "received": self.stats.received})

    def _port_lost(self, e: Exception) -> None:
        self.close_reason = f"port lost: {e}"
        self.closed.set()

    def _read_loop(self) -> None:
        while not self.closed.is_set():
            try:
                data = self.port.read()
            except PortClosed as e:
                self._port_lost(e)
                return
            if not data:
                continue
            self.session.recorder.write(RX, data)
            now = time.monotonic()
            for msg in self.codec.feed(data):
                self._handle(msg, now)

    def _handle(self, msg, now: float) -> None:
        self.stats.on_frame(msg.seq)
        self.latest[msg.name] = (msg, now)
        if msg.name == "HELLO":
            if msg.fields != self.hello:   # the vehicle repeats HELLO; write session.json once
                self.commands.on_hello(**msg.fields)
                self.session.update(fw_hash=f"{msg.fields['fw_hash']:#010x}",
                                    vehicle_proto_hash=f"{msg.fields['proto_hash']:#010x}")
                self.hello = msg.fields    # last: wait_hello() returning means the verdict is in
        self.commands.on_message(msg)
        for fn in self.listeners:
            fn(msg)
