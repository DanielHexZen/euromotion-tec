"""Commands are absolute and idempotent, so a retry or duplicate is harmless (R5).

One-shot commands (ack: true in protocol.yaml) are retried until ACKed or until FAIL_S,
then reported failed. Periodic ones (DRIVE, HEARTBEAT) are re-sent every slot and simply
replaced by the newest value. Nothing is queued past FAIL_S: a late command is worse than
a failed one. A slot is the gap right after a received telemetry frame (half-duplex radio).
"""

import threading
import time
from dataclasses import dataclass, field

from ..protocol.codec import Codec
from .link_stats import LinkStats
from .writer import Writer

ESTOP_BURST_S = (0.0, 0.15, 0.30)


class SendRefused(Exception):
    pass


@dataclass
class Pending:
    name: str
    fields: dict
    seq: int
    submitted: float
    last_sent: float | None = None
    state: str = "pending"          # pending | acked | failed
    reason: str = ""
    done: threading.Event = field(default_factory=threading.Event)


class CommandManager:
    RETRY_S = 0.10
    # Failure is noticed on the next tick, up to SLOT_TIMEOUT_S late, so FAIL_S + SLOT_TIMEOUT_S
    # must stay under the spec's 500 ms.
    FAIL_S = 0.38
    SLOT_TIMEOUT_S = 0.10  # send anyway when no telemetry arrives (link down)

    def __init__(self, codec: Codec, writer: Writer, stats: LinkStats, clock=time.monotonic):
        self._codec, self._writer, self._stats, self._clock = codec, writer, stats, clock
        self._lock = threading.RLock()
        self._seq = 0
        self._pending: dict[int, Pending] = {}
        self._periodic: dict[str, dict] = {}
        self.read_only_reason: str | None = "no HELLO from the vehicle yet"
        self.slot = threading.Event()
        self._running = False

    # ---- handshake (R4) ----------------------------------------------------------
    def on_hello(self, fw_hash: int, proto_hash: int) -> None:
        mine = self._codec.schema.hash
        self.read_only_reason = None if proto_hash == mine else (
            f"protocol hash mismatch: vehicle {proto_hash:#010x}, PC {mine:#010x}")

    # ---- sending -----------------------------------------------------------------
    def send(self, name: str, **fields) -> Pending | None:
        """ACKed message -> Pending to wait on. Others go out immediately, returns None."""
        m = self._codec.schema.by_name[name]
        with self._lock:
            self._check_allowed()
            if not m.ack:
                self._transmit(name, fields, self._next_seq())
                return None
            for old in [p for p in self._pending.values() if p.name == name]:
                self._finish(old, "failed", "superseded by a newer command")
            p = Pending(name, fields, self._next_seq(), self._clock())
            self._pending[p.seq] = p
            return p

    def set_periodic(self, name: str, **fields) -> None:
        with self._lock:
            self._check_allowed()
            self._periodic[name] = fields

    def clear_periodic(self, name: str | None = None) -> None:
        with self._lock:
            if name is None:
                self._periodic.clear()
            else:
                self._periodic.pop(name, None)

    def estop(self) -> threading.Thread:
        """Bypasses slots, retries and the UI thread. First frame is queued before returning.
        Also goes out in read-only mode: stopping must never depend on the version check."""
        with self._lock:
            for p in list(self._pending.values()):
                self._finish(p, "failed", "cancelled by E-stop")
            self._periodic.clear()
            self._transmit("ESTOP", {}, self._next_seq(), urgent=True)
        t = threading.Thread(target=self._estop_rest, name="estop-burst", daemon=True)
        t.start()
        return t

    def _estop_rest(self) -> None:
        t0 = time.monotonic()
        for at in ESTOP_BURST_S[1:]:
            time.sleep(max(0.0, t0 + at - time.monotonic()))
            with self._lock:
                self._transmit("ESTOP", {}, self._next_seq(), urgent=True)

    # ---- receiving ---------------------------------------------------------------
    def on_message(self, msg) -> None:
        if msg.name == "TELEM_FAST":
            self.slot.set()
        elif msg.name == "ACK":
            with self._lock:
                p = self._pending.get(msg.fields["acked_seq"])
                if p:
                    self._stats.add_rtt(self._clock() - p.submitted)
                    ok = msg.fields["status"] == 0
                    self._finish(p, "acked" if ok else "failed", "" if ok else f"vehicle status {msg.fields['status']}")

    # ---- slot loop ---------------------------------------------------------------
    def tick(self) -> None:
        now = self._clock()
        with self._lock:
            for p in list(self._pending.values()):
                if now - p.submitted > self.FAIL_S:
                    self._finish(p, "failed", "no ACK")
                elif p.last_sent is None or now - p.last_sent >= self.RETRY_S * 0.9:
                    p.last_sent = now
                    self._transmit(p.name, p.fields, p.seq)
            for name, fields in self._periodic.items():
                self._transmit(name, fields, self._next_seq())

    def start(self) -> None:
        self._running = True
        threading.Thread(target=self._loop, name="commands", daemon=True).start()

    def stop(self) -> None:
        self._running = False
        self.slot.set()

    def _loop(self) -> None:
        while self._running:
            self.slot.wait(self.SLOT_TIMEOUT_S)
            self.slot.clear()
            if self._running:
                self.tick()

    # ---- internals ---------------------------------------------------------------
    def _check_allowed(self) -> None:
        if self.read_only_reason:
            raise SendRefused(self.read_only_reason)

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    def _transmit(self, name, fields, seq, urgent=False) -> None:
        self._writer.send(self._codec.encode(name, seq, **fields), urgent=urgent)

    def _finish(self, p: Pending, state: str, reason: str = "") -> None:
        p.state, p.reason = state, reason
        self._pending.pop(p.seq, None)
        p.done.set()
