"""Link quality from sequence-number gaps and PING round trips (G2, G3)."""

import statistics
import time


class LinkStats:
    # A jump this large is a vehicle reboot (seq restarts), not 100+ lost frames.
    RESYNC_DELTA = 100

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self.reset()

    def reset(self) -> None:
        self.received = 0
        self.lost = 0
        self.rtts: list[float] = []
        self._last_seq: int | None = None
        self._last_rx: float | None = None

    def on_frame(self, seq: int) -> None:
        """Call for every valid vehicle->PC frame, whatever its type."""
        if self._last_seq is not None:
            delta = (seq - self._last_seq) & 0xFF
            if delta == 0:
                return                       # duplicate
            if delta < self.RESYNC_DELTA:
                self.lost += delta - 1
        self._last_seq = seq
        self._last_rx = self._clock()
        self.received += 1

    @property
    def loss_pct(self) -> float:
        total = self.received + self.lost
        return 100.0 * self.lost / total if total else 0.0

    def last_frame_age(self) -> float | None:
        return None if self._last_rx is None else self._clock() - self._last_rx

    def add_rtt(self, seconds: float) -> None:
        self.rtts.append(seconds)

    def rtt_p95(self) -> float | None:
        if len(self.rtts) < 2:
            return None
        return statistics.quantiles(self.rtts, n=20, method="inclusive")[18]
