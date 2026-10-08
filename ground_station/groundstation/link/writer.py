"""Single writer thread. Urgent frames (E-stop) jump the queue (R6)."""

import collections
import threading
import time

from ..record.recorder import TX, Recorder
from ..transport.serial_port import PortClosed


class Writer(threading.Thread):
    def __init__(self, port, recorder: Recorder, on_error=None):
        super().__init__(name="writer", daemon=True)
        self._port, self._rec, self._on_error = port, recorder, on_error
        self._urgent: collections.deque[bytes] = collections.deque()
        self._normal: collections.deque[bytes] = collections.deque()
        self._cv = threading.Condition()
        self._running = True
        self.last_urgent_write_perf: float | None = None   # for the <= 20 ms measurement

    def send(self, frame: bytes, urgent: bool = False) -> None:
        with self._cv:
            (self._urgent if urgent else self._normal).append(frame)
            self._cv.notify()

    def stop(self) -> None:
        with self._cv:
            self._running = False
            self._cv.notify()

    def run(self) -> None:
        while True:
            with self._cv:
                while self._running and not (self._urgent or self._normal):
                    self._cv.wait()
                if not self._running:
                    return
                urgent = bool(self._urgent)
                frame = (self._urgent if urgent else self._normal).popleft()
            try:
                self._port.write(frame)
            except PortClosed as e:
                if self._on_error:
                    self._on_error(e)
                return
            if urgent:
                self.last_urgent_write_perf = time.perf_counter()
            self._rec.write(TX, frame)
