"""Raw byte recorder. Standard library only, no knowledge of the protocol (R3).

Record: pc_time_ns u64 | direction u8 | length u16 | bytes   (little-endian, append-only)
"""

import struct
import threading
import time
from pathlib import Path
from typing import Iterator

RX, TX = 0, 1
_HDR = struct.Struct("<QBH")


class Recorder:
    def __init__(self, path: Path):
        self._f = open(path, "ab")
        self._lock = threading.Lock()

    def write(self, direction: int, data: bytes, pc_time_ns: int | None = None) -> None:
        if not data:
            return
        t = time.time_ns() if pc_time_ns is None else pc_time_ns
        with self._lock:
            self._f.write(_HDR.pack(t, direction, len(data)) + data)
            # Flush to the OS on every record: survives a killed process, which is the
            # failure we care about. ponytail: no fsync, so power loss can still cost the
            # OS cache; add a 1 s fsync timer if that ever matters.
            self._f.flush()

    def close(self) -> None:
        with self._lock:
            self._f.close()


def read_records(path: Path) -> Iterator[tuple[int, int, bytes]]:
    """Yields (pc_time_ns, direction, data). A truncated last record is ignored."""
    with open(path, "rb") as f:
        while head := f.read(_HDR.size):
            if len(head) < _HDR.size:
                return
            t, direction, n = _HDR.unpack(head)
            data = f.read(n)
            if len(data) < n:
                return
            yield t, direction, data
