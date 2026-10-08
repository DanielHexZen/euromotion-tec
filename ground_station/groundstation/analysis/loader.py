"""Offline decoding of a session folder (R9). raw.bin is the source of truth."""

import json
from pathlib import Path
from typing import Iterator

import pandas as pd

from ..protocol.codec import Codec, Decoded
from ..protocol.schema import Schema
from ..record.recorder import RX, read_records

_VEHICLE_ID_DEFAULT = 1


def open_session(path: Path) -> tuple[dict, Codec]:
    """Metadata plus a codec built from the protocol snapshot stored with the session,
    so a log stays decodable after protocol.yaml has moved on."""
    path = Path(path)
    meta = json.loads((path / "session.json").read_text())
    snap = path / "protocol.yaml"
    schema = Schema(snap.read_text()) if snap.exists() else Schema.load()
    return meta, Codec(schema, meta.get("vehicle_id", _VEHICLE_ID_DEFAULT))


def iter_messages(path: Path, codec: Codec) -> Iterator[tuple[int, int, Decoded]]:
    """Yields (pc_time_ns, direction, message) in log order. Direction is RX or TX.
    Each direction has its own byte stream, so each gets its own frame buffer."""
    tx_codec = Codec(codec.schema, codec.vehicle_id)
    for t, direction, data in read_records(Path(path) / "raw.bin"):
        for msg in (codec if direction == RX else tx_codec).feed(data):
            yield t, direction, msg


def load_session(path: Path) -> dict[str, pd.DataFrame]:
    """One DataFrame per message type. Columns: t_ms, pc_time_ns, seq, then the fields.

    t_ms is the vehicle clock. Messages that carry no clock of their own (ACK, HELLO and
    everything the PC sent) get the t_ms of the latest preceding TELEM_FAST, NaN before it."""
    meta, codec = open_session(path)
    rows: dict[str, list] = {}
    t_ms = float("nan")
    for t, _, msg in iter_messages(path, codec):
        t_ms = msg.fields.get("t_ms", t_ms)
        rows.setdefault(msg.name, []).append(
            {"t_ms": t_ms, "pc_time_ns": t, "seq": msg.seq, **msg.fields})
    return {name: pd.DataFrame(r) for name, r in rows.items()}
