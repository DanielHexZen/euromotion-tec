"""Frame codec. Never raises on bad input from the wire; it counts and drops."""

import binascii
import struct
from dataclasses import dataclass

from . import cobs
from .schema import Schema

HEADER = struct.Struct("<BBB")   # vehicle_id, type, seq
MAX_FRAME = 256                  # longest legal frame is far below this; longer means line noise


@dataclass(frozen=True)
class Decoded:
    name: str
    direction: str
    seq: int
    fields: dict


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)."""
    return binascii.crc_hqx(data, 0xFFFF)


class Codec:
    def __init__(self, schema: Schema, vehicle_id: int):
        self.schema = schema
        self.vehicle_id = vehicle_id
        self.frames_ok = 0
        self.crc_errors = 0
        self.malformed = 0       # bad COBS, wrong length, overlong
        self.unknown_type = 0
        self.foreign = 0         # valid frame, other vehicle ID
        self._buf = bytearray()

    def encode(self, name: str, seq: int, **fields) -> bytes:
        """Returns the frame including the trailing 0x00 delimiter."""
        m = self.schema.by_name[name]
        if set(fields) != {f.name for f in m.fields}:
            raise ValueError(f"{name} takes fields {[f.name for f in m.fields]}")
        values = [self._enum_value(f, fields[f.name]) for f in m.fields]
        body = HEADER.pack(self.vehicle_id, m.id, seq & 0xFF) + m.struct.pack(*values)
        return cobs.encode(body + struct.pack("<H", crc16(body))) + b"\x00"

    def _enum_value(self, f, v):
        return self.schema.enums[f.enum][v] if isinstance(v, str) and f.enum else v

    def feed(self, data: bytes) -> list[Decoded]:
        """Takes raw bytes from the port, returns every complete valid frame in them."""
        out = []
        for b in data:
            if b:
                self._buf.append(b)
                if len(self._buf) > MAX_FRAME:
                    self._buf.clear()
                    self.malformed += 1
            elif self._buf:
                msg = self.decode(bytes(self._buf))
                self._buf.clear()
                if msg:
                    out.append(msg)
        return out

    def decode(self, frame: bytes) -> Decoded | None:
        """frame is one COBS block without the delimiter."""
        try:
            raw = cobs.decode(frame)
        except ValueError:
            self.malformed += 1
            return None
        if len(raw) < HEADER.size + 2:
            self.malformed += 1
            return None
        body, (crc,) = raw[:-2], struct.unpack("<H", raw[-2:])
        if crc16(body) != crc:
            self.crc_errors += 1
            return None
        vid, type_id, seq = HEADER.unpack_from(body)
        if vid != self.vehicle_id:
            self.foreign += 1
            return None
        m = self.schema.by_id.get(type_id)
        if m is None:
            self.unknown_type += 1
            return None
        if len(body) - HEADER.size != m.struct.size:
            self.malformed += 1
            return None
        values = m.struct.unpack_from(body, HEADER.size)
        self.frames_ok += 1
        return Decoded(m.name, m.direction, seq, {f.name: v for f, v in zip(m.fields, values)})
