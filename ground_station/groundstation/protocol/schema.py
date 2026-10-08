"""Parses protocol.yaml. The same file drives the Python codec and the C generator."""

import hashlib
import json
import struct
from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_YAML = Path(__file__).resolve().parents[2] / "protocol.yaml"
_FMT = {"u8": "B", "i8": "b", "u16": "H", "i16": "h", "u32": "I", "i32": "i"}


@dataclass(frozen=True)
class Field:
    name: str
    type: str
    enum: str | None = None


@dataclass(frozen=True)
class MessageDef:
    name: str
    id: int
    direction: str          # "pc2veh" | "veh2pc"
    ack: bool
    fields: tuple[Field, ...]
    struct: struct.Struct


class Schema:
    def __init__(self, text: str):
        self.text = text
        raw = yaml.safe_load(text)
        self.enums: dict[str, dict[str, int]] = raw.get("enums", {})
        self.by_name: dict[str, MessageDef] = {}
        self.by_id: dict[int, MessageDef] = {}
        for name, m in raw["messages"].items():
            fields = tuple(Field(f["name"], f["type"], f.get("enum")) for f in m["fields"])
            fmt = "<" + "".join(_FMT[f.type] for f in fields)
            d = MessageDef(name, m["id"], m["dir"], bool(m.get("ack")), fields, struct.Struct(fmt))
            assert d.id not in self.by_id, f"duplicate message id {d.id:#x}"
            self.by_name[name] = self.by_id[d.id] = d
        # Hash of the parsed content, so comment and whitespace edits do not change it.
        canon = json.dumps(raw, sort_keys=True).encode()
        self.hash = int.from_bytes(hashlib.sha256(canon).digest()[:4], "little")

    @classmethod
    def load(cls, path: Path = DEFAULT_YAML) -> "Schema":
        return cls(Path(path).read_text())
