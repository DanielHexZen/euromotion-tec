import os
import random
import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest

from groundstation.protocol import cobs
from groundstation.protocol.codec import Codec, crc16
from groundstation.protocol.schema import Schema

ROOT = Path(__file__).resolve().parents[1]
RANGE = {"u8": (0, 255), "i8": (-128, 127), "u16": (0, 65535), "i16": (-32768, 32767),
         "u32": (0, 2**32 - 1), "i32": (-2**31, 2**31 - 1)}


def sample_fields(m, rng):
    return {f.name: rng.randint(*RANGE[f.type]) for f in m.fields}


def test_crc_check_value():
    assert crc16(b"123456789") == 0x29B1     # CRC-16/CCITT-FALSE reference


def test_cobs_roundtrip_and_no_zero():
    rng = random.Random(1)
    cases = [b"", b"\x00", b"\x00\x00", bytes(range(1, 255)), bytes(range(1, 256)) * 2, b"\x01" * 254]
    cases += [bytes(rng.choice([0, 0, 1, 255, rng.randrange(256)]) for _ in range(rng.randrange(600)))
              for _ in range(300)]
    for data in cases:
        enc = cobs.encode(data)
        assert 0 not in enc
        assert cobs.decode(enc) == data


def test_roundtrip_every_message_type(schema):
    rng = random.Random(2)
    codec = Codec(schema, 7)
    for m in schema.by_name.values():
        for _ in range(50):
            fields = sample_fields(m, rng)
            seq = rng.randrange(256)
            frame = codec.encode(m.name, seq, **fields)
            assert frame[-1] == 0 and 0 not in frame[:-1]
            (msg,) = codec.feed(frame)
            assert (msg.name, msg.seq, msg.fields) == (m.name, seq, fields)
    assert codec.crc_errors == codec.malformed == codec.foreign == 0


def test_enum_by_name(schema):
    codec = Codec(schema, 1)
    (msg,) = codec.feed(codec.encode("SET_MODE", 1, mode="AUTO"))
    assert msg.fields["mode"] == schema.enums["Mode"]["AUTO"]


def test_stream_split_across_chunks(schema):
    codec = Codec(schema, 1)
    stream = b"".join(codec.encode("DRIVE", i, v_mmps=i, w_mradps=-i) for i in range(20))
    got = []
    for i in range(0, len(stream), 3):
        got += codec.feed(stream[i:i + 3])
    assert [m.seq for m in got] == list(range(20))


def test_any_single_corruption_is_counted_never_raised(schema):
    codec = Codec(schema, 1)
    frame = codec.encode("TELEM_FAST", 5, **sample_fields(schema.by_name["TELEM_FAST"], random.Random(3)))
    for i in range(len(frame) - 1):          # every byte except the delimiter
        for flip in (0x01, 0x80, 0xFF):
            bad = bytearray(frame)
            bad[i] ^= flip
            bad = bytes(bad)
            if bad[i] == 0 or bad == frame:  # would change the framing itself; covered below
                continue
            assert codec.feed(bad) == []
    dropped = codec.crc_errors + codec.malformed + codec.unknown_type + codec.foreign
    assert dropped > 0 and codec.frames_ok == 0


def test_noise_never_raises(schema):
    codec = Codec(schema, 1)
    rng = random.Random(4)
    for _ in range(200):
        codec.feed(bytes(rng.randrange(256) for _ in range(rng.randrange(1, 400))))
    codec.feed(b"\x01" * 1000)               # overlong frame without delimiter
    codec.feed(b"\x00")
    assert codec.malformed > 0


def test_foreign_vehicle_dropped_and_counted(schema):
    other, mine = Codec(schema, 2), Codec(schema, 1)
    assert mine.feed(other.encode("PING", 0)) == []
    assert mine.foreign == 1 and mine.crc_errors == 0
    assert len(mine.feed(mine.encode("PING", 0))) == 1


def test_hash_ignores_comments_but_not_content(schema):
    assert Schema(schema.text + "\n# a comment\n").hash == schema.hash
    assert Schema(schema.text.replace("0x05, dir: pc2veh", "0x06, dir: pc2veh")).hash != schema.hash


@pytest.mark.skipif(shutil.which("cc") is None, reason="no C compiler")
def test_generated_c_matches_python(schema, tmp_path):
    """The firmware side packs exactly the bytes the Python codec expects."""
    import sys
    sys.path.insert(0, str(ROOT / "tools"))
    from gen_protocol import generate
    h, c = generate(schema)
    (tmp_path / "link_msgs.h").write_text(h)
    (tmp_path / "link_msgs.c").write_text(c)
    m = schema.by_name["TELEM_FAST"]
    vals = sample_fields(m, random.Random(5))
    inits = ", ".join(f".{k} = {v}" for k, v in vals.items())
    (tmp_path / "main.c").write_text(f'''
#include <stdio.h>
#include "link_msgs.h"
int main(void) {{
    link_TELEM_FAST_t t = {{ {inits} }}, back;
    uint8_t b[64];
    size_t n = link_TELEM_FAST_pack(&t, b);
    for (size_t i = 0; i < n; i++) printf("%02x", b[i]);
    printf(" %d %08x\\n", link_TELEM_FAST_unpack(&back, b, n) && back.x_mm == t.x_mm && back.cnt_l == t.cnt_l,
           LINK_PROTOCOL_HASH);
    return 0;
}}''')
    exe = tmp_path / "t"
    subprocess.run(["cc", "-Wall", "-Werror", "-I", str(tmp_path), "-o", exe, tmp_path / "main.c",
                    tmp_path / "link_msgs.c"], check=True)
    hexbytes, ok, h32 = subprocess.run([exe], capture_output=True, text=True, check=True).stdout.split()
    assert bytes.fromhex(hexbytes) == m.struct.pack(*vals.values())
    assert ok == "1" and int(h32, 16) == schema.hash
