import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from groundstation.record.recorder import RX, TX, Recorder, read_records
from groundstation.record.session import Session

ROOT = Path(__file__).resolve().parents[1]


def test_roundtrip_and_truncated_tail(tmp_path):
    p = tmp_path / "raw.bin"
    r = Recorder(p)
    r.write(RX, b"abc", 1)
    r.write(TX, b"\x00\x01", 2)
    r.close()
    assert list(read_records(p)) == [(1, RX, b"abc"), (2, TX, b"\x00\x01")]
    with open(p, "ab") as f:
        f.write(b"\x03\x00\x00")             # half a record, as after a crash
    assert len(list(read_records(p))) == 2


def test_session_files(tmp_path):
    s = Session(tmp_path, {"port": "x"}, "enums: {}\nmessages: {}\n")
    assert json.loads((s.path / "session.json").read_text())["port"] == "x"
    s.close(note="n")
    meta = json.loads((s.path / "session.json").read_text())
    assert meta["closed_cleanly"] and meta["note"] == "n"
    assert (s.path / "protocol.yaml").exists()


def test_kill_minus_9_loses_under_a_second(tmp_path):
    code = (
        "import sys, time\n"
        "from groundstation.record.recorder import Recorder, RX\n"
        "r = Recorder(sys.argv[1])\n"
        "i = 0\n"
        "while True:\n"
        "    r.write(RX, i.to_bytes(4, 'little')); i += 1; print(i, flush=True); time.sleep(0.001)\n")
    p = tmp_path / "raw.bin"
    proc = subprocess.Popen([sys.executable, "-c", code, str(p)], cwd=ROOT,
                            stdout=subprocess.PIPE, text=True)
    time.sleep(0.5)
    os.kill(proc.pid, signal.SIGKILL)
    written = int(proc.stdout.read().split()[-1])
    proc.wait()
    got = len(list(read_records(p)))
    assert written - got <= 1             # in practice 0: every record is flushed to the OS
