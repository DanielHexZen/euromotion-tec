import time

from conftest import wait_for
from groundstation.analysis.loader import load_session
from groundstation.analysis.report import build_report
from groundstation.protocol.codec import Codec
from groundstation.protocol.schema import Schema
from groundstation.record.recorder import RX, TX
from groundstation.record.session import Session


def go_auto(link):
    assert link.commands.send("SET_MODE", mode="AUTO").done.wait(1.0)
    assert wait_for(lambda: link.latest["TELEM_FAST"][0].fields["mode"] == 2)


def test_auto_report_clean_run(rig):
    sim, link = rig()
    go_auto(link)
    link.commands.set_periodic("HEARTBEAT")
    time.sleep(0.5)
    assert link.commands.send("SET_MODE", mode="IDLE").done.wait(1.0)   # allowed during AUTO
    link.close()
    text, clean = build_report(link.session.path)
    assert clean and "No navigation-relevant command sent" in text
    assert "HEARTBEAT" in text and "SET_MODE IDLE" in text


def test_auto_report_flags_drive_in_auto(rig):
    sim, link = rig()
    go_auto(link)
    link.commands.send("DRIVE", v_mmps=300, w_mradps=0)
    link.commands.send("SET_MODE", mode="MANUAL")
    time.sleep(0.3)
    link.close()
    text, clean = build_report(link.session.path)
    assert not clean and "2 navigation-relevant" in text
    assert text.count("FLAGGED") == 2


def test_report_without_auto_phase(rig):
    sim, link = rig()
    time.sleep(0.3)
    link.close()
    text, clean = build_report(link.session.path)
    assert clean and "No AUTO phase" in text


def test_loader_frames_and_time_base(rig):
    sim, link = rig()
    assert link.commands.send("PING").done.wait(1.0)
    assert wait_for(lambda: link.stats.received > 8)
    link.close()
    frames = load_session(link.session.path)
    assert {"HELLO", "TELEM_FAST", "ACK", "PING"} <= set(frames)
    t = frames["TELEM_FAST"]
    assert list(t.columns[:3]) == ["t_ms", "pc_time_ns", "seq"]
    assert t["t_ms"].is_monotonic_increasing and 80 < t["t_ms"].diff().median() < 120
    assert frames["PING"]["t_ms"].notna().all()           # carries the latest telemetry clock


def test_loader_uses_the_protocol_snapshot_of_the_session(rig, schema):
    old = Schema(schema.text.replace("vbat_mv", "vbat_old"))
    sim, link = rig(proto=old)
    assert wait_for(lambda: link.stats.received > 3)
    link.close()
    cols = load_session(link.session.path)["TELEM_FAST"].columns
    assert "vbat_old" in cols and "vbat_mv" not in cols


def test_ten_minute_session_loads_fast(tmp_path, schema):
    codec = Codec(schema, 1)
    s = Session(tmp_path, {"vehicle_id": 1}, schema.text)
    fields = {f.name: 0 for f in schema.by_name["TELEM_FAST"].fields}
    for i in range(6000):                                  # 10 min at 10 Hz
        s.recorder.write(RX, codec.encode("TELEM_FAST", i, **{**fields, "t_ms": i * 100}), i * 10**8)
        s.recorder.write(TX, codec.encode("HEARTBEAT", i), i * 10**8 + 1)
    s.close()
    t0 = time.perf_counter()
    frames = load_session(s.path)
    assert time.perf_counter() - t0 <= 5.0
    assert len(frames["TELEM_FAST"]) == len(frames["HEARTBEAT"]) == 6000
