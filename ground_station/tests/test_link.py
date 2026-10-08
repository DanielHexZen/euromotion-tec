import json
import time

import pytest

from conftest import wait_for
from groundstation.link.command_manager import SendRefused
from groundstation.link.manual import ManualDrive
from groundstation.record.recorder import RX, TX, read_records


def test_handshake_ok_and_session_metadata(rig):
    sim, link = rig()
    assert link.commands.read_only_reason is None
    link.close()
    meta = json.loads((link.session.path / "session.json").read_text())
    assert meta["fw_hash"] == "0xc0ffee01" and meta["closed_cleanly"]


def test_hash_mismatch_is_read_only_but_still_records(rig):
    sim, link = rig(proto_hash=0xDEADBEEF)
    assert "mismatch" in link.commands.read_only_reason
    for send in (lambda: link.commands.send("PING"),
                 lambda: link.commands.set_periodic("HEARTBEAT"),
                 lambda: link.commands.send("DRIVE", v_mmps=0, w_mradps=0)):
        with pytest.raises(SendRefused):
            send()
    assert wait_for(lambda: link.stats.received > 3)           # telemetry keeps flowing
    link.commands.estop().join(1.0)                            # the one thing that still goes out
    assert wait_for(lambda: sum(m.name == "ESTOP" for m in sim.received) == 3)
    link.close()
    raw = list(read_records(link.session.path / "raw.bin"))
    assert raw and sum(d == TX for _, d, _ in raw) == 3        # three ESTOP frames, nothing else


def test_one_shot_acked_and_rtt_measured(rig):
    sim, link = rig()
    assert wait_for(lambda: "TELEM_FAST" in link.latest)
    p = link.commands.send("SET_MODE", mode="MANUAL")
    assert p.done.wait(1.0) and p.state == "acked"
    assert wait_for(lambda: link.latest["TELEM_FAST"][0].fields["mode"] == 1)
    assert link.stats.rtts and link.stats.rtts[-1] < 0.25


def test_unanswered_command_fails_within_500ms(rig):
    sim, link = rig()
    sim.silent = True
    t0 = time.monotonic()
    p = link.commands.send("SET_MODE", mode="MANUAL")
    assert p.done.wait(1.0)
    assert p.state == "failed" and time.monotonic() - t0 < 0.5


def test_retries_are_idempotent(rig):
    sim, link = rig()
    sim.silent = True                       # ACKs never arrive, so the command is re-sent
    p = link.commands.send("SET_MODE", mode="MANUAL")
    p.done.wait(1.0)
    seen = [m for m in sim.received if m.name == "SET_MODE"]
    assert len(seen) >= 3 and len({m.seq for m in seen}) == 1   # same seq, same absolute value
    assert wait_for(lambda: link.latest["TELEM_FAST"][0].fields["mode"] == 1)


def test_newer_command_supersedes_older(rig):
    sim, link = rig()
    sim.silent = True
    a = link.commands.send("SET_MODE", mode="MANUAL")
    b = link.commands.send("SET_MODE", mode="IDLE")
    assert a.state == "failed" and "superseded" in a.reason and b.state == "pending"


def test_estop_latency_and_burst(rig):
    sim, link = rig()
    link.commands.set_periodic("HEARTBEAT")
    link.writer.last_urgent_write_perf = None
    t0 = time.perf_counter()
    burst = link.commands.estop()
    assert wait_for(lambda: link.writer.last_urgent_write_perf is not None, 0.5)
    assert (link.writer.last_urgent_write_perf - t0) * 1000 <= 20
    burst.join(1.0)
    assert wait_for(lambda: sum(m.name == "ESTOP" for m in sim.received) == 3)
    assert not link.commands._periodic                          # operator moved on
    assert wait_for(lambda: link.latest["TELEM_FAST"][0].fields["mode"] == 3)


def test_estop_jumps_the_queue(rig):
    sim, link = rig()
    link.writer.send(b"\x01" * 3 + b"\x00")                     # something already waiting
    for _ in range(50):
        link.writer.send(link.codec.encode("HEARTBEAT", 9))
    link.commands.estop()
    assert wait_for(lambda: any(m.name == "ESTOP" for m in sim.received))
    first_estop = next(i for i, m in enumerate(sim.received) if m.name == "ESTOP")
    assert first_estop <= 2


def test_deadman_release_stops_vehicle(rig):
    sim, link = rig()
    assert link.commands.send("SET_MODE", mode="MANUAL").done.wait(1.0)
    drive = ManualDrive(link.commands)
    drive.update(500, 0)                                        # ignored: dead-man not held
    assert not link.commands._periodic
    drive.deadman(True)
    drive.update(500, 100)
    assert wait_for(lambda: any(m.name == "DRIVE" and m.fields["v_mmps"] == 500 for m in sim.received))
    drive.deadman(False)
    assert wait_for(lambda: sim.received[-1].name == "DRIVE" and sim.received[-1].fields["v_mmps"] == 0)
    n = len(sim.received)
    time.sleep(0.35)
    assert len(sim.received) == n                               # heartbeat and DRIVE both stopped


def test_foreign_and_corrupt_bytes_counted_not_decoded(rig, schema):
    from groundstation.protocol.codec import Codec
    sim, link = rig(vehicle_id=1)
    foreign = Codec(schema, 9)
    sim.inject(foreign.encode("TELEM_FAST", 1, **{f.name: 0 for f in schema.by_name["TELEM_FAST"].fields}))
    sim.inject(b"\x05\x41\x42\x43\x00")                         # garbage frame
    assert wait_for(lambda: link.codec.foreign >= 1 and link.codec.malformed + link.codec.crc_errors >= 1)
    assert all(m.fields.get("x_mm", 0) == 0 for m in [link.latest["TELEM_FAST"][0]])


def test_loss_measured_from_sequence_gaps(rig):
    sim, link = rig(drop=0.3, seed=3)
    assert wait_for(lambda: link.stats.received + link.stats.lost > 40, timeout=8.0)
    assert 10 < link.stats.loss_pct < 55


def test_port_loss_ends_session_cleanly(rig):
    sim, link = rig()
    link.commands.set_periodic("HEARTBEAT")
    sim.stop()
    assert link.closed.wait(2.0)
    link.close()
    meta = json.loads((link.session.path / "session.json").read_text())
    assert meta["closed_cleanly"] and "port lost" in meta["close_reason"]
    assert list(read_records(link.session.path / "raw.bin"))    # log intact
