import time
from pathlib import Path

import pytest

from groundstation.link.link import Link
from groundstation.protocol.schema import Schema
from groundstation.record.session import Session
from groundstation.sim import SimVehicle
from groundstation.transport.serial_port import SerialPort


@pytest.fixture(scope="session")
def schema():
    return Schema.load()


@pytest.fixture
def rig(tmp_path, schema):
    """Factory: simulated vehicle + connected Link. Cleans up whatever it started."""
    started = []

    def make(vehicle_id=1, wait_hello=True, proto=None, **sim_kw):
        proto = proto or schema
        sim = SimVehicle(proto, vehicle_id, **sim_kw)
        sim.start()
        session = Session(tmp_path, {"vehicle_id": vehicle_id}, proto.text)
        link = Link(SerialPort(sim.port_name, 115200), session, proto, vehicle_id)
        link.start()
        started.append((sim, link))
        if wait_hello:
            assert link.wait_hello(2.0), "sim vehicle sent no HELLO"
        return sim, link

    yield make
    for sim, link in started:
        if not link.session.meta.get("closed_cleanly"):
            link.close()
        sim.stop()


def wait_for(cond, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.005)
    return False
