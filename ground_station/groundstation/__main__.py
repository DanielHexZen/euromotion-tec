"""python -m groundstation run|report|sim"""

import argparse
import sys
import time
from pathlib import Path

from .analysis.report import build_report
from .link.command_manager import SendRefused
from .link.link import Link
from .protocol.schema import Schema
from .record.session import Session
from .sim import SimVehicle
from .transport.serial_port import SerialPort

HELP = """commands: status | ping | mode idle|manual|auto | drive <mm/s> <mrad/s> | hb on|off
          stop (DRIVE 0, ends drive) | estop | quit"""


def cmd_run(a) -> int:
    schema = Schema.load()
    try:
        port = SerialPort(a.port, a.baud)
    except Exception as e:
        print(f"cannot open {a.port}: {e}")
        return 1
    session = Session(Path(a.out), {"port": a.port, "baud": a.baud, "vehicle_id": a.vehicle_id,
                                    "operator": a.operator, "notes": a.notes,
                                    "pc_proto_hash": f"{schema.hash:#010x}"}, schema.text)
    link = Link(port, session, schema, a.vehicle_id)
    link.start()
    print(f"recording to {session.path}")
    if link.wait_hello(a.hello_timeout):
        h = link.hello
        print(f"firmware {h['fw_hash']:#010x}, vehicle protocol {h['proto_hash']:#010x}")
    else:
        print(f"no HELLO within {a.hello_timeout:g} s")
    if link.commands.read_only_reason:
        print(f"READ-ONLY: {link.commands.read_only_reason}. Recording continues, sending is refused.")
    print(HELP)
    try:
        _console(link)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        link.commands.clear_periodic()
        link.close()
        print(f"session closed: {link.close_reason}")
    return 0


def _console(link: Link) -> None:
    cm = link.commands
    modes = link.codec.schema.enums["Mode"]
    while not link.closed.is_set():
        try:
            words = input("> ").split()
        except EOFError:
            return
        if not words:
            continue
        try:
            match words:
                case ["quit"]:
                    return
                case ["status"]:
                    t = link.latest.get("TELEM_FAST")
                    s = link.stats
                    age, p95 = s.last_frame_age(), s.rtt_p95()
                    print(f"confirmed mode {t[0].fields['mode'] if t else '?'} | loss {s.loss_pct:.1f} % | "
                          f"last frame {'-' if age is None else f'{age:.2f} s ago'} | "
                          f"rtt p95 {'-' if p95 is None else f'{p95 * 1000:.0f} ms'} | "
                          f"crc {link.codec.crc_errors} foreign {link.codec.foreign}")
                case ["ping"]:
                    p = cm.send("PING")
                    p.done.wait(1.0)
                    print(p.state, p.reason)
                case ["mode", m]:
                    p = cm.send("SET_MODE", mode=m.upper())
                    p.done.wait(1.0)
                    print(p.state, p.reason)
                case ["drive", v, w]:
                    cm.set_periodic("DRIVE", v_mmps=int(v), w_mradps=int(w))
                    cm.set_periodic("HEARTBEAT")
                case ["stop"]:
                    cm.clear_periodic("DRIVE")
                    cm.clear_periodic("HEARTBEAT")
                    cm.send("DRIVE", v_mmps=0, w_mradps=0)
                case ["hb", "on"]:
                    cm.set_periodic("HEARTBEAT")
                case ["hb", "off"]:
                    cm.clear_periodic("HEARTBEAT")
                case ["estop"]:
                    cm.estop()
                    print("E-STOP sent")
                case _:
                    print(HELP)
        except SendRefused as e:
            print(f"REFUSED: {e}")
        except (KeyError, ValueError) as e:
            print(f"bad command: {e!r}")


def cmd_report(a) -> int:
    text, clean = build_report(Path(a.session))
    print(text)
    return 0 if clean else 2


def cmd_sim(a) -> int:
    sim = SimVehicle(Schema.load(), a.vehicle_id, drop=a.drop)
    sim.start()
    print(f"simulated vehicle on {sim.port_name}  (python -m groundstation run --port {sim.port_name})")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        return 0


def main() -> int:
    ap = argparse.ArgumentParser(prog="groundstation")
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="connect, check versions, record")
    r.add_argument("--port", required=True)
    r.add_argument("--baud", type=int, default=115200)
    r.add_argument("--vehicle-id", type=int, default=1)
    r.add_argument("--out", default="sessions")
    r.add_argument("--operator", default="")
    r.add_argument("--notes", default="")
    r.add_argument("--hello-timeout", type=float, default=3.0)
    r.set_defaults(fn=cmd_run)
    p = sub.add_parser("report", help="AUTO run report for a session folder")
    p.add_argument("session")
    p.set_defaults(fn=cmd_report)
    s = sub.add_parser("sim", help="simulated vehicle on a pseudo-terminal")
    s.add_argument("--vehicle-id", type=int, default=1)
    s.add_argument("--drop", type=float, default=0.0, help="frame loss probability")
    s.set_defaults(fn=cmd_sim)
    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
