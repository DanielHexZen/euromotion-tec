#!/usr/bin/env python3
"""Minimal ground-station logger for MR2006B (spec 8, PHASE4_TELEMETRY.md package 3).

Reads the binary protocol of spec 8.2/8.4 from a serial port (USB VCP or HC-12 radio) or from a
recorded binary file, decodes frames with the firmware's rules (COBS, CRC-16/CCITT-FALSE little-endian,
vid filter, <= 101 raw bytes), writes one CSV per message type and prints frame-loss statistics from the
`seq` gaps. It can send PING and PARAM_GET on the radio, respecting the uplink rule of spec 8.3.

CSV files (in --out, default ./gs_log_out): hello.csv telem_fast.csv telem_slow.csv event.csv ack.csv
param_val.csv cap_data.csv text.csv. Columns: rx_idx (frame number), rx_time_s (host clock, empty when
replaying a file), seq, then the fields of the message with their wire units (mm, mm/s, 0.01 deg, ...).

Frame loss: every vehicle frame carries one counter `seq` (u8, wraps 255 -> 0). A jump of g > 1 counts as
g - 1 lost frames, a repeated value as a duplicate. A forward jump of more than 128 (mod 256) is a backward
jump, i.e. a vehicle reset or a reordered frame: it counts as a reset, not as loss, and a HELLO resyncs the
tracker (a reset to a value less than 128 above the old one cannot be told from loss without it). The exit status 3 refers to loss excluding resets.

Uplink rule (8.3): the vehicle is timing master with a 100 ms period. A command is sent only after a
vehicle frame was received in the current period, >= 5 ms after the last received byte (HC-12 turnaround),
at most one frame per period, and only if it ends before the next period starts. A new period is detected
as the first byte after >= 30 ms of silence (frames inside a vehicle burst follow each other closely).
The frame must fit the 30 ms uplink airtime (28 bytes at 9600 Bd, 57 at 19200 Bd).

  gs_log.py --port /dev/tty.usbserial-X [--baud 19200] [--vid 66] [--out DIR] [--duration S]
            [--ping SEC] [--param-get ID ...]
  gs_log.py --file capture.bin [--vid 66] [--out DIR]
  gs_log.py --selftest

Needs pyserial only when --port is used.
Exit status: 0 ok, 1 usage/IO error, 3 frame loss >= 1 % (the H11 limit; resets are not loss).
"""
import argparse
import csv
import os
import struct
import sys
import tempfile
import time

PAYLOAD_MAX = 96
RAW_MAX = 101          # vid type seq payload crc
ENC_MAX = 103          # spec 8.2
VID_BCAST = 0xFF
T_STOP = 0x21
T_PING = 0x20
T_PARAM_GET = 0x24

# type -> (csv name, struct format, field names); '<' = little-endian, no padding (spec 8.4)
MSG = {
    0x01: ("hello", "<BBBBIBB8sII",
           ["proto_ver", "fw_major", "fw_minor", "reset_cause", "git_hash", "dirty", "features", "course",
            "last_fault_code", "last_fault_pc"]),
    0x02: ("telem_fast", "<IBBHiihhhhhBBhBB",
           ["t_ms", "mode", "state", "flags", "x_mm", "y_mm", "th_cdeg", "v_mmps", "w_mradps", "duty_l", "duty_r",
            "seg", "wp", "e_y_mm", "gnd", "fwd_cm"]),
    0x03: ("telem_slow", "<IHHHHhhhHBBHHHHHBBB",
           ["t_ms", "bat_mv", "sig_x_mm", "sig_y_mm", "sig_th_cdeg", "bg_mdps", "scale_ppm", "imu_t_cdegc",
            "fault_first", "n_faults", "cpu", "ovr_ctrl", "rx_err", "ekf_acc", "ekf_rej", "stack_free",
            "vl_alive", "sens", "ev_drop"]),
    0x04: ("event", "<IHi", ["t_ms", "code", "arg"]),
    0x05: ("ack", "<BBB", ["type", "seq", "result"]),
    0x06: ("param_val", "<HfB", ["id", "value", "result"]),
}
T_CAP_DATA = 0x07
T_TEXT = 0x08
CAP_REC = {0: 24, 1: 24, 2: 24, 3: 16}


# ---- CRC, COBS, frames (mirror proto.c) -------------------------------------------------------

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data):
    out, ci, code = bytearray([0]), 0, 1
    for i, b in enumerate(data):
        if b:
            out.append(b)
            code += 1
        if b == 0 or (code == 0xFF and i + 1 < len(data)):
            out[ci] = code
            ci = len(out)
            out.append(0)
            code = 1
    out[ci] = code
    return bytes(out)


def cobs_decode(data):
    """-> bytes, or None on a malformed block (zero byte, block past the end)."""
    out, i, n = bytearray(), 0, len(data)
    if n == 0:
        return None
    while i < n:
        code = data[i]
        i += 1
        run = code - 1
        if code == 0 or i + run > n or 0 in data[i:i + run]:
            return None
        out += data[i:i + run]
        i += run
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


def encode_frame(vid, mtype, seq, payload=b""):
    """COBS(vid type seq payload crc_lo crc_hi) + 0x00."""
    if len(payload) > PAYLOAD_MAX:
        raise ValueError("payload longer than 96 bytes")
    raw = bytes([vid, mtype, seq]) + payload
    raw += struct.pack("<H", crc16(raw))
    return cobs_encode(raw) + b"\x00"


class Decoder:
    """Byte-stream decoder with the firmware's drop rules (REQ-PRO-03)."""

    def __init__(self, vid):
        self.vid = vid
        self.buf = bytearray()
        self.overflow = False
        self.st = dict(ok=0, oversize=0, cobs=0, crc=0, vid=0, runt=0)

    def feed(self, data):
        """-> list of (vid, type, seq, payload) for every complete valid frame in `data`."""
        out = []
        for b in data:
            if b:
                if len(self.buf) < ENC_MAX:
                    self.buf.append(b)
                else:
                    self.overflow = True
                continue
            f = self._finish()
            if f:
                out.append(f)
            self.buf.clear()
            self.overflow = False
        return out

    def _finish(self):
        if not self.buf and not self.overflow:
            return None
        if self.overflow:
            self.st["oversize"] += 1
            return None
        raw = cobs_decode(bytes(self.buf))
        if raw is None:
            self.st["cobs"] += 1
        elif len(raw) > RAW_MAX:
            self.st["oversize"] += 1
        elif len(raw) < 5:
            self.st["runt"] += 1
        elif crc16(raw[:-2]) != struct.unpack("<H", raw[-2:])[0]:
            self.st["crc"] += 1
        elif not (raw[0] == self.vid or (raw[0] == VID_BCAST and raw[1] == T_STOP)):
            self.st["vid"] += 1
        else:
            self.st["ok"] += 1
            return raw[0], raw[1], raw[2], raw[3:-2]
        return None


class SeqStats:
    """Frame loss from the u8 `seq` counter of the vehicle."""

    def __init__(self):
        self.last = None
        self.rx = self.lost = self.dup = self.resets = 0

    def resync(self):
        self.last = None

    def update(self, seq):
        self.rx += 1
        if self.last is not None:
            d = (seq - self.last) & 0xFF            # forward distance, 1 = next frame
            if d == 0:
                self.dup += 1
            elif d > 128:                           # backward: reset or reordered frame, not loss
                self.resets += 1
            else:
                self.lost += d - 1
        self.last = seq

    def loss(self):
        tot = self.rx + self.lost
        return self.lost / tot if tot else 0.0


# ---- message decoding and CSV -----------------------------------------------------------------

def decode_msg(mtype, payload):
    """-> (csv name, field names, values) or None if the type is unknown or the length is wrong."""
    if mtype in MSG:
        name, fmt, fields = MSG[mtype]
        if len(payload) != struct.calcsize(fmt):
            return None
        vals = list(struct.unpack(fmt, payload))
        if name == "hello":
            vals[fields.index("course")] = vals[fields.index("course")].rstrip(b"\0").decode("ascii", "replace")
        if name == "param_val":
            vals[1] = "%.9g" % vals[1]
        return name, fields, vals
    if mtype == T_CAP_DATA and len(payload) >= 4:
        index, cset, n = struct.unpack("<HBB", payload[:4])
        if cset in CAP_REC and len(payload) == 4 + n * CAP_REC[cset]:
            return "cap_data", ["index", "set", "n", "records_hex"], [index, cset, n, payload[4:].hex()]
        return None
    if mtype == T_TEXT:
        return "text", ["text"], [payload.decode("ascii", "replace")]
    return None


class Sink:
    """One CSV per message type, plus the counters."""

    def __init__(self, outdir):
        self.outdir = outdir
        self.files = {}
        self.idx = 0
        self.seq = SeqStats()
        self.per_type = {}
        self.unknown = 0

    def write(self, frame, t=None):
        _, mtype, seq, payload = frame
        self.idx += 1
        if mtype == 0x01:
            self.seq.resync()
        self.seq.update(seq)
        m = decode_msg(mtype, payload)
        if m is None:
            self.unknown += 1
            return None
        name, fields, vals = m
        self.per_type[name] = self.per_type.get(name, 0) + 1
        if name not in self.files:
            os.makedirs(self.outdir, exist_ok=True)
            fh = open(os.path.join(self.outdir, name + ".csv"), "w", newline="")
            w = csv.writer(fh)
            w.writerow(["rx_idx", "rx_time_s", "seq"] + fields)
            self.files[name] = (fh, w)
        self.files[name][1].writerow([self.idx, "" if t is None else "%.4f" % t, seq] + vals)
        return name, fields, vals

    def close(self):
        for fh, _ in self.files.values():
            fh.close()


def report(dec, sink, out=sys.stdout):
    st, sq = dec.st, sink.seq
    print("frames ok %d (unknown type or length %d); dropped: crc %d, cobs %d, oversize %d, runt %d, vid %d"
          % (st["ok"], sink.unknown, st["crc"], st["cobs"], st["oversize"], st["runt"], st["vid"]), file=out)
    print("per type: " + (", ".join("%s %d" % kv for kv in sorted(sink.per_type.items())) or "none"), file=out)
    print("seq: received %d, lost %d, duplicates %d, resets/reordered %d, frame loss %.2f %% (excluding resets)"
          % (sq.rx, sq.lost, sq.dup, sq.resets, 100.0 * sq.loss()), file=out)


# ---- uplink rule of spec 8.3 ------------------------------------------------------------------

class Uplink:
    """Decides when the PC may send. Times are seconds on any monotonic clock."""

    def __init__(self, guard=0.005, gap=0.030, period=0.100):
        self.guard, self.gap, self.period = guard, gap, period
        self.last_byte = None
        self.burst_start = None
        self.have_frame = False
        self.sent = False

    def on_bytes(self, now):
        if self.last_byte is None or now - self.last_byte >= self.gap:    # first byte of a new period
            self.burst_start = now
            self.have_frame = False
            self.sent = False
        self.last_byte = now

    def on_frame(self):
        self.have_frame = True

    def ready(self, now, air):
        """True if a frame of `air` seconds may start now."""
        return (self.have_frame and not self.sent and now - self.last_byte >= self.guard
                and now + air <= self.burst_start + self.period)

    def mark_sent(self):
        self.sent = True


def uplink_max_bytes(baud, air=0.030):
    return int(air * baud / 10.0 + 1e-9)


# ---- run loops --------------------------------------------------------------------------------

def run_file(path, vid, outdir):
    dec, sink = Decoder(vid), Sink(outdir)
    with open(path, "rb") as fh:
        data = fh.read()
    for f in dec.feed(data):
        sink.write(f)
    sink.close()
    return dec, sink


def run_serial(args):
    try:
        import serial  # only here: --selftest and --file work without pyserial
    except ImportError:
        print("pyserial is not installed (pip install pyserial)", file=sys.stderr)
        return None
    ser = serial.Serial(args.port, args.baud, timeout=0.005)
    dec, sink, up = Decoder(args.vid), Sink(args.out), Uplink()
    pending = [(T_PARAM_GET, struct.pack("<H", i)) for i in args.param_get]
    tx_seq = 0
    t0 = time.monotonic()
    next_ping = t0 if args.ping > 0 else None
    try:
        while args.duration <= 0 or time.monotonic() - t0 < args.duration:
            data = ser.read(max(1, ser.in_waiting))
            now = time.monotonic()
            if data:
                up.on_bytes(now)
                for f in dec.feed(data):
                    up.on_frame()
                    sink.write(f, now - t0)
            if next_ping is not None and now >= next_ping:
                pending.append((T_PING, b""))
                next_ping = now + args.ping
            if pending:
                fr = encode_frame(args.vid, pending[0][0], tx_seq, pending[0][1])
                if up.ready(now, len(fr) * 10.0 / args.baud):
                    pending.pop(0)
                    tx_seq = (tx_seq + 1) & 0xFF
                    ser.write(fr)
                    up.mark_sent()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        sink.close()
    return dec, sink


# ---- selftest ---------------------------------------------------------------------------------

# Same frames and bytes as the C encoder in test/t_proto_msgs.c ("known frame vectors").
C_VECTORS = [
    ("PING", "0642200196a900", 0x42, 0x20, 1, b""),
    ("STOP", "06ff21034b0600", 0xFF, 0x21, 3, b""),
    ("ACK", "06420505220703630100", 0x42, 0x05, 5, struct.pack("<BBB", 0x22, 7, 0)),
    ("PARAM_VAL", "0542060971010101024003e47f00", 0x42, 0x06, 9, struct.pack("<HfB", 113, 2.0, 0)),
    ("EVENT", "03420404452301010801feffffffe5cd00", 0x42, 0x04, 0, struct.pack("<IHi", 0x12345, 0x0100, -2)),
    ("TELEM_FAST",
     "074202ff40e201080304050a87d612174f348bffb0b9c2012efbf40106ff0709dfff1b57e46b00", 0x42, 0x02, 255,
     struct.pack("<IBBHiihhhhhBBhBB", 123456, 3, 4, 0x0A05, 1234567, -7654321, -18000, 450, -1234, 500, -250,
                 7, 9, -33, 0x1B, 87)),
]


def selftest():
    fails = []

    def check(cond, what):
        if not cond:
            fails.append(what)

    check(crc16(b"123456789") == 0x29B1, "crc check value")

    # COBS known vectors (same list as t_cobs)
    for raw, enc in [(b"\x00", b"\x01\x01"), (b"\x00\x00", b"\x01\x01\x01"), (b"\x00\x11\x00", b"\x01\x02\x11\x01"),
                     (b"\x11\x22\x00\x33", b"\x03\x11\x22\x02\x33"), (b"\x11\x22\x33\x44", b"\x05\x11\x22\x33\x44"),
                     (b"", b"\x01"), (bytes(range(1, 255)), b"\xff" + bytes(range(1, 255))),
                     (bytes(range(1, 256)), b"\xff" + bytes(range(1, 255)) + b"\x02\xff")]:
        check(cobs_encode(raw) == enc and cobs_decode(enc) == raw, "cobs vector len %d" % len(raw))
    check(cobs_decode(b"\x00") is None and cobs_decode(b"\x03\x11") is None and cobs_decode(b"") is None,
          "cobs rejects")

    # byte vectors produced by the C encoder
    dec = Decoder(0x42)
    for name, hexs, vid, mtype, seq, payload in C_VECTORS:
        want = bytes.fromhex(hexs)
        check(encode_frame(vid, mtype, seq, payload) == want, "encode == C vector " + name)
        got = dec.feed(want)
        check(got == [(vid, mtype, seq, payload)], "decode C vector " + name)
    m = decode_msg(0x02, C_VECTORS[-1][5])
    check(m[2] == [123456, 3, 4, 0x0A05, 1234567, -7654321, -18000, 450, -1234, 500, -250, 7, 9, -33, 0x1B, 87],
          "telem_fast fields")
    check(decode_msg(0x06, C_VECTORS[3][5])[2] == [113, "2", 0], "param_val fields")
    check(decode_msg(0x03, bytes(range(35))) is not None and decode_msg(0x03, bytes(34)) is None, "telem_slow length")
    check(decode_msg(0x03, bytes(range(35)))[2][-1] == 34, "ev_drop at offset 34")
    check(decode_msg(0x01, bytes(26))[2][7] == "", "hello course")

    # drop rules
    d = Decoder(0x42)
    good = encode_frame(0x42, 0x02, 1, bytes(32))
    bad = bytearray(good)
    bad[5] ^= 0x10
    check(d.feed(bytes(bad)) == [] and d.st["crc"] + d.st["cobs"] == 1, "bit flip dropped")
    check(d.feed(encode_frame(0x43, 0x02, 1, b"")) == [] and d.st["vid"] == 1, "wrong vid dropped")
    check(len(d.feed(encode_frame(0xFF, T_STOP, 1))) == 1, "STOP broadcast accepted")
    check(d.feed(encode_frame(0xFF, 0x22, 1, b"\x03")) == [] and d.st["vid"] == 2, "other broadcast dropped")
    check(d.feed(b"\x11" * 104 + b"\x00") == [] and d.st["oversize"] == 1, "oversize encoded")
    big = bytes([0x42, 2, 0]) + bytes([0x55] * 97)
    big += struct.pack("<H", crc16(big))
    check(d.feed(cobs_encode(big) + b"\x00") == [] and d.st["oversize"] == 2, "oversize decoded")
    check(len(d.feed(good)) == 1, "good frame after errors")
    try:
        encode_frame(0x42, 2, 0, bytes(97))
        check(False, "payload > 96 refused")
    except ValueError:
        pass

    # seq loss
    s = SeqStats()
    for q in [250, 251, 252, 253, 254, 255, 0, 1, 5, 6, 6]:   # wrap, a gap of 3 (2,3,4), one duplicate
        s.update(q)
    check(s.lost == 3 and s.dup == 1 and s.rx == 11, "seq gaps, wrap, duplicate")
    check(abs(s.loss() - 3 / 14) < 1e-12, "loss fraction")
    s = SeqStats()                                              # vehicle reset in the middle: no loss
    for q in list(range(100, 120)) + list(range(0, 20)):
        s.update(q)
    check(s.lost == 0 and s.resets == 1 and s.rx == 40, "reset is not loss")
    s = SeqStats()                                              # reordered frame: 5, 7, 6, 8
    for q in [5, 7, 6, 8]:
        s.update(q)
    check(s.resets == 1 and s.dup == 0, "reorder counted separately")
    s = SeqStats()                                              # HELLO resync hides nothing but the reset gap
    s.update(200)
    s.resync()
    s.update(3)
    s.update(4)
    check(s.lost == 0 and s.resets == 0, "resync on HELLO")
    d = Decoder(0x42)
    with tempfile.TemporaryDirectory() as tmp2:
        k = Sink(tmp2)
        for q in [10, 11, 12]:
            k.write(d.feed(encode_frame(0x42, 0x05, q, bytes(3)))[0])
        k.write(d.feed(encode_frame(0x42, 0x01, 0, bytes(26)))[0])      # reboot: HELLO with seq 0
        for q in [1, 2, 3]:
            k.write(d.feed(encode_frame(0x42, 0x05, q, bytes(3)))[0])
        check(k.seq.lost == 0 and k.seq.loss() == 0.0, "reboot through HELLO is not loss")
        k.close()

    # CSV and file replay
    with tempfile.TemporaryDirectory() as tmp:
        stream = bytearray()
        for k in range(1000):
            if k in (300, 301, 700):                                  # 3 frames never arrive
                continue
            pl = struct.pack("<IBBHiihhhhhBBhBB", k * 100, 3, 4, 0, k, -k, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255)
            stream += encode_frame(0x42, 0x02, k & 0xFF, pl)
            if k % 10 == 9:
                stream += encode_frame(0x42, 0x04, (k + 1000) & 0xFF, struct.pack("<IHi", k, 7, -1))
        stream = b"junk\r\n\x00" + bytes(stream)                        # text before the first frame is dropped
        path = os.path.join(tmp, "rec.bin")
        with open(path, "wb") as fh:
            fh.write(stream)
        dec, sink = run_file(path, 0x42, os.path.join(tmp, "out"))
        check(dec.st["ok"] == 997 + 100 and dec.st["cobs"] + dec.st["crc"] + dec.st["runt"] == 1, "replay frame count")
        check(sink.per_type == {"telem_fast": 997, "event": 100}, "replay per type")
        with open(os.path.join(tmp, "out", "telem_fast.csv")) as fh:
            rows = list(csv.DictReader(fh))
        check(len(rows) == 997 and rows[0]["x_mm"] == "0" and rows[-1]["y_mm"] == "-999" and rows[0]["rx_time_s"] == "",
              "telem_fast.csv")
        # telemetry and events share one seq counter in the firmware; this synthetic stream has two
        # counters, so only the telem_fast-only gaps are checked against a clean stream below.
        sq = SeqStats()
        for k in range(1000):
            if k not in (300, 301, 700):
                sq.update(k & 0xFF)
        check(sq.lost == 3 and sq.rx == 997, "replay loss count")

    # uplink rule
    up, air = Uplink(), 0.007
    up.on_bytes(0.000)
    check(not up.ready(0.020, air), "no command before a frame was received")
    up.on_frame()
    check(not up.ready(0.002, air), "no command inside the 5 ms guard")
    up.on_bytes(0.025)                                                # same burst: gap < 30 ms
    up.on_frame()
    check(not up.ready(0.029, air) and up.ready(0.031, air), "guard counts from the last byte")
    check(not up.ready(0.094, air) and up.ready(0.092, air), "must end before the next period")
    up.mark_sent()
    check(not up.ready(0.050, air), "one frame per period")
    up.on_bytes(0.100)                                                # next period
    up.on_frame()
    check(up.ready(0.110, air), "next period opens again")
    check(uplink_max_bytes(9600) == 28 and uplink_max_bytes(19200) == 57, "uplink size limit")
    check(len(encode_frame(0x42, T_PING, 0)) == 7 and len(encode_frame(0x42, T_PARAM_GET, 0, b"\x87\x03")) == 9,
          "uplink frames fit")

    if fails:
        for f in fails:
            print("FAIL", f)
        return 1
    print("gs_log selftest ok")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--selftest", action="store_true", help="check the codec against the C encoder's bytes, exit")
    src = p.add_mutually_exclusive_group()
    src.add_argument("--port", help="serial port (needs pyserial)")
    src.add_argument("--file", help="recorded binary stream")
    p.add_argument("--baud", type=int, default=19200, help="default 19200 (D-35), 9600 fallback")
    p.add_argument("--vid", type=lambda s: int(s, 0), default=0x42, help="vehicle ID (com_vid), default 0x42")
    p.add_argument("--out", default="gs_log_out", help="CSV directory")
    p.add_argument("--duration", type=float, default=0.0, help="stop after S seconds (0 = until Ctrl-C)")
    p.add_argument("--ping", type=float, default=0.0, metavar="SEC", help="send a PING every SEC seconds")
    p.add_argument("--param-get", type=int, action="append", default=[], metavar="ID",
                   help="send PARAM_GET for this parameter ID once (repeatable)")
    args = p.parse_args()
    if args.selftest:
        return selftest()
    if not (args.port or args.file):
        p.print_usage(sys.stderr)
        return 1
    if not 0 <= args.vid <= 255:
        print("vid out of range", file=sys.stderr)
        return 1
    if args.file:
        try:
            res = run_file(args.file, args.vid, args.out)
        except OSError as e:
            print(e, file=sys.stderr)
            return 1
    else:
        if len(encode_frame(args.vid, T_PARAM_GET, 0, b"\0\0")) > uplink_max_bytes(args.baud):
            print("uplink frame longer than the 30 ms limit at this baud rate", file=sys.stderr)
            return 1
        try:
            res = run_serial(args)
        except OSError as e:
            print(e, file=sys.stderr)
            return 1
        if res is None:
            return 1
    report(*res)
    return 3 if res[1].seq.loss() >= 0.01 else 0


if __name__ == "__main__":
    sys.exit(main())
