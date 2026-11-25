#!/usr/bin/env python3
"""S2 run statistics for MR2006B, tests H9b and H9 (spec 14.3, D-79), from the console `VLS` stream.

Firmware lines (USB console log, other lines are ignored):
    VLS ON <label>            start of a run (`VLS <label>` typed in the console)
    VLS <t_us> <seq> <range mm> <status> <signal kcps> <ambient kcps> <h 0.1 mm> <class>
    VLS OFF <label>           end of the run (`VLS` again)
h = range * sin(lat_tilt + roll) is the drop below the sensor that the classifier uses (REQ-LAT-07);
z = lat_h - h is the height of the hit point above the path (positive: raised edge, negative: drop).

Per run: samples, missing results (seq gaps), status counts, range median / mean / sd of the valid
samples and its deviation from the flat-path value lat_h / sin(tilt) (H9b: within +/- 5 % on flat
path), z mean / sd, and the class shares. With --expect LABEL=CLASS the share of that class in the run
is checked against --min-correct (H9b 95 %, H9 99 %).

  vls_eval.py LOG [--h 0.07] [--tilt-deg 15] [--expect d20=EDGE_UP ...] [--min-correct 95]
  vls_eval.py --selftest
Exit status: 0 ok, 1 usage/data error, 3 an --expect check failed.
"""
import argparse
import math
import random
import re
import sys

DATA = re.compile(r"VLS (\d+) (\d+) (\d+) (\d+) (-?\d+) (-?\d+) (-?\d+) ([A-Z_]+)")
MARK = re.compile(r"VLS (ON|OFF) ?(\S*)")
CLASSES = ("PATH", "GRASS", "EDGE_UP", "DROP", "UNKNOWN")


def parse(lines):
    """-> dict label -> list of (seq, range_m, status, signal, ambient, h_m, cls); unlabelled runs 'run<k>'."""
    runs, cur, k = {}, None, 0
    for ln in lines:
        ln = ln.strip()
        m = MARK.fullmatch(ln)
        if m:
            if m.group(1) == "ON":
                k += 1
                cur = m.group(2) or f"run{k}"
                runs.setdefault(cur, [])
            else:
                cur = None
            continue
        m = DATA.fullmatch(ln)
        if m and cur is not None:
            g = m.groups()
            runs[cur].append((int(g[1]), int(g[2]) / 1000.0, int(g[3]), int(g[4]), int(g[5]),
                              int(g[6]) / 10000.0, g[7]))
    return runs


def stats(rows, h0, tilt):
    n = len(rows)
    seqs = [r[0] for r in rows]
    missing = sum(((b - a) & 0xFFFF) - 1 for a, b in zip(seqs, seqs[1:]) if ((b - a) & 0xFFFF) > 1)
    valid = [r for r in rows if r[2] == 0]
    st = {}
    for r in rows:
        st[r[2]] = st.get(r[2], 0) + 1
    cls = {c: sum(1 for r in rows if r[6] == c) / n * 100.0 for c in CLASSES} if n else {}
    out = {"n": n, "missing": missing, "status": st, "cls": cls}
    if valid:
        rg = sorted(r[1] for r in valid)
        mean = sum(rg) / len(rg)
        z = [h0 - r[5] for r in valid]
        zm = sum(z) / len(z)
        out.update(med=rg[len(rg) // 2], mean=mean,
                   sd=math.sqrt(sum((x - mean) ** 2 for x in rg) / len(rg)),
                   flat_dev=(rg[len(rg) // 2] / (h0 / math.sin(tilt)) - 1.0) * 100.0,
                   z_mean=zm, z_sd=math.sqrt(sum((x - zm) ** 2 for x in z) / len(z)))
    return out


def report(runs, h0, tilt, expect, min_correct, out=print):
    out(f"flat-path slant range lat_h / sin(tilt) = {h0 / math.sin(tilt) * 1000:.1f} mm "
        f"(lat_h {h0 * 1000:.1f} mm, tilt {math.degrees(tilt):.2f} deg)")
    out("label          n  miss  valid%  r_med  r_sd   flat%   z_mean  z_sd   PATH GRASS EDGE_UP DROP UNKN")
    fail = False
    for lab, rows in runs.items():
        s = stats(rows, h0, tilt)
        if s["n"] == 0:
            out(f"{lab:12s} no samples")
            continue
        v = s["status"].get(0, 0) / s["n"] * 100.0
        c = s["cls"]
        if "med" in s:
            out(f"{lab:12s} {s['n']:5d} {s['missing']:4d} {v:6.1f}  {s['med'] * 1000:5.0f} {s['sd'] * 1000:5.1f} "
                f"{s['flat_dev']:+6.1f}  {s['z_mean'] * 1000:+6.1f} {s['z_sd'] * 1000:5.1f}   "
                f"{c['PATH']:4.0f} {c['GRASS']:5.0f} {c['EDGE_UP']:7.0f} {c['DROP']:4.0f} {c['UNKNOWN']:4.0f}")
        else:
            out(f"{lab:12s} {s['n']:5d} {s['missing']:4d} {v:6.1f}  no valid sample; status {s['status']}")
        if lab in expect:
            share = c.get(expect[lab], 0.0)
            ok = share >= min_correct
            fail = fail or not ok
            out(f"  {lab}: {expect[lab]} {share:.1f} % (>= {min_correct:g} %) {'PASS' if ok else 'FAIL'}")
    for lab in expect:
        if lab not in runs:
            out(f"  {lab}: no such run in the log")
            fail = True
    return 3 if fail else 0


def selftest():
    rnd = random.Random(3)
    h0, tilt = 0.07, math.radians(15.0)
    r0 = h0 / math.sin(tilt)
    lines, seq = ["> VLS flat", "VLS ON flat"], 0
    for k in range(1000):
        seq += 1
        if k == 500:
            seq += 2                                     # two results lost
        r = r0 * 1.01 + rnd.gauss(0.0, 0.002)
        lines.append(f"VLS {k * 20000} {seq} {round(r * 1000)} 0 900 40 {round(r * math.sin(tilt) * 10000)} PATH")
    lines += ["VLS OFF flat", "noise line", "VLS ON d20"]
    for k in range(200):
        seq += 1
        r = 0.20 + rnd.gauss(0.0, 0.002)
        cls = "EDGE_UP" if k % 20 else "GRASS"
        lines.append(f"VLS {k * 20000} {seq} {round(r * 1000)} {0 if k % 50 else 4} 900 40 "
                     f"{round(r * math.sin(tilt) * 10000)} {cls}")
    lines.append("VLS OFF d20")
    runs = parse(lines)
    msgs = []
    code = report(runs, h0, tilt, {"d20": "EDGE_UP"}, 95.0, out=msgs.append)
    s = stats(runs["flat"], h0, tilt)
    ok = True
    def chk(c, what):
        nonlocal ok
        print(("ok    " if c else "FAIL  ") + what)
        ok = ok and c
    chk(set(runs) == {"flat", "d20"} and len(runs["flat"]) == 1000, "runs split by VLS ON/OFF labels, other lines ignored")
    chk(s["missing"] == 2, "lost results counted from seq gaps")
    chk(abs(s["flat_dev"] - 1.0) < 0.3, f"flat-path deviation +1 % found ({s['flat_dev']:+.2f} %)")
    chk(abs(s["z_mean"] + 0.0007) < 0.0002, f"z = lat_h - h of a 1 % long range is -0.7 mm ({s['z_mean'] * 1000:+.2f} mm)")
    chk(code == 0 and any("EDGE_UP 95.0 %" in m and "PASS" in m for m in msgs), "EDGE_UP share 95 % passes at 95 %")
    code2 = report(runs, h0, tilt, {"d20": "EDGE_UP"}, 99.0, out=msgs.append)
    chk(code2 == 3, "the same run fails at 99 % (exit 3)")
    print("SELFTEST PASSED" if ok else "SELFTEST FAILED")
    return 0 if ok else 1


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("log", nargs="?")
    p.add_argument("--h", type=float, default=0.07, help="lat_h, optical centre of S2 above the path [m] (P671)")
    p.add_argument("--tilt-deg", type=float, default=15.0, help="lat_tilt [deg] (P670)")
    p.add_argument("--expect", action="append", default=[], metavar="LABEL=CLASS")
    p.add_argument("--min-correct", type=float, default=95.0, help="required class share in %% (H9b 95, H9 99)")
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args(argv)
    if a.selftest:
        return selftest()
    if not a.log:
        p.print_usage()
        return 1
    expect = {}
    for e in a.expect:
        lab, _, cls = e.partition("=")
        if cls not in CLASSES:
            print(f"--expect {e}: class must be one of {', '.join(CLASSES)}")
            return 1
        expect[lab] = cls
    with open(a.log, errors="replace") as f:
        runs = parse(f)
    if not runs:
        print("no VLS runs in the log")
        return 1
    return report(runs, a.h, math.radians(a.tilt_deg), expect, a.min_correct)


if __name__ == "__main__":
    sys.exit(main())
