#!/usr/bin/env python3
"""Host evaluation of the calibration firmware's log (APP_CALIB=1, spec 7.27, REQ-CAL-03, docs/CALIB_README.md).

Input is a captured text log (screen -L / CubeIDE terminal); no serial port is opened here.
Line formats are the contract of Challenge_TestHC/Core/Src/calib.c (units on the wire: counts, mm, mm/s, duty %):
  C,<L|R>,<fwd|rev>,<duty %>,<dcnt_l>,<dcnt_r>,<PASS|FAIL>        DONE C PASS|FAIL
  G,<signed duty %>,<v_l mm/s>,<v_r mm/s>                         second half of each 6 s step
  B,<L|R>,<fwd|rev>,<duty %>|NONE                                 breakaway, 1 % resolution
  K,<t_ms>,<duty_milli>,<dcnt_l>,<dcnt_r>                         counts per 20 ms, step at 200 ms
  L,<L|R>,<fwd_cnt>,<rev_cnt>,<fwd_latency_ms>,<rev_latency_ms>
  odo,<cnt_l>,<cnt_r>        FAULT ...        STOP        # comments, ERR ..., status lines (ignored)

Sub-commands
  check LOG         parse; C results, FAULT/STOP/ERR/WARN lines, malformed lines
  eval LOG [LOG..]  Ks, Kv from G (ffit.fit_wheel, per wheel), Ksb from B, tau_m from K; P<id>=<value> lines;
                    --header FILE writes calib_result.h (#define PAR_<name> <value>f, like the console `L`)
  selftest          synthetic logs with known answers

Methods
  G: |duty| = Ks + Kv |v| per wheel, points below --vmin dropped, pooled over both directions (as ffit.py steady).
     The speeds were computed by the firmware with the odo_m_per_cnt * odo_scale active at capture time.
  B: Ksb = mean of the fwd and rev breakaway duty of a wheel (NONE = did not move up to 34 %, not used).
  K: rise of the cumulative count X(t) after the step edge (first nonzero duty row minus 20 ms) is fitted with
     X = vss (t - tau (1 - exp(-t/tau))) (first-order speed, no dead time); vss linear, tau by 1 ms grid search.
     Counts are integers per 20 ms: tau of 50 ms is only 2.5 samples, expect about +-10 ms scatter and a bias
     upwards by any dead time. Repeat K and look at the spread. whl_tau_m = mean over wheels and runs.
  L: reported only. The encoders sit on the motor shaft, gear backlash is not observable (odo_backlash_* is not set).
  odo: counts only; odo_scale_*/odo_wheelbase come from straight_scale.py / umbmark.py.
Exit status: 0 ok, 1 usage/data error, 2 failed test, fault or refused result.
"""
import argparse
import contextlib
import io
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))     # also under python3 -I
from ffit import IDS, NAMES, RANGES, WHEEL, fit_wheel, mean  # noqa: E402

TAU_ID, TAU_NAME, TAU_RANGE = 115, "whl_tau_m", (0.0, 0.5)        # par_table.h
KEYS = ("C,", "G,", "B,", "K,", "L,", "odo,")
NAMES = dict(NAMES)
NAMES[TAU_ID] = TAU_NAME
DT_MS = 20


# ----------------------------------------------------------------------------- parsing
class Log:
    def __init__(self):
        self.c, self.g, self.b, self.l, self.odo = [], [], [], [], []
        self.k = []                                 # list of runs, each a list of (t_ms, duty_milli, dl, dr)
        self.done, self.fault, self.stop, self.err, self.warn, self.bad = [], [], 0, [], [], []


def _ints(f, n):
    if len(f) != n:
        raise ValueError
    return [int(x) for x in f]


def parse(text, log=None):
    log = log or Log()
    for no, raw in enumerate(text.splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#") and "WARN" not in line:
            continue
        try:
            if line.startswith("# WARN"):
                log.warn.append(line)
            elif line.startswith("C,"):
                f = line.split(",")
                if len(f) != 7 or f[2] not in ("fwd", "rev") or f[6] not in ("PASS", "FAIL") or f[1] not in "LR":
                    raise ValueError
                log.c.append((f[1], f[2], int(f[3]), int(f[4]), int(f[5]), f[6]))
            elif line.startswith("G,"):
                log.g.append(tuple(_ints(line.split(",")[1:], 3)))
            elif line.startswith("B,"):
                f = line.split(",")
                if len(f) != 4 or f[1] not in ("L", "R") or f[2] not in ("fwd", "rev"):
                    raise ValueError
                log.b.append((f[1], f[2], None if f[3] == "NONE" else int(f[3])))
            elif line.startswith("K,"):
                row = tuple(_ints(line.split(",")[1:], 4))
                if not log.k or row[0] <= log.k[-1][-1][0]:
                    log.k.append([])
                log.k[-1].append(row)
            elif line.startswith("L,"):
                f = line.split(",")
                if f[1] not in ("L", "R"):
                    raise ValueError
                log.l.append((f[1],) + tuple(_ints(f[2:], 4)))
            elif line.startswith("odo,"):
                log.odo.append(tuple(_ints(line.split(",")[1:], 2)))
            elif line.startswith("DONE"):
                log.done.append(line)
            elif line.startswith("FAULT"):
                log.fault.append(line)
            elif line == "STOP":
                log.stop += 1
            elif line.startswith("ERR"):
                log.err.append(line)
        except (ValueError, IndexError):
            log.bad.append((no, line))
    return log


def read_logs(paths):
    log = Log()
    for p in paths:
        with io.open(p, encoding="utf-8", errors="replace") as f:
            parse(f.read(), log)
    return log


# ----------------------------------------------------------------------------- check
def cmd_check(args):
    log = read_logs([args.log])
    rc = 0
    print("C (sign check):")
    for w, d, duty, dl, dr, res in log.c:
        print("  wheel %s %s at %d %%: dcnt L=%d R=%d  %s" % (w, d, duty, dl, dr, res))
    if log.c and len(log.c) != 4:
        print("  incomplete: %d of 4 results" % len(log.c))
        rc = 2
    if any(r[5] == "FAIL" for r in log.c):
        rc = 2
    if not log.c:
        print("  none in the log")
    for name, n in (("G", len(log.g)), ("B", len(log.b)), ("K runs", len(log.k)), ("L", len(log.l)), ("odo", len(log.odo))):
        print("%s: %d" % (name, n))
    for d in log.done:
        print(d)
    for title, items in (("FAULT", log.fault), ("ERR", log.err), ("WARN", log.warn)):
        for s in items:
            print(s)
    if log.stop:
        print("STOP (abort by 's'): %d" % log.stop)
    if log.fault:
        rc = 2
    for no, line in log.bad:
        print("malformed line %d: %r" % (no, line))
    if log.bad:
        rc = 2
    print("CHECK %s" % ("OK" if rc == 0 else "NOT OK"))
    return rc


# ----------------------------------------------------------------------------- eval
def eval_g(log, vmin, out):
    refused = False
    for i in (0, 1):
        pts = [(r[1 + i] / 1000.0, r[0] / 100.0) for r in log.g if abs(r[1 + i]) / 1000.0 >= vmin]
        f = fit_wheel(pts)
        print("G %s wheel: %d usable of %d points, speed span %.3f m/s" % (WHEEL[i], f["n"], len(log.g), f["span"]))
        if f["n"] < 4 or f["span"] < 0.15 or f["pool"] is None:
            print("  REFUSED: need >= 4 points and a span >= 0.15 m/s")
            refused = True
            continue
        p = f["pool"]
        print("  |duty| = Ks + Kv |v|: Ks = %.4f (se %.4f)  Kv = %.4f 1/(m/s) (se %.4f)  R2 = %.5f"
              % (p["a"], p["se_a"], p["b"], p["se_b"], p["r2"]))
        for key, val in (("ks", p["a"]), ("kv", p["b"])):
            lo, hi = RANGES[key]
            if not lo <= val <= hi:
                print("  REFUSED: %s = %.4f outside %.1f..%.1f (spec 10.1)" % (key.upper(), val, lo, hi))
                refused = True
            else:
                out[IDS[(key, i)]] = val
    return refused


def eval_b(log, out):
    for i, w in enumerate("LR"):
        got = {}
        for ww, d, pct in log.b:
            if ww == w:
                got.setdefault(d, []).append(pct)
        vals = []
        for d in ("fwd", "rev"):
            last = got.get(d, [None])[-1]
            print("B %s wheel %s: %s" % (WHEEL[i], d, "no result" if d not in got else ("NONE (no movement up to the limit)" if last is None else "%d %%" % last)))
            if last is not None:
                vals.append(last / 100.0)
        if len(vals) == 2 and abs(vals[0] - vals[1]) > 0.02:
            print("  note: fwd and rev differ by more than 2 %; the mean is used")
        if vals:
            v = mean(vals)
            if RANGES["ksb"][0] <= v <= RANGES["ksb"][1]:
                out[IDS[("ksb", i)]] = v
            else:
                print("  REFUSED: %.3f outside the parameter range" % v)


def fit_tau(rows, wheel):
    """rows: one K run. Returns (tau_s, vss_cnt_per_s) or None."""
    s = next((n for n, r in enumerate(rows) if r[1] > 0), None)
    if s is None:
        return None
    seg = []
    for r in rows[s:]:
        if r[1] <= 0:
            break
        seg.append(r)
    if len(seg) < 10:
        return None
    t0 = seg[0][0] - DT_MS
    t, x, acc = [], [], 0
    for r in seg:
        acc += r[2 + wheel]
        t.append((r[0] - t0) / 1000.0)
        x.append(float(acc))
    if x[-1] <= 0:
        return None
    best = None
    for k in range(2, 401):
        tau = k / 1000.0
        f = [ti - tau * (1.0 - math.exp(-ti / tau)) for ti in t]
        vss = sum(a * b for a, b in zip(f, x)) / sum(a * a for a in f)
        rss = sum((xi - vss * fi) ** 2 for xi, fi in zip(x, f))
        if best is None or rss < best[0]:
            best = (rss, tau, vss)
    return best[1], best[2]


def eval_k(log, out):
    taus = []
    for n, run in enumerate(log.k, 1):
        for i in (0, 1):
            r = fit_tau(run, i)
            if r is None:
                print("K run %d %s wheel: no usable step" % (n, WHEEL[i]))
                continue
            print("K run %d %s wheel: tau = %.0f ms, steady %.1f counts per 20 ms" % (n, WHEEL[i], r[0] * 1000.0, r[1] * DT_MS / 1000.0))
            taus.append(r[0])
    if taus:
        tau = mean(taus)
        print("K tau mean %.1f ms over %d fits (spread %.1f ms; resolution 20 ms)" % (tau * 1000.0, len(taus), (max(taus) - min(taus)) * 1000.0))
        if TAU_RANGE[0] <= tau <= TAU_RANGE[1]:
            out[TAU_ID] = tau
        else:
            print("  REFUSED: outside the parameter range")


def cmd_eval(args):
    log = read_logs(args.log)
    out, refused = {}, False
    if log.fault or log.bad:
        print("warning: %d FAULT, %d malformed lines in the log (run `check`)" % (len(log.fault), len(log.bad)))
    if log.g:
        refused = eval_g(log, args.vmin, out)
    if log.b:
        eval_b(log, out)
    if log.k:
        eval_k(log, out)
    for r in log.l:
        print("L %s wheel: fwd %d cnt, rev %d cnt, latency fwd %d ms / rev %d ms (20 ms steps)" % ((WHEEL[0 if r[0] == "L" else 1],) + r[1:]))
    if log.l:
        print("L: report only; motor-side encoders do not see gear backlash, odo_backlash_l/r is not derived")
    for r in log.odo:
        print("odo: L=%d R=%d counts (odo_scale_*, odo_wheelbase: use straight_scale.py / umbmark.py)" % r)
    if not out:
        print("nothing computable (need G, B or K)")
        return 2
    print("")
    print("Type into the console (live parameters) or paste into the defaults:")
    for pid in sorted(out):
        print("P%d=%.4g" % (pid, out[pid]) if pid == TAU_ID else "P%d=%.4f" % (pid, out[pid]))
    if args.header:
        with io.open(args.header, "w", encoding="utf-8") as f:
            f.write("/* calib_result.h: generated by tools/calib_host.py eval, SI units, review before use */\n")
            for pid in sorted(out):
                f.write("#define PAR_%s %.5ef\n" % (NAMES[pid], out[pid]))
        print("wrote %s" % args.header)
    return 2 if refused else 0


# ----------------------------------------------------------------------------- selftest
def synth_log(ks=(0.098, 0.041), kv=(1.275, 1.361), tau=0.060, vk=0.25):
    """Log in the exact line formats of calib.c, from a known plant."""
    m = ["# MR2006B calibration firmware, ? for help",
         "C,L,fwd,30,412,0,PASS", "C,L,rev,30,-398,2,PASS", "C,R,fwd,30,3,405,PASS", "C,R,rev,30,-1,-410,PASS",
         "DONE C PASS", "> 30", "L=10 (0.6 mm, 3 mm/s)  R=11 (0.7 mm, 3 mm/s)  CALIB OPEN"]
    for pct in (15, 20, 30, 40, 50, 60):
        for sg in (1, -1):
            v = [sg * (pct / 100.0 - ks[i]) / kv[i] * 1000.0 for i in (0, 1)]
            m.append("G,%d,%d,%d" % (sg * pct, round(v[0]), round(v[1])))
    m += ["DONE G", "B,L,fwd,14", "B,L,rev,15", "B,R,fwd,17", "B,R,rev,NONE", "DONE B",
          "L,L,150,-148,40,60", "L,R,151,-150,40,40", "odo,31416,31402", "# WARN tick overrun",
          "DONE L", "FAULT STALL_OR_SIGN wheel R", "> s", "STOP"]
    for wheel_scale in (1.0, 0.9):                  # two K runs, second wheel pair slower
        x_prev = [0, 0]
        for k in range(100):
            t = (k + 1) * DT_MS
            on = 10 <= k < 70
            te = max(0.0, t / 1000.0 - 0.2)
            d = []
            for i in (0, 1):
                v = vk * wheel_scale / 63.638e-6 * 0.001                     # counts per ms
                x = round(v * 1000.0 * (te - tau * (1.0 - math.exp(-te / tau)))) if te > 0 and k < 70 else x_prev[i]
                d.append(x - x_prev[i] if k < 70 else 0)
                x_prev[i] = x
            m.append("K,%d,%d,%d,%d" % (t, 400 if on else 0, d[0], d[1]))
    m.append("DONE K")
    m.append("G,notanumber")                          # malformed on purpose
    return "\n".join(m) + "\n"


def selftest():
    fails = []

    def check(cond, msg):
        print(("  ok    " if cond else "  FAIL  ") + msg)
        if not cond:
            fails.append(msg)

    text = synth_log()
    log = parse(text)
    check(len(log.c) == 4 and len(log.g) == 12 and len(log.b) == 4 and len(log.l) == 2 and log.odo == [(31416, 31402)],
          "parser: C, G, B, L, odo rows")
    check(len(log.k) == 2 and len(log.k[0]) == 100, "parser: two K runs of 100 rows")
    check(len(log.fault) == 1 and log.stop == 1 and len(log.warn) == 1 and len(log.done) == 5, "parser: FAULT, STOP, WARN, DONE")
    check(len(log.bad) == 1 and log.bad[0][1] == "G,notanumber", "parser: one malformed line found")

    out = {}
    with contextlib.redirect_stdout(io.StringIO()):
        eval_g(log, 0.02, out)
        eval_b(log, out)
        eval_k(log, out)
    for pid, true in ((116, 0.098), (118, 0.041)):
        check(abs(out.get(pid, 9) - true) < 0.005, "Ks P%d = %.4f (true %.3f)" % (pid, out.get(pid, float("nan")), true))
    for pid, true in ((117, 1.275), (119, 1.361)):
        check(abs(out.get(pid, 9) / true - 1.0) < 0.01, "Kv P%d = %.4f (true %.3f)" % (pid, out.get(pid, float("nan")), true))
    check(abs(out.get(120, 9) - 0.145) < 1e-9, "Ksb left = mean(14, 15) %")
    check(abs(out.get(121, 9) - 0.17) < 1e-9, "Ksb right = 17 % (rev NONE not used)")
    r = fit_tau(log.k[0], 0)
    check(r is not None and abs(r[0] - 0.060) < 0.010, "tau %.1f ms (true 60, tol 10 = half a sample)" % (r[0] * 1000 if r else -1))
    check(r is not None and abs(r[1] * DT_MS / 1000.0 / (0.25 / 63.638e-6 * 0.02) - 1.0) < 0.02, "steady count rate within 2 %")
    check(TAU_ID in out and 0.05 < out[TAU_ID] < 0.07, "whl_tau_m mean of both runs (the slower plant has the same tau)")

    with tempfile.TemporaryDirectory() as d:
        p, h = os.path.join(d, "x.log"), os.path.join(d, "calib_result.h")
        with open(p, "w") as f:
            f.write(text)
        a = argparse.Namespace(log=[p], vmin=0.02, header=h)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = cmd_eval(a)
            rc_check = cmd_check(argparse.Namespace(log=p))
        check(rc == 0 and "P116=" in buf.getvalue() and "P115=" in buf.getvalue(), "eval prints P lines")
        hdr = open(h).read()
        check("#define PAR_whl_ks_l " in hdr and "#define PAR_whl_tau_m " in hdr and hdr.count("#define") == 7, "header has 7 defines")
        check(rc_check == 2 and "malformed line" in buf.getvalue(), "check flags FAULT and malformed line")
        clean = "\n".join(l for l in text.splitlines() if not l.startswith(("FAULT", "G,notanumber"))) + "\n"
        with open(p, "w") as f:
            f.write(clean)
        with contextlib.redirect_stdout(io.StringIO()):
            rc = cmd_check(argparse.Namespace(log=p))
        check(rc == 0, "check passes a clean log")
        with open(p, "w") as f:
            f.write("G,20,50,50\nG,30,150,150\n")
        with contextlib.redirect_stdout(io.StringIO()):
            rc = cmd_eval(argparse.Namespace(log=[p], vmin=0.02, header=None))
        check(rc == 2, "eval refuses 2 G points")
    print("")
    print("SELFTEST FAILED (%d)" % len(fails) if fails else "SELFTEST PASSED")
    return 1 if fails else 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd")
    s = sub.add_parser("check")
    s.add_argument("log")
    s = sub.add_parser("eval")
    s.add_argument("log", nargs="+")
    s.add_argument("--vmin", type=float, default=0.02, help="drop G points slower than this, m/s (default 0.02)")
    s.add_argument("--header", help="write calib_result.h to this path")
    sub.add_parser("selftest")
    args = p.parse_args(argv)
    if not args.cmd:
        p.print_usage(sys.stderr)
        return 1
    try:
        return {"check": cmd_check, "eval": cmd_eval, "selftest": lambda a: selftest()}[args.cmd](args)
    except OSError as e:
        print("error: %s" % e, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
