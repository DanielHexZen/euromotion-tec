#!/usr/bin/env python3
"""Straight-line scale for MR2006B: odo_m_per_cnt (ID 200) from taped runs (PHASE1_CALIBRATION.md section 4).

Each run is a straight `D<mm>` drive on the floor. For every run you note the distance the vehicle
really rolled (tape measure, one value per wheel contact patch) and the final encoder counts of the
console (`p` after DONE, with the counters zeroed by `r` before the run, so L= and R= are the counts
of this run).

  ground per count, wheel w   k_w = tape_w / counts_w
  new odo_m_per_cnt           = mean over runs and wheels of k, divided by the mean of the scales
                                that were active when the counts were taken (default 1.0, 1.0)

The value is the mean ground distance per count of the two wheels, i.e. the mean scale. The ratio
k_r / k_l is printed as a cross-check of the UMBmark wheel-diameter ratio E_d (do not enter it; UMBmark
is the more precise measurement of it).

Input, either --csv or one --run per run:
  CSV columns (header optional, '#' comments):  tape_l_mm,tape_r_mm,cnt_l,cnt_r
  or, when you only wrote down the DONE line of the console:
        tape_l_mm,tape_r_mm,done_l_mm,done_r_mm   with --done  (needs --m-per-cnt, --scale-l, --scale-r:
        the values that were active during the runs; DONE is printed with 0.1 mm resolution)
  --run "tape_l,tape_r,cnt_l,cnt_r" may be repeated (write --run=... if the first number is negative;
  a backward run has all four numbers negative and is accepted).

Example (numbers are EXAMPLES, not measurements):
  straight_scale.py --run "2003,2004,31498,31501" --run "2002,2004,31496,31500" --run "2004,2005,31501,31503"
  straight_scale.py --csv tools/data/2026-11-12/straight.csv
  straight_scale.py --selftest

Refuses to print a P line when the result is outside 5e-5..8e-5 m (spec 10.2), when there are fewer
than 3 runs (--min-runs), or when the run-to-run spread is above 0.3 % (--max-spread-pct): a spread
that large means slip, a wrong tape reading or a dragged cable, not a scale.
Exit status: 0 ok, 1 usage/data error, 2 refused.
"""
import argparse
import csv
import io
import math
import sys

M_PER_CNT_RANGE = (5.0e-5, 8.0e-5)      # par_table.h ID 200
NOMINAL = 6.3638e-5                      # spec 7.6.2, for the plausibility remark only


class DataError(Exception):
    pass


def mean(v):
    return sum(v) / len(v)


def sd(v):
    if len(v) < 2:
        return float("nan")
    m = mean(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / (len(v) - 1))


def parse_row(vals, done, m_per_cnt, sl, sr):
    if len(vals) != 4:
        raise DataError("a run needs 4 numbers: tape_l,tape_r,%s" % ("done_l,done_r" if done else "cnt_l,cnt_r"))
    tl, tr, a, b = vals
    if done:
        cl = a / 1000.0 / (m_per_cnt * sl)
        cr = b / 1000.0 / (m_per_cnt * sr)
    else:
        cl, cr = a, b
    if cl == 0 or cr == 0:
        raise DataError("zero counts in a run")
    if tl * cl <= 0 or tr * cr <= 0:
        raise DataError("tape distance and counts have opposite signs: %s" % vals)
    return (tl, tr, cl, cr)


def parse_csv(text, done, m_per_cnt, sl, sr):
    runs = []
    lines = [ln for ln in text.splitlines() if ln.strip() and not ln.lstrip().startswith("#")]
    for n, row in enumerate(csv.reader(lines), 1):
        try:
            vals = [float(c) for c in row]
        except ValueError:
            if n == 1:
                continue                        # header
            raise DataError("row %d: non-numeric value" % n)
        runs.append(parse_row(vals, done, m_per_cnt, sl, sr))
    return runs


def evaluate(runs, scale_l=1.0, scale_r=1.0):
    """runs: list of (tape_l_mm, tape_r_mm, cnt_l, cnt_r). Returns a dict."""
    kl = [t / 1000.0 / c for t, _, c, _ in runs]
    kr = [t / 1000.0 / c for _, t, _, c in runs]
    k_run = [0.5 * (a + b) for a, b in zip(kl, kr)]
    scale_mean = 0.5 * (scale_l + scale_r)
    m_run = [k / scale_mean for k in k_run]
    m = mean(m_run)
    spread = sd(m_run) / m * 100.0 if len(m_run) > 1 else float("nan")
    return {"n": len(runs), "k_l": mean(kl), "k_r": mean(kr), "m_per_cnt": m, "m_run": m_run,
            "spread_pct": spread, "sem_pct": spread / math.sqrt(len(runs)) if len(runs) > 1 else float("nan"),
            "ratio_r_over_l": mean(kr) / mean(kl)}


def selftest():
    fails = []

    def check(cond, msg):
        print(("  ok    " if cond else "  FAIL  ") + msg)
        if not cond:
            fails.append(msg)

    import random
    rng = random.Random(7)
    true_m, ratio = 6.4012e-5, 1.0040            # right wheel rolls 0.4 % farther per count than the left
    kl_true = true_m * 2.0 / (1.0 + ratio)
    kr_true = true_m * 2.0 * ratio / (1.0 + ratio)
    runs = []
    for d in (2000.0, 2000.0, 2500.0, 3000.0, 2000.0):
        cl, cr = d / 1000.0 / kl_true, d / 1000.0 / kr_true
        runs.append((d + rng.gauss(0, 0.8), d + rng.gauss(0, 0.8), round(cl), round(cr)))
    r = evaluate(runs)
    check(abs(r["m_per_cnt"] / true_m - 1.0) < 5e-4, "recovers odo_m_per_cnt: %.5e (true %.5e)" % (r["m_per_cnt"], true_m))
    check(abs(r["ratio_r_over_l"] - ratio) < 1e-3, "per-wheel ratio k_r/k_l %.4f (true %.4f)" % (r["ratio_r_over_l"], ratio))
    check(r["spread_pct"] < 0.1, "spread %.3f %% small for 0.8 mm tape noise" % r["spread_pct"])
    r2 = evaluate(runs, 1.01, 0.99)
    check(abs(r2["m_per_cnt"] - r["m_per_cnt"]) < 1e-12, "mean scale 1.0 does not change the result")
    r3 = evaluate(runs, 1.02, 1.02)
    check(abs(r3["m_per_cnt"] * 1.02 / r["m_per_cnt"] - 1.0) < 1e-9, "a common scale of 1.02 is divided out")
    back = [(-t1, -t2, -c1, -c2) for t1, t2, c1, c2 in runs]
    check(abs(evaluate(back)["m_per_cnt"] - r["m_per_cnt"]) < 1e-15, "backward runs give the same value")
    row = parse_row([2000.0, 2000.0, 2000.0 * 0.99, 2000.0 * 0.99], True, 6.3638e-5, 1.0, 1.0)
    check(abs(row[2] - 0.99 * 2.0 / 6.3638e-5) < 1e-6, "--done converts mm back to counts")
    try:
        parse_row([100.0, 100.0, -5.0, 5.0], False, 0, 1, 1)
        check(False, "sign mismatch rejected")
    except DataError:
        check(True, "sign mismatch rejected")
    print("")
    if fails:
        print("SELFTEST FAILED (%d)" % len(fails))
        return 1
    print("SELFTEST PASSED")
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--selftest", action="store_true")
    p.add_argument("--csv", help="CSV file")
    p.add_argument("--run", action="append", default=[], help='"tape_l,tape_r,cnt_l,cnt_r" in mm and counts; repeatable')
    p.add_argument("--done", action="store_true", help="the last two numbers are the DONE L=, R= values in mm")
    p.add_argument("--m-per-cnt", type=float, help="odo_m_per_cnt active during the runs (with --done)")
    p.add_argument("--scale-l", type=float, default=1.0, help="odo_scale_l active during the runs (default 1.0)")
    p.add_argument("--scale-r", type=float, default=1.0, help="odo_scale_r active during the runs (default 1.0)")
    p.add_argument("--min-runs", type=int, default=3)
    p.add_argument("--max-spread-pct", type=float, default=0.3, help="refuse above this run-to-run sd in %% (default 0.3)")
    args = p.parse_args(argv)
    if args.selftest:
        return selftest()
    try:
        if args.done and args.m_per_cnt is None:
            raise DataError("--done needs --m-per-cnt (and --scale-l/--scale-r if they were not 1.0)")
        runs = []
        if args.csv:
            with io.open(args.csv, encoding="utf-8") as f:
                runs += parse_csv(f.read(), args.done, args.m_per_cnt, args.scale_l, args.scale_r)
        for s in args.run:
            try:
                vals = [float(x) for x in s.split(",")]
            except ValueError:
                raise DataError("cannot read --run '%s'" % s)
            runs.append(parse_row(vals, args.done, args.m_per_cnt, args.scale_l, args.scale_r))
        if not runs:
            raise DataError("no runs: give --csv or --run")
    except (DataError, OSError) as e:
        print("error: %s" % e, file=sys.stderr)
        return 1

    r = evaluate(runs, args.scale_l, args.scale_r)
    print("%d runs" % r["n"])
    print("run   m_per_cnt (mean of both wheels)   deviation from mean")
    for i, v in enumerate(r["m_run"], 1):
        print("%3d   %.5e                    %+.3f %%" % (i, v, (v / r["m_per_cnt"] - 1.0) * 100.0))
    print("mean odo_m_per_cnt = %.5e m   run-to-run sd %.3f %%   standard error %.3f %%"
          % (r["m_per_cnt"], r["spread_pct"], r["sem_pct"]))
    print("vs the nominal %.4e m (spec 7.6.2): %+.2f %%" % (NOMINAL, (r["m_per_cnt"] / NOMINAL - 1.0) * 100.0))
    print("per wheel: left %.5e m/count, right %.5e m/count, k_r/k_l = %.4f (cross-check of UMBmark E_d only)"
          % (r["k_l"], r["k_r"], r["ratio_r_over_l"]))

    if r["n"] < args.min_runs:
        print("REFUSED: %d runs, need at least %d (--min-runs)" % (r["n"], args.min_runs))
        return 2
    if not (r["spread_pct"] <= args.max_spread_pct):
        print("REFUSED: run-to-run spread %.3f %% above %.2f %%. Look for the outlier in the table, "
              "redo that run; do not average it in." % (r["spread_pct"], args.max_spread_pct))
        return 2
    lo, hi = M_PER_CNT_RANGE
    if not (lo <= r["m_per_cnt"] <= hi):
        print("REFUSED: %.5e is outside the parameter range %.0e..%.0e of spec 10.2 (units? mm instead of "
              "counts? wrong wheel?)" % (r["m_per_cnt"], lo, hi))
        return 2
    print("")
    print("Type into the console with the vehicle stationary (then P200? to read back):")
    print("P200=%.5e" % r["m_per_cnt"])
    print("For the compiled default: odo_m_per_cnt %.5ef" % r["m_per_cnt"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
