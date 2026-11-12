#!/usr/bin/env python3
"""UMBmark evaluation for MR2006B (PHASE1_CALIBRATION.md section 5, spec 10.2 IDs 201-203).

Borenstein & Feng, "Measurement and correction of systematic odometry errors in mobile robots",
IEEE Trans. Robotics and Automation 12(6), 1996: the vehicle drives a square of side L by dead
reckoning, 4 x (straight L, turn 90 deg), clockwise (CW) and counter-clockwise (CCW). The position
error at the end (final minus start) is measured. The centres of gravity of the CW and CCW errors
separate the two systematic errors that matter:

  type A  every turn is too large or too small  -> wheelbase error E_b
  type B  the straight legs curve               -> wheel diameter (ground per count) ratio E_d

Axes are the firmware's: x forward along the initial heading, y to the left, CCW positive, start
pose = origin, heading 0. Errors are entered in mm as (x, y) of the axle midpoint after the run.

Derivation. With e = error vector at the end of a run, sigma = +1 CCW / -1 CW, per-turn heading
error sigma*alpha (alpha > 0: every turn too large) and heading change gamma per leg (gamma > 0:
legs curve to the left, i.e. the right wheel travels farther than the left), to first order:
  e_x = 2 L (alpha + sigma gamma)         e_y = -sigma e_x      (the error lies on a diagonal)
  =>  alpha = (x_ccw + x_cw) / 4L = (y_cw - y_ccw) / 4L          type A
      gamma = (x_ccw - x_cw) / 4L = -(y_ccw + y_cw) / 4L         type B
  E_b = 1 / (1 + 2 alpha / pi)   (actual turn = commanded / E_b, mean scale error 0)
  E_d = (2L + gamma b_a) / (2L - gamma b_a),  b_a = b E_b        (Borenstein: R = L/gamma)
These closed forms are only good to about 10 % once the return error reaches several hundred mm
(gamma of 0.1 rad is a 1.2 % diameter mismatch on a 2 m leg), so the tool uses them as the starting
point and then solves for (E_b, E_d) with the exact kinematics of the square (Gauss-Newton on the
four centre-of-gravity coordinates). The residual of that fit is printed: if it is large, the data do
not fit the two-parameter model (wrong sign, wrong start pose, slipping wheel).
Turn shortfall: the firmware's SETTLE stops a wheel up to 2 mm short of the arc and never reverses
(section 5.1 of the procedure). --turn-done-mm puts the real arc into the model so that E_b is not biased.
The mean of the two wheel scales is assumed correct (straight-line test, section 4); the new scales keep
the mean of odo_scale_l and odo_scale_r:  rho = (s_r / s_l) E_d,  s_l' = (s_l + s_r)/(1 + rho),  s_r' = rho s_l'.
New odo_wheelbase = b E_b.

Examples (all numbers below are EXAMPLES, not measurements):
  umbmark.py --wheelbase 0.22325 --csv data/umb.csv
  umbmark.py --wheelbase 0.22325 --cw="-3.1,4.0;-2.2,3.1" --ccw="5.0,-6.1;4.4,-5.2"
  umbmark.py --wheelbase 0.22325 --csv data/verify.csv --verify
  umbmark.py --selftest

CSV: one run per line, '#' starts a comment, optional header. Either
  dir,x_mm,y_mm                     (axle midpoint relative to the start pose)
  dir,xl_mm,yl_mm,xr_mm,yr_mm       (contact points of the left and right wheel; midpoint is used
                                     and the heading error is reported)
with dir = cw or ccw.
Exit status: 0 ok, 1 usage/data error, 2 result refused (out of parameter range or data not credible),
3 --verify failed.
"""
import argparse
import csv
import io
import math
import random
import sys

PAR_RANGES = {  # spec 10.2 / par_table.h
    "odo_scale_l": (0.95, 1.05),
    "odo_scale_r": (0.95, 1.05),
    "odo_wheelbase": (0.15, 0.35),
}


class DataError(Exception):
    pass


# ----------------------------------------------------------------------------- statistics
def mean(v):
    return sum(v) / len(v)


def sd(v):
    if len(v) < 2:
        return float("nan")
    m = mean(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / (len(v) - 1))


# ----------------------------------------------------------------------------- input
def parse_pairs(text):
    out = []
    for item in text.split(";"):
        item = item.strip()
        if not item:
            continue
        try:
            x, y = item.split(",")
            out.append((float(x), float(y)))
        except ValueError:
            raise DataError("cannot read '%s', expected x,y in mm" % item)
    return out


def parse_csv(text):
    """Returns (cw, ccw, heading_errors) where cw/ccw are lists of (x_mm, y_mm)."""
    cw, ccw, hdg = [], [], {"cw": [], "ccw": []}
    lines = [ln for ln in text.splitlines() if ln.strip() and not ln.lstrip().startswith("#")]
    for n, row in enumerate(csv.reader(lines), 1):
        row = [c.strip() for c in row]
        if not row or not row[0]:
            continue
        d = row[0].lower()
        if d not in ("cw", "ccw"):
            if n == 1 and d in ("dir", "direction"):
                continue
            raise DataError("row %d: first column must be cw or ccw, got '%s'" % (n, row[0]))
        try:
            vals = [float(c) for c in row[1:]]
        except ValueError:
            raise DataError("row %d: non-numeric value" % n)
        if len(vals) == 2:
            x, y = vals
        elif len(vals) == 4:
            xl, yl, xr, yr = vals
            x, y = 0.5 * (xl + xr), 0.5 * (yl + yr)
            # axle line right minus left starts as (0, -b); rotated by theta it is b*(sin t, -cos t)
            hdg[d].append(math.degrees(math.atan2(xr - xl, -(yr - yl))))
        else:
            raise DataError("row %d: need dir,x,y or dir,xl,yl,xr,yr" % n)
        (cw if d == "cw" else ccw).append((x, y))
    return cw, ccw, hdg


# ----------------------------------------------------------------------------- core
def cog(runs):
    xs = [r[0] for r in runs]
    ys = [r[1] for r in runs]
    return {"n": len(runs), "x": mean(xs), "y": mean(ys), "sdx": sd(xs), "sdy": sd(ys),
            "r": math.hypot(mean(xs), mean(ys))}


def forward(e_d, e_b, side_m, b, sl, sr, short_m):
    """Exact return errors (cw_x, cw_y, ccw_x, ccw_y) in mm for hardware with ground-per-count ratio
    E_d (right/left, mean error 0) and wheelbase b*E_b, driven by firmware parameters (sl, sr, b)."""
    m0 = 1.0e-4
    hw = (m0 * sl * 2.0 / (1.0 + e_d), m0 * sr * 2.0 * e_d / (1.0 + e_d), b * e_b)
    model = (m0, sl, sr, b)
    cwx, cwy, _ = simulate_square(side_m, -1, hw, model, short_m)
    ccx, ccy, _ = simulate_square(side_m, +1, hw, model, short_m)
    return [cwx, cwy, ccx, ccy]


def solve_exact(target, side_m, b, sl, sr, short_m, e_d0, e_b0):
    """Gauss-Newton with a numerical Jacobian, Levenberg damping. Returns (E_d, E_b, rms_mm)."""
    p = [e_d0, e_b0]

    def res(q):
        f = forward(q[0], q[1], side_m, b, sl, sr, short_m)
        return [t - v for t, v in zip(target, f)]

    r = res(p)
    cost = sum(v * v for v in r)
    lam = 1.0e-6
    for _ in range(60):
        h = 1.0e-6
        cols = []
        for i in range(2):
            q = list(p)
            q[i] += h
            ri = res(q)
            cols.append([(a - c) / h for a, c in zip(ri, r)])   # d(residual)/dp, residual = target - model
        # minimise |r + J dp|^2 : (J^T J + lam I) dp = -J^T r
        jtj = [[sum(cols[i][k] * cols[j][k] for k in range(4)) for j in range(2)] for i in range(2)]
        jtr = [sum(cols[i][k] * r[k] for k in range(4)) for i in range(2)]
        improved = False
        for _try in range(12):
            a11, a12, a22 = jtj[0][0] + lam * (1 + jtj[0][0]), jtj[0][1], jtj[1][1] + lam * (1 + jtj[1][1])
            det = a11 * a22 - a12 * a12
            if det == 0.0:
                lam *= 10.0
                continue
            dp = [(-jtr[0] * a22 + jtr[1] * a12) / det, (-jtr[1] * a11 + jtr[0] * a12) / det]
            q = [p[0] + dp[0], p[1] + dp[1]]
            if not (0.5 < q[0] < 2.0 and 0.5 < q[1] < 2.0):
                lam *= 10.0
                continue
            rq = res(q)
            cq = sum(v * v for v in rq)
            if cq <= cost:
                p, r, cost = q, rq, cq
                lam = max(lam / 10.0, 1.0e-12)
                improved = True
                break
            lam *= 10.0
        if not improved or max(abs(dp[0]), abs(dp[1])) < 1.0e-12:
            break
    return p[0], p[1], math.sqrt(cost / 4.0)


def analyse(side_mm, b, sl, sr, cw, ccw, turn_done_mm=None):
    """UMBmark evaluation. b in m, side in mm, errors in mm. Returns a dict."""
    L = float(side_mm)
    c, a = cog(cw), cog(ccw)
    alpha_x = (a["x"] + c["x"]) / (4.0 * L)
    alpha_y = (c["y"] - a["y"]) / (4.0 * L)
    gamma_x = (a["x"] - c["x"]) / (4.0 * L)
    gamma_y = -(a["y"] + c["y"]) / (4.0 * L)
    alpha = 0.5 * (alpha_x + alpha_y)
    gamma = 0.5 * (gamma_x + gamma_y)

    short_m = 0.0
    short = 0.0
    if turn_done_mm is not None:
        arc = 0.5 * b * (math.pi / 2.0) * 1000.0
        short_m = (arc - turn_done_mm) / 1000.0
        short = (arc - turn_done_mm) / arc
    # first-order starting point
    e_b1 = (1.0 - short) / (1.0 + 2.0 * alpha / math.pi)
    den = 2.0 * (L / 1000.0) - gamma * b * e_b1
    if den <= 0.0:
        raise DataError("gamma too large for the side length: check the data and signs")
    e_d1 = (2.0 * (L / 1000.0) + gamma * b * e_b1) / den
    e_d, e_b, rms = solve_exact([c["x"], c["y"], a["x"], a["y"]], L / 1000.0, b, sl, sr, short_m, e_d1, e_b1)
    rho = (sr / sl) * e_d
    sl2 = (sl + sr) / (1.0 + rho)
    sr2 = rho * sl2
    return {"cw": c, "ccw": a, "alpha_x": alpha_x, "alpha_y": alpha_y, "gamma_x": gamma_x,
            "gamma_y": gamma_y, "alpha": alpha, "gamma": gamma, "turn_short": short,
            "E_b1": e_b1, "E_d1": e_d1, "fit_rms_mm": rms,
            "E_b": e_b, "E_d": e_d, "odo_wheelbase": b * e_b, "odo_scale_l": sl2, "odo_scale_r": sr2,
            "E_max_syst": max(c["r"], a["r"])}


def significance(res):
    """True when at least one centre-of-gravity coordinate is beyond 2 standard errors of zero,
    False when none is, None when a scatter estimate is impossible (one run in a direction)."""
    for k in ("cw", "ccw"):
        if res[k]["n"] < 2:
            return None
    for k in ("cw", "ccw"):
        g = res[k]
        for ax, s in (("x", "sdx"), ("y", "sdy")):
            if abs(g[ax]) >= 2.0 * g[s] / math.sqrt(g["n"]):
                return True
    return False


# ----------------------------------------------------------------------------- simulation
def _move(pose, s_l, s_r, b_true):
    x, y, th = pose
    dth = (s_r - s_l) / b_true
    ds = 0.5 * (s_l + s_r)
    if abs(dth) < 1e-12:
        return x + ds * math.cos(th), y + ds * math.sin(th), th
    r = ds / dth
    return (x + r * (math.sin(th + dth) - math.sin(th)), y - r * (math.cos(th + dth) - math.cos(th)),
            th + dth)


def simulate_square(side_m, sigma, hw, model, turn_short_m=0.0):
    """Dead-reckoning square on the true hardware.

    hw    = (ground m per count left, right, true wheelbase m)           the hardware
    model = (m_per_cnt, scale_l, scale_r, wheelbase m)                    the firmware parameters
    The firmware commands counts so that its own (believed) wheel distances are met exactly; the
    robot moves by counts times the true ground distance per count. Returns (x_mm, y_mm, theta_deg).
    """
    ml, mr, bt = hw
    m0, sl, sr, bm = model
    pose = (0.0, 0.0, 0.0)
    for _ in range(4):
        pose = _move(pose, ml * side_m / (m0 * sl), mr * side_m / (m0 * sr), bt)
        arc = 0.5 * bm * (math.pi / 2.0) - turn_short_m
        pose = _move(pose, -sigma * ml * arc / (m0 * sl), sigma * mr * arc / (m0 * sr), bt)
    return pose[0] * 1000.0, pose[1] * 1000.0, math.degrees(pose[2])


def make_hw(model, eps_l, eps_r, eb_true):
    m0, sl, sr, bm = model
    return (m0 * sl * (1.0 + eps_l), m0 * sr * (1.0 + eps_r), bm * eb_true)


def run_set(side_m, hw, model, n, noise_mm, rng, turn_short_m=0.0):
    out = {}
    for name, sig in (("cw", -1), ("ccw", 1)):
        runs = []
        for _ in range(n):
            x, y, _t = simulate_square(side_m, sig, hw, model, turn_short_m)
            runs.append((x + rng.gauss(0, noise_mm), y + rng.gauss(0, noise_mm)))
        out[name] = runs
    return out


def selftest():
    fails = []

    def check(cond, msg):
        print(("  ok    " if cond else "  FAIL  ") + msg)
        if not cond:
            fails.append(msg)

    rng = random.Random(20261110)
    side = 2.0
    nominal = (6.3638e-5, 1.0, 1.0, 0.22325)

    print("signs of the simulator (physical meaning of the documented conventions, small errors)")
    # type B only: right wheel travels farther (eps_r > eps_l, mean 0) -> legs curve to the left
    hw = make_hw(nominal, -0.0005, 0.0005, 1.0)
    cw_ = simulate_square(side, -1, hw, nominal)
    ccw_ = simulate_square(side, +1, hw, nominal)
    check(ccw_[0] > 0 > cw_[0], "type B, right wheel longer: x_ccw > 0 > x_cw (%.1f, %.1f mm)" % (ccw_[0], cw_[0]))
    check(abs(ccw_[1] + ccw_[0]) < 0.05 * abs(ccw_[0]) and abs(cw_[1] - cw_[0]) < 0.05 * abs(cw_[0]),
          "error on the diagonal to first order: y = -x (CCW), y = +x (CW)")
    # type A only: wheelbase smaller than assumed -> every turn too large (alpha > 0)
    hw = make_hw(nominal, 0.0, 0.0, 0.995)
    a_cw = simulate_square(side, -1, hw, nominal)
    a_ccw = simulate_square(side, +1, hw, nominal)
    check(a_cw[0] > 0 and a_ccw[0] > 0 and abs(a_cw[0] - a_ccw[0]) < 0.05 * a_cw[0],
          "type A, wheelbase too small (turns too large): x > 0 in both directions (%.1f, %.1f mm)" % (a_cw[0], a_ccw[0]))
    check(a_ccw[1] < 0 < a_cw[1], "type A: y_ccw < 0 < y_cw")
    # first-order closed forms against the simulator for small errors
    r = analyse(2000.0, 0.22325, 1.0, 1.0, [tuple(a_cw[:2])], [tuple(a_ccw[:2])])
    check(abs(r["alpha"] - (math.pi / 2) * (1 / 0.995 - 1)) < 0.03 * abs(r["alpha"]),
          "first-order alpha %.5f vs exact per-turn error %.5f rad" % (r["alpha"], (math.pi / 2) * (1 / 0.995 - 1)))

    print("recovery of injected errors, exact kinematics, noiseless: E_b and E_d within 1e-4,")
    print("residual return error after applying the corrections < 1.5 mm on a 2 m square")
    cases = [  # eps_l, eps_r, E_b true, current s_l, s_r, wheelbase
        (-0.006, 0.006, 1.020, 1.0, 1.0, 0.22325),
        (0.010, -0.010, 0.985, 1.0, 1.0, 0.22325),
        (-0.003, 0.003, 1.008, 1.0123, 0.9877, 0.22325),
        (0.012, -0.012, 1.015, 0.99, 1.01, 0.2300),
        (-0.0015, 0.0015, 0.997, 1.0, 1.0, 0.22325),
        (0.0, 0.0, 1.0, 1.0, 1.0, 0.22325),
    ]
    for el, er, eb, sl, sr, b in cases:
        model = (6.3638e-5, sl, sr, b)
        hw = make_hw(model, el, er, eb)
        data = run_set(side, hw, model, 5, 0.0, rng)
        r = analyse(side * 1000.0, b, sl, sr, data["cw"], data["ccw"])
        ed_true = (1.0 + er) / (1.0 + el)
        tag = "eps=(%+.4f,%+.4f) E_b=%.3f" % (el, er, eb)
        check(abs(r["E_b"] - eb) < 1e-4, "%s: E_b %.5f (true %.5f; first order %.5f)" % (tag, r["E_b"], eb, r["E_b1"]))
        check(abs(r["E_d"] - ed_true) < 1e-4, "%s: E_d %.5f (true %.5f; first order %.5f)" % (tag, r["E_d"], ed_true, r["E_d1"]))
        check(abs(0.5 * (r["odo_scale_l"] + r["odo_scale_r"]) - 0.5 * (sl + sr)) < 1e-9,
              "%s: mean scale unchanged (%.6f)" % (tag, 0.5 * (r["odo_scale_l"] + r["odo_scale_r"])))
        model2 = (6.3638e-5, r["odo_scale_l"], r["odo_scale_r"], r["odo_wheelbase"])
        worst = 0.0
        for sig in (-1, 1):
            x, y, _ = simulate_square(side, sig, hw, model2)
            worst = max(worst, math.hypot(x, y))
        check(worst < 1.5, "%s: residual after correction %.3f mm" % (tag, worst))

    print("turn shortfall (SETTLE stops each wheel short of the arc), 1.5 mm of a 175 mm arc")
    b = 0.22325
    model = (6.3638e-5, 1.0, 1.0, b)
    hw = make_hw(model, -0.004, 0.004, 1.01)
    data = run_set(side, hw, model, 5, 0.0, rng, turn_short_m=1.5e-3)
    arc_mm = 0.5 * b * (math.pi / 2.0) * 1000.0
    r_plain = analyse(2000.0, b, 1.0, 1.0, data["cw"], data["ccw"])
    r_corr = analyse(2000.0, b, 1.0, 1.0, data["cw"], data["ccw"], turn_done_mm=arc_mm - 1.5)
    check(abs(r_corr["E_b"] - 1.01) < 1e-4, "with --turn-done-mm: E_b %.5f (true 1.01000)" % r_corr["E_b"])
    check(abs(r_plain["E_b"] - 1.01) > 3e-3,
          "without it the shortfall biases E_b: %.5f (documented caveat, about -0.9 %%)" % r_plain["E_b"])

    print("noise: 5 runs per direction, 2 mm scatter per axis, 30 random cases, mean scale error")
    print("up to 0.2 % (the method assumes 0; E_b is then compared with the value it can see)")
    worst_eb = worst_ed = worst_res = 0.0
    for _ in range(30):
        el = rng.uniform(-0.01, 0.01)
        er = -el + rng.uniform(-0.002, 0.002)
        eb = rng.uniform(0.98, 1.02)
        model = (6.3638e-5, 1.0, 1.0, 0.22325)
        hw = make_hw(model, el, er, eb)
        data = run_set(side, hw, model, 5, 2.0, rng)
        r = analyse(2000.0, 0.22325, 1.0, 1.0, data["cw"], data["ccw"])
        ed_true = (1.0 + er) / (1.0 + el)
        worst_ed = max(worst_ed, abs(r["E_d"] - ed_true))
        worst_eb = max(worst_eb, abs(r["E_b"] - eb / (1.0 + 0.5 * (el + er))))
        model2 = (6.3638e-5, r["odo_scale_l"], r["odo_scale_r"], r["odo_wheelbase"])
        for sig in (-1, 1):
            x, y, _ = simulate_square(side, sig, hw, model2)
            worst_res = max(worst_res, math.hypot(x, y))
    check(worst_eb < 1e-3, "worst E_b error %.5f < 0.001" % worst_eb)
    check(worst_ed < 1e-3, "worst E_d error %.5f < 0.001" % worst_ed)
    check(worst_res < 8.0, "worst residual return error %.2f mm < 8 mm (scatter 2 mm per axis, 0.2 %% mean error)" % worst_res)

    print("input parsing")
    cw, ccw, h = parse_csv("# c\ndir,x_mm,y_mm\ncw,-3.1,4.0\nCCW,5.0,-6.1\n")
    check(cw == [(-3.1, 4.0)] and ccw == [(5.0, -6.1)], "CSV with header, case-insensitive dir")
    cw, ccw, h = parse_csv("cw,10,100,10,-100\nccw,0,0,0,0\n")
    check(abs(cw[0][0] - 10.0) < 1e-9 and abs(cw[0][1]) < 1e-9, "wheel-point CSV gives the midpoint")
    check(abs(h["cw"][0]) < 1e-9, "wheel-point CSV: aligned axle line = 0 deg")
    cw, ccw, h = parse_csv("ccw,-10,100,10,-100\n")     # right wheel ahead of the left one: CCW rotation
    check(h["ccw"][0] > 0, "wheel-point CSV: right wheel ahead = CCW = positive heading (%.2f deg)" % h["ccw"][0])
    check(parse_pairs("-3.1,4.0; 2,1") == [(-3.1, 4.0), (2.0, 1.0)], "command-line pairs")
    try:
        parse_csv("up,1,2\n")
        check(False, "bad direction rejected")
    except DataError:
        check(True, "bad direction rejected")

    print("")
    if fails:
        print("SELFTEST FAILED (%d)" % len(fails))
        return 1
    print("SELFTEST PASSED")
    return 0


# ----------------------------------------------------------------------------- report
def fmt_par(name, pid, value):
    lo, hi = PAR_RANGES[name]
    return "P%d=%.5f" % (pid, value), lo <= value <= hi


def report(args, cw, ccw, hdg):
    if len(cw) < args.min_runs or len(ccw) < args.min_runs:
        raise DataError("need at least %d runs per direction (have CW %d, CCW %d); --min-runs lowers this"
                        % (args.min_runs, len(cw), len(ccw)))
    c, a = cog(cw), cog(ccw)
    print("UMBmark, side %.0f mm, runs: CW %d, CCW %d" % (args.side_mm, c["n"], a["n"]))
    print("axes: x forward, y left, CCW positive; errors are final minus start of the axle midpoint")
    print("")
    print("      x_cg mm   y_cg mm   |r_cg| mm   sd_x mm  sd_y mm")
    for name, g in (("CW ", c), ("CCW", a)):
        print("%s %9.2f %9.2f %10.2f %9.2f %8.2f" % (name, g["x"], g["y"], g["r"], g["sd" + "x"], g["sdy"]))
    print("E_max,syst = max(|r_cg,cw|, |r_cg,ccw|) = %.2f mm" % max(c["r"], a["r"]))
    for name, runs in (("CW", cw), ("CCW", ccw)):
        print("single-run |e| %-3s: %s mm" % (name, ", ".join("%.1f" % math.hypot(*r) for r in runs)))
    for name in ("cw", "ccw"):
        if hdg[name]:
            print("final heading error %-3s: mean %.2f deg (%d runs, from the wheel points)"
                  % (name.upper(), mean(hdg[name]), len(hdg[name])))

    if args.verify:
        worst = max(math.hypot(*r) for r in cw + ccw)
        print("")
        print("VERIFY against %.1f mm (spec 1.4: return error on a 2 m square, both directions):" % args.limit_mm)
        print("  every single run:   max |e| = %.2f mm  -> %s" % (worst, "PASS" if worst < args.limit_mm else "FAIL"))
        print("  centres of gravity: max |r_cg| = %.2f mm -> %s"
              % (max(c["r"], a["r"]), "PASS" if max(c["r"], a["r"]) < args.limit_mm else "FAIL"))
        return 0 if worst < args.limit_mm else 3

    res = analyse(args.side_mm, args.wheelbase, args.scale_l, args.scale_r, cw, ccw, args.turn_done_mm)
    print("")
    print("first order (starting values):")
    print("  type A  alpha = %+.5f rad (%+.3f deg per turn)   from x: %+.5f  from y: %+.5f"
          % (res["alpha"], math.degrees(res["alpha"]), res["alpha_x"], res["alpha_y"]))
    print("  type B  gamma = %+.5f rad (%+.3f deg per leg)    from x: %+.5f  from y: %+.5f"
          % (res["gamma"], math.degrees(res["gamma"]), res["gamma_x"], res["gamma_y"]))
    print("  E_b = %.5f   E_d = %.5f" % (res["E_b1"], res["E_d1"]))
    if args.turn_done_mm is not None:
        print("turn shortfall used: %.2f %% of the arc (turn-done %.2f mm)" % (100.0 * res["turn_short"], args.turn_done_mm))
    print("exact kinematics (used for the result):")
    print("  E_b = b_actual / b_model = %.5f      E_d = D_right / D_left (ground per count) = %.5f"
          % (res["E_b"], res["E_d"]))
    print("  fit residual (rms over the 4 centre-of-gravity coordinates): %.2f mm" % res["fit_rms_mm"])

    warn = []
    scat = max([g[k] for g in (res["cw"], res["ccw"]) for k in ("sdx", "sdy") if g[k] == g[k]] + [0.0])
    if res["fit_rms_mm"] > max(3.0, 2.0 * scat):
        warn.append("the exact fit leaves %.1f mm of residual (run scatter %.1f mm): the data do not fit the "
                    "two-parameter model. Check the axes and the signs of the measurements, that the start pose is "
                    "the same for every run, and that no wheel slipped." % (res["fit_rms_mm"], scat))
    if abs(res["alpha"]) > 0.1 or abs(res["gamma"]) > 0.15:
        warn.append("alpha or gamma is very large: errors this size point to wrong data, not to odometry")
    sig = significance(res)
    if sig is False:
        warn.append("both centres of gravity are within 2 standard errors of zero: the correction below is "
                    "indistinguishable from noise. Do not enter it; the vehicle already meets the data.")
    elif sig is None:
        warn.append("fewer than 2 runs in one direction: no scatter estimate, significance not checked")
    for w in warn:
        print("WARNING: " + w)
    refuse = [w for w in warn if w.startswith(("the exact fit", "alpha or gamma", "both centres"))]

    lines, ok = [], True
    for name, pid in (("odo_wheelbase", 203), ("odo_scale_l", 201), ("odo_scale_r", 202)):
        s, inrange = fmt_par(name, pid, res[name])
        lines.append((s, name, inrange, res[name]))
        ok = ok and inrange
    print("")
    if refuse and not args.force:
        print("REFUSED: no console lines are printed because of the warning(s) above (--force overrides, "
              "only to look at the numbers).")
        return 2
    if not ok:
        print("REFUSED: result outside the parameter range of spec 10.2:")
        for s, name, inrange, v in lines:
            lo, hi = PAR_RANGES[name]
            print("  %s = %.5f  range %.2f..%.2f %s" % (name, v, lo, hi, "" if inrange else "<-- out"))
        return 2
    print("Type into the console with the vehicle stationary (then P201? P202? P203? to read back):")
    for s, name, inrange, v in lines:
        print(s)
    print("(mean scale kept: (s_l + s_r)/2 = %.6f; old: %.6f)"
          % (0.5 * (res["odo_scale_l"] + res["odo_scale_r"]), 0.5 * (args.scale_l + args.scale_r)))
    print("For the compiled defaults: odo_scale_l %.5ff, odo_scale_r %.5ff, odo_wheelbase %.5ff"
          % (res["odo_scale_l"], res["odo_scale_r"], res["odo_wheelbase"]))
    print("Repeat the 10 runs with the new values (--verify) before trusting them.")
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--selftest", action="store_true", help="simulate known errors, check the recovery, exit")
    p.add_argument("--side-mm", type=float, default=2000.0, help="side of the square in mm (default 2000)")
    p.add_argument("--wheelbase", type=float, help="odo_wheelbase currently in the firmware, m (P203?)")
    p.add_argument("--scale-l", type=float, default=1.0, help="odo_scale_l currently in the firmware (P201?)")
    p.add_argument("--scale-r", type=float, default=1.0, help="odo_scale_r currently in the firmware (P202?)")
    p.add_argument("--csv", help="CSV file, see above")
    p.add_argument("--cw", help="CW runs as 'x,y;x,y;...' in mm; write --cw=... when the first number is negative")
    p.add_argument("--ccw", help="CCW runs, same format")
    p.add_argument("--turn-done-mm", type=float,
                   help="mean |DONE L| and |DONE R| of the W90/W-90 turns in mm; corrects E_b for the SETTLE shortfall")
    p.add_argument("--verify", action="store_true", help="data are from a run with the corrected values: pass/fail only")
    p.add_argument("--limit-mm", type=float, default=10.0, help="verify limit in mm (default 10, spec 1.4)")
    p.add_argument("--force", action="store_true", help="print the P lines even when the data fail the plausibility checks")
    p.add_argument("--min-runs", type=int, default=3, help="minimum runs per direction (default 3, procedure uses 5)")
    args = p.parse_args(argv)

    if args.selftest:
        return selftest()
    try:
        cw, ccw, hdg = [], [], {"cw": [], "ccw": []}
        if args.csv:
            with io.open(args.csv, encoding="utf-8") as f:
                cw, ccw, hdg = parse_csv(f.read())
        if args.cw:
            cw += parse_pairs(args.cw)
        if args.ccw:
            ccw += parse_pairs(args.ccw)
        if not cw and not ccw:
            raise DataError("no data: give --csv, or --cw and --ccw")
        if args.wheelbase is None and not args.verify:
            raise DataError("--wheelbase is required (the odo_wheelbase now in the firmware, in m)")
        for name, v in (("--scale-l", args.scale_l), ("--scale-r", args.scale_r)):
            if not 0.5 < v < 2.0:
                raise DataError("%s %.4f is not a plausible scale" % (name, v))
        return report(args, cw, ccw, hdg)
    except (DataError, OSError) as e:
        print("error: %s" % e, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
