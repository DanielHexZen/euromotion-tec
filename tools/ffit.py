#!/usr/bin/env python3
"""Feedforward identification for MR2006B from a captured console log (PHASE1_CALIBRATION.md section 6).

What the firmware gives (Challenge_TestHC/Core/Src/app.c, app_cmd.c, console.c, read on 2026-11-10):
  * open-loop duty commands `<-100..100>` (both wheels) and `PWM <l%> <r%>`, integer percent, with the
    duty slew limit mot_slew_ol (2 /s) and the 2 s dead-man (any received byte, also a bare Enter,
    keeps it alive);
  * `a` prints a status line every 300 ms (TLM slot 100 ms, print when >= 250 ms elapsed):
        L=<cnt> (<x mm>, <v mm/s>)  R=<cnt> (<x mm>, <v mm/s>)  <mode> <phase>
    x is cnt * odo_m_per_cnt * odo_scale with 0.1 mm resolution, v is the filtered speed estimate
    (3.2 mm/s per count at 50 Hz, so single values are noisy), phase is OPEN in open loop;
  * every received line is echoed as `> <line>`, which is how this tool knows the duty of each step.
There are no time stamps in the lines. The tool therefore uses the print period (--period, default 0.300 s,
derived from the code; verify it once with the `period` sub-command) and takes the speed from the
difference of the printed positions over the steady window (resolution well below 1 mm/s), cross-checked
against the mean of the printed v values.

Model (spec 7.8, REQ-WHL-01):  |duty| = Ks + Kv |v|  per wheel while rolling, sign of v = sign of duty.

Sub-commands
  steady LOG      least squares of duty versus steady speed per wheel -> Ks, Kv (IDs 116-119)
  breakaway LOG   lowest duty at which a wheel starts from rest -> Ks_break (IDs 120, 121)
  period LOG      --seconds N: status lines per second in a log of N seconds of `a` -> print period
  --selftest      synthetic logs in the real format with known Ks, Kv, Ksb; checks the recovery

Refuses to print P lines when a wheel has fewer than 4 usable points or a speed span below 0.15 m/s
(intercept would be extrapolated), or when the result is outside the parameter range of spec 10.1
(Ks 0..0.5, Kv 0.5..3, Ksb 0..0.6).
Exit status: 0 ok, 1 usage/data error, 2 refused.
"""
import argparse
import io
import math
import re
import sys

RANGES = {"ks": (0.0, 0.5), "kv": (0.5, 3.0), "ksb": (0.0, 0.6)}
IDS = {("ks", 0): 116, ("kv", 0): 117, ("ks", 1): 118, ("kv", 1): 119, ("ksb", 0): 120, ("ksb", 1): 121}
NAMES = {116: "whl_ks_l", 117: "whl_kv_l", 118: "whl_ks_r", 119: "whl_kv_r", 120: "whl_ksb_l", 121: "whl_ksb_r"}
WHEEL = ("left", "right")

STATUS = re.compile(r"L=(-?\d+) \((-?\d+(?:\.\d+)?) mm, (-?\d+) mm/s\)\s+R=(-?\d+) \((-?\d+(?:\.\d+)?) mm, "
                    r"(-?\d+) mm/s\)\s+(\w+) (\w+)")
ECHO = re.compile(r"(?:^|\s)> (.*)$")
INTCMD = re.compile(r"^[+-]?\d+$")
PWMCMD = re.compile(r"^PWM\s+([+-]?\d+)\s+([+-]?\d+)$")
KEEP = {"a", "p", "S", "I", "k", "?", "L", "EV", "FAULTS", ""}


class DataError(Exception):
    pass


# ----------------------------------------------------------------------------- parsing
class Seg:
    def __init__(self, duty, rest_before):
        self.duty = duty                # (u_l, u_r) as fractions
        self.rest_before = rest_before  # True when the last status line before the command showed both wheels at rest
        self.s = []                     # samples: (cnt_l, x_l_mm, v_l_mms, cnt_r, x_r_mm, v_r_mms)


def parse_log(text):
    """Returns the list of open-loop segments (phase OPEN, duty != 0), in log order."""
    segs, cur, last = [], None, None
    for raw in text.splitlines():
        line = raw.strip("\r\n")
        m = STATUS.search(line)
        if m:
            cl, xl, vl, cr, xr, vr, mode, phase = m.groups()
            sample = (int(cl), float(xl), float(vl), int(cr), float(xr), float(vr))
            if cur is not None and phase == "OPEN":
                cur.s.append(sample)
            last = sample
            continue
        e = ECHO.search(line)
        if not e:
            continue
        cmd = e.group(1).strip()
        new = "same"
        if INTCMD.match(cmd):
            u = int(cmd) / 100.0
            new = (u, u)
        else:
            mp = PWMCMD.match(cmd)
            if mp:
                new = (int(mp.group(1)) / 100.0, int(mp.group(2)) / 100.0)
            elif cmd in ("s", "STOP"):
                new = (0.0, 0.0)
            elif cmd in KEEP or cmd.startswith("P"):
                new = "same"
            else:
                new = None              # D, W, MODE, r, CLEAR ...: no longer an open-loop step
        if new == "same":
            continue
        if new is not None and cur is not None and cur.duty == new:
            continue                    # the same duty sent again (keep-alive)
        if new is None or new == (0.0, 0.0):
            cur = None
        else:
            rest = last is None or (abs(last[2]) < 5.0 and abs(last[5]) < 5.0)
            cur = Seg(new, rest)
            segs.append(cur)
    return [s for s in segs if s.s]


# ----------------------------------------------------------------------------- statistics
def mean(v):
    return sum(v) / len(v)


def ols(points):
    """y = a + b x. Returns dict with a, b, se_a, se_b, r2, rms or None for < 3 points / no spread."""
    n = len(points)
    if n < 3:
        return None
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    mx, my = mean(xs), mean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0.0:
        return None
    b = sum((x - mx) * (y - my) for x, y in points) / sxx
    a = my - b * mx
    ss = sum((y - a - b * x) ** 2 for x, y in points)
    s2 = ss / (n - 2)
    syy = sum((y - my) ** 2 for y in ys)
    return {"a": a, "b": b, "se_b": math.sqrt(s2 / sxx), "se_a": math.sqrt(s2 * (1.0 / n + mx * mx / sxx)),
            "r2": 1.0 - ss / syy if syy > 0 else float("nan"), "rms": math.sqrt(ss / n), "n": n}


# ----------------------------------------------------------------------------- steady state
def steady_points(segs, period, skip_s, min_s, vmin):
    """Per segment and wheel: steady speed from the window after skip_s. Returns (rows, pts[2])."""
    n_skip = int(math.ceil(skip_s / period - 1e-9))
    n_min = int(math.ceil(min_s / period - 1e-9))
    rows, pts = [], ([], [])
    for k, sg in enumerate(segs, 1):
        w = sg.s[n_skip:]
        row = {"seg": k, "duty": sg.duty, "n": len(w), "v": [None, None], "vhat": [None, None],
               "flag": ["", ""]}
        rows.append(row)
        if len(w) - 1 < n_min:
            row["flag"] = ["short", "short"]
            continue
        n_int = len(w) - 1
        half = n_int // 2
        for i in (0, 1):
            xi, vi = (1, 2) if i == 0 else (4, 5)
            u = sg.duty[i]
            if u == 0.0:
                row["flag"][i] = "off"
                continue
            v = (w[-1][xi] - w[0][xi]) / (n_int * period) / 1000.0
            v1 = (w[half][xi] - w[0][xi]) / (half * period) / 1000.0 if half > 0 else v
            v2 = (w[-1][xi] - w[half][xi]) / ((n_int - half) * period) / 1000.0
            vh = mean([s[vi] for s in w]) / 1000.0
            row["v"][i], row["vhat"][i] = v, vh
            if abs(v) < vmin:
                row["flag"][i] = "slow"
            elif v * u < 0:
                row["flag"][i] = "sign"
            elif abs(v1 - v2) > max(0.01, 0.05 * abs(v)):
                row["flag"][i] = "unsteady"
            elif abs(v - vh) > max(0.01, 0.03 * abs(v)):
                row["flag"][i] = "disagree"
            else:
                row["flag"][i] = "ok"
                pts[i].append((v, u))
    return rows, pts


def fit_wheel(points, positive_only_note=True):
    """points: (v, u) with signs. Pooled fit on (|v|, |u|), plus per direction."""
    pool = [(abs(v), abs(u)) for v, u in points]
    fw = [(v, u) for v, u in points if u > 0]
    rv = [(-v, -u) for v, u in points if u < 0]
    return {"pool": ols(pool), "fwd": ols(fw), "rev": ols(rv), "n": len(points),
            "span": (max(abs(v) for v, _ in points) - min(abs(v) for v, _ in points)) if points else 0.0}


def cmd_steady(args, text):
    segs = parse_log(text)
    if not segs:
        raise DataError("no open-loop status lines found (need `a` on, phase OPEN, and the `> <duty>` echoes)")
    rows, pts = steady_points(segs, args.period, args.skip_s, args.min_s, args.vmin)
    print("%d open-loop steps; print period %.3f s, skip %.1f s, window >= %.1f s" % (len(segs), args.period, args.skip_s, args.min_s))
    print("")
    print("step  duty L/R %   n   v_L m/s (flag)        v_R m/s (flag)        v-hat L / R m/s")
    for r in rows:
        def cell(i):
            return ("%8.4f (%s)" % (r["v"][i], r["flag"][i])) if r["v"][i] is not None else "       -  (%s)" % r["flag"][i]
        vh = "%.4f / %.4f" % (r["vhat"][0], r["vhat"][1]) if r["vhat"][0] is not None else "-"
        print("%3d  %4.0f /%4.0f  %3d  %-21s %-21s %s" % (r["seg"], r["duty"][0] * 100, r["duty"][1] * 100, r["n"],
                                                       cell(0), cell(1), vh))
    print("flags: ok used | short window too short | slow below %.2f m/s | sign wheel moved against the duty | "
          "unsteady speed still changing | disagree position and v-hat differ (lost lines?) | off duty 0" % args.vmin)
    print("")
    refused = False
    out = []
    for i in (0, 1):
        f = fit_wheel(pts[i])
        print("%s wheel: %d usable points, speed span %.3f m/s" % (WHEEL[i], f["n"], f["span"]))
        if f["n"] < 4 or f["span"] < 0.15 or f["pool"] is None:
            print("  REFUSED: need >= 4 points and a span >= 0.15 m/s (steps from about 15 % to 60 %, both directions)")
            refused = True
            continue
        p = f["pool"]
        print("  pooled |duty| = Ks + Kv |v|: Ks = %.4f (se %.4f)  Kv = %.4f 1/(m/s) (se %.4f)  R2 = %.5f  rms %.4f duty"
              % (p["a"], p["se_a"], p["b"], p["se_b"], p["r2"], p["rms"]))
        for name, g in (("forward", f["fwd"]), ("reverse", f["rev"])):
            if g:
                print("  %-7s only:                Ks = %.4f           Kv = %.4f                    (%d points)"
                      % (name, g["a"], g["b"], g["n"]))
        if f["fwd"] and f["rev"] and abs(f["fwd"]["b"] - f["rev"]["b"]) > 0.05 * p["b"]:
            print("  note: Kv differs by more than 5 %% between the directions; the firmware uses one Kv per wheel")
        if p["se_a"] > 0.02:
            print("  note: se(Ks) above 0.02: more low-speed steps (15-30 %) would help")
        for key, val in (("ks", p["a"]), ("kv", p["b"])):
            lo, hi = RANGES[key]
            if not lo <= val <= hi:
                print("  REFUSED: %s = %.4f outside the parameter range %.1f..%.1f of spec 10.1" % (key.upper(), val, lo, hi))
                refused = True
        out.append((i, p["a"], p["b"]))
    print("")
    if refused or len(out) < 2:
        return 2
    print("Type into the console (live parameters; then P116? .. P119? to read back):")
    for i, ks, kv in out:
        print("P%d=%.4f" % (IDS[("ks", i)], ks))
        print("P%d=%.4f" % (IDS[("kv", i)], kv))
    print("Check afterwards with a closed-loop run (D2000): both wheels must end in the gate of H3.")
    return 0


# ----------------------------------------------------------------------------- breakaway
def cmd_breakaway(args, text):
    segs = parse_log(text)
    n_min = int(math.ceil(args.min_s / args.period - 1e-9))
    trials = {}       # (wheel, sign) -> {duty_pct: [moved, ...]}
    skipped = 0
    for sg in segs:
        if not sg.rest_before or len(sg.s) - 1 < n_min:
            skipped += 1
            continue
        for i in (0, 1):
            u = sg.duty[i]
            if u == 0.0:
                continue
            ci = 0 if i == 0 else 3
            moved = abs(sg.s[-1][ci] - sg.s[0][ci]) >= args.move_counts
            trials.setdefault((i, 1 if u > 0 else -1), {}).setdefault(int(round(abs(u) * 100)), []).append(moved)
    print("%d open-loop steps in the log, %d not used (not started from rest, or shorter than %.1f s)"
          % (len(segs), skipped, args.min_s))
    print("moved = at least %d counts between the first and last status line of the step" % args.move_counts)
    print("")
    res = {}
    refused = False
    for i in (0, 1):
        for sign, nm in ((1, "forward"), (-1, "reverse")):
            t = trials.get((i, sign))
            if not t:
                continue
            line = ", ".join("%d%%:%s" % (d, "".join("M" if m else "." for m in t[d])) for d in sorted(t))
            print("%s wheel %s  (M moved, . stayed)  %s" % (WHEEL[i], nm, line))
            ok = [d for d in sorted(t) if all(t[d])]
            if not ok:
                print("  no duty tested made it move every time: extend the range upwards")
                continue
            ksb = ok[0]
            below = [d for d in sorted(t) if d < ksb and not any(t[d])]
            above_bad = [d for d in t if d > ksb and not all(t[d])]
            res[(i, sign)] = ksb / 100.0
            print("  lowest duty at which it always moved: %d %%%s" % (ksb, ("; last step that stayed: %d %%" % below[-1]) if below else
                                                                          "  (nothing below was tested: start lower)"))
            if above_bad:
                print("  note: it did not move at %s %% although it moved at %d %%: noisy, repeat those steps"
                      % (",".join(str(d) for d in sorted(above_bad)), ksb))
    print("")
    out = []
    for i in (0, 1):
        vals = [res[(i, s)] for s in (1, -1) if (i, s) in res]
        if not vals:
            print("%s wheel: no result" % WHEEL[i])
            refused = True
            continue
        v = mean(vals)
        lo, hi = RANGES["ksb"]
        if not lo <= v <= hi:
            print("%s wheel: %.3f outside the parameter range %.1f..%.1f of spec 10.1" % (WHEEL[i], v, lo, hi))
            refused = True
            continue
        if len(vals) == 2 and abs(vals[0] - vals[1]) > 0.02:
            print("note: %s wheel forward %.2f and reverse %.2f differ by more than 0.02; the mean is entered" % (WHEEL[i], vals[0], vals[1]))
        out.append((i, v))
    if refused or len(out) < 2:
        return 2
    print("Type into the console (live parameters; then P120? P121? to read back). Compare with Ks from `steady`:")
    print("Ks_break should be at or above Ks of the same wheel.")
    for i, v in out:
        print("P%d=%.3f" % (IDS[("ksb", i)], v))
    return 0


def cmd_period(args, text):
    n = len(STATUS.findall(text))
    if args.seconds is None or args.seconds <= 0:
        raise DataError("period needs --seconds N (length of the capture, wall clock)")
    print("%d status lines in %.1f s -> period %.3f s (the code predicts 0.300 s; use --period for the other sub-commands)"
          % (n, args.seconds, args.seconds / n if n else float("nan")))
    return 0


# ----------------------------------------------------------------------------- self-test
class Plant:
    """One wheel: speed = (|u| - Ks)/Kv once rolling, breakaway at Ksb, first-order lag, 50 Hz."""

    def __init__(self, ks, kv, ksb, tau=0.05):
        self.ks, self.kv, self.ksb, self.tau = ks, kv, ksb, tau
        self.v = 0.0
        self.cnt = 0.0

    def step(self, u, dt):
        rolling = abs(self.v) > 0.008
        thr = self.ks if rolling else self.ksb
        tgt = math.copysign(max(0.0, abs(u) - thr) / self.kv, u) if abs(u) > thr else 0.0
        self.v += (tgt - self.v) * dt / self.tau
        if abs(self.v) < 1e-4 and tgt == 0.0:
            self.v = 0.0
        return self.v


def synth_log(steps, pl, pr, seed=1):
    """steps: list of (duty_l %, duty_r %, hold_s). Emits a log like the firmware's: echo, reply, `a`
    status lines every 0.3 s (phase OPEN), `s`, then 1.5 s of HOLD lines."""
    import random
    rng = random.Random(seed)
    m_per_cnt = 6.3638e-5
    out = ["HELLO MR2006B fw 0.1-phase0 abc1234 Debug reset=0x00 calib=0", "> a", "Auto print ON"]
    t = 0.0
    dt = 0.02
    cur = [0.0, 0.0]
    vh = [0.0, 0.0]
    prev_cnt = [0, 0]
    next_print = 0.3
    cnt_i = [0, 0]

    def advance(seconds, target, phase):
        nonlocal t, next_print
        n = int(round(seconds / dt))
        for _ in range(n):
            for i, p in enumerate((pl, pr)):
                c = cur[i]
                c += max(-2.0 * dt, min(2.0 * dt, target[i] - c))      # mot_slew_ol
                cur[i] = c
                v = p.step(c, dt)
                p.cnt += v / m_per_cnt * dt * (1.0 + rng.gauss(0.0, 0.10))     # 10 % speed ripple per 20 ms
                ci = int(math.floor(p.cnt + 0.5))
                raw = (ci - prev_cnt[i]) * m_per_cnt / dt
                prev_cnt[i] = ci
                cnt_i[i] = ci
                vh[i] += 0.5 * (raw - vh[i])
            t += dt
            if t >= next_print - 1e-9:
                next_print += 0.3
                out.append("L=%d (%.1f mm, %d mm/s)  R=%d (%.1f mm, %d mm/s)  BENCH %s"
                           % (cnt_i[0], cnt_i[0] * m_per_cnt * 1000.0, round(vh[0] * 1000), cnt_i[1],
                              cnt_i[1] * m_per_cnt * 1000.0, round(vh[1] * 1000), phase))

    for dl, dr, hold in steps:
        if dl == dr:
            out.append("> %d" % dl)
        else:
            out.append("> PWM %d %d" % (dl, dr))
        out.append("Open loop duty %d %% %d %%" % (dl, dr))
        advance(hold, (dl / 100.0, dr / 100.0), "OPEN")
        out.append("> s")
        out.append("STOP")
        cur[0] = cur[1] = 0.0
        for p in (pl, pr):
            p.v = 0.0
        advance(1.5, (0.0, 0.0), "HOLD")
    return "\n".join(out) + "\n"


def selftest():
    fails = []

    def check(cond, msg):
        print(("  ok    " if cond else "  FAIL  ") + msg)
        if not cond:
            fails.append(msg)

    print("parser on lines produced by the real console.c (captured from the firmware sources, 2026-11-10)")
    real = ["L=31416 (1999.5 mm, 301 mm/s)  R=-12 (-0.8 mm, 0 mm/s)  BENCH OPEN",
            "> PWM 20 -20", "> 30"]
    m = STATUS.search(real[0])
    check(m is not None and m.group(1) == "31416" and m.group(6) == "0" and m.group(8) == "OPEN", "status line")
    check(ECHO.search(real[1]).group(1) == "PWM 20 -20" and ECHO.search(real[2]).group(1) == "30", "echo lines")

    print("steady fit: synthetic wheels with known Ks, Kv (left 0.098/1.275, right 0.041/1.361 = the 9 Oct defaults)")
    pl, pr = Plant(0.098, 1.275, 0.141), Plant(0.041, 1.361, 0.169)
    steps = []
    for u in (20, 30, 40, 50, 60):
        steps += [(u, u, 6.0), (-u, -u, 6.0)]
    text = synth_log(steps, pl, pr)
    ns = steps and len(parse_log(text))
    check(ns == 10, "10 open-loop steps found (%d)" % ns)

    class A:
        pass
    a = A()
    a.period, a.skip_s, a.min_s, a.vmin = 0.3, 2.0, 2.4, 0.02
    segs = parse_log(text)
    rows, pts = steady_points(segs, a.period, a.skip_s, a.min_s, a.vmin)
    for i, (ks, kv) in enumerate(((0.098, 1.275), (0.041, 1.361))):
        f = fit_wheel(pts[i])
        p = f["pool"]
        check(f["n"] >= 8, "%s: %d usable steps" % (WHEEL[i], f["n"]))
        check(abs(p["a"] - ks) < 0.01, "%s: Ks %.4f (true %.3f, tol 0.01)" % (WHEEL[i], p["a"], ks))
        check(abs(p["b"] / kv - 1.0) < 0.02, "%s: Kv %.4f (true %.3f, tol 2 %%)" % (WHEEL[i], p["b"], kv))
    import contextlib
    buf = io.StringIO()
    a.cmd = "steady"
    with contextlib.redirect_stdout(buf):
        rc = cmd_steady(a, text)
    check(rc == 0 and "P117=" in buf.getvalue() and "P118=" in buf.getvalue(), "sub-command prints P116..P119 lines")

    print("steady fit refuses a narrow speed span and a ramp-only log")
    narrow = synth_log([(40, 40, 6.0), (-40, -40, 6.0), (42, 42, 6.0), (-42, -42, 6.0)], Plant(0.098, 1.275, 0.141),
                       Plant(0.041, 1.361, 0.169))
    with contextlib.redirect_stdout(io.StringIO()):
        rc = cmd_steady(a, narrow)
    check(rc == 2, "span below 0.15 m/s refused")
    short = synth_log([(30, 30, 2.0), (-30, -30, 2.0)], Plant(0.098, 1.275, 0.141), Plant(0.041, 1.361, 0.169))
    rows, pts = steady_points(parse_log(short), 0.3, 2.0, 2.4, 0.02)
    check(not pts[0] and not pts[1], "2 s steps are too short for the window (nothing used)")

    print("breakaway: true Ksb 0.141 / 0.169, steps of 1 % from rest, 2 s each, two repeats")
    steps = []
    for u in range(8, 21):
        for _ in range(2):
            steps.append((u, u, 2.0))
    text = synth_log(steps, Plant(0.098, 1.275, 0.141), Plant(0.041, 1.361, 0.169))
    # reverse
    steps = [(-u, -u, 2.0) for u in range(8, 21) for _ in range(2)]
    text += synth_log(steps, Plant(0.098, 1.275, 0.141), Plant(0.041, 1.361, 0.169), seed=2)
    a.move_counts, a.min_s = 10, 1.2
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = cmd_breakaway(a, text)
    o = buf.getvalue()
    check(rc == 0, "sub-command ok")
    mm = re.findall(r"P(\d+)=([0-9.]+)", o)
    got = {int(k): float(v) for k, v in mm}
    check(120 in got and abs(got[120] - 0.141) <= 0.0101, "Ksb left %s (true 0.141, step 0.01)" % got.get(120))
    check(121 in got and abs(got[121] - 0.169) <= 0.0101, "Ksb right %s (true 0.169, step 0.01)" % got.get(121))

    print("period sub-command")
    a.seconds = 30.0
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        cmd_period(a, "\n".join(["L=1 (0.1 mm, 0 mm/s)  R=1 (0.1 mm, 0 mm/s)  IDLE HOLD"] * 100))
    check("period 0.300" in buf.getvalue(), "100 lines in 30 s = 0.300 s")
    print("")
    if fails:
        print("SELFTEST FAILED (%d)" % len(fails))
        return 1
    print("SELFTEST PASSED")
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--selftest", action="store_true")
    sub = p.add_subparsers(dest="cmd")
    for name in ("steady", "breakaway", "period"):
        s = sub.add_parser(name)
        s.add_argument("log", help="captured console log (text)")
        s.add_argument("--period", type=float, default=0.300, help="status print period in s (default 0.300)")
        if name == "steady":
            s.add_argument("--skip-s", type=float, default=2.0, help="seconds to skip after each duty change (default 2.0)")
            s.add_argument("--min-s", type=float, default=2.4, help="minimum steady window in s (default 2.4)")
            s.add_argument("--vmin", type=float, default=0.02, help="ignore steps slower than this, m/s (default 0.02)")
        if name == "breakaway":
            s.add_argument("--min-s", type=float, default=1.2, help="minimum step length in s (default 1.2)")
            s.add_argument("--move-counts", type=int, default=10, help="counts that mean 'moved' (default 10)")
        if name == "period":
            s.add_argument("--seconds", type=float, help="wall-clock length of the capture")
    args = p.parse_args(argv)
    if args.selftest:
        return selftest()
    if not args.cmd:
        p.print_usage(sys.stderr)
        return 1
    try:
        with io.open(args.log, encoding="utf-8", errors="replace") as f:
            text = f.read()
        return {"steady": cmd_steady, "breakaway": cmd_breakaway, "period": cmd_period}[args.cmd](args, text)
    except (DataError, OSError) as e:
        print("error: %s" % e, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
