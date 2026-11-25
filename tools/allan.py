#!/usr/bin/env python3
"""IMU standstill analysis for MR2006B, gate H7 (spec 14.3) and EKF noise parameters (spec 10.3).

Input: a USB console log with the `IMUS` stream (spec D-74), one line per S_CTRL period (20 ms):
    IMUS <t_us> <n> <wx> <wy> <wz> <T>
t_us   time of the last IMU sample in the window (TIM2, 1 us, wraps every 71.6 min)
n      total processed IMU samples since boot
w*     mean body rate over the window in mdps (z after imu_gz_scale and the temperature model)
T      temperature in 0.01 degC
Other lines in the log are ignored, so a plain `screen -L` capture works.

Procedure (PHASE2_IMU_EKF.md): vehicle powered >= 5 min, standing on the floor, motors off,
nobody touching it. `IMUS` to start, 30 min (at least 10), `IMUS` to stop.

Output
  1. DRDY rate = (n_last - n_first) / (t_last - t_first) against --odr (H7: +/- 0.1 %)
  2. mean rate (bias) and temperature span per axis, z drift against temperature
  3. overlapping Allan deviation per axis (table)
  4. from the z axis: angle random walk N (-> ekf_gyro_nd, P503), bias instability, ZUPT floor
     sqrt(sigma_z(T)^2 - N^2/T) at T = --zupt-T (-> ekf_zupt_floor, P508; the ZUPT variance of
     spec 7.17.3 is ekf_gyro_nd^2/T + ekf_zupt_floor^2), rate random walk K (-> ekf_q_bg = K^2, P505)
     when a +1/2 slope is visible after the minimum.
Refuses to print P lines (exit 2) when the log is shorter than 600 s, more than 1 % of the windows
are missing, or the vehicle moved (a 1 s mean of z differs from the overall mean by > 1 deg/s).
Exit status: 0 ok, 1 usage/data error, 2 refused, 3 rate outside +/- 0.1 % (H7 failed; P lines still printed).

  allan.py LOG [--odr 833] [--zupt-T 1.0]
  allan.py --selftest
"""
import argparse
import math
import random
import re
import sys

LINE = re.compile(r"IMUS (\d+) (\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+)")
D2R = math.pi / 180.0


def parse(lines):
    """-> list of (t_s, n, wx, wy, wz [deg/s], T [degC]) with t unwrapped."""
    out, wrap, prev = [], 0, None
    for ln in lines:
        m = LINE.fullmatch(ln.strip())
        if not m:
            continue                                  # also lines merged by a lost CR/LF in the capture
        t, n = int(m.group(1)), int(m.group(2))
        if prev is not None and t < prev:
            wrap += 1 << 32
        prev = t
        w = [int(m.group(k)) / 1000.0 for k in (3, 4, 5)]
        out.append(((t + wrap) * 1e-6, n, w[0], w[1], w[2], int(m.group(6)) / 100.0))
    return out


def adev(t, w, taus):
    """Overlapping Allan deviation of window means w at times t (unequal dt allowed): -> [(tau, sigma)]."""
    th = [0.0]
    for k in range(1, len(w)):
        th.append(th[-1] + w[k] * (t[k] - t[k - 1]))
    tau0 = (t[-1] - t[0]) / (len(t) - 1)
    res = []
    for tau in taus:
        m = max(1, int(round(tau / tau0)))
        n = len(th) - 2 * m
        if n < 8:
            break
        s = 0.0
        for k in range(n):
            d = th[k + 2 * m] - 2.0 * th[k + m] + th[k]
            s += d * d
        tau_m = m * tau0
        res.append((tau_m, math.sqrt(s / (2.0 * tau_m * tau_m * n))))
    return res


def log_taus(tau0, t_total):
    taus, tau = [], tau0
    while tau <= t_total / 4.0:
        taus.append(tau)
        tau *= 10 ** 0.1
    return taus


def interp(ad, tau):
    """log-log interpolation of sigma at tau."""
    for (t1, s1), (t2, s2) in zip(ad, ad[1:]):
        if t1 <= tau <= t2:
            f = (math.log(tau) - math.log(t1)) / (math.log(t2) - math.log(t1))
            return math.exp(math.log(s1) + f * (math.log(s2) - math.log(s1)))
    return None


def analyse(rows, odr, zupt_t, out=print, spike_lim=3.0, despike=False):
    if len(rows) < 100:
        out("ERROR: fewer than 100 IMUS lines")
        return 1, None
    # DRDY rate from the board's own counters: valid across gaps in the capture
    rate = (rows[-1][1] - rows[0][1]) / (rows[-1][0] - rows[0][0])
    dev = rate / odr - 1.0
    dts = sorted(rows[k][0] - rows[k - 1][0] for k in range(1, len(rows)))
    dt_med = dts[len(dts) // 2]
    # Allan needs contiguous windows: split at gaps (lost lines) and keep the longest piece
    segs, cur = [], [rows[0]]
    for a, b in zip(rows, rows[1:]):
        if b[0] - a[0] > 1.5 * dt_med or b[1] <= a[1]:
            segs.append(cur)
            cur = []
        cur.append(b)
    segs.append(cur)
    out(f"lines {len(rows)}, span {rows[-1][0] - rows[0][0]:.1f} s, window {dt_med * 1e3:.2f} ms, "
        f"gaps {len(segs) - 1}: pieces " + ", ".join(f"{s_[-1][0] - s_[0][0]:.0f} s" for s_ in segs))
    out(f"DRDY rate {rate:.3f} Hz against ODR {odr:g} Hz: {dev * 100:+.3f} %  "
        f"{'PASS' if abs(dev) <= 1e-3 else 'FAIL'} (H7: +/- 0.1 %)")
    rows = max(segs, key=lambda s_: s_[-1][0] - s_[0][0])
    t = [r[0] for r in rows]
    dur = t[-1] - t[0]
    out(f"analysed piece: {dur:.1f} s, {len(rows)} windows")
    # single-window spikes (one corrupted IMU sample moves a 20 ms mean by ~17 deg/s)
    n_spk = [0, 0, 0]
    rows = [list(r) for r in rows]
    for i, ax in enumerate((2, 3, 4)):
        med = sorted(r[ax] for r in rows)[len(rows) // 2]
        for r in rows:
            if abs(r[ax] - med) > spike_lim:
                n_spk[i] += 1
                if despike:
                    r[ax] = med
    if sum(n_spk):
        out(f"SPIKES: {n_spk[0]} x, {n_spk[1]} y, {n_spk[2]} z windows off by > {spike_lim:g} deg/s from the median "
            f"({'replaced by the median (--despike)' if despike else 'not removed'})")

    temps = [r[5] for r in rows]
    for ax, name in ((2, "x"), (3, "y"), (4, "z")):
        w = [r[ax] for r in rows]
        out(f"mean w{name} {sum(w) / len(w):+.4f} deg/s")
    out(f"temperature {min(temps):.2f} .. {max(temps):.2f} degC")
    wz = [r[4] for r in rows]
    if max(temps) - min(temps) >= 2.0:
        mt, mw = sum(temps) / len(temps), sum(wz) / len(wz)
        b = sum((a - mt) * (c - mw) for a, c in zip(temps, wz)) / sum((a - mt) ** 2 for a in temps)
        out(f"z drift against temperature {b * 1e3:+.2f} mdps/K (imu_tc1 would be {b * D2R:+.3e} rad/s/K; "
            "only with a real warm-up span, REQ-IMU-03)")
    else:
        out("temperature span < 2 K: no temperature coefficient (REQ-IMU-03 needs a warm-up run)")

    taus = log_taus(dt_med, dur)
    ads = [adev(t, [r[ax] for r in rows], taus) for ax in (2, 3, 4)]
    out("tau [s]      sigma_x       sigma_y       sigma_z   [deg/s]")
    for k in range(len(ads[2])):
        out(f"{ads[2][k][0]:8.3f}  {ads[0][k][1]:.6f}  {ads[1][k][1]:.6f}  {ads[2][k][1]:.6f}")

    adz = ads[2]
    white = [(ta, s) for ta, s in adz if 0.1 <= ta <= 1.0]
    if not white:
        out("ERROR: no tau between 0.1 and 1 s")
        return 1, None
    N = math.exp(sum(math.log(s) + 0.5 * math.log(ta) for ta, s in white) / len(white))   # sigma = N / sqrt(tau)
    i_min = min(range(len(adz)), key=lambda k: adz[k][1])
    B = adz[i_min][1] / 0.664
    s_T = interp(adz, zupt_t)
    floor = math.sqrt(max(0.0, s_T * s_T - N * N / zupt_t)) if s_T is not None else None
    rrw = [(ta, s) for ta, s in adz[i_min + 1:] if ta >= 3.0 * adz[i_min][0]]
    K = None
    if len(rrw) >= 3:
        slope = (math.log(rrw[-1][1]) - math.log(rrw[0][1])) / (math.log(rrw[-1][0]) - math.log(rrw[0][0]))
        if slope >= 0.3:
            K = math.exp(sum(math.log(s) - 0.5 * math.log(ta / 3.0) for ta, s in rrw) / len(rrw))   # sigma = K sqrt(tau/3)
    out(f"z: angle random walk N = {N * 1e3:.2f} mdps/rtHz = {N * D2R:.3e} rad/s/rtHz")
    out(f"z: bias instability {B * 1e3:.2f} mdps at tau {adz[i_min][0]:.1f} s")
    if floor is not None:
        out(f"z: sigma({zupt_t:g} s) {s_T * 1e3:.2f} mdps, white part {N / math.sqrt(zupt_t) * 1e3:.2f} mdps, "
            f"floor {floor * 1e3:.2f} mdps")
    out(f"z: rate random walk K = {K * D2R:.3e} rad/s/rts" if K else
        "z: no +1/2 slope after the minimum (rate random walk not visible; log longer or keep ekf_q_bg)")

    moved = False
    nwin = max(1, int(round(1.0 / dt_med)))
    mz = sum(wz) / len(wz)
    for k in range(0, len(wz) - nwin, nwin):
        if abs(sum(wz[k:k + nwin]) / nwin - mz) > 1.0:
            moved = True
            break
    refuse = []
    if dur < 600.0:
        refuse.append("longest contiguous piece shorter than 600 s")
    if sum(n_spk) and not despike:
        refuse.append("single-window spikes in the data (corrupted samples?); check them, --despike to analyse anyway")
    if moved:
        refuse.append("vehicle moved (a 1 s mean of z is > 1 deg/s off)")
    if refuse:
        out("REFUSED: " + "; ".join(refuse))
        return 2, None
    p = {503: N * D2R}
    if floor is not None:
        p[508] = floor * D2R
    if K:
        p[505] = (K * D2R) ** 2
    out(f"P503={p[503]:.3e}   # ekf_gyro_nd")
    if 508 in p:
        out(f"P508={p[508]:.3e}   # ekf_zupt_floor")
    if 505 in p:
        out(f"P505={p[505]:.3e}   # ekf_q_bg")
    return (0 if abs(dev) <= 1e-3 else 3), {"rate": rate, "N": N, "floor": floor, "K": K}


def selftest():
    """Synthetic stream in the real format: ODR 829.5 Hz, bias, white noise N, rate random walk K."""
    rnd = random.Random(1)
    odr_true, N, K, dt = 829.5, 0.006, 3e-4, 0.02           # deg/s/rtHz, deg/s/rts
    lines, t, n, b = [], 1000.0, 0, 0.0
    t_us = 4294000000                                        # wraps after ~1 s
    for k in range(int(1800 / dt)):
        t += dt
        n_new = int(t * odr_true) - n
        n += n_new
        b += K * math.sqrt(dt) * rnd.gauss(0.0, 1.0)
        w = [2.0 + b + N / math.sqrt(dt) * rnd.gauss(0.0, 1.0) for _ in range(3)]
        t_us = (t_us + int(round(dt * 1e6))) & 0xFFFFFFFF
        lines.append("IMUS %d %d %d %d %d %d" % (t_us, n, *(round(x * 1000) for x in w), 2100 + k // 9000))
        if k % 997 == 0:
            lines.append("> IMU")                         # other console lines are ignored
    quiet = []
    code, r = analyse(parse(lines), 833.0, 1.0, out=quiet.append)
    ok = True
    def chk(cond, what):
        nonlocal ok
        print(("ok    " if cond else "FAIL  ") + what)
        ok = ok and cond
    chk(code == 3, f"rate {r and r['rate']:.3f} Hz -> H7 FAIL reported (exit {code})")
    chk(r is not None and abs(r["rate"] / odr_true - 1.0) < 2e-5, "DRDY rate within 20 ppm of the true 829.5 Hz")
    chk(r is not None and abs(r["N"] / N - 1.0) < 0.10, f"N {r and r['N'] * 1e3:.2f} mdps/rtHz within 10 % of 6.00")
    chk(r is not None and r["K"] is not None and 0.3 < r["K"] / K < 3.0, "rate random walk seen, within a factor 3")
    moved = [ln for ln in lines]
    for k in range(20000, 20100):
        p = moved[k].split()
        if p[0] == "IMUS":
            p[5] = str(int(p[5]) + 50000)
            moved[k] = " ".join(p)
    code2, _ = analyse(parse(moved), 833.0, 1.0, out=quiet.append)
    chk(code2 == 2, "a 50 deg/s turn in the log is refused")
    bad = list(lines)
    bad[1000] = bad[1000][:-6] + bad[1001]                # two lines merged by a lost CR/LF
    del bad[2000:2500]                                    # 10 s missing in the capture
    for k in (5000, 40000, 80000):                        # single corrupted samples on y
        p = bad[k].split()
        if p[0] == "IMUS":
            p[4] = str(int(p[4]) + 17500)
            bad[k] = " ".join(p)
    rows_b = parse(bad)
    chk(len(rows_b) == len([x for x in bad if x.startswith("IMUS")]) - 1, "a merged line is skipped, not parsed")
    code3, _ = analyse(rows_b, 833.0, 1.0, out=quiet.append)
    chk(code3 == 2, "spikes refuse the P lines without --despike")
    code4, r4 = analyse(rows_b, 833.0, 1.0, out=quiet.append, despike=True)
    chk(r4 is not None and abs(r4["rate"] / odr_true - 1.0) < 2e-5, "rate unchanged across a gap")
    chk(r4 is not None and abs(r4["N"] / N - 1.0) < 0.10, "N within 10 % on the longest piece after --despike")
    print("SELFTEST PASSED" if ok else "SELFTEST FAILED")
    return 0 if ok else 1


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("log", nargs="?")
    p.add_argument("--odr", type=float, default=833.0, help="nominal ODR in Hz (spec 7.11.1)")
    p.add_argument("--zupt-T", type=float, default=1.0, help="ZUPT window, ekf_zupt_T (P509)")
    p.add_argument("--despike", action="store_true", help="replace single-window spikes by the median")
    p.add_argument("--spike-lim", type=float, default=3.0, help="spike threshold in deg/s")
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args(argv)
    if a.selftest:
        return selftest()
    if not a.log:
        p.print_usage()
        return 1
    with open(a.log, errors="replace") as f:
        code, _ = analyse(parse(f), a.odr, a.zupt_T, spike_lim=a.spike_lim, despike=a.despike)
    return code


if __name__ == "__main__":
    sys.exit(main())
