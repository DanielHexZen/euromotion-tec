# Calibration Firmware (APP_CALIB=1)

Optional second route to the feedforward numbers (`FIRMWARE_SPEC.md` §7.27, REQ-CAL-01…03, Appendix 19.2). Phase 1 normally runs with the **vehicle firmware** and its console (`PHASE1_CALIBRATION.md`, D-31). Use this firmware when you need `whl_tau_m` (K) or a 50 Hz step record, or want G/B steps without typing duty commands by hand.

**Status: not yet run on hardware.** The formats below come from reading `Core/Src/calib.c`; `tools/calib_host.py selftest` checks the parser against them, nothing more. Treat the first run as a commissioning test, not a calibration.

## 1. Build

- Set `#define APP_CALIB 1` in `Core/Inc/app_config.h`, or add `-DAPP_CALIB=1` to the compiler symbols. Rebuild everything (clean).
- `calib.c` stays unchanged (CLAUDE.md). `app.c` is not initialised in this build.
- TIM6 is generated at 1 kHz; `main.c` (USER CODE 2, `APP_CALIB` branch) writes ARR = 19999 before `Calib_Init`, so the control tick is **50 Hz** (REQ-CAL-02). If it is not, every K time and every G speed is wrong.
- Switch back with `APP_CALIB 0` before any vehicle-track work. Never flash this build for a gate test of the vehicle firmware.

## 2. Safety

- **Wheels in the air** (vehicle on blocks) for C, B, K, L. G may run on the floor with free run-out of ≥ 2 m; it drives both wheels forward and back for 6 s per step.
- `s` (any UART, also from the radio) aborts at once: bridge off in the receive interrupt, then `STOP`.
- Duty is clamped to 60 % on every command. Duty ramps at `mot_slew_ol` except in K (a real step).
- Stall guard: at duty ≥ 35 % a wheel that does not move in the commanded direction for 0.5 s ends the test with `FAULT STALL_OR_SIGN wheel <L|R>`. A fault usually means a wrong sign or a blocked wheel, not a calibration result.
- Run C first. Do nothing else if it does not pass.

## 3. Commands

Console: USB (USART2, 115200 8N1), mirrored on the HC-12 (USART1). Prefer USB: the radio drains 96 bytes per 100 ms and drops lines when the capture is long. `?` prints help.

| Cmd | Test | Duration | Output (SI/legacy units on the wire: counts, mm, mm/s, duty %) |
|---|---|---|---|
| `C` | sign check: each wheel alone, fwd and rev, 30 % for 0.8 s | ~6 s | `C,<L\|R>,<fwd\|rev>,30,<dcnt_l>,<dcnt_r>,PASS\|FAIL`, then `DONE C PASS\|FAIL`. PASS: ≥ 100 counts on the driven wheel, < 100 on the other. Check by eye that the wheel really turned forward. |
| `G` | steady steps 15, 20, 30, 40, 50, 60 %, fwd and rev, 6 s each | ~90 s | `G,<signed duty %>,<v_l mm/s>,<v_r mm/s>` (mean of the second half of the step), `DONE G`. |
| `B` | breakaway: from 8 % in 1 % steps up to 34 %, each wheel, fwd and rev, until ≥ 10 counts | up to ~5 min | `B,<L\|R>,<fwd\|rev>,<duty %>` or `NONE`, `DONE B`. |
| `K [pct]` | step response, default 40 %, max 60 % | 2 s run, then a dump of a few s | `K,<t_ms>,<duty_milli>,<dcnt_l>,<dcnt_r>` × 100 (counts per 20 ms), step at 200 ms, off at 1400 ms; `DONE K`. |
| `L` | reversal latency, 25 %, each wheel | ~6 s | `L,<L\|R>,<fwd_cnt>,<rev_cnt>,<fwd_latency_ms>,<rev_latency_ms>`, `DONE L`. |
| `odo` | raw counts since `r` | – | `odo,<cnt_l>,<cnt_r>` |
| `r`, `s`, `?` | reset counters (not while a test runs), stop, help | – | `Counters reset`, `STOP` |
| `U`, `A` | not available | – | `ERR not available` |

G and B also print the vehicle console's status line (`L=… R=… CALIB OPEN`) every 0.3 s and echo `> <duty>`, so `tools/ffit.py steady|breakaway` can read the same log. `calib_host.py` uses the `G,`/`B,` lines instead.

## 4. Capture a log

No serial code on the host. Capture every received byte to a file, as in `PHASE1_CALIBRATION.md` §3:

```
screen -L -Logfile tools/data/<date>/calib.log /dev/cu.usbmodem11403 115200
```

Old `screen` (macOS) has no `-Logfile` and writes `screenlog.0`; the CubeIDE terminal log works too. Send the commands, wait for `DONE …`, then end `screen` with `Ctrl-A K`. One log per session is fine; `eval` accepts several files. Keep logs with the commit that changes the defaults.

## 5. Evaluate

From the repository root (Python 3 standard library only):

```
python3 -I tools/calib_host.py selftest
python3 tools/calib_host.py check tools/data/<date>/calib.log
python3 tools/calib_host.py eval  tools/data/<date>/calib.log [--header calib_result.h]
```

- `check`: C PASS/FAIL per wheel and direction, FAULT/STOP/ERR/WARN lines, malformed lines (radio drops, truncated capture). Exit 2 on any failure.
- `eval`: fits what the log holds, prints `P<id>=<value>` lines for the console (live, RAM only, D-7) and, with `--header`, `#define PAR_<name> <value>f` lines (the format of the console `L`) to paste into `par_table.h` defaults.
- Methods: G by `ffit.fit_wheel` (least squares |duty| = Ks + Kv |v| per wheel, points below 20 mm/s dropped, refused with < 4 points or a speed span < 0.15 m/s or outside the §10 range). B: mean of the fwd and rev duty per wheel. K: fit of the cumulative count after the step edge with a first-order rise, 1 ms grid over τ.

## 6. Results to parameters (Appendix 19.2)

| Log | Parameter (ID) | Note |
|---|---|---|
| `G` | `whl_ks_l/r` (116, 118), `whl_kv_l/r` (117, 119) | speeds use the `odo_m_per_cnt · odo_scale` active during capture: calibrate the odometry scale first (`straight_scale.py`) or repeat G after changing it |
| `B` | `whl_ksb_l/r` (120, 121) | 1 % resolution; should be ≥ Ks of the same wheel |
| `K` | `whl_tau_m` (115) | mean of both wheels, all runs; see limits |
| `odo` | counts only | `odo_scale_l/r` (201, 202), `odo_wheelbase` (203) come from `umbmark.py`, `odo_m_per_cnt` (200) from `straight_scale.py` |
| `L` | none | report only |

Check afterwards with a closed-loop run (`D2000`): both wheels end inside the H3 gate. Defaults stay `[MEASURE]` until committed with their logs.

## 7. Known limits

- **Not yet run on hardware.** Parser and fits are verified on synthetic logs only; real-data limits (noise, stick-slip, radio drops) are unknown.
- `U` and `A` are not available: `whl_ff_u_ref` (129) and `whl_ff_norm_en` (128) cannot be measured (battery divider not wired).
- **L does not measure gear backlash.** The encoders are on the motor shaft, so lash between motor and wheel is invisible; the test reports counts and the time to the first count after a reversal (20 ms steps), which is drive-train dead time at best. `odo_backlash_l/r` (204, 205) stay at 0 and are not derived (phase 5, §7.6.3).
- **K is quantised to 20 ms** (integer counts per interval). A τ of 50 ms spans 2.5 samples: expect ±10 ms scatter, and any dead time (driver, slew, stiction) biases τ upwards. Run K several times and at two duty levels; use the mean and keep the spread.
- G speeds are the mean of the second half of a 6 s step with 3.2 mm/s per count at 50 Hz resolution; low steps (15 %) may not roll and are dropped. Fit sees only forward-rolling wheels in the air or on the floor, not the loaded vehicle.
- B ends at 34 % (`NONE`) so that it never trips the stall guard.
- Fault or `s` mid-test leaves a partial log: `check` reports it, `eval` still fits what is complete.
