# Phase 1 Calibration

Gate for `FIRMWARE_SPEC.md` §15 phase 1 and test H6 (§14.3): the odometry and feedforward parameters are measured on the floor, entered, made the compiled defaults and committed with their logs. Success criterion (§1.4): **return error on a 2 m square, both directions, < 10 mm after correction**.

Do the steps in order. Stop at the first failure and write it down. Mark each box, keep the console log of every part (`screen`/CubeIDE terminal capture) and the tool output.

**How this differs from the spec text.** Version 1.0 of the spec (§15 and H6) assumed the separate calibration firmware (`calib.c`, `calib.h`, `CALIB_README.md`, `tools/calib_host.py`). They exist in the repository since D-47 (`calib.c`/`calib.h` in `Challenge_TestHC/Core`, `docs/CALIB_README.md`, `tools/calib_host.py`), but this procedure does not depend on them. Decision D-31 (§18.2, spec version 1.1 follows this procedure in §14.3 H6 and §15): phase 1 is done with the **normal vehicle firmware and its text console**, plus the host tools in `tools/` (standard-library Python 3). The calibration firmware stays optional. Everything below is based on what `console.c`, `app_cmd.c`, `app.c` and `drv.c` do today, read on 2026-11-10; nothing here is promised from a later phase.

## Status

**Not started.** Nothing has been measured. Every table below is empty and every number in the tools' `--help` text is labelled as an example. Do not copy example numbers into a parameter.

## 1. What this procedure covers

| Parameter (ID) | How | Section | State |
|---|---|---|---|
| `odo_m_per_cnt` (200) | taped straight runs, `tools/straight_scale.py` | 4 | covered |
| `odo_scale_l`, `odo_scale_r` (201, 202) | UMBmark, `tools/umbmark.py` | 5 | covered |
| `odo_wheelbase` (203) | UMBmark, `tools/umbmark.py` | 5 | covered |
| `whl_ks_l/r`, `whl_kv_l/r` (116–119) | open-loop duty steps, steady speed from the `a` print, `tools/ffit.py steady` | 6 | covered, see limits in 6 |
| `whl_ksb_l/r` (120, 121) | open-loop duty steps from rest, `tools/ffit.py breakaway` | 6 | covered, 1 % resolution |
| `whl_tau_m` (115) | needs a step response with ≤ 20 ms resolution | – | **deferred**: the console prints every 300 ms and there is no capture (`CAP`, spec §7.25) before phase 2. Needs the calibration firmware (`K`, 50 Hz step, §19.2) or phase 2. Default 0.050 s stays `[MEASURE]`. |
| `odo_backlash_l/r` (204, 205) | reversal test | – | **deferred**: the parameters do not exist in `par_table.h` yet and are used only in phase 5 (§7.6.3). Needs phase 5: calibration `L` reports motor-side counts and latency only, real gear backlash is not observable with motor-side encoders. |
| `whl_ff_u_ref` (129), `whl_ff_norm_en` (128) | feedforward at a measured battery voltage | – | **deferred**: the battery divider is not wired (`7 bat absent`), so there is no voltage to normalise with. Keep `whl_ff_norm_en = 0`. Read the pack with a multimeter and record it (section 3). |

## 2. What the console really gives (checked in the code)

| Command | Range | Reply, units |
|---|---|---|
| `D<mm>` | 5 ≤ \|mm\| ≤ 20000 | `Move 2000 mm: Vmax 300 mm/s, T 7667 ms`, then at the end `DONE L=<mm> R=<mm> sync_max=<mm> t=<ms>`, or `TIMEOUT …`. L, R are the encoder-frame distance of this move with the current scales (0.1 mm). `Busy, send s to stop` while a move runs. |
| `W<deg>` | 1 ≤ \|deg\| ≤ 3600, **CCW positive** | `Turn 90 deg`, then `DONE …` with L negative and R positive for CCW (left wheel backwards). Wheelbase from `odo_wheelbase`, speed `drv_v_nom`, ramp `drv_t_acc` (D-24). |
| `V<mm/s>`, `T<ms>[ <ms>]` | 20..`drv_v_max`, 100..5000 | defaults `V300`, `T1000 1000`. Leave them at the defaults for all calibration runs. |
| `r` | stationary only | `Counters reset`; `Busy, send s first` while moving. |
| `p`, `a` | any | status line (below). `a` toggles a print every ≥ 250 ms. The TLM slot runs every 100 ms, so the real period is **300 ms** (derived from the code, check it once in section 3). |
| `-100…100`, `PWM <l> <r>` | integer %, ±100 | `Open loop duty 30 % 30 %`. Duty slews at `mot_slew_ol` (2 /s), 50 ms of zero on a sign change. **Dead-man 2 s:** any received byte, also a bare Enter, keeps it alive; then `DEADMAN: no input for 2 s, motors stopped`. `0` = brake. |
| `s`, `STOP` | any | `STOP`; ends in IDLE, EN low. |
| `P<id\|name>=<value>` | SI units | `P201 odo_scale_l=1.01230e+00 -`. `ERR state BENCH` (or the mode name) if a `S`-flag parameter (IDs 200–203, 128, 129) is set while the vehicle is not at rest; `ERR range <name> <lo>..<hi> <unit> (SI)` outside the range. Values are in **RAM only**: a reset restores the compiled defaults (D-7). |
| `P<id\|name>?`, `k`, `L`, `I` | any | `P201 odo_scale_l=1.00000e+00 -`; `k` = IDs < 200 as `116 whl_ks_l=9.80000e-02 -`; `L` = every parameter as `#define PAR_odo_scale_l 1.00000e+00f`; `I` = version, hash, tick, `m/cnt` in nm. Values are printed with 6 significant digits. |

Status line (`p`, `a`, `S`), exactly as printed by `con_print_status`:

```
L=31416 (1999.5 mm, 301 mm/s)  R=-12 (-0.8 mm, 0 mm/s)  BENCH OPEN
```

`L=`/`R=` are the accumulated counts (forward positive on both wheels, REQ-ENC-01). The mm value is `counts · odo_m_per_cnt · odo_scale`. The speed is the filtered estimate (resolution 3.2 mm/s per count at 50 Hz, noisy). Last two words: mode (`IDLE`, `BENCH`, `FAULT`) and phase (`OFF`, `HOLD`, `OPEN`, `MOVE`, `SETTLE`, `STOPPING`).

**Things this firmware cannot do for calibration** (reported, not added): no time stamps in any line; no capture of speed, duty or `e_x` at 50 Hz (`CAP` is phase 2); no battery voltage; no per-wheel step-response data; no backlash parameter; `W` uses the firmware's own `SETTLE` stop (see 5.1), so a turn is never exactly 90°.

## 3. Preparation

- [ ] Phase 0 is closed (`git tag phase0`, H1–H5 passed, see `PHASE0_COMMISSIONING.md`). CLAUDE.md: no phase starts before the previous gate passes. If the team decides to calibrate earlier, write that into the session record below.
- [ ] Host tests green: `make -C test` → `0 failed`.
- [ ] Tool self-tests green (from the repository root; they need nothing but Python 3):
  ```
  python3 tools/umbmark.py --selftest
  python3 tools/straight_scale.py --selftest
  python3 tools/ffit.py --selftest
  ```
  Each ends with `SELFTEST PASSED`.
- [ ] Flat, hard, clean floor, the same surface for the whole session. At least **3 m × 3 m** free for the UMBmark square (2 m side plus run-out) and a **4 m straight** runway (6 m if you do the `D5000` runs). Note the surface: the scale depends on the tyre and the floor.
- [ ] Material: steel tape measure ≥ 5 m (mm), masking tape, fine marker, a carpenter's/set square, a plumb line or second square for dropping a point from the hub to the floor, laptop with the USB console.
- [ ] Battery fully charged. Measure the pack with a multimeter now and after each part (there is no `bat` reading). Record it.
- [ ] Same firmware build for the whole session, flashed once. **Do not reset the board during a part:** `P` values are RAM only. After any reset (USB plug, button, fault, watchdog) re-enter every `P` line of this session before you continue.
- [ ] Open the console (USB, 115200 8N1) **with logging to a file**, e.g. `screen -L -Logfile tools/data/<date>/session.log /dev/cu.usbmodem11403 115200`. (Old `screen` versions, such as the one macOS ships, have no `-Logfile` and write `screenlog.0` in the current directory; CubeIDE's terminal log works too. Any capture that stores every received byte is enough. USB only: the radio drops lines.)
- [ ] Record the session header, and save the output of `I`, `k` and `L` into the log:

  | Item | Value |
  |---|---|
  | Date, people | |
  | Firmware hash (`I`, first line), Debug/Release | |
  | Floor, tyres | |
  | Pack voltage at start / after 3 / after 4 / after 5 | |
  | `P200?` `P201?` `P202?` `P203?` at start | |
  | `V`, `T` (leave at 300 / 1000 1000) | |

- [ ] Make the straight-line section start from clean scales: `P201=1`, `P202=1`. Read back with `P201?`, `P202?`. (`P203` stays at its default; the tool takes the value you give it.)
- [ ] **Signs on the floor** (a calibration with a wrong sign is worse than none): `D300` drives forward; `W90` turns **left** (CCW seen from above, left wheel backwards); `W-90` turns right. If not: stop, this is an H4 failure.
- [ ] Check the print period once: send `a`, let it run exactly 30 s (stopwatch), send `a` again. Save that stretch of the log as `period.log`, then `python3 tools/ffit.py period period.log --seconds 30`. Expected: **100 lines, 0.300 s**. If it differs, use `--period <value>` in section 6.

## 4. Straight-line scale (`odo_m_per_cnt`, ID 200)

Purpose: the mean ground distance per encoder count. The user observation of 2026-11-09 (the vehicle rolls farther than the encoder frame says) is exactly what this measures.

Method: for each run, zero the counters, drive `D2000`, read the counts, measure with the tape how far each wheel rolled. Use the **counts**, not the `DONE` mm: the vehicle stops up to 2 mm short of the target by design (`SETTLE`), and the tool divides real tape distance by real counts.

Marking each wheel (same method at start and end, both wheels): lay a strip of masking tape on the floor beside each wheel, parallel to the travel. At the start, drop a vertical line from the **hub centre** to the tape (set square or plumb line) and mark it with a tick; after the run do the same for the new hub position. The distance between the two ticks on the tape is that wheel's ground distance. The path may drift a few millimetres sideways; that is irrelevant if the tape strips are parallel to the travel within a degree.

- [ ] `P201?` and `P202?` read `1.00000e+00`.
- [ ] 5 runs of **`D2000`** (forward). Per run: place the vehicle, mark both start ticks, `r` (`Counters reset`), `D2000`, wait for `DONE`, `p` (note `L=`, `R=` counts), mark both end ticks, measure. The next run starts from wherever the vehicle stands (new start ticks). Any `TIMEOUT`, `FAULT` or visible slip: redo the run and note it.
- [ ] Optional if the room allows 6 m (H6 lists `D5000`): 3 runs of **`D5000`** in the same table.
- [ ] Write the runs into `tools/data/<date>/straight.csv` (one line per run, `tape_l_mm,tape_r_mm,cnt_l,cnt_r`, `#` comments allowed):
  ```
  # tape_l_mm,tape_r_mm,cnt_l,cnt_r
  ```
  (Use `--done` and the `DONE` values instead of counts only if you forgot to read `p`: `straight_scale.py --done --m-per-cnt 6.3638e-5 …`.)
- [ ] Run `python3 tools/straight_scale.py --csv tools/data/<date>/straight.csv | tee tools/data/<date>/straight.txt`.
  Expected: per-run table, mean `odo_m_per_cnt`, run-to-run sd, per-wheel values, a line `P200=…`.
  **Refusals (exit status 2) are findings:** fewer than 3 runs; sd above 0.3 % `[DECIDE]` (acceptance limit proposed here, not in the spec; a larger spread is slip or a bad tape reading, redo the outlier); value outside 5e-5…8e-5 m (spec §10.2).
- [ ] `k_r/k_l` printed by the tool is a cross-check of the UMBmark ratio `E_d`; **do not enter it**.
- [ ] Enter the value with the vehicle at rest: `P200=<value>`; read back `P200?`, `I` (line 4 shows m/cnt in nm).
- [ ] Check: 3 × `D2000` again, mark and measure the same way. The tape distance of each wheel must agree with that run's `DONE` value within 3 mm `[DECIDE]` (≈ 0.15 % of 2 m; limited by the tape reading itself). A systematic difference means the first fit was wrong, not the firmware.

| Run | Distance cmd | Tape L mm | Tape R mm | Counts L | Counts R | `DONE L / R` | Notes |
|---|---|---|---|---|---|---|---|
| 1 | D2000 | | | | | | |
| 2 | D2000 | | | | | | |
| 3 | D2000 | | | | | | |
| 4 | D2000 | | | | | | |
| 5 | D2000 | | | | | | |
| 6 (opt.) | D5000 | | | | | | |
| 7 (opt.) | D5000 | | | | | | |
| 8 (opt.) | D5000 | | | | | | |
| check 1…3 | D2000 | | | | | | after `P200` |

Result: `odo_m_per_cnt` = ______ m, sd ______ %, entered at ____:____.

## 5. UMBmark (`odo_scale_l`, `odo_scale_r`, `odo_wheelbase`, IDs 201–203)

Borenstein & Feng, *Measurement and correction of systematic odometry errors in mobile robots*, IEEE Trans. Robotics and Automation 12(6), 1996. The vehicle drives a 2 m square by dead reckoning, **4 × (straight 2 m, turn 90°)**, once clockwise and once counter-clockwise. The return error separates two systematic errors:

- **type A**, turns too large or too small → wheelbase (`odo_wheelbase`);
- **type B**, the straight legs curve → ratio of the two wheels' ground distance per count (`odo_scale_l/r`).

`tools/umbmark.py` derives and solves this for the firmware's axes; its header and `--selftest` show the derivation and prove the signs with a simulated vehicle that has known errors.

### 5.0 Sign conventions (read before you measure)

- **Vehicle and start frame:** origin O = midpoint between the two hub centres at the start pose. **x** points forward (the direction of the first leg), **y** to the **left** of the vehicle (the left wheel is at +y, the right wheel at −y), **CCW positive** seen from above. `W+90` = CCW = turn left.
- The frame stays fixed on the floor (the **start** frame), not on the vehicle at the end.
- **Return error** e = (x, y) = final position − start position, of the midpoint between the hub centres, in the start frame. x > 0: the vehicle ended in front of the start. y > 0: it ended to the left of the start.
- `cw` = the run with `W-90` (right turns); `ccw` = the run with `W90` (left turns).
- Plausibility before you run the tool, to first order: the errors lie on a diagonal, **CCW: y ≈ −x, CW: y ≈ +x**. If your CCW run has x and y of the same sign, one axis or one sign is probably mixed up.

| What is wrong | Typical return error |
|---|---|
| Right wheel rolls farther per count than the left (type B) | legs veer left. CCW: x > 0, y < 0. CW: x < 0, y < 0. |
| Left wheel rolls farther per count (type B) | the opposite: CCW: x < 0, y > 0. CW: x > 0, y > 0. |
| Real wheelbase smaller than `odo_wheelbase` (turns too large, type A) | CW: x > 0, y > 0. CCW: x > 0, y < 0. |
| Real wheelbase larger (turns too small, type A) | CW: x < 0, y < 0. CCW: x < 0, y > 0. |

### 5.1 One caveat that is specific to this firmware

`SETTLE` (REQ-DRV-01) brakes a wheel as soon as it is within `drv_pos_tol` (2 mm) of its target and never drives backwards. Measured on 2026-11-09: every `D300` ends 1.4–2.0 mm short. A turn is a move of ±(b/2)·θ per wheel (175.3 mm at the default wheelbase), so it ends about 1.5 mm short too, about **0.9 % of the turn angle**, in the same sense for CW and CCW. That looks exactly like a wheelbase error (type A) and would be absorbed into `odo_wheelbase`, which later phases use for continuous encoder heading, where there is no such shortfall.

The tool removes it if you tell it how far the turns really went: note the `DONE L=… R=…` line of **every** `W90`/`W-90` (table 5.4) and pass the mean of all |L| and |R| as `--turn-done-mm`. The straight legs also end short by up to 2 mm; to first order that does not change the closing error (it is the same in every leg) and is ignored. Without `--turn-done-mm` the result is still valid for "this firmware, these commands", and the verification run will pass, but `odo_wheelbase` is then an effective value that includes the shortfall. Say which you did.

### 5.2 Set-up on the floor

- [ ] Stick a long strip of tape along the floor as **T_x**, the x axis, from about 0.5 m behind O to 2.5 m in front of it, and a perpendicular strip **T_y** at O (use the square). Fasten the tape measure along T_x with its 0 at O.
- [ ] Measure the distance between the two hub centres (track) with the ruler. Mark two start ticks on T_y at ±(track/2) from O. Track measured: ______ mm.
- [ ] The start pose: both hub centres vertically above the two ticks (drop a plumb line), the vehicle square to T_y. This is the pose for every run. In IDLE (EN low) the wheels turn freely, so the vehicle can be lifted or pushed back by hand between runs.
- [ ] Write `P201?`, `P202?`, `P203?` (current values, needed by the tool): ______ / ______ / ______. For the first iteration these are `1.0`, `1.0` and the default `0.22325` (after section 4 `P200` is already updated, that is intended: the mean scale is then right and the tool assumes so).

### 5.3 Runs

5 runs CW and 5 runs CCW, **alternating** (CW1, CCW1, CW2, …) so that drift in battery or tyres averages out. Per run, with the vehicle in the start pose and at rest:

```
CCW:  D2000   W90    D2000   W90    D2000   W90    D2000   W90
CW:   D2000   W-90   D2000   W-90   D2000   W-90   D2000   W-90
```

- [ ] Send one command, **wait for its `DONE`**, then the next (a command sent while a move runs answers `Busy, send s to stop`). Keep your finger ready to send `s`. Copy the 4 turn `DONE` lines into table 5.4.
- [ ] After the last turn the vehicle is back near O. **Do not touch it.** Drop the hub centres to the floor and mark the two end points P_L (left wheel) and P_R (right wheel). Measure each in the start frame: x = reading on the tape at the foot of the perpendicular from the point onto T_x (set square), y = perpendicular distance from T_x to the point, **positive to the left of T_x when looking along +x**. Enter the four numbers in table 5.5 (mm, signed).
- [ ] Any `TIMEOUT`, `FAULT`, a wheel that visibly slipped, or a start pose that was not exact: redo that run, note why. Do not throw away a run just because it is far from the others.
- [ ] Per run time: about 1 minute driving plus the measurement. 10 runs ≈ 30–40 minutes; check the pack.

### 5.4 Turn `DONE` lines (for `--turn-done-mm`)

Copy `L` and `R` of each `W` leg. Expected for the default wheelbase: |L|, |R| a little below 175.3 mm (the commanded arc is `odo_wheelbase/2 · π/2`; for another `P203` it is different, the tool recomputes it).

| Run | Dir | Turn 1 L / R | Turn 2 L / R | Turn 3 L / R | Turn 4 L / R |
|---|---|---|---|---|---|
| 1 | | | | | |
| … | | | | | |
| 10 | | | | | |

Mean of all |L| and |R|: ______ mm → `--turn-done-mm`.

### 5.5 Return errors (before correction)

| Run | Dir | x_L mm | y_L mm | x_R mm | y_R mm | x_mid = (x_L+x_R)/2 | y_mid = (y_L+y_R)/2 |
|---|---|---|---|---|---|---|---|
| 1 | cw | | | | | | |
| 2 | ccw | | | | | | |
| 3 | cw | | | | | | |
| 4 | ccw | | | | | | |
| 5 | cw | | | | | | |
| 6 | ccw | | | | | | |
| 7 | cw | | | | | | |
| 8 | ccw | | | | | | |
| 9 | cw | | | | | | |
| 10 | ccw | | | | | | |

### 5.6 Evaluate and enter

- [ ] Write `tools/data/<date>/umb_before.csv`, one line per run, direction first (`cw` or `ccw`), the four wheel-point numbers, `#` comments allowed:
  ```
  # dir,xl_mm,yl_mm,xr_mm,yr_mm   (start frame, mm; x forward, y left)
  ```
  (If you only have the midpoint, use `dir,x_mm,y_mm`; you then lose the heading check.)
- [ ] Run (substitute your numbers for `P203`, `P201`, `P202` as read in 5.2, and the turn mean from 5.4):
  ```
  python3 tools/umbmark.py --wheelbase 0.22325 --scale-l 1.0 --scale-r 1.0 \
      --turn-done-mm <mean> --csv tools/data/<date>/umb_before.csv | tee tools/data/<date>/umb_before.txt
  ```
  Expected: table of the centres of gravity (x_cg, y_cg, |r_cg|, scatter) per direction; E_max,syst; first-order type A/B values; the exact E_b, E_d; the fit residual; then three console lines `P203=…`, `P201=…`, `P202=…`.
  Checks the tool does for you: the exact fit residual must stay below max(3 mm, 2 × the run scatter) `[DECIDE]`; at least one centre-of-gravity coordinate must be more than 2 standard errors from zero. If a check fails it **refuses** (exit status 2, no `P` lines): that means wrong signs or axes, a start pose that moved, a slipping wheel, or an error that is already below the noise. Look at the numbers before anything else.
  Also refused: results outside the parameter ranges of §10.2 (scales 0.95…1.05, wheelbase 0.15…0.35 m).
- [ ] Enter the three lines, with the vehicle at rest. Read back `P201?`, `P202?`, `P203?`. The tool keeps the mean of the two scales, so `odo_m_per_cnt` stays valid.
- [ ] The tool says **"Repeat the 10 runs with the new values"**: that is section 7.

Result: E_b = ______, E_d = ______, `P201` = ______, `P202` = ______, `P203` = ______, E_max,syst before = ______ mm.

## 6. Feedforward (`whl_ks_*`, `whl_kv_*`, `whl_ksb_*`, IDs 116–121)

Model (REQ-WHL-01): while the wheel rolls `|duty| = Ks + Kv·|v|`; from rest the first term is `Ks_break`. Open-loop duty commands plus the `a` print are enough for this, with these limits:

- No time stamps. `tools/ffit.py` takes the speed from the **difference of the printed positions over the steady window** divided by the print period (300 ms, section 3). That resolves well below 1 mm/s; the printed `mm/s` is only used as a cross-check (lost lines show up as a disagreement).
- Duty has 1 % resolution. `Ks_break` is therefore found to ±1 %.
- Only steady states are identified. The dynamic part (`Kv·τ_m·a_ref`, `whl_tau_m`) is deferred (above).
- The values depend on the battery voltage (no `bat` reading) and the floor. Note the pack voltage; use a charged pack.
- **Validation is weak.** The closed loop (PI + position loop) hides small feedforward errors, and the console cannot log `e_x`. Judge the fit by its statistics (R², scatter between the directions) and by H3-style `D` runs, not by "the vehicle looks better".

Do section 6 **after** 4 and 5: speed in m/s uses `odo_m_per_cnt`.

### 6.1 Steady-state steps

- [ ] `a` (reply `Auto print ON`; if it says OFF send `a` again). Use a separate log file for this part if you can (`screen`: Ctrl-A H toggles logging), e.g. `ff_steady.log`; one file for sections 6.1 and 6.2 also works, each sub-command ignores the steps it cannot use.
- [ ] The vehicle on the floor with at least 3 m free ahead and behind. For each duty in **20, 25, 30, 40, 50, 60 %**, forward **and** reverse (alternate `+`/`−` so the vehicle stays in the area): send the duty (e.g. `30`), **press Enter about once a second** (dead-man) for **6 s**, then `s`, wait 2 s until the lines show `HOLD`/`IDLE`. At 60 % the vehicle reaches about 0.4 m/s; be ready to send `s`.
  Order: `20`, `-20`, `25`, `-25`, … `60`, `-60`. 12 steps, about 3 minutes.
- [ ] Do not type anything else between the steps except Enter and `s` (every other command ends the open-loop step in the log).
- [ ] `python3 tools/ffit.py steady ff_steady.log | tee ff_steady.txt`.
  Expected: a table of the 12 steps with the speed of each wheel and a flag (`ok`, or why a step was not used: `short`, `slow`, `unsteady`, `disagree`, `sign`), then per wheel the pooled fit, the forward-only and reverse-only fits, and `P116=… P117=… P118=… P119=…`.
  **Refusals:** fewer than 4 usable points per wheel or a speed span below 0.15 m/s (the intercept would be extrapolated), results outside Ks 0…0.5, Kv 0.5…3 (§10.1).
- [ ] Plausibility: R² close to 1; forward and reverse Kv within about 5 %; se(Ks) below 0.02. For comparison the 9 Oct values were L 0.098 / 1.275, R 0.041 / 1.361 (in the air; on the floor Ks is expected to be larger).
- [ ] Enter `P116`…`P119` (live parameters, allowed while moving; do it at rest anyway). Read back with `k`.

### 6.2 Breakaway (`whl_ksb_l/r`)

- [ ] Log file `ff_breakaway.log` (or the same file), `a` on. Coarse pass first: from rest (after `s`, wait 2 s), `8`, hold 2 s (Enter once), `s`, wait 2 s; then 10, 12, … 24 %. Look at the counts: the duty between the last step where a wheel did not move and the first where it did is the bracket. Then the **fine pass**: every 1 % over the bracket (about 6 values), each **twice**, forward and then reverse (the vehicle moves only a few centimetres per step). Each trial **must start from rest**.
- [ ] `python3 tools/ffit.py breakaway ff_breakaway.log | tee ff_breakaway.txt`.
  Expected per wheel and direction: the tested duties as `M` (moved, at least 10 counts) or `.` (stayed), the lowest duty at which it **always** moved, a note if the picture is not monotonic, and `P120=…`, `P121=…` (mean of both directions). Refuses when no tested duty moved every time or the value is outside 0…0.6.
- [ ] Ks_break should be at or above Ks of the same wheel; if it is below, the steps were not from rest or the fit is wrong.
- [ ] Enter `P120`, `P121`; read back with `k`.

### 6.3 Check

- [ ] 3 × `D2000` and `D300`: ends within 2 mm per wheel, no `TIMEOUT`, no `FAULT`, `sync_max` not larger than before the change (H3 data: 2.4–3.4 mm for `D300`). If the closed loop gets worse, put the old values back (`P116`…`P121` with the values read in section 3) and report it as a finding.

| Step | Duty % | v_L m/s | v_R m/s | Flag L / R |
|---|---|---|---|---|
| … | | | | |

| Wheel | Ks | Kv | Ksb fwd / rev | Ksb entered |
|---|---|---|---|---|
| L | | | | |
| R | | | | |

## 7. Verification (H6)

Same set-up, same commands, same V and T as section 5, with the corrected parameters in RAM (re-enter them if the board was reset).

- [ ] 5 runs CW and 5 runs CCW alternating, marking and measuring exactly as in 5.3, into `tools/data/<date>/umb_after.csv`. Also copy the turn `DONE` lines.
- [ ] `python3 tools/umbmark.py --verify --csv tools/data/<date>/umb_after.csv | tee tools/data/<date>/umb_after.txt`.
  Expected: the centres of gravity and the single-run |e|, then `PASS`/`FAIL` against 10 mm (exit status 0 / 3).
- [ ] **Pass (H6):** every single run of both directions below **10 mm**. The spec says only "return error < 10 mm"; the tool prints the per-run and the centre-of-gravity verdict; the strict per-run reading is the pass criterion here `[DECIDE]`.
- [ ] If it fails: one more iteration. Run `umbmark.py` on the **after** data with the **currently active** `--wheelbase`, `--scale-l`, `--scale-r` (the tool then returns the incremental correction), enter, and verify again (10 more runs). Two iterations at most; if it still fails it is a finding (look for slip, a loose wheel, a floor that is not flat, or errors that depend on speed).
- [ ] 3 × `D5000` (room permitting): both wheels end within 2 mm of the target (as in H3), no reversal, no `TIMEOUT`; the tape distance of each wheel within 0.2 % `[DECIDE]` of the `DONE` value.
- [ ] Record the final pack voltage.

| Run | Dir | x_mid mm | y_mid mm | \|e\| mm |
|---|---|---|---|---|
| 1…10 | | | | |

Gate: max single-run \|e\| = ______ mm (< 10) at iteration ____. Max centre of gravity = ______ mm.

## 8. Compiled defaults and commit

The `P` values are RAM only. The gate is passed only when they are the **compiled defaults**.

- [ ] Edit the default column of these rows in `Challenge_TestHC/Core/Inc/par_table.h` (nothing else on the rows: IDs, ranges and flags stay; keep the `f` suffix and the 5–6 significant digits the tools print): 200, 201, 202, 203, and 116–121 if section 6 was done.
- [ ] `make -C test`. **Expect failures, checked on a scratch copy with changed defaults:** changing 116–121 breaks 9 checks in `t_whl_law` (they hard-code the 9 Oct defaults, `0.098 + 1.275·v` …); an `odo_scale_l/r` default other than 1.0 breaks the `D1000`/`D5000` checks in `t_drv_move` because `test/sim_plant.c` converts with `odo_m_per_cnt` only and ignores the scales. `odo_m_per_cnt` and `odo_wheelbase` alone broke nothing. Fix the tests in the same commit (set the parameters explicitly with `par_set` in the test, or let `sim_plant` apply the scales); do not leave a failing test and do not revert the calibrated defaults.
- [ ] Rebuild, flash, **reset**, then `L`: the lines `#define PAR_odo_m_per_cnt …`, `PAR_odo_scale_l`, `PAR_odo_scale_r`, `PAR_odo_wheelbase` (and the feedforward lines) equal the values you entered. Save this `L` dump.
- [ ] Repeat once the check of section 7 from the freshly flashed defaults: 2 runs CW and 2 runs CCW, all < 10 mm; and `D300`, `D1000`, `D5000` once each within the H3 gate.
- [ ] Update spec §10 (default column and source `M` for the changed rows) and Appendix 19.2 where it names these constants; remove `[MEASURE]` only for what was measured. Anything deferred above keeps its tag. Record deviations in §18.2.
- [ ] Save under `tools/data/<date>/` (the date of the session): `session.log`, `period.log`, `straight.csv`, `straight.txt`, `umb_before.csv`, `umb_before.txt`, `umb_after.csv`, `umb_after.txt`, `ff_steady.log`, `ff_steady.txt`, `ff_breakaway.log`, `ff_breakaway.txt`, the final `L` dump (`params_after.txt`), photos of the floor marks, and a `README.md` naming: firmware hash and build (`I`), the `L` dump **before** the session, floor, tyres, pack voltage at each part, who measured, the iteration count, anything refused, and which steps were skipped.
- [ ] Commit (REQ IDs in the message, CLAUDE.md): `Phase 1: calibration defaults from floor measurements, H6 passed (REQ-WHL-03, REQ-DRV-08, spec 7.6.2, REQ-PAR-02)`; includes `par_table.h`, the test change, the spec update, `tools/umbmark.py`, `tools/straight_scale.py`, `tools/ffit.py`, `tools/data/<date>/` and this file with the tables filled in.
- [ ] `git tag phase1`.

## 9. Deferred, and why

| Item | Why it is not done here | What it needs |
|---|---|---|
| `whl_tau_m` | no 50 Hz data on the console, 300 ms print period | calibration firmware `K` (50 Hz step), or `CAP` (phase 2) |
| `odo_backlash_l/r` | IDs 204/205 not in `par_table.h`; consumed only in phase 5 | `L` shows motor-side counts and latency only (no gear backlash); reversal test with `CAP` once the parameters exist |
| `whl_ff_u_ref`, `whl_ff_norm_en = 1` | no battery divider, no voltage reading | divider wired and `bat` OK, then the same fit at two known voltages |
| Spec 1.0 H6 `A`, `L`, `G` | named calibration-firmware commands; `CALIB_README.md` exists (D-47); `A` and `U` answer `ERR not available` (battery divider not wired) | covered here: `G` (feedforward) by 5 and the odometry sequence by 3–4; `L` deferred as above; `A` not available |
| Calibration firmware (`APP_CALIB=1`) | `calib.c`/`calib.h` exist (D-47), unchanged-by-review rule as for any file | optional second route (`CALIB_README.md`, `tools/calib_host.py`); its results map onto the same parameters (§19.2) |
| Closed-loop feedforward validation | the console shows no tracking error | `CAP` (phase 2) |

## 10. Problems seen on the floor

| Symptom | Cause | Action |
|---|---|---|
| `ERR state BENCH` on a `P20x=` | the vehicle is not at rest yet | wait for `STOP`/IDLE lines to stop, send again |
| `Busy, send s to stop` | a move is still running | wait for `DONE`, or `s` |
| Everything reverts after a while | the board reset (power, fault, watchdog) | re-enter all `P` lines of the session; check `I` reset cause |
| `DEADMAN` in the middle of a feedforward step | no byte received for 2 s | press Enter every second; that step is short and is not used |
| `umbmark.py` refuses: residual / diagonal | wrong sign or axis, start pose moved, slip | compare the data with the table in 5.0; redo the odd runs |
| `umbmark.py` refuses: within noise | the odometry is already better than your measuring | do not enter anything; run 6 instead |
| `straight_scale.py` refuses on spread | slip, a dragged cable, a tape misread | find the outlier row, redo that run |
| `ffit.py steady`: many `disagree` | status lines were lost, or the period is not 0.300 s | check the capture (USB, no radio), redo `period` in section 3 |

## 11. Sign-off

- [ ] Section 3 done, session header filled.
- [ ] 4: `odo_m_per_cnt` measured and checked.
- [ ] 5: UMBmark corrections entered.
- [ ] 6: feedforward done or explicitly skipped with a reason (write it in the README).
- [ ] 7: H6 passed, or the finding written down.
- [ ] 8: defaults compiled, host tests green, `L` dump equals the entered values, logs committed, `git tag phase1`.
- [ ] Deferred items (section 9) copied into the open-findings list of the next phase.
