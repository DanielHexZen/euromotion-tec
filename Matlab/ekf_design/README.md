# EKF design and verification (MATLAB / Simulink)

MATLAB reference, simulation and acceptance tests for the 5-state EKF of
`FIRMWARE_SPEC.md` §7.17 (`ekf.c`, not written yet). The code in `core/` is
meant to be ported line by line to C and afterwards to serve as the reference
that `ekf.c` is checked against (same inputs, same outputs).

All parameter values come from `ekf_params.m`, which copies names and
defaults from spec §10. Values tagged `[MEASURE]`, `[VERIFY]` or `[DECIDE]`
in the spec are placeholders. Nothing in this folder has been measured on the
vehicle; every number below is from simulation.

## How to run

```matlab
cd Matlab/ekf_design
S = run_all;                  % everything: design, verification, Simulink build + comparison
S = run_all('quick', true);   % shorter run (MC N = 3, 1e5-step long test, one seed)
S = run_all('mc_N', 20, 'simulink', false);
```

`run_all` adds the subfolders to the path, prints one pass/fail table and
writes `results/summary.txt`, `results/summary.mat` and (where figures can be
rendered) PNG figures into `results/` (ignored by Git). Single parts can be
run alone, e.g. `verify_line`, `verify_rate_50_200(1:3)`,
`verify_monte_carlo(10)`, `design_observability`.

Long stages can be split over processes: `verify_monte_carlo(1:5, 'rect10', 'tag', 'x')`
and `verify_monte_carlo(6:10, 'rect10', 'tag', 'x')` cache each run in
`results/mc_cache/`; `verify_monte_carlo(10, 'rect10', 'tag', 'x')` then pools them.
Delete the cache after any code or parameter change (the tag is not checked).

Requirements: base MATLAB and Simulink only. No Statistics, Control System,
Navigation or Sensor Fusion Toolbox functions are used (chi-square quantiles
come from `chi2_quantile.m`, built on base-MATLAB `gammainc`; the random
numbers come from an own portable generator, `sim_rand.m`). The script part
also runs in GNU Octave 8.

**Minimum release targeted: MATLAB/Simulink R2020b.** The model builder uses
`new_system`, `add_block`, `add_line`, `set_param`, atomic subsystems with
`SystemSampleTime`, MATLAB Function blocks whose code is set through
`sfroot` / `Stateflow.EMChart.Script` (no Stateflow licence needed) and
`Simulink.SimulationOutput`.

## What was executed and what was not

| Part | Status |
|---|---|
| `core/`, `sim/`, `design/`, `verify/` | executed in **GNU Octave 8.4** (Linux), single precision filter, in two rounds: round 1 with the 960 Hz IMU rate of the LSM6DSV (quick settings), round 2 with the **833 Hz** rate of the LSM6DS3. Round 2 ran, each as its own Octave process: the sensitivity sweep, the 50/200 Hz comparison (seed 1), the Monte Carlo with N = 10 for the default model **and** for the literal spec scale-factor model (same seeds), the slip study, the in-place-turn study, the Jacobian / static (1e5 steps) / gate / back-shift / float32 / `t_ekf_line` checks. **Not repeated at 833 Hz:** `design_observability`, `design_gates` (their numbers below are from round 1); the 1e6-step static test and seed 2 of the rate comparison were never run |
| `run_all.m` as one call | **not executed in round 2** (it would run for hours in Octave and the container restarts); the steps were called one by one with the same arguments. The two new steps (`verify_slip`, `verify_turn_nees`) are in `run_all` but that file itself was only parsed, not run end to end |
| figures | **produced** in headless Octave after a fix of `can_plot` (Octave's font directory is missing in the Debian package; `can_plot` copies the system FreeSans fonts to `results/octave_fonts` and sets `OCTAVE_FONTS_DIR`; the gnuplot toolkit prints the PNGs). In `results/` (git-ignored): `allan_demo.png`, `line40_{path,errors,nees,nis}.png`, `rect10_{path,errors,nees,nis}.png`. The plain look is gnuplot's; MATLAB will render them better |
| `simulink/build_ekf_model.m`, `simulink/run_ekf_model.m` | **never executed** (no MATLAB/Simulink available). Written against documented APIs; expect small fixes on the first run in MATLAB. The change to 833 Hz made the base step 1/1666000 s (it was 1/48000 s), i.e. 35 times more base steps per simulated second; untested |
| MATLAB Function block code generation | **not checked**. The step functions were written for `%#codegen` (fixed sizes, no cell arrays, no dynamic fields, type-stable structs) but no code generator has seen them |

## Files

### `core/` — the filter (port these to C)

| File | Spec | C function |
|---|---|---|
| `ekf_params.m` | §10 | `par_table.h` defaults (plus `mx_*` MATLAB-only switches: `mx_still_init`, `mx_still_F_bg`, `mx_hist_corr`, `mx_sf_mode`, `mx_sf_corr`, `mx_slip_abs`, `mx_freeze_s`) |
| `ekf_param_cast.m` | — | the single/double switch (firmware: float) |
| `ekf_init.m` | §7.17.1, §7.17.5 | `ekf_init` |
| `ekf_reset_pose.m` | §7.17.1 | `ekf_reset_pose` |
| `ekf_still_update.m` | §7.17.4, REQ-EKF-09 | standstill flag + ZUPT accumulator (inside `ekf_predict` or `app_est`) |
| `ekf_predict.m` | §7.17.2 | `ekf_predict` (dispatch on `dpsi = NaN`, history push) |
| `ekf_predict_imu.m`, `ekf_model_imu.m` | §7.17.2, REQ-EKF-03 | IMU-mode prediction, F, Q |
| `ekf_predict_enc.m`, `ekf_model_enc.m` | §7.17.7, REQ-EKF-10 | encoder-heading mode |
| `ekf_slip_detect.m` | REQ-EKF-04 | slip detector (count `ekf_slip_n`; see F-10). The inflation of the distance noise while `slip` is in `ekf_predict_imu.m` (D-45, F-11) |
| `ekf_update_scalar.m`, `ekf_update_2d.m` | REQ-EKF-05 | common gate / Joseph / health / NIS procedure |
| `ekf_health.m` | REQ-EKF-05 step 5 | health check → `F_EKF` |
| `ekf_update_zupt.m` | §7.17.3 | `ekf_update_zupt` |
| `ekf_update_edge.m`, `ekf_meas_edge.m` | §7.17.3 | `ekf_update_edge` |
| `ekf_update_face_range.m`, `ekf_meas_face.m` | §7.17.3, REQ-EKF-06 | `ekf_update_face_range` |
| `ekf_update_heading.m` | §7.17.3 | `ekf_update_heading` |
| `ekf_update_point.m`, `ekf_meas_point.m` | §7.17.3 | `ekf_update_point` |
| `ekf_hist_push.m`, `ekf_back_at.m`, `ekf_pose_at.m` | §7.17.1, REQ-EKF-02 | pose history, back-shift, `ekf_pose_at` |
| `ekf_get.m` | §7.17.6 | `ekf_get` (flattened `ekf_out_t`) |
| `ekf_wrap.m` | — | `wrap_pi` |
| `imu_yaw_init.m`, `imu_yaw_push.m`, `imu_yaw_integral_at.m` | §7.11.3 | yaw-integral ring in `imu.c`, `imu_yaw_integral_at` |
| `map_line_frame.m`, `map_boundary.m` | §7.18.2 | `map_seg_frame` (LINE), `map_boundary` |
| `lat_init.m`, `lat_step.m` | §7.15 | simplified `lat` (face fit as specified, post notch simplified) |

Port notes: state order `[x y θ b_g s]`; every operation is written in the
order the C code should use (`PHt = P*H'`, `S = H*PHt + R`, `K = PHt/S`,
Joseph form, then `P = (P+P')/2`). Timestamps are integer microseconds
(`uint32_t` in C, wrap-safe differences, `dt = (float)(t1 - t0)*1e-6f`).
`e.rec`/`e.n_rec` are MATLAB-only logging.

### `sim/` — truth, sensors, scheduler

| File | Content |
|---|---|
| `sim_config.m` | scenario, truth model and run options (rate, precision, truth mode, ground-sensor mode) |
| `sim_course.m` | `rect10` (§7.18.5), `rect10_lm` (YAML landmarks), `rect10_all`, `line40` (t_ekf_line), `maneuver` (open-loop arcs and spins) |
| `sim_plant_*.m` | 1 kHz differential drive: first-order closed speed loops, wheel-diameter mismatch, odometry scale, effective wheelbase ≠ `odo_wheelbase`, slip episodes |
| `sim_enc_step.m` | 1 kHz snapshots, `floor(distance/odo_m_per_cnt)` |
| `sim_imu_*.m` | DRDY samples at `imu_odr_hz` = 833 Hz (LSM6DS3, `[VERIFY]`): exact true rate inside the tick, constant bias + random walk + ramp, white noise, scale factor, 17.5 mdps LSB |
| `sim_yaw_*.m` | `imu_drain`: yaw-integral ring at 1 kHz (pack/unpack = the signal between blocks) |
| `sim_gnd_*.m` | G1–G4: 50 Hz ranging, staggered, 5 ms polling, REQ-VL-03 timestamps, spot noise, ragged true boundary, debounce, edge events; or periodic synthetic events (t_ekf_line), optional false events |
| `sim_lat_*.m` | S1/S2: 50 Hz, EXTI timestamps, ray-cast against true faces and posts (survey error) |
| `sim_est_*.m` | `app_est`: odometry increment, `imu_yaw_integral_at` at the encoder time, standstill, predict, ZUPT, edge events (REQ-EKF-07 skips, back-shift), lateral updates |
| `sim_gdn_*.m` | mission (INIT, DRIVE, FINAL, HOLD, PRETURN, TURN, FINISHED), `nav_follow`, encoder-frame final stop, `nav_turn`, `drv_vel` limits |
| `sim_run.m` | the multirate scheduler (identical order to the Simulink model) |
| `sim_rng_init.m`, `sim_rand.m`, `sim_randn.m` | portable deterministic PRNG (L'Ecuyer 1988 combined LCG, Box–Muller) |

Simplifications: no wheel PI/position cascade (the closed speed loop is a
first-order lag, τ = 1/6.8 s); the final stop is a √(2ad) speed profile on the
encoder distance; level ground (roll = pitch = 0, `att` not simulated); no
obstacles, evasion or forward sensors; the post notch and the face windows
use the EKF along-segment position; ARC segments are not used by the
closed-loop courses (the arc case is in the open-loop `maneuver` course).

### `design/`

| File | Content |
|---|---|
| `design_noise_models.m` | how each `ekf_*` noise parameter maps to a physical quantity and how to measure it; consequences of the defaults; Allan demo on simulated data |
| `allan_gyro.m` | overlapping Allan deviation; fit `N²/τ + K²τ/3` → `ekf_gyro_nd = N`, `ekf_q_bg = K²` |
| `design_observability.m` | covariance/observability analysis per scenario with the filter's own code; s estimated vs frozen |
| `design_sensitivity.m` | sweeps of `ekf_kd`, `ekf_gyro_nd`, `ekf_sig_edge`, `ekf_q_bg` with fixed truth |
| `design_gates.m` | false-rejection and outlier-rejection probabilities; t_ekf_line with 10 % false edge events, gate vs no gate |
| `chi2_cdf.m`, `chi2_quantile.m` | chi-square without toolboxes |
| `save_fig.m`, `can_plot.m` | figure helpers (`can_plot` sets up the fonts for headless Octave) |

### `verify/`

| File | Test |
|---|---|
| `verify_jacobians.m` | `t_ekf_jacobians`: F (IMU moving/still, encoder) and every H against central differences, 1e-3 rel |
| `verify_static.m` | `t_ekf_static` (60 s, bias 0.5 °/s, real yaw ring) + symmetry/PD after 1e6 float32 steps |
| `verify_gate.m` | `t_ekf_gate`, chi-square table check, 2-D sequential = batch |
| `verify_backshift.m` | history interpolation at 20 ms spacing; corrections must not leak into the back-shift |
| `verify_line.m` | `t_ekf_line` |
| `verify_float32.m` | float32 vs float64 on line40 and rect10; float32 yaw integral at large Ψ |
| `verify_rate_50_200.m` | D-30: 50 Hz vs 200 Hz |
| `verify_slip.m` | REQ-EKF-04 at 50 Hz against 200 Hz: slip events (delay, error before/after detection, covariance added, also by direction for the in-place events) and false detections without slip; counts 1, 2, 3; adopted slip inflation (D-45) against the form of spec version ≤ 1.4 |
| `verify_turn_nees.m` | cause of the overconfidence in the in-place turns: truth variants and the three scale-factor Q models (`mx_sf_mode` 0/1/2) |
| `verify_monte_carlo.m` | NEES/NIS, closure and waypoint targets over N runs; waypoint error by direction (along/cross-track, scale state, edge events per leg); `'tag'` caches each run in `results/mc_cache/` so the seeds can be split over processes |
| `sim_metrics.m`, `plot_run.m`, `vres.m` | metrics, figures, result rows |

### `simulink/`

| File | Content |
|---|---|
| `build_ekf_model.m` | builds `ekf_mr2006b.slx`: Plant (1 kHz), Sensors/{Encoders 1 kHz, IMU 833 Hz, Ground 1 kHz, Lateral 1 kHz}, Estimator/{YawIntegral 1 kHz, EKF 50 Hz offset 1 ms}, Guidance (50 Hz offset 1 ms), CmdDelay (1 kHz unit delay), StatusDelay (50 Hz unit delay); fixed-step discrete 1/1666000 s, single-tasking, algebraic loop = error; logs `sl_est`, `sl_upd`, `sl_gdn`, `sl_plant` |
| `run_ekf_model.m` | builds, simulates, runs `sim_run` with the same configuration and compares sample by sample |

Every MATLAB Function block contains a persistent state and one call of the
same `sim_*_step` function that `sim_run.m` calls, so the filter exists once.
The base step is `1/(2·lcm(1000, imu_odr_hz))` = 1/1666000 s for 833 Hz, which
makes 1 kHz (1666 steps), 833 Hz (2000 steps, offset 1 step) and 50 Hz
(33320 steps, offset 1666 steps) exact, and the IMU samples never coincide
with a 1 kHz tick (odd against even step numbers), so the execution order is unambiguous and
equal to `sim_run`'s. The command and status feedbacks pass through unit
delays (as in the firmware, where `mot_apply` acts after the slot and
`app_est` sees the drive status of the previous control tick), so there is no
algebraic loop.

## Mapping onto the firmware under D-30

D-30 moves the EKF from the 200 Hz `S_EST` slot into `S_CTRL` (50 Hz, phase
1), immediately before `gnd_step/fwd_step → msn_step → drv_step`:

```
S_FAST 1 kHz : enc_update, imu_drain (yaw ring, 833 Hz samples), vl_service -> gnd/lat queues
S_CTRL 50 Hz : app_est  = sim_est_step : odo from the encoder snapshot of this tick,
                                          imu_yaw_integral_at(t_enc), still, predict, ZUPT,
                                          queued edge / face / heading / post updates
               msn_step -> nav -> drv  = sim_gdn_step
```

The pose history then holds one entry per 20 ms; 64 entries already cover the
1.0 s age limit (the spec's 256 entries = 5.12 s). The yaw ring (32 samples at
833 Hz) spans 37.2 ms, more than one 20 ms period; `app_est` only reads it at
the encoder timestamp of the current tick.

## Spec findings

Each was checked numerically; the code implements the corrected form and the
literal form stays selectable through an `mx_*` switch for comparison.

| ID | Finding | Evidence (Octave run) | Switch for the literal spec form |
|---|---|---|---|
| F-1 | §7.17.2 lists `F(3,4) = -dt` unconditionally. While `still` the heading does not depend on `b_g`, so the entry must be 0; otherwise heading variance grows at standstill and every ZUPT moves the frozen heading (contradicts D-12). | 60 s static: heading drift 0.000 deg as designed, 0.450 deg with the literal matrix | `mx_still_F_bg = 1` |
| F-2 | With the literal start the first 0.2 s count as motion although the vehicle stands. Starting with the standstill timer full avoids it. | static: σ(b_g) after 3 s 0.0035 deg/s (design) against 0.0043 deg/s, drift 0.0004 deg | `mx_still_init = 0` |
| F-3 | `Q_tt += (sf·dθ)²` per step treats a constant gyro scale error as white noise: the variance depends on the step rate and is about ten times too small in σ for one 90° turn (0.027 deg at 50 Hz, 0.014 deg at 200 Hz, true 0.27 deg). Proposed: per-radian model `Q_tt += sf²·c·|dθ|`, rate-independent. The correlation angle `c` is a `[DECIDE]` (2π used). | `P_tt` after the turns identical at 50 and 200 Hz (0.00219 rad²) with the proposed model | `mx_sf_mode = 0` |
| F-6 | The back-shift of a delayed measurement must contain motion only. With raw poses in the history, every correction applied after the measurement time leaks into the back-shift. Proposed: store pose minus accumulated corrections (3 floats, 3 additions per update). | leak 0 m with the correction, 19.9 mm without | `mx_hist_corr = 0` |
| F-7 | The yaw integral Ψ grows without bound; in float32 its resolution degrades. Re-base the ring or keep Ψ in integer/double in the C code. | ZUPT bias error −1.9e-5 rad/s at Ψ = 0, −9.4e-5 at 100 rad, −8.7e-3 at 1000 rad (float32); unchanged in double | none |
| F-8 | Accumulating `dt` in float for the standstill timer is precision-dependent (50 × 0.02f < 1.0f), which shifts the ZUPT by one step between float32 and float64. Accumulate integer microseconds. | by construction | none |
| F-9 | The per-radian scale-factor model of F-3 (`c` = 2π) is exact after one revolution, conservative below it (σ twice the true value at 90°) and **overconfident beyond it**: a constant scale error gives the heading error `sf·Θ`, the model gives `sf·√(c·Θ)`, so the ratio is `√(Θ/c)` for net turning Θ in one direction. This is the whole cause of the in-place-turn NEES figure of the 50/200 Hz comparison (the `maneuver` course turns 12π the same way). `rect10` turns 3π/2 in total, so the default is conservative there. Alternative `mx_sf_mode = 2`: `Q_θθ += sf²·max(0, (Θ+dθ)² − Θ²)` with Θ = net angle since init/pose reset (one more float); exact for any number of turns in one direction, conservative after reversals and after heading corrections (it grows with every lap). Not made the default; `[DECIDE]` together with `ekf_sf_corr`. A gyro scale-factor state would be the exact treatment. | seed 1 (`sf` = −0.0041 = −1.4 σ): heading error −142 mrad against σ 47 mrad (ratio 3.0), NEES ≤ upper bound in 0.82 of the turn steps; plant `sf` = 0 → 1.00; plant wheelbase = `odo_wheelbase` → 0.81; no wheel mismatch → 0.81; `mx_sf_corr` = 12π → 1.00 (ratio 1.26); mode 2 → 1.00 (ratio 1.27); literal spec form → 0.00 (ratio 14.7). Seed 2 (`sf` = −0.27 σ): 1.00 in all but the literal form (0.18) | `mx_sf_mode` = 1 (default) / 2 / 0 |
| F-10 | Slip detector count at 50 Hz. "3 consecutive steps" is 41–61 ms after the onset at 50 Hz (15 ms at 200 Hz). The noise margin went the other way: one encoder count of wheel difference is 0.014 rad/s per 50 Hz step against 0.057 rad/s per 200 Hz step, and without slip the largest residual of the whole manoeuvre course is 0.055 rad/s at 50 Hz against 0.142 rad/s at 200 Hz (threshold 0.15). A count of 2 at 50 Hz (≤ 41 ms) therefore has more margin than 3 at 200 Hz had. Proposed: `ekf_slip_n` = 2 at the 50 Hz step, or a time `ekf_slip_T` = 0.04 s with n = ceil(T/dt) so the definition does not depend on the step rate. | `verify_slip`, 35 events above the threshold: detected 27 / 33 / 35 / 35 for 50 Hz n = 3 / 2 / 1 and 200 Hz n = 3; delay max 61 / 41 / 21 / 15 ms; position error before the flag (not covered by any inflation) max 9.1 / 6.1 / 3.1 / 3.1 mm; error left by the missed events max 4.4 / 0.9 / 0 / 0 mm; false episodes without slip 0 for every count at both rates (manoeuvre course and one rect10 mission with n = 1) | `ekf_slip_n` (parameter) |
| F-11 | **Adopted, spec D-45 (version 1.5).** The slip inflation `σ²_ds = ekf_kd·\|Δs_e\|·ekf_slip_infl` of spec version ≤ 1.4 added **nothing in an in-place turn**, because Δs_e ≈ 0 there, although a slipping wheel makes the vehicle pivot about the other wheel and translate. Adopted form: while `slip` is set, `σ²_ds = ekf_kd·(\|ds_l\| + \|ds_r\|)/2·ekf_slip_infl`; without slip the term is unchanged (`ekf_kd·\|Δs_e\|`), and with both wheels moving in the same direction the two forms are equal. The mapping along the heading (`g`) is kept: the simulated error lies along the heading. The plant has no lateral wheel slip, so a sideways slide is not tested and not covered by this term. Encoder-heading mode (§7.17.7) has no slip detector and is unchanged. Still open, for any manoeuvre: the inflated σ grows with √t and the slip error with t: a full one-wheel slip at 0.3 m/s leaves 3 σ after about 1.2 s (computed from the defaults: 0.15 m/s against 55 mm·√t), so `ekf_slip_infl` = 100 covers short slips only. | in-place turn at 0.8 rad/s, one wheel without grip for 300 ms, flag after 41 ms: 13.4 mm position error, σ added 0.00 mm (form of version ≤ 1.4) against 15.8 mm (adopted), error/σ 0.85; the five detected in-place events (30–300 ms): error/σ 0.32–0.85. By direction (principal axes of the added covariance): along the main axis 13.43 mm against σ 15.81 mm, across it 0.12 mm against σ 0.50 mm (all events: ≤ 0.12 mm across). Straight and arc events: identical to the run before the change. Straight, same slip: 43.6 mm against σ added 27.8 mm (ratio 1.6); arc 62.0 mm against 29.1 mm (ratio 2.1) | `mx_slip_abs` = 1 (adopted, default) / 0 (spec version ≤ 1.4) |
| F-12 | §14.2 "every waypoint estimate within 5 cm of truth" cannot be met with the §10 start uncertainties on `course_rect10`; it is a statement about the priors, not about the filter (NEES and NIS are in band). At waypoint 1 (10 m, no boundary) the filter's own σ is 89 mm cross-track (`ekf_p0_th` 0.5° × 10 m) and 60 mm along-track (`ekf_p0_s` 0.5 % × 10 m = 50 mm, `ekf_kd` 1·10⁻⁴ m × 10 m → 32 mm, `ekf_p0_xy` 10 mm). Each later waypoint inherits the direction the edge sensors do not measure: the along-track error of a leg is the cross-track error of the leg before plus scale and `ekf_kd`. Second effect: the edge events only occur while the boundary crosses a ground sensor, i.e. for a cross-track offset near ±(`gnd_y_out` − `gnd_y_in`)/2 = ±5 cm; a vehicle that is centred, or more than about 5 cm + raggedness off, gets no events at all. | MC N = 10: waypoint 1 rms 53 mm along / 80 mm cross (8 of 10 runs above 5 cm), waypoint 2 63 / 58 mm, waypoint 3 44 / 37 mm; legs with a boundary but no edge event: 3 of 30 (run 6, leg 2: the vehicle was 7 to 18 cm towards the boundary, both right-hand sensors beyond it, waypoint 2 estimate error 18 cm). With `ekf_p0_th` = 0.1°, `ekf_p0_s` = 0.1 %, `ekf_kd` = 1·10⁻⁵ m and a boundary on leg 1 (`rect10_all`), N = 3: max 3.3 cm (σ about 2 cm per axis, so 5 cm is about 2.5 σ) | none (limit and priors are spec values) |

F-1, F-2, F-3, F-6, F-7 and F-8 are in `FIRMWARE_SPEC.md` as D-36 to D-41 (the correlation angle of F-3 is parameter 521 `ekf_sf_corr` = 2π `[DECIDE]`, `mx_sf_corr` here). F-9 to F-12 are new and not in the spec yet; they are proposals for §18.2.

## Results of the Octave runs

All noise parameters are placeholders, so these numbers check the filter logic, not the tuning. The raw output of both rounds is in `octave_quick_run.txt` (round 2 below the `SECOND RUN` banner) and, per stage, in `logs/`.

### Round 2: LSM6DS3, IMU sample rate 833 Hz (current)

The IMU is the plain LSM6DS3 (WHO_AM_I 0x69, spec D-42). `imu_odr_hz` = 833 is `[VERIFY]` pending the firmware decision (1666 Hz is the alternative; the simulation needs an integer rate below 1000 Hz). The gyro noise is still the placeholder `ekf_gyro_nd` = 1·10⁻⁴ rad/s/√Hz for truth and filter; no LSM6DS3 datasheet value is used here (`[VERIFY]` against the datasheet, then `[MEASURE]` by Allan variance).

| Check | Result | Limit | Pass |
|---|---|---|---|
| Yaw ring, 32 samples at 833 Hz | spans 37.2 ms (31 intervals; "32/833 = 38.4 ms" counts one interval too many); a query 20 ms and 37 ms back is answered, 38 ms back is `ST_RANGE` | ≥ 20 ms (one EKF period) | yes |
| Jacobians against finite differences | worst relative error 1.2e-9 | 1e-3 | yes |
| Static, float32, 1e5 steps | P symmetric, min eigenvalue 1.0e-9, no faults; drift 0.000 deg (literal F while still: 0.458 deg) | positive definite | yes |
| Gate probabilities | 0.99730 (scalar, 9.00), 0.99000 (2-D, 9.21) | — | yes |
| Back-shift interpolation | 0.015 mm, leak 0 m (19.9 mm without F-6) | — | yes |
| `t_ekf_line` | cross-track max 24.7 mm, NIS 97.1 %, NEES 94.6 %, 79 edges accepted, 0 rejected | < 30 mm, ≥ 90 % | yes |
| float32 against float64 | manoeuvre 0.7 mm / 0.03 deg; line40 3.8 mm; rect10 138 mm / 21.7 deg mid-run (closed-loop timing diverges), final errors 8.8 mm and 33.0 mm | informational | — |
| Sensitivity (`line40`, each parameter ×0.25 and ×4) | cross-track rms 8.8–9.0 mm for `ekf_kd`, `ekf_gyro_nd`, `ekf_q_bg` (nominal 8.9 mm); `ekf_sig_edge` ×0.25: rms 26.8 mm, max 71.2 mm, NIS 57 %, NEES 13 %; ×4: rms 12.8 mm, NIS 89 %; along-track error at the end −199 mm in every run | informational | — |
| **D-30:** position RMS difference 50 against 200 Hz | 0.35 mm (straight 0.01, arc 0.08, in-place turn 0.35 mm) | < 2 mm | yes |
| **D-30:** back-shift max error at 50 Hz | 1.251 mm (200 Hz: 1.251 mm) | < 2 mm | yes |
| **D-30:** `P_θθ` after the turns 50 / 200 Hz | 0.00219 / 0.00219 rad² | informational | — |
| **D-30:** rect10 (seed 1) closure 50 / 200 Hz | 16.0 / 17.5 cm | informational | — |
| **D-30:** rect10 (seed 1) max waypoint estimate error 50 / 200 Hz | 23.7 / 24.3 cm | informational | — |
| **D-30:** NEES below upper bound 50 / 200 Hz | 1.00 / 1.00 straight and arc, 0.82 / 0.82 in-place turn (cause: F-9) | informational | — |
| Slip: false episodes without slip, 50 Hz, count 1 / 2 / 3 | 0 / 0 / 0 on the manoeuvre course (max residual 0.055 rad/s against 0.15); 0 on one rect10 mission with count 1 | none | yes |
| Slip: same at 200 Hz | 0 / 0 / 0, but max residual 0.142 rad/s (5 % below the threshold); 0 on one rect10 mission with count 1 | informational | — |
| Slip: events above the threshold detected, 50 Hz n = 3 / n = 2 / n = 1 / 200 Hz n = 3 | 27 / 33 / 35 / 35 of 35 | informational | — |
| Slip: detection delay, max | 61 / 41 / 21 / 15 ms | informational | — |
| Slip: position error before the flag, max | 9.1 / 6.1 / 3.1 / 3.1 mm | informational | — |
| Slip: error left by undetected events, max | 4.4 / 0.9 / 0 / 0 mm | informational | — |
| Slip: in-place turn, σ added by the flag for 13.4 mm of error | 15.8 mm (adopted form, D-45, default; error/σ 0.85) / 0.00 mm (form of spec version ≤ 1.4, `mx_slip_abs` = 0) | informational (F-11) | — |
| Slip: same events by direction (principal axes of the added covariance) | along 13.43 mm against σ 15.81 mm; across 0.12 mm against σ 0.50 mm (five detected events: across ≤ 0.12 mm, ratio ≤ 0.93) | informational (F-11) | — |
| After D-45 (`mx_slip_abs` = 1 default), repeated: Jacobians, `t_ekf_line`, static 1e5 steps, 50/200 Hz comparison (seed 1), MC rect10 N = 3 (seeds 101–103) | all printed numbers identical to the runs before the change (slip never triggers there); straight and arc slip events identical | no change | yes |
| MC rect10, N = 10: all runs finished | 10/10 | all | yes |
| MC NIS in 95 % band | 97.3 % (edge 98.8 %, ZUPT 96.6 %, n = 1286) | ≥ 90 % | yes |
| MC NEES per sample in 95 % band | 93.0 % (≤ upper bound: 97.3 %) | ≥ 90 % | yes |
| MC averaged NEES in the χ²₃ₙ/N band | 93.4 % | ≥ 90 % | yes |
| MC rectangle closure | max 11.3 cm, mean 3.9 cm | < 30 cm | yes |
| MC waypoint estimate against truth (§14.2) | max 18.0 cm; above 5 cm at 17 of 30 waypoints | < 5 cm | **no** |
| MC true waypoint error (§14.2) | max 17.6 cm | < 15 cm | **no** |
| MC true waypoint error (§1.4, week 16) | max 17.6 cm | ≤ 20 cm | yes |
| MC, literal spec scale-factor Q (`mx_sf_mode` = 0), N = 10, same seeds: NIS | 96.8 % | ≥ 90 % | (yes) |
| same: NEES per sample in 95 % band | 77.4 % (≤ upper bound: 79.1 %) | ≥ 90 % | **no** |
| same: averaged NEES in band | 39.1 % | ≥ 90 % | **no** |
| same: closure / waypoint estimate / true waypoint error | max 12.3 cm (mean 5.0) / 18.0 cm / 17.6 cm | as above | yes / **no** / **no** |
| MC, `mx_sf_mode` = 2, N = 3 (seeds 101–103; default on the same seeds in brackets) | NIS 95.4 % (98.5 %), NEES 88.4 % (84.0 %), waypoint estimate max 8.0 cm (8.0 cm) | comparison | — |
| MC `rect10_all` (boundary on leg 1), default parameters, N = 3 | waypoint estimate max 7.7 cm; cross-track at waypoint 1 rms 5.9 mm, along-track rms 62.9 mm | < 5 cm | **no** |
| MC `rect10_all`, `ekf_p0_th` 0.1°, `ekf_p0_s` 0.1 %, `ekf_kd` 1·10⁻⁵ m (truth drawn from the same), N = 3 | waypoint estimate max 3.3 cm, true waypoint error max 3.8 cm, NIS 96.0 %, NEES 96.3 % | < 5 cm | yes (N = 3 only) |

The N = 3 subsets fail the 90 % NEES line (84.0 % for the default model on seeds 101–103, one run at 61 % from too *small* NEES); with N = 10 the default model passes. Three runs are too few for that check.

Waypoint estimate error by direction, default model, N = 10 (mm; along/cross with respect to the leg that ends at the waypoint; σ is the filter's own):

| Waypoint | along rms / max / σ | cross rms / max / σ | runs above 5 cm | scale error rms / σ | edge events on the leg (mean) | legs without any event |
|---|---|---|---|---|---|---|
| 1 (leg 1, no boundary) | 53 / 81 / 60 | 80 / 132 / 89 | 8 of 10 | 0.55 % / 0.50 % | 0 | 10 (by design) |
| 2 | 63 / 118 / 90 | 58 / 178 / 68 | 5 of 10 | 0.35 % / 0.32 % | 14.3 | 1 |
| 3 | 44 / 88 / 62 | 37 / 98 / 68 | 4 of 10 | 0.37 % / 0.30 % | 11.3 | 2 |
| finish | 43 / 97 / 66 | 12 / 28 / 27 | 2 of 10 | 0.28 % / 0.26 % | 15.4 | 0 |

### Round 1: 960 Hz (LSM6DSV, superseded)

Quick settings (one seed for the rate comparison, Monte Carlo N = 3, long test 1e5 steps). `design_observability` and `design_gates` were run only in this round.

| Check | Result | Limit | Pass |
|---|---|---|---|
| **D-30:** position RMS difference 50 against 200 Hz | 0.25 mm | < 2 mm | yes |
| **D-30:** back-shift max error at 50 Hz | 1.265 mm (200 Hz: 1.262 mm) | < 2 mm | yes |
| **D-30:** rect10 closure / max waypoint estimate error 50 / 200 Hz | 13.6 / 13.9 cm, 16.9 / 16.5 cm | informational | — |
| **D-30:** NEES below upper bound 50 / 200 Hz | 1.00 / 1.00 straight and arc, 0.74 / 0.74 in-place turn | informational | — |
| MC rect10, N = 3 | NIS 97.3 %, NEES 90.9 %, closure max 6.2 cm, waypoint estimate max 8.4 cm (**fails** < 5 cm), true waypoint error max 7.1 cm | as above | — |
| Observability (rect10) | σ of the scale state falls to 0.37–0.42 of the prior; freezing it raises mean NEES from about 1.2 to 3.8–4.9 | informational | — |

### Reading of the results

* **50 Hz is sufficient in simulation, also at the 833 Hz sample rate.** 50 Hz and 200 Hz agree to 0.35 mm RMS on identical sensor data and the back-shift error is 1.25 mm at both rates. This supports D-30; it does not replace a check on the vehicle. The yaw ring covers the 20 ms period with margin (37.2 ms).
* **Slip detector (F-10, F-11).** Keeping "3 steps" at 50 Hz costs at most 9 mm of position error that no inflation covers (3 mm at 200 Hz) and misses slips shorter than about 60 ms (≤ 4.4 mm each). That is small against the waypoint budget. A count of 2 costs nothing: the per-step residual without slip is 0.055 rad/s at 50 Hz against 0.142 rad/s at 200 Hz, because one encoder count weighs four times less in a 20 ms step. A time threshold (`ekf_slip_T` = 0.04 s, n = ceil(T/dt)) says the same thing independent of the step rate. The detector only inflates Q; it sees only *differential* slip (both wheels slipping equally give no residual), added nothing in an in-place turn until D-45 (F-11; now 15.8 mm of σ for 13.4 mm of error), and its inflation is outrun by a slip that lasts longer than about a second.
* **The in-place-turn overconfidence is the scale-factor Q model (F-9), not the wheelbase, the wheel mismatch or the rate.** It appears only when the vehicle turns more than one revolution net in one direction *and* the drawn scale error is large; on `rect10` (3π/2 net) the default model is consistent (NEES 93 %). The literal spec form is overconfident on `rect10` too (NEES 77 %, averaged NEES 39 %), which confirms F-3 with N = 10.
* **The 5 cm waypoint-estimate limit of §14.2 is not reachable with the §10 start uncertainties (F-12).** The error at waypoint 1 is the start heading (0.5° over 10 m = 87 mm) and the odometry scale and distance noise (60 mm); the filter reports exactly that σ, so this is not a filter fault. Later waypoints are limited along-track, the direction the boundary does not measure. What it takes: start heading known to about 0.1°, scale calibrated to about 0.1 %, `ekf_kd` ≤ 1·10⁻⁵ m (1 cm σ over 10 m; the placeholder 1·10⁻⁴ m alone gives 32 mm), and a boundary on leg 1; with all four the three runs stay below 3.3 cm, which is still only about 2.5 σ. Without them an along-track reference near each waypoint (post or face landmark, a cross boundary) is needed, or the limit must be relaxed to what the priors give (about 3 σ = 20–25 cm). The 15 cm true-error limit is also missed at N = 10 (17.6 cm).
* **Edge events need the boundary under a sensor.** With the boundary nominally midway between the inner and outer sensor, a well-centred vehicle produces no events, and one that is more than about 5 cm plus the raggedness off produces none either. 3 of 30 boundary legs had no event at all; seed 3 of `rect10_all` got none on leg 1 and ran identically to `rect10`. The event rate on the real boundary is the number to measure (H9).
* **Sensitivity:** on the straight only `ekf_sig_edge` matters. Set four times too small it breaks consistency and the 3 cm cross-track limit (71 mm); the other three parameters change nothing within ×0.25 to ×4. The along-track error (−199 mm = 0.5 % of 40 m) does not react to any of them: the scale is unobservable on one straight.
* **The odometry scale state is observable on the rectangle** (round 1): its σ falls to 0.37–0.42 of the prior, and freezing it raises mean NEES from about 1.2 to 3.8–4.9.

## What must be measured before the tuning is real

| Quantity | Parameter(s) | How |
|---|---|---|
| gyro white noise and bias random walk of the chosen IMU, mounted | `ekf_gyro_nd`, `ekf_q_bg` | ≥ 2 h stationary log at the configured ODR (833 Hz) → `allan_gyro` (below) |
| gyro scale factor residual | `imu_gz_scale`, `ekf_gyro_sf` | H7 10-turn spin; `ekf_gyro_sf` = residual/(20π) |
| random distance noise | `ekf_kd` | repeated `D5000`, variance of the end error about its mean / 5 m |
| odometry scale and its change with surface | `ekf_p0_s`, `ekf_q_s` | `D5000`×3 per surface |
| effective wheelbase, wheel mismatch | `odo_wheelbase`, `odo_scale_l/r` | UMBmark (affects slip detector and encoder-heading mode) |
| edge-event accuracy and RATE on the real boundary | `ekf_sig_edge`, `gnd_*` thresholds | H9 outdoors: log events against a taped line; count events per metre at nominal tracking |
| ground-sensor geometry | `gnd_x`, `gnd_y_in/out`, `gnd_h0` | chassis survey (OQ-2) |
| start pose accuracy | `ekf_p0_xy`, `ekf_p0_th` | start jig (OQ-8): repeat placement, measure spread |
| lateral sensor noise, landmark survey | `lat_*`, `ekf_sig_post`, landmark σ | course survey (OQ-3), static range logs |
| wheel speed loop time constant | (simulation only) | step response on the floor |

## Loading real data later

* **Stationary gyro log** (Allan): load the z rates in rad/s after axis
  mapping into a vector `w`, then `A = allan_gyro(w, 833)` (the ODR the log was taken at); use
  `A.ekf_gyro_nd`, `A.ekf_q_bg`, and look at `loglog(A.tau, A.adev)` for a
  flat bias-instability region (see the help of `allan_gyro`).
* **Replaying a capture through the filter**: build per estimator step the
  same inputs `sim_est_step` gets — encoder counts and `t_us` of the tick, the
  IMU samples `(t_us, rate)` pushed with `imu_yaw_push`, the edge events
  `(t_us, sensor id)` and the current segment — and call `sim_est_step` (or
  the `ekf_*` functions directly) in time order. NIS (no truth needed) is the
  consistency measure on real data (§1.4); compare with `sim_metrics`'s NIS
  bands. The C implementation is checked by feeding `ekf.c` (via
  `test/replay.c`) and this code the same input file and comparing `ekf_get`
  outputs step by step (float32 tolerance).
