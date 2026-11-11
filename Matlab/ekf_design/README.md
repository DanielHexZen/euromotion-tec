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
| `core/`, `sim/`, `design/`, `verify/`, `run_all.m` | executed in **GNU Octave 8.4** (Linux), single precision filter, results below |
| figures | **not produced**: headless Octave here could not render text (`can_plot` returns false); MATLAB will write them |
| `simulink/build_ekf_model.m`, `simulink/run_ekf_model.m` | **never executed** (no MATLAB/Simulink available). Written against documented APIs; expect small fixes on the first run in MATLAB |
| MATLAB Function block code generation | **not checked**. The step functions were written for `%#codegen` (fixed sizes, no cell arrays, no dynamic fields, type-stable structs) but no code generator has seen them |

## Files

### `core/` — the filter (port these to C)

| File | Spec | C function |
|---|---|---|
| `ekf_params.m` | §10 | `par_table.h` defaults (plus `mx_*` MATLAB-only switches) |
| `ekf_param_cast.m` | — | the single/double switch (firmware: float) |
| `ekf_init.m` | §7.17.1, §7.17.5 | `ekf_init` |
| `ekf_reset_pose.m` | §7.17.1 | `ekf_reset_pose` |
| `ekf_still_update.m` | §7.17.4, REQ-EKF-09 | standstill flag + ZUPT accumulator (inside `ekf_predict` or `app_est`) |
| `ekf_predict.m` | §7.17.2 | `ekf_predict` (dispatch on `dpsi = NaN`, history push) |
| `ekf_predict_imu.m`, `ekf_model_imu.m` | §7.17.2, REQ-EKF-03 | IMU-mode prediction, F, Q |
| `ekf_predict_enc.m`, `ekf_model_enc.m` | §7.17.7, REQ-EKF-10 | encoder-heading mode |
| `ekf_slip_detect.m` | REQ-EKF-04 | slip detector |
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
| `sim_imu_*.m` | 960 Hz DRDY samples: exact true rate inside the tick, constant bias + random walk + ramp, white noise, scale factor, 17.5 mdps LSB |
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
| `save_fig.m`, `can_plot.m` | figure helpers |

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
| `verify_monte_carlo.m` | NEES/NIS, closure and waypoint targets over N runs |
| `sim_metrics.m`, `plot_run.m`, `vres.m` | metrics, figures, result rows |

### `simulink/`

| File | Content |
|---|---|
| `build_ekf_model.m` | builds `ekf_mr2006b.slx`: Plant (1 kHz), Sensors/{Encoders 1 kHz, IMU 960 Hz, Ground 1 kHz, Lateral 1 kHz}, Estimator/{YawIntegral 1 kHz, EKF 50 Hz offset 1 ms}, Guidance (50 Hz offset 1 ms), CmdDelay (1 kHz unit delay), StatusDelay (50 Hz unit delay); fixed-step discrete 1/48000 s, single-tasking, algebraic loop = error; logs `sl_est`, `sl_upd`, `sl_gdn`, `sl_plant` |
| `run_ekf_model.m` | builds, simulates, runs `sim_run` with the same configuration and compares sample by sample |

Every MATLAB Function block contains a persistent state and one call of the
same `sim_*_step` function that `sim_run.m` calls, so the filter exists once.
The base step 1/48000 s makes 1 kHz (48 steps), 960 Hz (50 steps, offset 1
step) and 50 Hz (960 steps, offset 48 steps) exact, and the 960 Hz samples
never coincide with a 1 kHz tick, so the execution order is unambiguous and
equal to `sim_run`'s. The command and status feedbacks pass through unit
delays (as in the firmware, where `mot_apply` acts after the slot and
`app_est` sees the drive status of the previous control tick), so there is no
algebraic loop.

## Mapping onto the firmware under D-30

D-30 moves the EKF from the 200 Hz `S_EST` slot into `S_CTRL` (50 Hz, phase
1), immediately before `gnd_step/fwd_step → msn_step → drv_step`:

```
S_FAST 1 kHz : enc_update, imu_drain (yaw ring, 960 Hz samples), vl_service -> gnd/lat queues
S_CTRL 50 Hz : app_est  = sim_est_step : odo from the encoder snapshot of this tick,
                                          imu_yaw_integral_at(t_enc), still, predict, ZUPT,
                                          queued edge / face / heading / post updates
               msn_step -> nav -> drv  = sim_gdn_step
```

The pose history then holds one entry per 20 ms; 64 entries already cover the
1.0 s age limit (the spec's 256 entries = 5.12 s).

## Spec findings

Each was checked numerically; the code implements the corrected form and the
literal form stays selectable through an `mx_*` switch for comparison.

RESULTS_FINDINGS_PLACEHOLDER

## Results of the Octave run

RESULTS_PLACEHOLDER

## What must be measured before the tuning is real

| Quantity | Parameter(s) | How |
|---|---|---|
| gyro white noise and bias random walk of the chosen IMU, mounted | `ekf_gyro_nd`, `ekf_q_bg` | ≥ 2 h stationary log at 960 Hz → `allan_gyro` (below) |
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
  mapping into a vector `w`, then `A = allan_gyro(w, 960)`; use
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
