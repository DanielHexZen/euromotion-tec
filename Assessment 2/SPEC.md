# Assessment 2 – Discrete PID + Linear Kalman Filter for autonomous navigation
Shared contract for all work packages. Read fully before starting.

## Assignment (verbatim intent)
- Obj. 1: discrete PID controller (pole placement) regulating the vehicle position in simulation.
- Obj. 2: discrete **Linear** Kalman Filter (LKF) estimating position from noisy longitudinal a_x
  and lateral a_y acceleration measurements.
- Spec 1 SysID: identify discrete longitudinal AND lateral dynamics with Least Squares (LS),
  fit >= 90 % (NRMSE fit, report also R^2), convert transfer functions to a controllable /
  observable state-space model at fixed sampling time T. Must be exact and verifiable
  (show tf -> ss equivalence, same T everywhere).
- Spec 2 PID: analytically compute discrete PID gains by pole placement on the identified model.
  Targets: zeta >= 0.7, t_ss <= 2.5 s (2 % criterion), overshoot < 10 %. Handle actuator
  saturation cleanly (anti-windup).
- Spec 3 LKF: discrete LKF on the state-space model, fusing noisy a_x, a_y. Position RMSE must be
  reduced by >= 70 % vs raw double integration of the accelerometers (and vs real measured
  position where available). Q and R must be justified (from sensor noise statistics), not guessed.
- Spec 4 Navigation: LKF position estimates as feedback to the discrete PIDs, complete a full lap of
  the scaled Mexico City F1 circuit (Autodromo Hermanos Rodriguez) without leaving the track.
- Spec 5 Metrics: at least two indices: CTE (max, RMSE), ATE, RMSE, IAE.
- Spec 6 Report: IEEE conference paper (LaTeX IEEEtran), <= 8 pages, in English.
- Grading "Outstanding" thresholds: fit >= 90 %; zeta >= 0.7, t_ss < 2.5 s, OS < 10 %, saturation
  handled; RMSE reduction >= 70 % with justified Q/R; CTE_max < 1.5 m and RMSE_CTE < 0.5 m
  **scaled to vehicle dimensions**; >= 2 indices; good conclusions + paper.

## Vehicle and data
- Real F1TENTH Ackermann car "McQueen56": wheelbase L = 0.324 m, width ~0.30 m, length ~0.54 m.
- Data: `data/openloop_test01_synced.mat` (100 Hz grid, open-loop joystick drive, 105 s, loops).
  Fields (all column vectors): t, cmd_speed [m/s], servo_steering [rad, + left] (steering actually
  sent to servo), vesc_speed [m/s] (wheel speed), imu_ax, imu_ay [m/s^2, body frame],
  imu_wz [rad/s], imu_yaw, pf_x, pf_y, pf_yaw [map frame, reference pose from LiDAR particle
  filter, unwrapped yaw], odom_*. Some NaN at gaps -> handle (ZOH for commands, skip for sensors).
  gains.wheelbase etc. Previous (nonlinear) analysis found: tau_v ~ 0.28 s, steering lag ~0.1 s,
  effective steering gain ~1.39 on r = v/L*tan(delta), gyro bias ~1e-4 rad/s, gyro std 0.0052.
- Steering limits: servo 0..1 -> delta in [-0.366, +0.374] rad; use |delta| <= 0.34 rad.
  Speed command limit: 0 ... 3.0 m/s (car max seen 2.45 m/s; use v_max = 3.0 m/s).
- Track: `data/MexicoCity.csv` (TUMFTM racetrack-database): x_m, y_m, w_tr_right_m, w_tr_left_m,
  closed loop, 4297 m long, 9.7-14 m wide.
  **Scale 1:10** (F1 car ~5.5 m vs our car ~0.54 m): track ~430 m, width ~1.0-1.4 m.
  Scaled Outstanding limits: CTE_max <= 0.15 m, RMSE_CTE <= 0.05 m (report also the
  full-scale-equivalent = value x 10 to compare with the rubric 1.5 m / 0.5 m).

## Global conventions
- Sampling time **T = 0.02 s** (50 Hz = native rate of VESC and IMU) for identification,
  controllers, LKF and simulation. Data at 100 Hz are decimated by 2 (take every 2nd sample).
- Code must run in **MATLAB R2026a** (Control System, System Identification, Optimization,
  Symbolic toolboxes available). Implement LS explicitly (theta = Phi\Y), do not rely only on
  `arx`/`tfest` (you may use them as cross-check). Test your code in the container with
  **Octave** (`octave --no-gui -q`, packages control/signal may be installed: `pkg load control`);
  guard toolbox-only calls if Octave lacks them. Scripts: no local functions inside scripts
  (put helpers in separate function files) so they run in both.
- Figures: save as PNG (and .pdf if easy) into `<package>/fig/`, clean, labelled axes with units,
  English labels, readable at IEEE column width (~8.8 cm). Use `print -dpng -r200`.
- Each package writes a short `RESULTS.md` (numbers the paper needs) and a .mat with its outputs.
- Everything in English (code comments, figures). Keep code clean and commented.

## Folder layout (root /home/claude/A2)
```
data/            input data (do not modify)
common/          shared helper functions (e.g. fit metrics, decimation, track loading)
01_sysid/        sysid_main.m -> model_id.mat, fig/, RESULTS.md
02_pid/          pid_design.m -> pid_gains.mat, fig/, RESULTS.md
03_lkf/          lkf_main.m  -> lkf_params.mat, fig/, RESULTS.md
04_navigation/   track_prepare.m, nav_main.m, build_nav_simulink.m -> fig/, RESULTS.md
paper/           IEEEtran paper (main.tex, figures copied from packages)
```

## Interfaces
`01_sysid/model_id.mat` must contain:
- `T`
- `Gv`  : struct with discrete tf of longitudinal dynamics  v_cmd -> v   (num, den in z^-1 or z,
          state clearly), plus `Gv_ss` with A,B,C,D (controllable canonical) and observable form.
- `Gr`  : lateral dynamics. Recommended structure: input u_lat = v*tan(delta_cmd)/L
          (kinematic yaw-rate demand, makes the model linear and speed independent),
          output r = yaw rate (imu_wz). Same fields as Gv. Document the scheduling idea.
- fit values (NRMSE fit %, R^2) for identification AND validation data (split the run or
  use cross-validation segments), one-step-ahead and free-run simulation.
- `par` : L, delta_max, v_max, k_delta etc.

`02_pid/pid_gains.mat`: struct `pid_lon`, `pid_lat` with Kp, Ki, Kd, (N filter), form of the
discrete PID (state it exactly, e.g. parallel with backward Euler integrator and filtered
derivative), desired poles, achieved closed-loop poles, step metrics, saturation limits,
anti-windup method.

`03_lkf/lkf_params.mat`: matrices F, G (input), H, Q, R, T and the sensor-noise statistics used.

## Simulation plant (for 02 and 04)
Nonlinear kinematic bicycle at T with identified actuator dynamics:
v(k+1) from Gv(v_cmd), yaw rate from Gr(u_lat), psi integrates r, x,y integrate v*cos/sin(psi).
Sensors: accelerometer a_x = dv/dt, a_y = v*r (+ noise std and bias from real standstill data),
gyro r (+ noise), optional low-rate position fix (e.g. 10 Hz, noise std estimated from the real
particle-filter pose). Use fixed random seed (rng(1)) for reproducibility.
