# Assessment 2 – Discrete PID + Linear Kalman Filter for autonomous navigation

MR2006B Automatización Industrial, Module 4 (Control). Daniel Appel (A01833285), Jan Mosler (A01833812).

Report: `paper/main.pdf` (IEEE conference format, 7 pages). All numbers come from MATLAB R2026a runs.

| Package | Content | Entry point |
|---|---|---|
| `01_sysid/` | Least-squares ARX identification (longitudinal v_cmd→v, lateral u_lat→yaw rate), controllable/observable state space, T = 0.02 s | `sysid_main.m` |
| `02_pid/` | Discrete PID gains by pole placement (Diophantine), prefilter, back-calculation anti-windup | `pid_main.m` |
| `03_lkf/` | Linear Kalman filter with a_x, a_y inputs, Q/R from sensor statistics, simulation + real data | `lkf_main.m` |
| `04_navigation/` | Closed lap on the 1:10 Autódromo Hermanos Rodríguez with LKF feedback, CTE/ATE/IAE/RMSE, seed and plant-mismatch study, Simulink model `nav_closed_loop.slx` | `track_prepare.m`, `nav_main.m`, `build_nav_simulink.m` |
| `common/` | Shared helper functions | – |
| `data/` | `openloop_test01_synced.mat`, track `MexicoCity.csv` (TUMFTM racetrack database), portable noise `noise_nav.mat` | – |
| `paper/` | LaTeX source (`main.tex`, `numbers.tex`, `refs.bib`, `figs/`) | – |

Run order (each script from its own folder): `01_sysid/sysid_main` → `02_pid/pid_main` → `03_lkf/lkf_main` → `04_navigation/nav_main` → `04_navigation/build_nav_simulink`. Each package writes a `RESULTS.md`.

Key results (scaled 1:10, LKF feedback): validation fit 93.1 % / 91.7 %; PID ζ 0.85 / 0.75, t_ss 1.56 s / 2.10 s, overshoot 0.6 % / 1.2 %; LKF position RMSE −99.99 % vs. raw double integration; CTE_max 0.101 m, RMSE_CTE 0.025 m (1.01 m / 0.25 m full scale); Simulink = script to 1e-13 m.

`SPEC.md` is the shared design contract. `KalmanFilter/` and `Assessment-2_EuroMotion.tex` are the earlier EKF draft for the previous version of the assignment.
