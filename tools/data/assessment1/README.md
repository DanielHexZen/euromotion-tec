# Assessment 1 calibration logs

Drop the raw firmware logs of the identification runs here (screen sessions or single runs).
Picked up: `*.log` and `screenlog.*` in this folder. One file may hold several runs.
Suggested name: `<date>_<profile>_<n>.log`, e.g. `2026-11-20_P_1.log` (profiles S, P, N, L, D).
Needed at least: P, S, N (longitudinal) and D, L (lateral). Extra runs only enter the cross-validation table.
Runs without a closing `DONE X` line are truncated: they are skipped with a warning.
Evaluate (MATLAB, from anywhere):

    addpath Matlab/assessment1; S = run_assessment1;

Without any `X` run in this folder the pipeline falls back to synthetic data (`\DataSource` = synthetic).
