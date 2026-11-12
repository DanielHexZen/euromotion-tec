function [R, out] = run_ekf_model(scenario, T_end, seed)
%RUN_EKF_MODEL  Build and simulate the Simulink model, compare it sample by
%   sample with the script reference sim_run.m on the same configuration.
%
%   [R, out] = run_ekf_model(scenario, T_end, seed)
%   defaults: 'rect10', 60 s, seed 1.
%
%   NOT EXECUTED BY THE AUTHOR (no Simulink available). Expected result: the
%   two implementations call the same step functions with the same inputs in
%   the same order, so they agree to floating-point rounding (generated C
%   versus the MATLAB interpreter: libm sin/cos and fused operations may
%   differ in the last bit). Tolerances: position 1e-6 m, heading 1e-6 rad
%   (single precision filter), state machine and update counters exactly.
if nargin < 1 || isempty(scenario), scenario = 'rect10'; end
if nargin < 2 || isempty(T_end), T_end = 60; end
if nargin < 3, seed = 1; end
R = [];
out = struct();
if ~exist('sim', 'builtin') && ~exist('sim', 'file') || exist('OCTAVE_VERSION', 'builtin')
    R = vres('Simulink model vs script', -1, 'not run', 'Simulink not available');
    fprintf('run_ekf_model: Simulink is not available here; skipped.\n');
    return;
end
cfg = sim_config(scenario, seed, 'T_end', T_end);
mdl = build_ekf_model(cfg);
so  = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
Es  = so.get('sl_est');
Gs  = so.get('sl_gdn');
L   = sim_run(cfg);
n   = min(size(Es, 1), L.n);
out.n_sl = size(Es, 1);  out.n_ml = L.n;
d   = abs(Es(1:n, :) - L.est(1:n, :));
d(:, 3) = abs(mod(Es(1:n, 3) - L.est(1:n, 3) + pi, 2*pi) - pi);
out.dpos = max(max(d(:, 1:2)));
out.dth  = max(d(:, 3));
out.dP   = max(max(d(:, 34:58)));
out.dcnt = max(max(d(:, 17:26)));
out.dgdn_state = max(abs(Gs(1:min(n, size(Gs,1)), 8) - L.gdn(1:min(n, size(Gs,1)), 8)));
k1 = find(max(d(:, 1:3), [], 2) > 1e-9, 1);
if isempty(k1), k1 = 0; end
out.first_diff = k1;
fprintf('run_ekf_model: %d / %d samples, max |dpos| %.3g m, |dth| %.3g rad, |dP| %.3g, counters %g, state %g, first > 1e-9 at %d\n', ...
    out.n_sl, out.n_ml, out.dpos, out.dth, out.dP, out.dcnt, out.dgdn_state, k1);
ok = out.n_sl == out.n_ml && out.dpos < 1e-6 && out.dth < 1e-6 && out.dcnt == 0 && out.dgdn_state == 0;
R = vres('Simulink model vs script (sample by sample)', double(ok), ...
    sprintf('dpos %.2g m, dth %.2g rad, %d samples', out.dpos, out.dth, n), '< 1e-6, counters equal');
end
