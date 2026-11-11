function [R, out] = design_sensitivity(factors, scenario)
%DESIGN_SENSITIVITY  Sweep the main tuning parameters with the truth held fixed.
%
%   [R, out] = design_sensitivity(factors, scenario)
%   factors   multipliers applied to the FILTER's value (default [0.25 4])
%   scenario  'line40' (default; t_ekf_line truth, edge events every 0.5 m) or 'rect10'
%
%   The truth keeps the default noise levels, so factor 1 is "correctly
%   tuned"; < 1 is an overconfident filter, > 1 a conservative one. Swept:
%   ekf_kd, ekf_gyro_nd, ekf_sig_edge, ekf_q_bg. Metrics: RMS and max
%   cross-track estimation error, along-track error at the end, NIS and NEES
%   in-band fractions. Same seed for every run.
if nargin < 1 || isempty(factors), factors = [0.25 4]; end
if nargin < 2, scenario = 'line40'; end
names = {'ekf_kd', 'ekf_gyro_nd', 'ekf_sig_edge', 'ekf_q_bg'};
p0 = ekf_params();
rows = {};
base = one(p0, scenario);
rows(end+1, :) = {'nominal', 1, base};
for i = 1:numel(names)
    for f = factors
        p = p0;  p.(names{i}) = p0.(names{i})*f;
        rows(end+1, :) = {names{i}, f, one(p, scenario)}; %#ok<AGROW>
    end
end
fprintf('\n=== Sensitivity (%s, truth fixed) ===\n', scenario);
fprintf('%-14s %6s  %9s %9s %10s %6s %6s\n', 'parameter', 'factor', 'ct rms mm', 'ct max mm', 'at end mm', 'NIS', 'NEES');
for r = 1:size(rows, 1)
    m = rows{r, 3};
    fprintf('%-14s %6.2f  %9.1f %9.1f %10.1f %6.2f %6.2f\n', rows{r, 1}, rows{r, 2}, 1e3*m.ct_rms, ...
        1e3*m.ct_max, 1e3*m.at_end, m.f_nis, m.f_nees);
end
out.rows = rows;
R = vres(['sensitivity sweep ' scenario], -1, sprintf('%d runs', size(rows, 1)), 'see table');
end

function m = one(p, scenario)
if strcmp(scenario, 'line40')
    cfg = sim_config('line40', 5, 'truth', 'spec_line', 'gnd', 2, 'p', p);
    pn = ekf_params();                       % truth fixed at the nominal values
    cfg.tr.nd = pn.ekf_gyro_nd;  cfg.tr.q_bg = pn.ekf_q_bg;  cfg.tr.sig_edge_true = pn.ekf_sig_edge;
else
    cfg = sim_config(scenario, 5, 'p', p);
    pn = ekf_params();
    cfg.tr.nd = pn.ekf_gyro_nd;  cfg.tr.q_bg = pn.ekf_q_bg;
end
L = sim_run(cfg);
M = sim_metrics(L);
ct = M.e_ct(M.i0:end);  ct = ct(~isnan(ct));
at = M.e_at(~isnan(M.e_at));
m.ct_rms = sqrt(mean(ct.^2));  m.ct_max = max(abs(ct));
m.at_end = at(end);
m.f_nis = M.f_nis;  m.f_nees = M.f_nees3;
end
