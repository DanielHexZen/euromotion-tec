function [R, L, M] = verify_line(cls, seed)
%VERIFY_LINE  t_ekf_line (§14.1): 40 m straight, scale error +0.5 %, gyro bias
%   drift (random walk + 1e-6 rad/s^2 ramp), edge events every 0.5 m, closed
%   loop with nav_follow on the estimate, EKF at 50 Hz (D-30).
%   Pass: |cross-track estimation error| < 3 cm over the whole leg;
%   NIS within the two-sided 95 % band >= 90 %.
%   Reported, not graded by the spec: along-track error at the waypoint (the
%   scale error is unobservable from edges on one straight, see design_observability).
if nargin < 1, cls = 'single'; end
if nargin < 2, seed = 1; end
cfg = sim_config('line40', seed, 'truth', 'spec_line', 'gnd', 2, 'cls', cls);
L = sim_run(cfg);
M = sim_metrics(L);
ect = M.e_ct(M.i0:end);  ect = ect(~isnan(ect));
R = [];
R = [R, vres('t_ekf_line cross-track error', double(max(abs(ect)) < 0.03), ...
    sprintf('max %.1f mm, rms %.1f mm', 1e3*max(abs(ect)), 1e3*sqrt(mean(ect.^2))), '< 30 mm')];
R = [R, vres('t_ekf_line NIS in 95 % band', double(M.f_nis >= 0.90), ...
    sprintf('%.1f %% of %d', 100*M.f_nis, size(M.nis, 1)), '>= 90 %')];
R = [R, vres('t_ekf_line NEES(x,y,th) in 95 % band', -1, sprintf('%.1f %%', 100*M.f_nees3), 'informational')];
if ~isempty(M.wp)
    R = [R, vres('t_ekf_line waypoint estimate error (along-track)', -1, ...
        sprintf('%.1f mm (truth scale %.3f, est %.4f)', 1e3*M.wp(1,2), cfg.tr.scale, L.est(end,5)), ...
        'informational: s unobservable on one straight')];
end
fprintf('verify_line: ct max %.1f mm, NIS %.1f %%, NEES %.1f %%, edges %d acc / %d rej\n', ...
    1e3*max(abs(ect)), 100*M.f_nis, 100*M.f_nees3, M.n_edge_acc, M.n_edge_rej);
end
