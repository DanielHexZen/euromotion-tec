function [R, out] = verify_monte_carlo(N, scenario, varargin)
%VERIFY_MONTE_CARLO  Consistency and accuracy over N seeded runs of the rectangle.
%
%   [R, out] = verify_monte_carlo(N, scenario, 'name', value, ...)
%   N         number of runs (default 10; each rect10 run is ~240 s simulated)
%   scenario  'rect10' (default), 'rect10_lm', 'rect10_all'
%   options are passed to sim_config (e.g. 'ekf_rate', 200, 'p', p)
%
%   Truth 'prior': start pose, initial bias, odometry scale and gyro scale
%   factor are drawn from the filter's own P0/Q model, so NEES tests the
%   equations. Checks:
%     - spec §1.4: NIS within the two-sided 95 % chi2 band >= 90 % of the updates
%     - NEES(x,y,theta) in the 95 % chi2_3 band, per sample, pooled over runs
%     - averaged NEES over N runs against the chi2_{3N}/N 95 % band
%     - §1.4 rectangle closes within 30 cm (true end position vs start)
%     - §14.2 every waypoint ESTIMATE within 5 cm of truth; every TRUE waypoint
%       error < 15 cm; §1.4 waypoint <= 20 cm (week 16) / 40 cm (week 15)
if nargin < 1 || isempty(N), N = 10; end
if nargin < 2 || isempty(scenario), scenario = 'rect10'; end
nees = [];  fin = zeros(N, 1);  clos = zeros(N, 1);  wpe = [];  wpt = [];  nis = [];
neesmat = [];
for s = 1:N
    cfg = sim_config(scenario, 100 + s, varargin{:});
    L = sim_run(cfg);
    M = sim_metrics(L);
    nees = [nees; M.nees3(M.i0:end)]; %#ok<AGROW>
    nn = min(size(neesmat, 1), L.n);
    if s == 1
        neesmat = M.nees3;
    else
        neesmat = neesmat(1:nn, :);  neesmat(:, end+1) = M.nees3(1:nn); %#ok<AGROW>
    end
    fin(s) = M.finished;  clos(s) = M.closure;
    w = M.wp(M.wp(:,1) < 99, :);
    wpe = [wpe; w(:,2)]; %#ok<AGROW>
    wpt = [wpt; w(:,3)]; %#ok<AGROW>
    nis = [nis; M.nis]; %#ok<AGROW>
    fprintf('  MC run %2d: finished %d, closure %.3f m, wp est err %s, wp true err %s, NEES in band %.2f, NIS %.2f, edges %d/%d\n', ...
        s, M.finished, M.closure, mat2str(w(:,2)', 3), mat2str(w(:,3)', 3), M.f_nees3, M.f_nis, ...
        M.n_edge_acc, M.n_edge_rej);
    out.runs(s) = struct('closure', M.closure, 'f_nees3', M.f_nees3, 'f_nis', M.f_nis, ...
        'n_edge', M.n_edge_acc, 'n_edge_rej', M.n_edge_rej, 'finished', M.finished);
end
b3 = chi2_quantile([0.025 0.975], 3);
f_nees = mean(nees >= b3(1) & nees <= b3(2));
f_nees_hi = mean(nees <= b3(2));
bN = chi2_quantile([0.025 0.975], 3*N)/N;
anees = mean(neesmat, 2);
i0 = find(anees > 0, 1);
f_anees = mean(anees(i0:end) >= bN(1) & anees(i0:end) <= bN(2));
b1 = chi2_quantile([0.025 0.975], 1);  b2 = chi2_quantile([0.025 0.975], 2);
if isempty(nis)
    f_nis = NaN;  f_nis_edge = NaN;  f_nis_zupt = NaN;
else
    dof = 1 + (nis(:,1) == 5);
    lo = b1(1)*(dof == 1) + b2(1)*(dof == 2);  hi = b1(2)*(dof == 1) + b2(2)*(dof == 2);
    inb = nis(:,3) >= lo & nis(:,3) <= hi & nis(:,2) == 0;
    f_nis = mean(inb);
    f_nis_edge = mean(inb(nis(:,1) == 2));
    f_nis_zupt = mean(inb(nis(:,1) == 1));
end
out.f_nees = f_nees;  out.f_anees = f_anees;  out.f_nis = f_nis;  out.clos = clos;
out.wpe = wpe;  out.wpt = wpt;  out.anees = anees;
R = [];
R = [R, vres(sprintf('MC %s N=%d all runs finished', scenario, N), double(all(fin)), sprintf('%d/%d', sum(fin), N), 'all')];
R = [R, vres('MC NIS in 95 % band (spec §1.4)', double(f_nis >= 0.90), ...
    sprintf('%.1f %% (edge %.1f %%, ZUPT %.1f %%, n = %d)', 100*f_nis, 100*f_nis_edge, 100*f_nis_zupt, size(nis,1)), '>= 90 %')];
R = [R, vres('MC NEES(x,y,th) per sample in 95 % band', double(f_nees >= 0.90), ...
    sprintf('%.1f %% (<= upper: %.1f %%)', 100*f_nees, 100*f_nees_hi), '>= 90 %')];
R = [R, vres(sprintf('MC averaged NEES in chi2_{3N}/N band'), double(f_anees >= 0.90), ...
    sprintf('%.1f %%', 100*f_anees), '>= 90 %')];
R = [R, vres('rectangle closure (§1.4)', double(max(clos) < 0.30), ...
    sprintf('max %.1f cm, mean %.1f cm', 100*max(clos), 100*mean(clos)), '< 30 cm')];
R = [R, vres('waypoint estimate vs truth (§14.2)', double(max([wpe; 0]) < 0.05), ...
    sprintf('max %.1f cm', 100*max([wpe; 0])), '< 5 cm')];
R = [R, vres('true waypoint error (§14.2)', double(max([wpt; 0]) < 0.15), ...
    sprintf('max %.1f cm', 100*max([wpt; 0])), '< 15 cm')];
R = [R, vres('true waypoint error (§1.4 week 16)', double(max([wpt; 0]) <= 0.20), ...
    sprintf('max %.1f cm', 100*max([wpt; 0])), '<= 20 cm')];
fprintf('verify_monte_carlo: NIS %.3f, NEES %.3f, avg NEES %.3f, closure max %.3f m, wp est max %.3f, wp true max %.3f\n', ...
    f_nis, f_nees, f_anees, max(clos), max([wpe; 0]), max([wpt; 0]));
end
