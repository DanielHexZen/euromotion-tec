function [R, out] = verify_monte_carlo(N, scenario, varargin)
%VERIFY_MONTE_CARLO  Consistency and accuracy over N seeded runs of the rectangle.
%
%   [R, out] = verify_monte_carlo(N, scenario, 'name', value, ...)
%   N         number of runs (default 10; each rect10 run is ~240 s simulated),
%             or a vector of run indices (seed = 100 + index), e.g. 4:6
%   scenario  'rect10' (default), 'rect10_lm', 'rect10_all'
%   options are passed to sim_config (e.g. 'ekf_rate', 200, 'p', p), except
%     'tag', name   cache every run's metrics in results/mc_cache/<name>_<scenario>_<seed>.mat
%                   and reuse them on the next call (the runs can then be split over
%                   several processes and pooled afterwards; the tag must identify
%                   the configuration, nothing checks it)
%   out.wpd: one row per waypoint, see wp_detail below.
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
if numel(N) > 1
    runs = N(:)';  N = numel(runs);
else
    runs = 1:N;
end
% own options (removed before the rest goes to sim_config)
tag = '';
keep = true(1, numel(varargin));
for k = 1:2:numel(varargin)
    if strcmp(varargin{k}, 'tag'), tag = varargin{k+1};  keep(k:k+1) = false; end
end
varargin = varargin(keep);
cdir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results', 'mc_cache');
if ~isempty(tag) && ~exist(cdir, 'dir'), mkdir(cdir); end
nees = [];  fin = zeros(N, 1);  clos = zeros(N, 1);  wpe = [];  wpt = [];  nis = [];
neesmat = [];  wpd = [];
for q = 1:N
    s = runs(q);
    cf = fullfile(cdir, sprintf('%s_%s_%03d.mat', tag, scenario, 100 + s));
    if ~isempty(tag) && exist(cf, 'file')
        C = load(cf);  M = C.M;  D = C.D;  nL = C.nL;
    else
        cfg = sim_config(scenario, 100 + s, varargin{:});
        L = sim_run(cfg);
        M = sim_metrics(L);
        D = wp_detail(L, M, 100 + s);
        nL = L.n;
        if ~isempty(tag), save('-v7', cf, 'M', 'D', 'nL'); end
    end
    nees = [nees; M.nees3(M.i0:end)]; %#ok<AGROW>
    nn = min(size(neesmat, 1), nL);
    if q == 1
        neesmat = M.nees3;
    else
        neesmat = neesmat(1:nn, :);  neesmat(:, end+1) = M.nees3(1:nn); %#ok<AGROW>
    end
    fin(q) = M.finished;  clos(q) = M.closure;
    w = M.wp(M.wp(:,1) < 99, :);
    wpe = [wpe; w(:,2)]; %#ok<AGROW>
    wpt = [wpt; w(:,3)]; %#ok<AGROW>
    wpd = [wpd; D]; %#ok<AGROW>
    nis = [nis; M.nis]; %#ok<AGROW>
    fprintf('  MC run %2d: finished %d, closure %.3f m, wp est err %s, wp true err %s, NEES in band %.2f, NIS %.2f, edges %d/%d\n', ...
        s, M.finished, M.closure, mat2str(w(:,2)', 3), mat2str(w(:,3)', 3), M.f_nees3, M.f_nis, ...
        M.n_edge_acc, M.n_edge_rej);
    out.runs(q) = struct('closure', M.closure, 'f_nees3', M.f_nees3, 'f_nis', M.f_nis, ...
        'n_edge', M.n_edge_acc, 'n_edge_rej', M.n_edge_rej, 'finished', M.finished);
end
out.wpd = wpd;
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
% ---- waypoint budget: which direction and which state carries the error (§14.2)
ids = unique(wpd(:, 2))';
fprintf('\nWaypoint estimate error by direction (along/cross = w.r.t. the leg that ends at the waypoint), N = %d\n', N);
fprintf('%-4s | %-22s | %-22s | %-9s | %-17s | %-9s | %s\n', 'wp', 'along: rms max sig mm', 'cross: rms max sig mm', '|e|>5 cm', 'scale: rms sig %', 'edges/leg', 'legs with 0 edges');
out.wp_budget = zeros(numel(ids), 12);
for i = 1:numel(ids)
    m = wpd(:, 2) == ids(i);
    W = wpd(m, :);
    ne = W(:, 12);
    if i > 1
        for q = 1:size(W, 1)
            pr = wpd(wpd(:, 1) == W(q, 1) & wpd(:, 2) == ids(i-1), 12);
            if ~isempty(pr), ne(q) = ne(q) - pr(1); end
        end
    end
    en = hypot(W(:, 4), W(:, 5));
    b = [sqrt(mean(W(:,4).^2)), max(abs(W(:,4))), sqrt(mean(W(:,8).^2)), ...
         sqrt(mean(W(:,5).^2)), max(abs(W(:,5))), sqrt(mean(W(:,9).^2)), ...
         sum(en > 0.05), size(W, 1), sqrt(mean(W(:,7).^2)), sqrt(mean(W(:,11).^2)), mean(ne), sum(ne == 0)];
    out.wp_budget(i, :) = b;
    fprintf('%-4d | %6.1f %6.1f %6.1f   | %6.1f %6.1f %6.1f   | %2d of %-3d | %6.3f %6.3f     | %7.1f   | %d\n', ids(i), ...
        1e3*b(1:6), b(7), b(8), 100*b(9), 100*b(10), b(11), b(12));
end
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

function D = wp_detail(L, M, seed)
% One row per waypoint (and the finish, id 99), taken at the estimator step of
% the waypoint record:
%   [seed id t  e_along e_cross e_theta e_s  sig_along sig_cross sig_theta sig_s
%    edges_accepted_so_far  d_since_last_edge  true_along_err true_cross_err]
% along/cross are relative to the LINE segment that ends at the waypoint;
% errors are estimate minus truth, sigmas from P.
c = L.cfg.course;
D = zeros(size(L.wp, 1), 15);
for i = 1:size(L.wp, 1)
    id = L.wp(i, 1);
    if id == 99
        k = find(c.type == 6, 1) - 1;
    else
        k = find(c.type == 4 & c.wp_id == id, 1) - 1;
    end
    dx = c.x1(k) - c.x0(k);  dy = c.y1(k) - c.y0(k);  Ls = hypot(dx, dy);
    tx = dx/Ls;  ty = dy/Ls;
    [~, n] = min(abs(L.t - L.wp(i, 2)));
    e = M.e(n, :);
    P = reshape(L.est(n, 34:58), 5, 5);
    Rm = [tx ty; -ty tx];
    Pr = Rm*P(1:2,1:2)*Rm';
    ne = L.est(1:n, 18);
    j = find(ne < ne(n), 1, 'last');
    if isempty(j), dlast = NaN; else, dlast = L.est(n, 59) - L.est(j + 1, 59); end
    pt = [L.wp(i,5) - c.x1(k), L.wp(i,6) - c.y1(k)];
    D(i, :) = [seed, id, L.wp(i,2), e(1)*tx + e(2)*ty, -e(1)*ty + e(2)*tx, e(3), e(5), ...
        sqrt(Pr(1,1)), sqrt(Pr(2,2)), sqrt(P(3,3)), sqrt(P(5,5)), ne(n), dlast, ...
        pt(1)*tx + pt(2)*ty, -pt(1)*ty + pt(2)*tx];
end
end
