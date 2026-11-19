function S = run_assessment1(varargin)
%RUN_ASSESSMENT1  Least-squares identification of longitudinal and lateral dynamics.
%   S = run_assessment1('data', folder, 'synthetic', tf, 'out', folder)
%   Reads firmware logs (id_read_log), fits by explicit linear least squares, validates by
%   free-run simulation, writes figures, results.tex and cv_table.tex.
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
ip = inputParser;
addParameter(ip, 'data', fullfile(root, 'tools', 'data', 'assessment1'));
addParameter(ip, 'synthetic', []);
addParameter(ip, 'label', '');            % overrides the DataSource label (default: from 'synthetic')
addParameter(ip, 'out', fullfile(root, 'reports', 'assessment1'));
parse(ip, varargin{:});
o = ip.Results;
figdir = fullfile(o.out, 'figures');
if ~exist(figdir, 'dir'), mkdir(figdir); end

f = [dir(fullfile(o.data, '*.log')); dir(fullfile(o.data, 'screenlog.*'))];
runs = id_read_log(unique(fullfile({f.folder}, {f.name})));
if isempty(o.synthetic), o.synthetic = isempty(runs); end
if o.synthetic
    d = tempname;
    id_synth(d);
    runs = id_read_log(fullfile(d, 'screenlog.0'));
end
src = o.label;
if isempty(src), src = ternary(o.synthetic, 'synthetic', 'experimental'); end
Ts = runs(1).Ts;

% signals: v, r from encoders; u, dl = common / differential duty
for i = 1:numel(runs)
    q = runs(i);
    vL = q.dL * q.m_per_cnt_l / Ts;  vR = q.dR * q.m_per_cnt_r / Ts;
    sg(i) = struct('t', q.t, 'v', (vL + vR) / 2, 'r', (vR - vL) / q.b, ...
        'u', (q.uL + q.uR) / 2, 'dl', (q.uR - q.uL) / 2, 'ud', [(q.uR - q.uL) / 2, (q.uL + q.uR) / 2]); %#ok<AGROW>
end
pf = [runs.profile];
pick = @(c) find(pf == c, 1);
for c = 'SPNLD'
    if isempty(pick(c)), error('run_assessment1:missing', 'No complete run of profile %s found.', c); end
end
iS = pick('S'); iP = pick('P'); iN = pick('N'); iL = pick('L'); iD = pick('D');

%% structure selection by validation R2 (free run): delay d, Coulomb term c
longrid = [];
for d = 0:2
    for c = 0:1
        m = fit_lsq(sg(iP).v, sg(iP).u, d, c, 1);
        longrid(end+1, :) = [d c mean([score(m, sg(iS).v, sg(iS).u), score(m, sg(iN).v, sg(iN).u)])]; %#ok<AGROW>
    end
end
[~, k] = max(longrid(:, 3));
lon = fit_lsq(sg(iP).v, sg(iP).u, longrid(k, 1), longrid(k, 2), 1);
arx = fit_lsq(sg(iP).v, sg(iP).u, longrid(k, 1), longrid(k, 2), 2);
latgrid = [];
for d = 0:2
    for e = 0:1
        m = fit_lsq(sg(iD).r, sg(iD).ud(:, 1:1+e), d, 0, 1);
        latgrid(end+1, :) = [d e score(m, sg(iL).r, sg(iL).ud(:, 1:1+e))]; %#ok<AGROW>
    end
end
[~, k] = max(latgrid(:, 3));
lat = fit_lsq(sg(iD).r, sg(iD).ud(:, 1:1+latgrid(k, 2)), latgrid(k, 1), 0, 1);

%% scores
R.lonTrain = score(lon, sg(iP).v, sg(iP).u);
R.lonValS = score(lon, sg(iS).v, sg(iS).u);
R.lonValN = score(lon, sg(iN).v, sg(iN).u);
R.arxValS = score(arx, sg(iS).v, sg(iS).u);
R.arxValN = score(arx, sg(iN).v, sg(iN).u);
R.latTrain = score(lat, sg(iD).r, sg(iD).ud(:, 1:lat.nu));
R.latVal = score(lat, sg(iL).r, sg(iL).ud(:, 1:lat.nu));
[~, R.lonTrain1] = score(lon, sg(iP).v, sg(iP).u);
[~, R.lonValS1] = score(lon, sg(iS).v, sg(iS).u);
[~, R.lonValN1] = score(lon, sg(iN).v, sg(iN).u);
[~, R.latTrain1] = score(lat, sg(iD).r, sg(iD).ud(:, 1:lat.nu));
[~, R.latVal1] = score(lat, sg(iL).r, sg(iL).ud(:, 1:lat.nu));
R.arxVal = mean([R.arxValS R.arxValN]);

%% continuous parameters, sampling-time numbers
lon.tau = -Ts / log(lon.th(1));  lon.K = lon.th(2) / (1 - lon.th(1));
lat.tau = -Ts / log(lat.th(1));  lat.K = lat.th(2) / (1 - lat.th(1));
lat.e = 0;  if lat.nu == 2, lat.e = lat.th(3); end
mq = mean([runs(1).m_per_cnt_l runs(1).m_per_cnt_r]);
vquant = mq / Ts;
sam.vquant_mms = vquant * 1e3;
sam.rquant = mq / Ts / runs(1).b;                % yaw rate per one count of difference
sam.TsOverTau = [Ts / lon.tau, Ts / lat.tau];
sam.tauOverTs = 1 ./ sam.TsOverTau;
sam.f3dB = 1 ./ (2*pi*[lon.tau lat.tau]);
sam.fNyq = 1 / (2*Ts);
sam.nyqOver3dB = sam.fNyq ./ sam.f3dB;

%% cross-validation matrix: train on each run, validate on every other run of the same axis
lab = arrayfun(@(i) sprintf('%s%d', pf(i), sum(pf(1:i) == pf(i))), 1:numel(runs), 'UniformOutput', false);
cv = {};  axname = {'lon', 'lat'};
for ax = 1:2
    ids = find(ismember(pf, 'SPN') == (ax == 1));
    for a = ids
        for b = ids(ids ~= a)
            if ax == 1
                m = fit_lsq(sg(a).v, sg(a).u, lon.d, lon.c, 1);  r2v = score(m, sg(b).v, sg(b).u);
            else
                m = fit_lsq(sg(a).r, sg(a).ud(:, 1:lat.nu), lat.d, 0, 1);  r2v = score(m, sg(b).r, sg(b).ud(:, 1:lat.nu));
            end
            cv(end+1, :) = {axname{ax}, lab{a}, lab{b}, r2v}; %#ok<AGROW>
        end
    end
end

%% path of the lane changes: measurement vs simulation (v from lon model, r from lat model)
for i = [iL iD]
    q = sg(i);
    pm = integrate_path(q.v, q.r, Ts);
    ps = integrate_path(simulate(lon, q.u, q.v), simulate(lat, q.ud(:, 1:lat.nu), q.r), Ts);
    lane(i) = struct('meas', pm, 'sim', ps); %#ok<AGROW>
end

%% console
fprintf('\nData source: %s (%d complete runs)\n', src, numel(runs));
fprintf('Structure selection (free-run R2 on validation):\n  lon: d c R2 =%s\n  lat: d e R2 =%s\n', ...
    sprintf(' [%d %d %.3f]', longrid'), sprintf(' [%d %d %.3f]', latgrid'));
fprintf('\n%-4s %2s %8s %8s %9s %8s %9s %8s\n', 'axis', 'd', 'a', 'b', 'c/e', 'tau[ms]', 'K', 'Ts/tau');
fprintf('%-4s %2d %8.4f %8.4f %9.5f %8.1f %9.4f %8.3f\n', 'lon', lon.d, lon.th(1), lon.th(2), lon.cval, 1e3*lon.tau, lon.K, sam.TsOverTau(1));
fprintf('%-4s %2d %8.4f %8.4f %9.5f %8.1f %9.4f %8.3f\n', 'lat', lat.d, lat.th(1), lat.th(2), lat.e, 1e3*lat.tau, lat.K, sam.TsOverTau(2));
fprintf('\nR2 (free run | one step)\n');
fprintf('  lon train P      %7.4f | %7.4f\n  lon val   S      %7.4f | %7.4f\n  lon val   N      %7.4f | %7.4f\n', ...
    R.lonTrain, R.lonTrain1, R.lonValS, R.lonValS1, R.lonValN, R.lonValN1);
fprintf('  lon ARX2 val S/N %7.4f / %7.4f (1st order: %7.4f / %7.4f)\n', R.arxValS, R.arxValN, R.lonValS, R.lonValN);
fprintf('  lat train D      %7.4f | %7.4f\n  lat val   L      %7.4f | %7.4f\n', R.latTrain, R.latTrain1, R.latVal, R.latVal1);
fprintf('\nCross-validation (free-run R2)\n');
for i = 1:size(cv, 1), fprintf('  %s  train %-3s val %-3s  %7.4f\n', cv{i, 1:3}, cv{i, 4}); end
fprintf('\nSampling time: Ts = %.0f ms, tau = %.0f / %.0f ms (lon/lat) -> %.1f / %.1f samples per tau\n', ...
    1e3*Ts, 1e3*lon.tau, 1e3*lat.tau, sam.tauOverTs);
fprintf('  -3 dB frequency 1/(2 pi tau) = %.2f / %.2f Hz, Nyquist %.0f Hz = %.0f / %.0f x that\n', sam.f3dB, sam.fNyq, sam.nyqOver3dB);
fprintf('  speed quantisation %.2f mm/s per count, yaw-rate quantisation %.4f rad/s per count\n\n', sam.vquant_mms, sam.rquant);

%% figures
t = tiledlayout(new_fig(5.6), 5, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for c = 'SPNLD'
    i = pick(c);
    nexttile(t);  plot(runs(i).t, runs(i).uL, runs(i).t, runs(i).uR);  grid on;  ylabel(sprintf('%s: duty', c));
    ylim([-0.05 0.55]);
end
xlabel(t, 't [s]', 'FontSize', 8);  legend('u_L', 'u_R', 'Box', 'off', 'Location', 'northeast');
save_fig(gcf, figdir, 'fig_inputs');

fit_fig(new_fig(3.8), figdir, 'fig_lon_train', sg(iP), 'v', lon, R.lonTrain, 'P', 'v [m/s]');
fit_fig(new_fig(5.4), figdir, 'fig_lon_val', sg([iS iN]), 'v', lon, [R.lonValS R.lonValN], 'SN', 'v [m/s]');
fit_fig(new_fig(3.8), figdir, 'fig_lat_train', sg(iD), 'r', lat, R.latTrain, 'D', 'r [rad/s]');
fit_fig(new_fig(3.8), figdir, 'fig_lat_val', sg(iL), 'r', lat, R.latVal, 'L', 'r [rad/s]');

new_fig(3.0);
plot(lane(iL).meas.x, lane(iL).meas.y, lane(iL).sim.x, lane(iL).sim.y);  grid on;  axis equal;
xlabel('x [m]');  ylabel('y [m]');  legend('measured', 'simulated', 'Box', 'off', 'Location', 'best');
save_fig(gcf, figdir, 'fig_lat_xy');

%% LaTeX
fid = fopen(fullfile(o.out, 'results.tex'), 'w');
mac = @(n, v) fprintf(fid, '\\newcommand{\\%s}{%s}\n', n, v);
tx = @(x, fmt) strrep(sprintf(fmt, x), '-', '\ensuremath{-}');
fprintf(fid, '%% generated by run_assessment1.m\n');
mac('DataSource', src);            mac('TsMs', tx(1e3*Ts, '%g'));
mac('LonA', tx(lon.th(1), '%.4f'));  mac('LonB', tx(lon.th(2), '%.4f'));  mac('LonC', tx(lon.cval, '%.4f'));
mac('LonDelay', tx(lon.d, '%d'));  mac('LonTauMs', tx(1e3*lon.tau, '%.0f'));  mac('LonK', tx(lon.K, '%.3f'));
mac('LatA', tx(lat.th(1), '%.4f'));  mac('LatB', tx(lat.th(2), '%.4f'));
mac('LatE', tx(lat.e, '%.4f'));  mac('LatDelay', tx(lat.d, '%d'));  mac('LatTauMs', tx(1e3*lat.tau, '%.0f'));  mac('LatK', tx(lat.K, '%.2f'));
mac('RtwoLonTrain', tx(R.lonTrain, '%.3f'));  mac('RtwoLonValStep', tx(R.lonValS, '%.3f'));
mac('RtwoLonValSine', tx(R.lonValN, '%.3f'));  mac('RtwoLatTrain', tx(R.latTrain, '%.3f'));
mac('RtwoLatVal', tx(R.latVal, '%.3f'));  mac('RtwoLonArxTwoVal', tx(R.arxVal, '%.3f'));
mac('VquantMms', tx(sam.vquant_mms, '%.2f'));
kchk = 2 * lon.K / runs(1).b;                       % K_r ~ 2K/b, eq. (lat)
mac('LatKcheck', tx(kchk, '%.2f'));  mac('LatKcheckPct', tx(100 * (lat.K / kchk - 1), '%+.1f'));
mac('TsOverTauLon', tx(sam.TsOverTau(1), '%.2f'));  mac('TsOverTauLat', tx(sam.TsOverTau(2), '%.2f'));
fclose(fid);
fid = fopen(fullfile(o.out, 'cv_table.tex'), 'w');
for i = 1:size(cv, 1), fprintf(fid, '%s & %s & %s & %s \\\\\n', cv{i, 1:3}, tx(cv{i, 4}, '%.3f')); end
fclose(fid);

S = struct('source', src, 'runs', runs, 'lon', lon, 'lat', lat, 'arx2', arx, 'R2', R, ...
    'grid', longrid, 'latgrid', latgrid, 'cv', {cv}, 'sampling', sam, 'lane', lane);
end

%% ---- local functions ----
function m = fit_lsq(y, U, d, c, na)
% y(k+1) = a1 y(k) [+ a2 y(k-1)] + sum_j b_j U_j(k-d) [+ c sign(y(k))], plain linear least squares.
m = struct('d', d, 'c', c, 'na', na, 'nu', size(U, 2), 'th', [], 'cval', 0);
[Phi, k] = regressors(y, U, d, c, na);
m.th = Phi \ y(k + 1);
if c, m.cval = m.th(end); end
end

function [Phi, k] = regressors(y, U, d, c, na)
k = (max(na, d + 1):numel(y) - 1)';
Phi = [y(k), U(k - d, :)];
if na == 2, Phi = [y(k), y(k - 1), U(k - d, :)]; end
if c, Phi = [Phi, sign(y(k))]; end
end

function yh = simulate(m, U, y)
% Free run: only the measured input and the model's own past outputs; start from the measured rest state.
a = m.th(1:m.na);  b = m.th(m.na + (1:m.nu));
yh = zeros(size(y));  yh(1:m.na) = y(1:m.na);
for k = max(m.na, m.d + 1):numel(y) - 1
    yh(k + 1) = a(1) * yh(k) + U(k - m.d, :) * b + m.cval * sign(yh(k));
    if m.na == 2, yh(k + 1) = yh(k + 1) + a(2) * yh(k - 1); end
end
end

function [rf, r1] = score(m, y, U)
% R2 on the excited part (first nonzero of the first input column to end of run): free run and one step ahead.
win = find(U(:, 1) ~= 0, 1):numel(y);
rf = r2(y, simulate(m, U, y), win);
[Phi, k] = regressors(y, U, m.d, m.c, m.na);
y1 = nan(size(y));  y1(k + 1) = Phi * m.th;
r1 = r2(y, y1, intersect(win, k + 1));
end

function r = r2(y, yh, i)
r = 1 - sum((y(i) - yh(i)).^2) / sum((y(i) - mean(y(i))).^2);
end

function p = integrate_path(v, r, Ts)
psi = [0; cumsum(r(1:end-1)) * Ts];
p.x = cumsum(v .* cos(psi)) * Ts;
p.y = cumsum(v .* sin(psi)) * Ts;
end

function fit_fig(f, dir, name, sg, ysel, m, r2s, tag, ylab)
% One tile per run: measured vs free-run simulation (left), input (right, grey).
t = tiledlayout(f, numel(sg), 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:numel(sg)
    y = sg(i).(ysel);
    if ysel == 'v', u = sg(i).u; else, u = sg(i).ud(:, 1:m.nu); end
    nexttile(t);
    yyaxis left;   h = plot(sg(i).t, y, sg(i).t, simulate(m, u, y));  ylabel(ylab);
    yyaxis right;  plot(sg(i).t, u(:, 1), 'Color', [0.6 0.6 0.6]);  ylabel('duty');
    ax = gca;  ax.YAxis(1).Color = 'k';  ax.YAxis(2).Color = [0.4 0.4 0.4];  grid on;
    title(sprintf('%s, R^2 = %.3f', tag(i), r2s(i)), 'FontWeight', 'normal');
    if i == 1, legend(h, 'measured', 'simulated', 'Box', 'off', 'Location', 'best'); end
end
xlabel(t, 't [s]', 'FontSize', 8);
save_fig(f, dir, name);
end

function save_fig(f, dir, name)
exportgraphics(f, fullfile(dir, [name '.pdf']), 'ContentType', 'vector');
exportgraphics(f, fullfile(dir, [name '.png']), 'Resolution', 300);
close(f);
end

function f = new_fig(h)
f = figure('Visible', 'off', 'Units', 'inches', 'Position', [1 1 3.5 h], ...
    'DefaultAxesFontSize', 8, 'DefaultTextFontSize', 8, 'DefaultLineLineWidth', 0.8);
theme(f, 'light');
end

function x = ternary(c, a, b)
if c, x = a; else, x = b; end
end
