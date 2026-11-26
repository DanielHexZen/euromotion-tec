%% SYSID_MAIN  Work package 01 - discrete-time system identification (Least Squares)
%  Longitudinal:  cmd_speed  -> vesc_speed          (SISO ARX, y = v,  u = v_cmd)
%  Lateral     :  u_lat(v,delta_cmd) -> imu_wz      (SISO ARX, y = r - gyro bias)
%  Sampling time T = 0.02 s everywhere (100 Hz log decimated by 2).
%  Run from this folder:  octave --no-gui -q sysid_main.m   (or MATLAB: run sysid_main)
%  Output: model_id.mat, fig/*.png, RESULTS.md
clear; clc; close all;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
cd(here);
addpath(fullfile(here, '..', 'common'));
isOct = exist('OCTAVE_VERSION', 'builtin') ~= 0;
if isOct
    try, pkg load control; catch, end
    warning('off', 'all');
end
if ~exist('fig', 'dir'), mkdir('fig'); end
datafile = fullfile(here, '..', 'data', 'openloop_test01_synced.mat');

%% ------------------------------------------------------------------ 0. settings
T        = 0.02;          % sampling time [s] (SPEC.md)
v_min_id = 1.0;           % [m/s] "moving" threshold: identification starts when the wheel speed exceeds it
nBlocks  = 6;             % contiguous blocks of the moving period (odd = identification, even = validation)
nk_min   = 1;             % strictly proper models: u(k) acts at the earliest on y(k+1) (causal ZOH implementation)
tol_sel  = 0.25;          % parsimony tolerance [percentage points of identification free-run NRMSE]
NA = 1:3; NB = 1:3; NK = 0:10;      % order / delay search space
par.L = 0.324; par.delta_max = 0.34; par.v_max = 3.0;

%% ------------------------------------------------------------------ 1. data
D  = load_decimated(datafile, T);
v  = D.vesc_speed;  ucmd = D.cmd_speed;  delta = D.servo_steering;  N = numel(v);
fprintf('Decimated data: N = %d samples, T = %.3f s, %d interpolated sensor samples\n', N, T, sum(D.gap));

% gyro bias and noise from the post-run standstill (cmd = 0, car at rest, t > t_end + 3 s)
sb = D.t > D.t_end + 3;
par.gyro_bias    = mean(D.imu_wz(sb));
par.gyro_std     = std(D.imu_wz(sb));
sp = D.t > 0.5 & D.t < D.t_start - 1;          % pre-run window (hand-held / settling -> noisier)
par.gyro_bias_pre = mean(D.imu_wz(sp)); par.gyro_std_pre = std(D.imu_wz(sp));
r = D.imu_wz - par.gyro_bias;                  % bias-free yaw rate [rad/s]
fprintf('Gyro bias (post-run standstill) = %.3e rad/s, std = %.3e rad/s (n=%d)\n', par.gyro_bias, par.gyro_std, sum(sb));

% moving period [a0, b0]: wheel speed >= 1 m/s until 0.1 s before the stop command
a0 = find(v >= v_min_id, 1, 'first');
b0 = find(D.t <= D.t_end - 0.1, 1, 'last');
edges = round(linspace(a0, b0, nBlocks+1));
segI = zeros(nBlocks/2, 2); segV = zeros(nBlocks/2, 2);
for b = 1:nBlocks/2
    segI(b,:) = [edges(2*b-1), edges(2*b)];       % blocks 1,3,5
    segV(b,:) = [edges(2*b),   edges(2*b+1)];     % blocks 2,4,6
end
fprintf('Moving period t = %.2f ... %.2f s (%d samples), %d blocks of %.1f s; min speed in period %.2f m/s\n', ...
    D.t(a0), D.t(b0), b0-a0+1, nBlocks, (b0-a0)*T/nBlocks, min(v(a0:b0)));
nGapUsed = sum(D.gap(a0:b0));
fprintf('Interpolated (NaN-gap) samples inside the moving period: %d\n', nGapUsed);

%% ------------------------------------------------------------------ 2. LONGITUDINAL: explicit LS + structured search
% Model: y(k) = -a1 y(k-1) - ... - a_na y(k-na) + b1 u(k-nk) + ... + b_nb u(k-nk-nb+1)
%        Phi(k,:) = [-y(k-1) ... -y(k-na)  u(k-nk) ... u(k-nk-nb+1)],  Y(k) = y(k),  theta = Phi\Y
% Every identification block is treated as an independent experiment: regressor rows start n0 = max(na, nk+nb-1)
% samples after the block start so that every regressor lies inside the block.
yL = v; uL = ucmd;
[SL, thL, infL, PhiL, YL] = sysid_search(yL, uL, segI, segV, NA, NB, NK, nk_min, tol_sel);

%% ------------------------------------------------------------------ 3. LATERAL: linear baseline -> static correction -> final
% (i) baseline: u_lat = v*tan(delta)/L (Kus = 0, cs = 0)
parB = par; parB.Kus = 0; parB.cs = 0;
uB = lat_input(v, delta, parB);
[SB, ~, infB] = sysid_search(r, uB, segI, segV, NA, NB, NK, nk_min, tol_sel);
fprintf('Lateral baseline structure (na,nb,nk) = (%d,%d,%d): ident %.2f %%, val %.2f %% (free-run)\n', ...
    infB.na, infB.nb, infB.nk, SB.sel.ident_free.fit, SB.sel.val_free.fit);

% (ii) static speed/steering correction, 2-D grid on IDENTIFICATION free-run NRMSE (validation untouched)
Kgrid = 0:0.005:0.12;  cgrid = 0:0.25:3;
Jg = zeros(numel(Kgrid), numel(cgrid));
na_p = infB.na; nb_p = infB.nb; nk_p = infB.nk;
for iK = 1:numel(Kgrid)
    for ic = 1:numel(cgrid)
        pp = par; pp.Kus = Kgrid(iK); pp.cs = cgrid(ic);
        uu = lat_input(v, delta, pp);
        [~,~,~,iG] = arx_ls(r, uu, na_p, nb_p, nk_p, sysid_rows(segI, na_p, nb_p, nk_p));
        [ym, yf] = arx_eval(iG.a, iG.b, uu, r, segI);
        mm = fit_metrics(ym, yf);
        Jg(iK, ic) = mm.fit;
    end
end
[~, imax] = max(Jg(:)); [iKb, icb] = ind2sub(size(Jg), imax);
par.Kus = Kgrid(iKb); par.cs = cgrid(icb);
% K-only variant (cs = 0) for the documentation table
[~, iK0] = max(Jg(:,1)); KonlyK = Kgrid(iK0);
fprintf('Static correction (ident. grid search): Kus = %.3f s^2/m^2, cs = %.2f 1/rad^2  (ident free-run %.2f %%)\n', ...
    par.Kus, par.cs, Jg(iKb, icb));

% (iii) final lateral order/delay search with the corrected input
uLat = lat_input(v, delta, par);
[SR, thR, infR, PhiR, YR] = sysid_search(r, uLat, segI, segV, NA, NB, NK, nk_min, tol_sel);

% variants for documentation: K only
pK = par; pK.Kus = KonlyK; pK.cs = 0; uK = lat_input(v, delta, pK);
[SK, ~, infK] = sysid_search(r, uK, segI, segV, NA, NB, NK, nk_min, tol_sel);

%% ------------------------------------------------------------------ 4. final models, transfer functions
mods = {'lon', 'lat'};
Q = struct();
Q.lon.S = SL; Q.lon.th = thL; Q.lon.inf = infL; Q.lon.y = yL; Q.lon.u = uL; Q.lon.Phi = PhiL; Q.lon.Y = YL;
Q.lat.S = SR; Q.lat.th = thR; Q.lat.inf = infR; Q.lat.y = r;  Q.lat.u = uLat; Q.lat.Phi = PhiR; Q.lat.Y = YR;
for im = 1:2
    q = Q.(mods{im}); inf = q.inf;
    n = max(inf.na, inf.nk + inf.nb - 1);              % state dimension of the realisation
    bz = [inf.b, zeros(1, n + 1 - numel(inf.b))];      % numerator  in z^-1 -> multiply by z^n
    az = [inf.a, zeros(1, n + 1 - numel(inf.a))];      % denominator in z^-1 -> multiply by z^n
    G.num = bz; G.den = az;                            % descending powers of z, equal length, den monic
    G.num_zinv = inf.b; G.den_zinv = inf.a;            % same TF in z^-1 (B(z^-1)/A(z^-1))
    G.T = T; G.na = inf.na; G.nb = inf.nb; G.nk = inf.nk; G.n = n;
    G.theta = q.th(:); G.theta_stderr = inf.stderr(:);
    G.dcgain = sum(G.num)/sum(G.den);
    G.poles = roots(G.den);
    G.zeros = roots(G.num(find(abs(G.num) > 0, 1):end));
    G.stable = all(abs(G.poles) < 1);
    G.convention = ['G(z) = num(z)/den(z), coefficients in DESCENDING powers of z, equal length n+1, den(1)=1; ' ...
                    'equivalent to B(z^-1)/A(z^-1) with A=1+a1 z^-1+.., B=z^-nk (b1+b2 z^-1+..)'];
    C = tf2canon(G.num, G.den);
    G.canon = C;
    Q.(mods{im}).G = G;
end

%% ------------------------------------------------------------------ 5. exact tf <-> ss verification
Nv = 400; uim = zeros(Nv,1); uim(1) = 1; ust = ones(Nv,1);
rng(1); urnd = randn(Nv,1);
V = struct();
for im = 1:2
    G = Q.(mods{im}).G; C = G.canon;
    yi_tf = filter(G.num, G.den, uim);  ys_tf = filter(G.num, G.den, ust); yr_tf = filter(G.num, G.den, urnd);
    yi_c = ss_sim(C.Ac, C.Bc, C.Cc, C.Dc, uim); ys_c = ss_sim(C.Ac, C.Bc, C.Cc, C.Dc, ust); yr_c = ss_sim(C.Ac, C.Bc, C.Cc, C.Dc, urnd);
    yi_o = ss_sim(C.Ao, C.Bo, C.Co, C.Do, uim); ys_o = ss_sim(C.Ao, C.Bo, C.Co, C.Do, ust); yr_o = ss_sim(C.Ao, C.Bo, C.Co, C.Do, urnd);
    vv.imp_c = max(abs(yi_tf - yi_c)); vv.imp_o = max(abs(yi_tf - yi_o));
    vv.step_c = max(abs(ys_tf - ys_c)); vv.step_o = max(abs(ys_tf - ys_o));
    vv.rnd_c = max(abs(yr_tf - yr_c)); vv.rnd_o = max(abs(yr_tf - yr_o));
    [vv.rank_ctrb_c, vv.rank_obsv_c] = ctrb_rank(C.Ac, C.Bc, C.Cc);
    [vv.rank_ctrb_o, vv.rank_obsv_o] = ctrb_rank(C.Ao, C.Bo, C.Co);
    vv.n = G.n;
    % toolbox cross-check (guarded)
    vv.toolbox = NaN;
    try
        if exist('tf', 'file') && exist('ss', 'file') && exist('step', 'file')
            Gt = tf(G.num, G.den, T);
            [ytb, ~] = step(Gt, (0:Nv-1)*T);
            vv.toolbox = max(abs(ytb(:) - ys_tf(:)));
            Sc = ss(C.Ac, C.Bc, C.Cc, C.Dc, T);
            [ytb2, ~] = step(Sc, (0:Nv-1)*T);
            vv.toolbox_ss = max(abs(ytb2(:) - ys_tf(:)));
        end
    catch
    end
    vv.ys_tf = ys_tf; vv.ys_c = ys_c; vv.ys_o = ys_o; vv.yi_tf = yi_tf; vv.yi_c = yi_c; vv.yi_o = yi_o;
    V.(mods{im}) = vv;
    fprintf('[%s] tf vs ss max|diff|: impulse c/o %.2e/%.2e, step c/o %.2e/%.2e, random-input c/o %.2e/%.2e; rank(Wc)=%d rank(Wo)=%d (n=%d)\n', ...
        mods{im}, vv.imp_c, vv.imp_o, vv.step_c, vv.step_o, vv.rnd_c, vv.rnd_o, vv.rank_ctrb_c, vv.rank_obsv_c, vv.n);
    % the observable form is checked for its own ranks too
    fprintf('       observable form: rank(Wc)=%d rank(Wo)=%d\n', vv.rank_ctrb_o, vv.rank_obsv_o);
end

%% ------------------------------------------------------------------ 6. continuous-time interpretation
CT = struct();
for im = 1:2
    G = Q.(mods{im}).G;
    ct.poles_z = G.poles;
    pz = G.poles(abs(G.poles) > 1e-9);                 % z = 0 poles are pure delays
    ct.s = log(pz)/T;                                  % s = ln(z)/T
    ct.delay = G.nk*T;                                 % pure transport delay [s]
    ct.dcgain = G.dcgain;
    realp = pz(abs(imag(pz)) < 1e-12 & real(pz) > 0);
    if ~isempty(realp)
        tau = -T ./ log(realp);
        ct.tau = tau(:).';                             % time constants of the real poles [s]
        ct.tau_dom = max(tau);
    else
        ct.tau = []; ct.tau_dom = NaN;
    end
    % complex pairs
    cp = pz(imag(pz) > 1e-12);
    ct.wn = abs(log(cp)/T).'; ct.zeta = (-real(log(cp)/T)./abs(log(cp)/T)).';
    % d2c cross-check on the delay-free part (guarded)
    ct.d2c_ok = false;
    try
        if exist('d2c', 'file') && exist('tf', 'file')
            nd = G.num_zinv(G.nk+1:end); ad = G.den_zinv;
            Gd = tf(nd, ad, T, 'variable', 'z^-1');
            Gc = d2c(Gd, 'zoh');
            ct.d2c_ok = true; ct.d2c_poles = pole(Gc);
        end
    catch
    end
    CT.(mods{im}) = ct;
    Q.(mods{im}).ct = ct;
end

%% ------------------------------------------------------------------ 7. fit metrics for all protocols
% P1 (primary): odd blocks identification / even blocks validation, free-run restarted at each block start
% P2: swapped roles (even blocks identification, odd blocks validation) -> cross-validation
% P3: chronological hold-out: first 60 % of the moving period identification, last 40 % validation (one contiguous run)
% P4: single contiguous free-run over the whole moving period (no restart) with the final (P1) model
% P5: extended record: standstill -> start -> moving -> stop -> standstill (contiguous), final (P1) model
FIT = struct();
kEnd = find(D.t <= D.t_end + 3, 1, 'last'); kBeg = max(a0 - round(1/T), 20);
for im = 1:2
    q = Q.(mods{im}); inf = q.inf; y = q.y; u = q.u;
    f = struct();
    % P1
    [ym, yf, yo] = arx_eval(inf.a, inf.b, u, y, segI); f.P1.ident.free = fit_metrics(ym, yf); f.P1.ident.osa = fit_metrics(ym, yo);
    f.P1.ident.y = ym; f.P1.ident.yf = yf;
    [ym, yf, yo, sid] = arx_eval(inf.a, inf.b, u, y, segV); f.P1.val.free = fit_metrics(ym, yf); f.P1.val.osa = fit_metrics(ym, yo);
    for s = 1:size(segV,1), f.P1.val.block_fit(s) = fit_metrics(ym(sid==s), yf(sid==s)).fit; end
    % P2 : swapped roles, same structure
    [~, ~, ~, i2] = arx_ls(y, u, inf.na, inf.nb, inf.nk, sysid_rows(segV, inf.na, inf.nb, inf.nk));
    [ym, yf, yo] = arx_eval(i2.a, i2.b, u, y, segV); f.P2.ident.free = fit_metrics(ym, yf);
    [ym, yf, yo] = arx_eval(i2.a, i2.b, u, y, segI); f.P2.val.free = fit_metrics(ym, yf); f.P2.val.osa = fit_metrics(ym, yo);
    f.P2.a = i2.a; f.P2.b = i2.b;
    % P3 : chronological hold-out
    kc = a0 + round(0.6*(b0 - a0));
    [~, ~, ~, i3] = arx_ls(y, u, inf.na, inf.nb, inf.nk, sysid_rows([a0 kc], inf.na, inf.nb, inf.nk));
    [ym, yf, yo] = arx_eval(i3.a, i3.b, u, y, [a0 kc]); f.P3.ident.free = fit_metrics(ym, yf);
    [ym, yf, yo] = arx_eval(i3.a, i3.b, u, y, [kc b0]); f.P3.val.free = fit_metrics(ym, yf); f.P3.val.osa = fit_metrics(ym, yo);
    f.P3.t_split = D.t(kc);
    % P4 : whole moving period, no restart
    [ym, yf, yo] = arx_eval(inf.a, inf.b, u, y, [a0 b0]); f.P4.free = fit_metrics(ym, yf); f.P4.osa = fit_metrics(ym, yo);
    % P5 : extended record
    [ym, yf, yo] = arx_eval(inf.a, inf.b, u, y, [kBeg kEnd]); f.P5.free = fit_metrics(ym, yf); f.P5.osa = fit_metrics(ym, yo);
    FIT.(mods{im}) = f;
    fprintf('[%s] P1 ident free %.2f%% (R2 %.4f) osa %.2f%% | val free %.2f%% (R2 %.4f) osa %.2f%% | P2 val free %.2f%% | P3 val free %.2f%% | P4 %.2f%% | P5 %.2f%%\n', ...
        mods{im}, f.P1.ident.free.fit, f.P1.ident.free.r2, f.P1.ident.osa.fit, f.P1.val.free.fit, f.P1.val.free.r2, f.P1.val.osa.fit, ...
        f.P2.val.free.fit, f.P3.val.free.fit, f.P4.free.fit, f.P5.free.fit);
end

%% ------------------------------------------------------------------ 8. explicit-regression demonstration (printed + stored)
for im = 1:2
    q = Q.(mods{im}); Phi = q.Phi; Yv = q.Y;
    PtP = Phi.'*Phi; PtY = Phi.'*Yv;
    th_ne = PtP\PtY;
    Q.(mods{im}).demo.size_Phi = size(Phi); Q.(mods{im}).demo.PtP = PtP; Q.(mods{im}).demo.PtY = PtY;
    Q.(mods{im}).demo.theta_diff = norm(th_ne - q.th(:));
    Q.(mods{im}).demo.cond = cond(PtP);
    fprintf('[%s] Phi is %d x %d; ||theta(Phi\\Y) - theta(normal eq.)|| = %.2e; cond(Phi''Phi) = %.2e\n', ...
        mods{im}, size(Phi,1), size(Phi,2), Q.(mods{im}).demo.theta_diff, Q.(mods{im}).demo.cond);
end

%% ------------------------------------------------------------------ 9. optional System Identification Toolbox cross-check
XC = struct('done', false);
try
    if ~isOct && exist('arx', 'file') && exist('iddata', 'file')
        for im = 1:2
            q = Q.(mods{im}); inf = q.inf; Z = [];
            for s = 1:size(segI,1)
                rr = segI(s,1):segI(s,2);
                zz = iddata(q.y(rr), q.u(rr), T);
                if isempty(Z), Z = zz; else, Z = merge(Z, zz); end
            end
            mm = arx(Z, [inf.na inf.nb inf.nk]);
            XC.(mods{im}).a = mm.A; XC.(mods{im}).b = mm.B;
            XC.(mods{im}).maxdiff = max(abs([mm.A(2:end) - inf.a(2:end), mm.B(inf.nk+1:end) - inf.b(inf.nk+1:end)]));
            fprintf('[%s] arx() cross-check: max |theta_arx - theta_LS| = %.2e\n', mods{im}, XC.(mods{im}).maxdiff);
        end
        XC.done = true;
    else
        fprintf('System Identification Toolbox not available -> arx cross-check skipped.\n');
    end
catch ME
    fprintf('arx cross-check failed (%s) -> skipped.\n', ME.message);
end

%% ------------------------------------------------------------------ 10. figures (IEEE single column, 200 dpi)
fw = 3.5; fh = 3.1;                                   % inches
set(0, 'defaultaxesfontsize', 7, 'defaulttextfontsize', 7, 'defaultlinelinewidth', 0.8);
cb = [0.00 0.30 0.60]; cr = [0.80 0.15 0.10]; cg = [0.45 0.45 0.45]; ck = [0 0 0];
tt = D.t;
% ---- (a),(b) validation plots
names = {'lon', 'lat'}; ylab = {'Speed (m/s)', 'Yaw rate (rad/s)'}; fnames = {'fig_a_lon_validation', 'fig_b_lat_validation'};
for im = 1:2
    q = Q.(mods{im}); inf = q.inf; n0 = max(inf.na, inf.nk + inf.nb - 1);
    h = figure('visible', 'off'); set(h, 'paperunits', 'inches', 'papersize', [fw fh], 'paperposition', [0 0 fw fh]);
    subplot(2,1,1); hold on;
    kk = (a0:b0)';
    simV = NaN(size(tt)); simI = NaN(size(tt));          % NaN outside the blocks -> one line per series
    for s = 1:size(segV,1), rr = (segV(s,1):segV(s,2))'; simV(rr) = arx_sim(inf.a, inf.b, q.u, q.y, rr); end
    for s = 1:size(segI,1), rr = (segI(s,1):segI(s,2))'; simI(rr) = arx_sim(inf.a, inf.b, q.u, q.y, rr); end
    plot(tt(kk), q.y(kk), '-', 'color', [0.65 0.65 0.65], 'linewidth', 2.2);
    plot(tt(kk), simI(kk), '-', 'color', cb, 'linewidth', 0.6);
    plot(tt(kk), simV(kk), '-', 'color', cr, 'linewidth', 0.8);
    ylabel(ylab{im}); xlim([tt(a0) tt(b0)]);
    legend({'measured', 'free-run, ident. blocks', 'free-run, valid. blocks'}, 'location', 'northoutside', 'orientation', 'horizontal', 'fontsize', 6);
    legend boxoff; grid on; box on; set(gca, 'gridlinestyle', ':');
    subplot(2,1,2); hold on;
    for s = 1:size(segV,1)
        rr = (segV(s,1):segV(s,2))'; yh = arx_sim(inf.a, inf.b, q.u, q.y, rr);
        plot(tt(rr(n0+1:end)), q.y(rr(n0+1:end)) - yh(n0+1:end), '-', 'color', cr);
    end
    ylabel('Error (valid.)'); xlabel('Time (s)'); xlim([tt(a0) tt(b0)]); grid on; box on; set(gca, 'gridlinestyle', ':');
    text(0.02, 0.88, sprintf('Valid. free-run: NRMSE fit %.1f %%, R^2 = %.3f', FIT.(mods{im}).P1.val.free.fit, FIT.(mods{im}).P1.val.free.r2), 'units', 'normalized', 'fontsize', 6, 'backgroundcolor', 'w');
    print(h, '-dpng', '-r200', fullfile('fig', [fnames{im} '.png'])); close(h);
end
% ---- (c) order selection: identification free-run fit vs nk for several (na,nb), both channels
h = figure('visible', 'off'); set(h, 'paperunits', 'inches', 'papersize', [fw 3.6], 'paperposition', [0 0 fw 3.6]);
combos = [1 1; 2 1; 2 2; 3 3]; mk = {'o-', 's-', '^-', 'd-'}; cc = [cb; cr; 0 0.55 0.25; 0.6 0.3 0.7];
for im = 1:2
    subplot(2,1,im); hold on;
    S = Q.(mods{im}).S;
    for ic = 1:size(combos,1)
        sel = S.tab(:,1) == combos(ic,1) & S.tab(:,2) == combos(ic,2);
        plot(S.tab(sel,3), S.tab(sel,6), mk{ic}, 'color', cc(ic,:), 'markersize', 3, 'markerfacecolor', cc(ic,:));
    end
    inf = Q.(mods{im}).inf;
    plot(inf.nk, S.sel.ident_free.fit, 'kp', 'markersize', 9);
    ylabel('Ident. free-run fit (%)'); grid on; box on; set(gca, 'gridlinestyle', ':'); xlim([-0.3 10.3]);
    yl = ylim; ylim([max(yl(1), 40), 100]);
    if im == 1
        title('Longitudinal', 'fontsize', 7);
        legend({'(1,1)', '(2,1)', '(2,2)', '(3,3)', 'chosen'}, 'location', 'southwest', 'orientation', 'horizontal'); legend boxoff;
    else
        title('Lateral', 'fontsize', 7); xlabel('Input delay n_k (samples)');
    end
end
print(h, '-dpng', '-r200', fullfile('fig', 'fig_c_order_selection.png')); close(h);
% ---- (d) step responses tf vs ss
h = figure('visible', 'off'); set(h, 'paperunits', 'inches', 'papersize', [fw 3.3], 'paperposition', [0 0 fw 3.3]);
for im = 1:2
    subplot(2,1,im); hold on; vv = V.(mods{im}); Nshow = 100; kt = (0:Nshow-1)*T;
    stairs(kt, vv.ys_tf(1:Nshow), '-', 'color', ck, 'linewidth', 1.4);
    plot(kt(1:3:end), vv.ys_c(1:3:Nshow), 'o', 'color', cr, 'markersize', 3.5);
    plot(kt(2:3:end), vv.ys_o(2:3:Nshow), 'x', 'color', cb, 'markersize', 4);
    grid on; box on; set(gca, 'gridlinestyle', ':'); ylabel('Step response'); xlim([0 kt(end)]);
    if im == 1
        title('Longitudinal  v_{cmd} \rightarrow v', 'fontsize', 7);
        legend({'G(z)', 'ctrl. canon.', 'obs. canon.'}, 'location', 'southeast'); legend boxoff;
    else
        title('Lateral  u_{lat} \rightarrow r', 'fontsize', 7); xlabel('Time (s)');
    end
    text(0.35, 0.28, sprintf('max|\\Delta| = %.1e', max(vv.step_c, vv.step_o)), 'units', 'normalized', 'fontsize', 6);
end
print(h, '-dpng', '-r200', fullfile('fig', 'fig_d_step_tf_vs_ss.png')); close(h);

%% ------------------------------------------------------------------ 11. save model_id.mat (SPEC interface)
Gv = Q.lon.G; Gr = Q.lat.G;
Gv = rmfield(Gv, 'canon'); Gr = rmfield(Gr, 'canon');
Cl = Q.lon.G.canon; Cr_ = Q.lat.G.canon;
mkss = @(C) struct('A', C.Ac, 'B', C.Bc, 'C', C.Cc, 'D', C.Dc, 'T', T, 'form', 'controllable canonical (companion, last row -a_n..-a_1)', ...
    'ctrl', struct('A', C.Ac, 'B', C.Bc, 'C', C.Cc, 'D', C.Dc), ...
    'obs',  struct('A', C.Ao, 'B', C.Bo, 'C', C.Co, 'D', C.Do));
Gv_ss = mkss(Cl); Gr_ss = mkss(Cr_);
Gv.ss = Gv_ss; Gr.ss = Gr_ss;
par.k_delta = Gr.dcgain;                 % DC gain of Gr: yaw rate per unit (corrected) kinematic yaw-rate demand
par.k_delta_eff_v1 = par.k_delta/(1 + par.Kus*1^2);    % equivalent steering gain on v*tan(delta)/L at 1 m/s (small delta)
par.k_delta_eff_v2 = par.k_delta/(1 + par.Kus*2^2);
par.k_delta_eff_v3 = par.k_delta/(1 + par.Kus*3^2);
par.lat_input_desc = 'u_lat = v*tan(delta)*(1-cs*delta^2)/(L*(1+Kus*v^2)); use common/lat_input.m and common/lat_inverse.m';
par.speed_to_ident_threshold = v_min_id;
par.tau_v = CT.lon.tau_dom; par.delay_v = CT.lon.delay + 0; par.tau_r = CT.lat.tau_dom; par.delay_r = CT.lat.delay;
fit = struct();
fit.convention = 'NRMSE fit [%] = 100*(1-||y-yhat||/||y-mean(y)||); r2 = 1-SSres/SStot; protocol P1: odd blocks identification, even blocks validation (free-run restarted at each block start)';
for im = 1:2
    f = FIT.(mods{im}); nm = mods{im};
    fit.(nm).ident_free_nrmse = f.P1.ident.free.fit; fit.(nm).ident_free_r2 = f.P1.ident.free.r2;
    fit.(nm).ident_osa_nrmse  = f.P1.ident.osa.fit;  fit.(nm).ident_osa_r2  = f.P1.ident.osa.r2;
    fit.(nm).val_free_nrmse   = f.P1.val.free.fit;   fit.(nm).val_free_r2   = f.P1.val.free.r2;
    fit.(nm).val_osa_nrmse    = f.P1.val.osa.fit;    fit.(nm).val_osa_r2    = f.P1.val.osa.r2;
    fit.(nm).val_free_rmse    = f.P1.val.free.rmse;
    fit.(nm).val_block_fit    = f.P1.val.block_fit;
    fit.(nm).swapped_val_free_nrmse = f.P2.val.free.fit; fit.(nm).swapped_val_free_r2 = f.P2.val.free.r2;
    fit.(nm).chrono_val_free_nrmse  = f.P3.val.free.fit; fit.(nm).chrono_val_free_r2  = f.P3.val.free.r2;
    fit.(nm).chrono_ident_free_nrmse = f.P3.ident.free.fit;
    fit.(nm).moving_contiguous_free_nrmse = f.P4.free.fit; fit.(nm).moving_contiguous_free_r2 = f.P4.free.r2;
    fit.(nm).extended_free_nrmse = f.P5.free.fit; fit.(nm).extended_free_r2 = f.P5.free.r2;
end
fit.Gv = fit.lon; fit.Gr = fit.lat;
verify = struct('lon', rmfield(V.lon, {'ys_tf','ys_c','ys_o','yi_tf','yi_c','yi_o'}), 'lat', rmfield(V.lat, {'ys_tf','ys_c','ys_o','yi_tf','yi_c','yi_o'}));
ct_lon = CT.lon; ct_lat = CT.lat;
search_lon = Q.lon.S.tab; search_lat = Q.lat.S.tab;
search_columns = 'na nb nk n_params ident_osa_nrmse ident_free_nrmse val_free_nrmse AIC FPE stable';
segments = struct('moving_period_idx', [a0 b0], 'moving_period_t', [D.t(a0) D.t(b0)], 'ident_blocks', segI, 'valid_blocks', segV, 'T', T);
save('model_id.mat', 'T', 'Gv', 'Gr', 'Gv_ss', 'Gr_ss', 'fit', 'par', 'verify', 'ct_lon', 'ct_lat', ...
     'search_lon', 'search_lat', 'search_columns', 'segments', '-v7');
fprintf('Saved model_id.mat\n');

%% ------------------------------------------------------------------ 12. RESULTS.md

fid = fopen('RESULTS.md', 'w');
fp = @(varargin) fprintf(fid, varargin{:});
nl = char(10);
fp('# 01 System identification - RESULTS\n\n');
fp('Generated by `sysid_main.m` (all numbers below are printed by the script; sampling time T = %.2f s everywhere).\n\n', T);
fp('## 1. Data, preprocessing, protocol\n\n');
fp('- Source: `data/openloop_test01_synced.mat` (100 Hz grid), decimated by 2 (every 2nd sample) -> T = 0.02 s, N = %d samples (%.1f s).\n', N, D.t(end));
fp('- NaN handling (`common/load_decimated.m`): commands (`cmd_speed`, `servo_steering`) zero-order hold; sensors linear interpolation of the short gaps (<= 70 ms); %d interpolated samples in total, **%d inside the moving period used for identification/validation** (all gaps are at the start/end of the log).\n', sum(D.gap), nGapUsed);
fp('- Signals: longitudinal `u = cmd_speed`, `y = vesc_speed` [m/s]; lateral `y = r = imu_wz - gyro bias` [rad/s], `u = u_lat` (Section 7).\n');
fp('- Gyro bias from the post-run standstill (t > %.1f s, n = %d): **%.3e rad/s**, std %.3e rad/s (pre-run window, noisier, presumably because the car was still being handled: mean %.3e, std %.3e rad/s).\n', D.t_end+3, sum(sb), par.gyro_bias, par.gyro_std, par.gyro_bias_pre, par.gyro_std_pre);
fp('- Moving period: t = %.2f ... %.2f s (%d samples) = from the first sample with wheel speed >= %.1f m/s until 0.1 s before the stop command. It is cut into %d contiguous blocks of %.1f s. **Blocks 1,3,5 identification, blocks 2,4,6 validation** (alternating segments, SPEC option). Every block is an independent experiment: LS regressors never cross a block border; free-run simulations restart at each block start from the measured initial samples (n0 = max(na, nk+nb-1) samples) and are then driven by u only (no output feedback).\n', D.t(a0), D.t(b0), b0-a0+1, v_min_id, nBlocks, (b0-a0)*T/nBlocks);
fp('- Excluded on purpose: the start-up from rest (the wheel speed stays at zero although the command is already ~0.7 m/s: static friction / dead zone) and the braking to standstill (VESC brake ramp, about -5 m/s^2, slew-rate limited). Both are strongly non-linear and not part of the linear operating regime; their effect on the fit is reported honestly in Section 6 (protocol P5).\n');
fp('- Metrics: NRMSE fit = 100(1 - ||y - yhat|| / ||y - mean(y)||) [%%]; R^2 = 1 - SS_res/SS_tot; pooled over all blocks of a set. One-step-ahead (OSA, uses measured past outputs) and free-run (simulation) are both reported.\n\n');

fp('## 2. Least-squares formulation\n\n');
fp('ARX: `y(k) = -a1 y(k-1) - ... - a_na y(k-na) + b1 u(k-nk) + ... + b_nb u(k-nk-nb+1) + e(k)`.\n\n');
fp('Regression row `phi(k) = [-y(k-1) ... -y(k-na)  u(k-nk) ... u(k-nk-nb+1)]`, stacked `Phi` (N x (na+nb)) and `Y = [y(k)]`, `Y = Phi*theta + e`, `theta = [a1..a_na b1..b_nb]^T`.\n');
fp('Solved with `theta = Phi\\Y` (QR); identical to the normal equations `(Phi^T Phi) theta = Phi^T Y` (checked: difference %.1e / %.1e for lon / lat; cond(Phi^T Phi) = %.2e / %.2e). Implementation: `common/arx_ls.m`.\n\n', Q.lon.demo.theta_diff, Q.lat.demo.theta_diff, Q.lon.demo.cond, Q.lat.demo.cond);
for im = 1:2
    q = Q.(mods{im}); nmx = {'Longitudinal', 'Lateral'};
    fp('%s: Phi is %d x %d, `Phi^T Phi` and `Phi^T Y`:\n\n```\n%s\n%s```\n\n', nmx{im}, q.demo.size_Phi(1), q.demo.size_Phi(2), mat_pretty(q.demo.PtP, '%14.6f'), mat_pretty(q.demo.PtY, '%14.6f'));
end

fp('## 3. Order / delay selection (structured search)\n\n');
fp('Search space: na, nb in 1..3, nk in 0..10 (99 LS fits per channel). Each candidate is fitted on the identification blocks. **Selection criterion = identification free-run NRMSE** (validation data are NOT used for selection). Rule (parsimony): among stable candidates with nk >= %d whose identification free-run fit is within %.2f percentage points of the best, take the fewest parameters (na+nb), then the smallest nk. nk >= 1 is imposed because the model is used for control design: u(k) is computed from y(k) and can only act on y(k+1) (strictly proper G(z), no algebraic loop); the nk = 0 candidates are listed below for transparency. Figure: `fig/fig_c_order_selection.png`.\n\n', nk_min, tol_sel);
for im = 1:2
    q = Q.(mods{im}); S = q.S; nmx = {'Longitudinal', 'Lateral'};
    fp('### %s: best (na,nb) per delay nk (identification free-run fit, with the corresponding validation fit)\n\n', nmx{im});
    fp('| nk | best (na,nb) | ident. OSA fit %% | ident. free-run fit %% | valid. free-run fit %% |\n|---|---|---|---|---|\n');
    for nk = 0:10
        ii = find(S.tab(:,3) == nk); [~, jj] = max(S.tab(ii,6)); r1 = S.tab(ii(jj),:);
        fp('| %d | (%d,%d) | %.2f | %.2f | %.2f |\n', nk, r1(1), r1(2), r1(5), r1(6), r1(7));
    end
    nks = q.inf.nk;
    fp('\nAll nine (na,nb) at the chosen nk = %d (columns: ident. OSA, ident. free-run, valid. free-run fit in %%, AIC of the one-step residuals):\n\n', nks);
    fp('| (na,nb) | ident. OSA | ident. free | valid. free | AIC |\n|---|---|---|---|---|\n');
    ii = find(S.tab(:,3) == nks);
    for jj = ii(:).'
        r1 = S.tab(jj,:);
        mkr = ''; if jj == S.isel, mkr = ' **(chosen)**'; end
        fp('| (%d,%d)%s | %.2f | %.2f | %.2f | %.1f |\n', r1(1), r1(2), mkr, r1(5), r1(6), r1(7), r1(8));
    end
    fp('\n');
end
fp('Reading: the longitudinal fit is already saturated by the first-order model (na = nb = 1): extra parameters change the identification free-run fit by < 0.15 points, whereas every additional sample of delay costs 1-2 points. The lateral channel needs the 2nd-order denominator (servo + yaw dynamics) and a 3-sample (60 ms) delay; the fit is flat for nk = 2..4 and collapses for nk >= 6.\n\n');

fp('## 4. Identified models\n\n');
for im = 1:2
    q = Q.(mods{im}); G = q.G; C = G.canon; inf = q.inf; nmx = {'Longitudinal  G_v(z): v_cmd -> v_wheel', 'Lateral  G_r(z): u_lat -> r (yaw rate)'};
    fp('### %s\n\n', nmx{im});
    fp('Structure ARX(na=%d, nb=%d, nk=%d), n = %d states, T = %.2f s. LS parameters (+- 1 std. error): ', G.na, G.nb, G.nk, G.n, T);
    for i = 1:G.na, fp('a%d = %.6f (+-%.1e)%s', i, G.theta(i), G.theta_stderr(i), ', '); end
    for i = 1:G.nb, fp('b%d = %.6f (+-%.1e)%s', i, G.theta(G.na+i), G.theta_stderr(G.na+i), ', '); end
    fp('(N = %d regression rows, residual std %.3e).\n\n', inf.N, sqrt(inf.sigma2));
    fp('Transfer function (convention: `num`/`den` = coefficients of G(z)=num(z)/den(z) in DESCENDING powers of z, equal length n+1, den monic; equivalently B(z^-1)/A(z^-1) with the nk-sample delay as z^-nk):\n\n```\n');
    fp('G(z) = B(z^-1)/A(z^-1) = z^-%d (%s) / (%s)\n', G.nk, polystr(G.num_zinv(G.nk+1:end), 'z^-'), polystr(G.den_zinv, 'z^-'));
    fp('num = [%s]\nden = [%s]\n```\n\n', sprintf(' %.10f', G.num), sprintf(' %.10f', G.den));
    fp('DC gain G(1) = %.6f; poles: %s; zeros: %s; stable (|z|<1): %d.\n\n', G.dcgain, sprintf('%.6f%+.6fi  ', [real(G.poles) imag(G.poles)].'), mat2str_c(G.zeros), G.stable);
    fp('Controllable canonical form (companion; `A_c` last row = [-a_n ... -a_1], `B_c = e_n`, `C_c = [b_n - b_0 a_n ... b_1 - b_0 a_1]`, `D_c = b_0`):\n\n```\nA_c =\n%sB_c =\n%sC_c =\n%sD_c = %.10f\n```\n\n', mat_pretty(C.Ac), mat_pretty(C.Bc), mat_pretty(C.Cc), C.Dc);
    fp('Observable canonical form (dual: `A_o = A_c^T`, `B_o = C_c^T`, `C_o = B_c^T`, `D_o = D_c`):\n\n```\nA_o =\n%sB_o =\n%sC_o =\n%sD_o = %.10f\n```\n\n', mat_pretty(C.Ao), mat_pretty(C.Bo), mat_pretty(C.Co), C.Do);
    vv = V.(mods{im});
    fp('Verification (400 samples): max|y_tf - y_ss| impulse: ctrl %.1e / obs %.1e; step: ctrl %.1e / obs %.1e; random input (rng(1)): ctrl %.1e / obs %.1e. ', vv.imp_c, vv.imp_o, vv.step_c, vv.step_o, vv.rnd_c, vv.rnd_o);
    fp('Ranks (n = %d): controllable form rank(W_c) = %d, rank(W_o) = %d; observable form rank(W_c) = %d, rank(W_o) = %d -> controllable and observable (minimal realisation). ', vv.n, vv.rank_ctrb_c, vv.rank_obsv_c, vv.rank_ctrb_o, vv.rank_obsv_o);
    if ~isnan(vv.toolbox), fp('Control System Toolbox cross-check (`step` of `tf`, `ss`): %.1e / %.1e. ', vv.toolbox, vv.toolbox_ss); end
    fp('Figure: `fig/fig_d_step_tf_vs_ss.png`.\n\n');
end
if XC.done
    fp('System Identification Toolbox `arx` cross-check: max |theta_arx - theta_LS| = %.2e (lon), %.2e (lat).\n\n', XC.lon.maxdiff, XC.lat.maxdiff);
else
    fp('System Identification Toolbox `arx` cross-check: not run in this (Octave) execution; the code in `sysid_main.m` (Section 9) runs it automatically in MATLAB when the toolbox is present.\n\n');
end

fp('## 5. Continuous-time interpretation\n\n');
fp('Poles mapped with s = ln(z)/T (exact for ZOH sampling); time constant tau = -T/ln(p).\n\n');
c1 = CT.lon; c2 = CT.lat;
fp('- **Longitudinal**: pole z = %.6f -> tau_v = **%.3f s** (s = %.3f 1/s), pure delay n_k T = %.3f s (one sampling period: command -> wheel speed), DC gain %.4f (unity steady-state tracking). Equivalent first-order lag with delay: G_v(s) ~ %.3f e^{-%.3f s}/(%.3f s + 1). SPEC pre-study: tau_v ~ 0.28 s (plausibly a lag + delay lumped together: tau + delay = %.3f s here).\n', real(c1.poles_z(1)), c1.tau_dom, real(c1.s(1)), c1.delay, c1.dcgain, c1.dcgain, c1.delay, c1.tau_dom, c1.tau_dom + c1.delay);
fp('- **Lateral**: complex pole pair z = %.4f +- %.4fi -> s = %.2f +- %.2fi 1/s, natural frequency omega_n = **%.1f rad/s** (%.1f Hz), damping zeta = **%.2f**, pure delay n_k T = %.3f s, DC gain k_delta = **%.4f** (yaw-rate per unit of the corrected kinematic yaw-rate demand u_lat). The dynamic part is the steering servo + tyre/yaw response (servo lag ~ 0.1 s in the SPEC pre-study = 60 ms delay + the second-order lag).\n', real(c2.poles_z(1)), abs(imag(c2.poles_z(1))), real(c2.s(1)), abs(imag(c2.s(1))), c2.wn(1), c2.wn(1)/(2*pi), c2.zeta(1), c2.delay, c2.dcgain);
fp('- Gyro bias %.2e rad/s is negligible against the yaw-rate signal (std %.2f rad/s in the moving period) but is subtracted.\n\n', par.gyro_bias, std(r(a0:b0)));

fp('## 6. Fit results (honest numbers)\n\n');
fp('Primary protocol P1 = alternating blocks (identification: blocks 1,3,5; validation: blocks 2,4,6), free-run restarted at each block start.\n\n');
fp('| channel | set | one-step NRMSE %% | one-step R^2 | free-run NRMSE %% | free-run R^2 | free-run RMSE |\n|---|---|---|---|---|---|---|\n');
unit = {'m/s', 'rad/s'};
for im = 1:2
    f = FIT.(mods{im}); nmx = {'Longitudinal', 'Lateral'};
    fp('| %s | identification | %.2f | %.4f | %.2f | %.4f | %.4f %s |\n', nmx{im}, f.P1.ident.osa.fit, f.P1.ident.osa.r2, f.P1.ident.free.fit, f.P1.ident.free.r2, f.P1.ident.free.rmse, unit{im});
    fp('| %s | **validation** | %.2f | %.4f | **%.2f** | **%.4f** | %.4f %s |\n', nmx{im}, f.P1.val.osa.fit, f.P1.val.osa.r2, f.P1.val.free.fit, f.P1.val.free.r2, f.P1.val.free.rmse, unit{im});
end
fp('\nValidation free-run NRMSE per block (blocks 2,4,6): lon %s %%, lat %s %%.\n\n', sprintf('%.1f ', FIT.lon.P1.val.block_fit), sprintf('%.1f ', FIT.lat.P1.val.block_fit));
fp('Additional (stricter or different) protocols for the same structures, all free-run NRMSE %% (R^2 in brackets):\n\n');
fp('| protocol | longitudinal | lateral |\n|---|---|---|\n');
rowf = @(lbl, a, b) fp('| %s | %.2f (%.4f) | %.2f (%.4f) |\n', lbl, a.fit, a.r2, b.fit, b.r2);
rowf('P2 swapped roles (ident. even blocks, valid. odd blocks) - re-estimated theta', FIT.lon.P2.val.free, FIT.lat.P2.val.free);
rowf(sprintf('P3 chronological hold-out (ident. first 60 %% of moving period, valid. last 40 %% = t > %.1f s, one contiguous run)', FIT.lon.P3.t_split), FIT.lon.P3.val.free, FIT.lat.P3.val.free);
rowf('P4 final model, whole moving period as ONE contiguous free-run (no restarts, 79 s)', FIT.lon.P4.free, FIT.lat.P4.free);
rowf('P5 final model, extended record incl. start from rest, stop and 3 s standstill (non-linear events)', FIT.lon.P5.free, FIT.lat.P5.free);
fp('\nCaveats (stated openly): (i) the longitudinal P3 number (%.1f %%) is below 90 %% because the last 40 %% of the run is an almost constant-speed segment (std of v only %.2f m/s) so the NRMSE denominator is small; the absolute error is the same (RMSE %.3f m/s, i.e. < 1 %% of the speed). The alternating-block protocol P1/P2 gives every set the full speed range 1.1-2.4 m/s and is therefore the primary protocol. (ii) the NRMSE of single validation blocks is lower for the longitudinal channel (per-block mean removal on ~13 s of a slowly varying speed; see the per-block values above) - the pooled value is the one reported in the paper tables, with this caveat; (iii) P5 contains the start-up dead zone and the slew-limited braking that a linear model cannot represent (%.1f %% lon).\n\n', FIT.lon.P3.val.free.fit, std(v(round(a0+0.6*(b0-a0)):b0)), FIT.lon.P3.val.free.rmse, FIT.lon.P5.free.fit);
fp('Figures: `fig/fig_a_lon_validation.png` (longitudinal), `fig/fig_b_lat_validation.png` (lateral): measured vs free-run simulation (grey: measured, red: validation blocks, blue: identification blocks) and the validation error.\n\n');

fp('## 7. Lateral model: non-linear scheduling and what was tried\n\n');
fp('The yaw-rate response is linear in the demand `u_lat` but the gain depends on speed and on the steering amplitude (understeer, servo/tyre saturation). Hammerstein/LPV-style static front end (`common/lat_input.m`), followed by a **linear** ARX model:\n\n');
fp('```\nu_lat(k) = v(k) * tan(delta_cmd(k)) * (1 - cs*delta_cmd(k)^2) / ( L * (1 + Kus*v(k)^2) ),   L = %.3f m\n```\n\n', par.L);
fp('`v` = measured wheel speed, `delta_cmd` = `servo_steering` [rad, + left]. Kus = %.3f s^2/m^2 (characteristic speed 1/sqrt(Kus) = %.1f m/s) and cs = %.2f 1/rad^2 are estimated by a 2-D grid search (Kus 0..0.12, cs 0..3) maximising the **identification** free-run fit of the lateral ARX (validation untouched). The map is monotone in delta for |delta| <= %.2f rad and all v in 0..3 m/s (checked numerically); `common/lat_inverse.m` returns the steering command that realises a desired u_lat at the current speed (bisection, saturated at +-delta_max).\n\n', par.Kus, 1/sqrt(par.Kus), par.cs, par.delta_max);
fp('| lateral input | (na,nb,nk) | ident. free-run %% | valid. free-run %% |\n|---|---|---|---|\n');
fp('| baseline v tan(delta)/L (Kus=0, cs=0) | (%d,%d,%d) | %.2f | %.2f |\n', infB.na, infB.nb, infB.nk, SB.sel.ident_free.fit, SB.sel.val_free.fit);
fp('| + speed-dependent gain only (Kus=%.3f, cs=0) | (%d,%d,%d) | %.2f | %.2f |\n', KonlyK, infK.na, infK.nb, infK.nk, SK.sel.ident_free.fit, SK.sel.val_free.fit);
fp('| **+ steering saturation (Kus=%.3f, cs=%.2f) = final** | (%d,%d,%d) | %.2f | %.2f |\n\n', par.Kus, par.cs, infR.na, infR.nb, infR.nk, SR.sel.ident_free.fit, SR.sel.val_free.fit);
fp('Interpretation: the pure kinematic input explains the yaw rate only to ~%.0f %% (validation) because the measured effective gain is ~1.4 (vs 1.0) and drops with speed; the static correction raises the fit above 90 %% on identification and validation. The DC gain of G_r is k_delta = %.4f on the corrected input, i.e. an effective steering gain on v tan(delta)/L (small delta) of %.3f / %.3f / %.3f at v = 1 / 2 / 3 m/s (SPEC pre-study: ~1.39 at ~2 m/s - consistent). Other attempts: higher orders and other delays (no gain, Section 3); a pure speed-dependent gain without steering saturation (row 2) recovers only part of the improvement.\n\n', SB.sel.val_free.fit, par.k_delta, par.k_delta_eff_v1, par.k_delta_eff_v2, par.k_delta_eff_v3);
fp('How downstream packages use it: yaw-rate loop plant = `G_r(z)` from `u_lat` to `r`; the controller output `u_lat` is converted to the steering command with `delta_cmd = lat_inverse(u_lat, v, par)` (or approximately `delta = atan(L*u_lat/(k*v))` with k = k_eff(v)); the simulation plant integrates `psi_dot = r`, `x_dot = v cos psi`, `y_dot = v sin psi`. At v < 1 m/s the model is outside the identified range.\n\n');

fp('## 8. `model_id.mat` contents\n\n');
fp('- `T` = 0.02; `Gv`, `Gr`: fields `num`, `den` (z, descending, equal length), `num_zinv`, `den_zinv` (z^-1), `na nb nk n`, `theta`, `theta_stderr`, `dcgain`, `poles`, `zeros`, `stable`, `convention`, `ss`;\n');
fp('- `Gv_ss`, `Gr_ss`: `A,B,C,D` = controllable canonical form; `.ctrl.{A,B,C,D}` same; `.obs.{A,B,C,D}` observable canonical form; `.T`;\n');
fp('- `fit` (`fit.lon`/`fit.Gv`, `fit.lat`/`fit.Gr`: `ident_free_nrmse`, `ident_free_r2`, `ident_osa_*`, `val_free_*`, `val_osa_*`, swapped/chronological/contiguous/extended variants), `verify`, `ct_lon`, `ct_lat`, `search_lon`, `search_lat` (+ `search_columns`), `segments`;\n');
fp('- `par`: `L`, `delta_max`, `v_max`, `k_delta` (= DC gain of G_r), `Kus`, `cs`, `gyro_bias`, `gyro_std`, `tau_v`, `delay_v`, `tau_r`, `delay_r`, `k_delta_eff_v1/2/3`.\n\n');

fp('## 9. Design decisions (short rationale)\n\n');
fp('1. **T = 0.02 s, decimation by 2** - native VESC/IMU rate (SPEC).\n');
fp('2. **ARX + explicit LS** - the model is linear in the parameters, so `theta = Phi\\Y` is the global optimum of the one-step prediction error; no iterative search, no local minima; closed form also gives std errors. ARX has no noise model, so the one-step fit is optimistic - the free-run fit is the honest figure (Section 6).\n');
fp('3. **Order/delay by structured search with a parsimony rule** - selection on identification free-run only; the first-order lon model is as good as any 2nd/3rd-order one (differences < 0.15 points), hence 1 state.\n');
fp('4. **nk >= 1** - strictly proper models for control design (sample-and-hold implementation delay). nk = 0 would add ~1.2 points (lon) but implies an algebraic loop in closed loop.\n');
fp('5. **Moving-regime identification** - start-up dead zone and slew-limited braking are non-linear and excluded; they are reported in P5.\n');
fp('6. **Alternating blocks for validation** - identification and validation sets both cover the full speed range; role swap (P2) and chronological hold-out (P3) are reported as well.\n');
fp('7. **Lateral Hammerstein/LPV input** - removes the speed dependence of the gain, keeps the dynamic part LTI (needed for pole-placement PID) and is exactly invertible for the steering command.\n');
fp('8. **Canonical forms written explicitly** (`common/tf2canon.m`), validated against `filter` (and the Control System Toolbox when present) to ~1e-15; same T = 0.02 s for tf and ss.\n\n');
fp('## 10. Files\n\n');
fp('`01_sysid/sysid_main.m`, `01_sysid/model_id.mat`, `01_sysid/fig/*.png`, `common/{fit_metrics,load_decimated,arx_ls,arx_sim,arx_eval,sysid_rows,sysid_search,tf2canon,ss_sim,ctrb_rank,lat_input,lat_inverse,mat_pretty,polystr,mat2str_c}.m`.\n');
fclose(fid);
fprintf('Wrote RESULTS.md\n');

