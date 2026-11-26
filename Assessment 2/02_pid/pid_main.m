%% PID_MAIN  Work package 02: pole-placement discrete PID (longitudinal position, lateral CTE)
%  Run from this folder:  octave --no-gui -q pid_main.m   (or  run('pid_main.m')  in MATLAB)
%  Outputs: pid_gains.mat, fig/*.png, RESULTS.md.  All numbers printed here are the ones in RESULTS.md.
clear; close all; clc;
try, pkg load control; catch, end                        % Octave: control package (optional here)
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
cd(here); addpath(here, fullfile(here, '..', 'common'), fullfile(here, '..', '01_sysid'));
if ~exist('fig', 'dir'), mkdir('fig'); end

%% ------------------------------------------------------------------ 1. identified models and specs
M = load(fullfile(here, '..', '01_sysid', 'model_id.mat'));
mdl.T = M.T; mdl.Gv = M.Gv; mdl.Gr = M.Gr; mdl.par = M.par; T = mdl.T; par = mdl.par;
fprintf('T = %.3f s | Gv: a = %.6f, b = %.6f | Gr: a1 = %.6f, a2 = %.6f, b = %.6f, DC gain %.4f\n', T, ...
    -mdl.Gv.den(2), mdl.Gv.num(2), mdl.Gr.den(2), mdl.Gr.den(3), mdl.Gr.num(end), mdl.Gr.dcgain);

spec_targets = struct('zeta', 0.7, 'ts', 2.5, 'os', 10);      % assignment targets
v0 = 2.0;                                                      % lateral design speed [m/s]
spec_lon = struct('zeta', 0.85, 'ts', 1.5, 'kappa', 4, 'umin', 0, 'umax', par.v_max, 'aw', 1);
spec_lat = struct('zeta', 0.75, 'ts', 2.0, 'kappa', 2, 'v0', v0, 'aw', 1);
spec_lat.umax = pid_ulat_limit(v0, par); spec_lat.umin = -spec_lat.umax;

%% ------------------------------------------------------------------ 2. design (analytic pole placement)
gl = pid_design('lon', mdl, spec_lon);
ga = pid_design('lat', mdl, spec_lat);
glon = gl; glat = ga;                      % short names used below
fprintf('\n--- desired poles (z = exp(sT)) ---\n');
for ii = 1:2
    g = {gl, ga}; g = g{ii}; nm = {'lon', 'lat'}; nm = nm{ii};
    s_des = log(g.desired_poles)/T;
    fprintf('[%s] zeta = %.2f, ts_dom = %.2f s -> wn = %.4f rad/s; s-poles: %s\n', nm, g.zeta, g.ts_dom, g.wn, sprintf('%.3f%+.3fi  ', [real(s_des) imag(s_des)].'));
    fprintf('     z-poles: %s\n', sprintf('%.6f%+.6fi  ', [real(g.desired_poles) imag(g.desired_poles)].'));
    fprintf('     Kp = %.5f  Ki = %.5f  Kd = %.5f  pf = %.5f (N = %.2f rad/s)  Nc = [%.5f %.5f %.5f]\n', g.Kp, g.Ki, g.Kd, g.pf, g.N, g.Nc);
    fprintf('     Diophantine residual %.2e, cond %.2e, residual poles of R(z): %s\n', g.dioph_resid, g.dioph_cond, sprintf('%.4f%+.4fi ', [real(g.R_poles) imag(g.R_poles)].'));
end
% closed-form check of the lon gains (reduced second-order formulas)
a_l = -mdl.Gv.den(2); b_l = mdl.Gv.num(2);
dchk = pid_coeffmatch(-(1 + a_l), a_l, b_l*T, gl.desired_poles, T);
fprintf('lon closed-form (pid_coeffmatch) vs Diophantine: max diff %.2e; 3x3 solve check %.2e\n', ...
    max(abs([dchk.Kp dchk.Ki dchk.Kd dchk.pf] - [gl.Kp gl.Ki gl.Kd gl.pf])), dchk.solve_check);

%% ------------------------------------------------------------------ 3. verification of the linear closed loops
t = (0:500)'*T;                                           % 10 s
V = struct();
loops = {gl, ga}; names = {'lon', 'lat'};
for ii = 1:2
    g = loops{ii}; nm = names{ii};
    [Delta, Dc] = pid_clpoly(g);
    pc = roots(Delta);                                      % achieved closed-loop poles (full order)
    [zd, wnd, sdom] = pid_domzeta(pc, T);
    gu = g; gu.umin = -Inf; gu.umax = Inf;
    r1 = ones(size(t));
    pst = pid_prefilt_init(0); rf = zeros(size(t));
    for k = 1:numel(t), [rf(k), pst] = pid_prefilt(r1(k), pst, g); end
    y1 = pid_linsim(g.full_num, g.full_den, gu, r1);        % 1-DOF (no prefilter)
    y2 = pid_linsim(g.full_num, g.full_den, gu, rf);        % with zero-cancelling prefilter
    V.(nm).poles = pc; V.(nm).zeta_dom = zd; V.(nm).wn_dom = wnd; V.(nm).s_dom = sdom;
    V.(nm).y1 = y1; V.(nm).y2 = y2; V.(nm).rf = rf;
    V.(nm).m1 = pid_stepinfo(t, y1, 1); V.(nm).m2 = pid_stepinfo(t, y2, 1);
    % independent check of the derivation: closed-form T_r(z) = Nc(1) Np z^2 / (Ad(z) R(z)) vs simulated difference equations
    denTr = conv(g.desired_poly, g.R); numTr = [zeros(1, numel(denTr) - 3) g.Nc1*g.full_num(end)*[1 0 0]];
    V.(nm).tr_diff = max(abs(filter(numTr, denTr, ones(size(t))) - y2));
    sc = log(pc(abs(pc) > 1e-3))/T; V.(nm).zeta_min_all = min(-real(sc)./abs(sc));
    V.(nm).zeros = roots(g.Nc);
    % pole match: every desired pole must be a closed-loop pole
    dd = zeros(4,1); for kk = 1:4, dd(kk) = min(abs(pc - g.desired_poles(kk))); end
    V.(nm).match = max(dd);
    % cross-check with the Control System Toolbox (if present): poles of feedback(C*G,1)
    V.(nm).cst_diff = NaN;
    try
        Gp = tf(g.full_num, g.full_den, T); Cc = tf(g.Nc, Dc, T);
        pcs = pole(minreal(feedback(Cc*Gp, 1), 1e-8));
        V.(nm).cst_diff = max(arrayfun(@(p) min(abs(pcs - p)), pc(abs(pc) > 1e-3)));
    catch
    end
    fprintf('\n[%s] achieved poles: %s\n', nm, sprintf('%.5f%+.5fi ', [real(pc) imag(pc)].'));
    fprintf('[%s] max distance desired->achieved poles %.2e; dominant: s = %.3f%+.3fi, zeta = %.4f, wn = %.4f; CST cross-check %.2e\n', ...
        nm, V.(nm).match, real(sdom), imag(sdom), zd, wnd, V.(nm).cst_diff);
    fprintf('[%s] closed-form T_r(z) = Nc(1) Np z^2/(Ad R) vs simulated prefiltered step: max diff %.2e; min zeta over all poles %.3f\n', nm, V.(nm).tr_diff, V.(nm).zeta_min_all);
    fprintf('[%s] PID zeros: %s (|z| = %s)\n', nm, sprintf('%.5f%+.5fi ', [real(V.(nm).zeros) imag(V.(nm).zeros)].'), sprintf('%.4f ', abs(V.(nm).zeros)));
    mm = V.(nm).m1; fprintf('[%s] 1-DOF      : OS = %6.2f %%  rise(10-90) = %.3f s  ts(2%%) = %.3f s\n', nm, mm.overshoot, mm.rise, mm.ts);
    mm = V.(nm).m2; fprintf('[%s] prefilter  : OS = %6.2f %%  rise(10-90) = %.3f s  ts(2%%) = %.3f s\n', nm, mm.overshoot, mm.rise, mm.ts);
end

%% ------------------------------------------------------------------ 4. design-space evidence (lat) and naive reduction
fprintf('\n--- lat design space (full model verification, prefilter step) ---\n');
sweep = struct('label', {}, 'zeta', {}, 'kap', {}, 'pf', {}, 'maxp', {}, 'zd', {}, 'os', {}, 'ts', {});
cases = [0.70 2; 0.75 2; 0.80 2; 0.85 2; 0.90 2; 0.75 1.5; 0.75 3; 0.75 4; 0.75 5];
for ic = 1:size(cases, 1)
    sp = spec_lat; sp.zeta = cases(ic,1); sp.kappa = cases(ic,2);
    gg = pid_design('lat', mdl, sp); pcs = roots(pid_clpoly(gg));
    row.label = sprintf('zeta=%.2f kappa=%.1f', cases(ic,1), cases(ic,2)); row.zeta = cases(ic,1); row.kap = cases(ic,2);
    row.pf = gg.pf; row.maxp = max(abs(pcs)); row.zd = pid_domzeta(pcs, T);
    if row.maxp < 1 && gg.pf > 0 && gg.pf < 1
        gu = gg; gu.umin = -Inf; gu.umax = Inf; pst = pid_prefilt_init(0); rff = zeros(size(t));
        for k = 1:numel(t), [rff(k), pst] = pid_prefilt(1, pst, gg); end
        mq = pid_stepinfo(t, pid_linsim(gg.full_num, gg.full_den, gu, rff), 1); row.os = mq.overshoot; row.ts = mq.ts;
    else
        row.os = NaN; row.ts = NaN;
    end
    sweep(end+1) = row; %#ok<SAGROW>
    fprintf('%s: pf = %.3f, max|p| = %.4f, zeta_dom = %.3f, OS = %.2f %%, ts = %.2f s\n', row.label, row.pf, row.maxp, row.zd, row.os, row.ts);
end
% naive design: reduce Gr to its DC gain, exact coefficient matching on G_red = k_delta*v0*T^2/(z-1)^2
dn = pid_coeffmatch(-2, 1, mdl.Gr.dcgain*v0*T^2, ga.desired_poles, T);
gn = ga; gn.Kp = dn.Kp; gn.Ki = dn.Ki; gn.Kd = dn.Kd; gn.pf = dn.pf; gn.Nc = dn.Nc; gn.Nc1 = dn.Nc1;
pcn = roots(pid_clpoly(gn)); [zn, ~] = pid_domzeta(pcn, T);
gnu = gn; gnu.umin = -Inf; gnu.umax = Inf; pst = pid_prefilt_init(0); rfn = zeros(size(t));
for k = 1:numel(t), [rfn(k), pst] = pid_prefilt(1, pst, gn); end
mn = pid_stepinfo(t, pid_linsim(gn.full_num, gn.full_den, gnu, rfn), 1);
szs = log(pcn)/T; zzs = -real(szs)./abs(szs);
naive.Kp = dn.Kp; naive.Ki = dn.Ki; naive.Kd = dn.Kd; naive.pf = dn.pf; naive.maxp = max(abs(pcn)); naive.zmin = min(zzs(abs(pcn) > 1e-3));
naive.zdom = zn; naive.ts = mn.ts; naive.os = mn.overshoot;
fprintf('Naive static reduction: Kp = %.3f Ki = %.3f Kd = %.3f pf = %.3f -> full model max|p| = %.4f, min zeta = %.3f, zeta_dom = %.3f, OS = %.1f %%, ts = %.2f s\n', ...
    naive.Kp, naive.Ki, naive.Kd, naive.pf, naive.maxp, naive.zmin, naive.zdom, naive.os, naive.ts);

%% ------------------------------------------------------------------ 5. robustness (linear)
fprintf('\n--- robustness ---\n');
qgrid = logspace(-1.5, 1.0, 801); R = struct();
for ii = 1:2
    g = loops{ii}; nm = names{ii}; ok = false(size(qgrid));
    Dc = conv([1 -1], [1 -g.pf]);
    for iq = 1:numel(qgrid)
        D = conv(Dc, g.full_den); B = qgrid(iq)*conv(g.Nc, g.full_num); D(end-numel(B)+1:end) = D(end-numel(B)+1:end) + B;
        ok(iq) = max(abs(roots(D))) < 1;
    end
    R.(nm).q_lo = qgrid(find(ok, 1, 'first')); R.(nm).q_hi = qgrid(find(ok, 1, 'last'));
    fprintf('[%s] loop-gain multiplier range with a stable closed loop: %.3f ... %.3f (gain margins: down %.1f dB, up %.1f dB; contiguous = %d)\n', ...
        nm, R.(nm).q_lo, R.(nm).q_hi, 20*log10(1/R.(nm).q_lo), 20*log10(R.(nm).q_hi), all(ok(find(ok,1):find(ok,1,'last'))));
end
% lateral speed dependence: fixed gains (designed at v0) vs scheduled gains (K ~ v0/v)
vv = [1 2 3]; Rv = struct('v', {}, 'fix_zd', {}, 'fix_os', {}, 'fix_ts', {}, 'fix_maxp', {}, 'sch_diff', {});
Yrob_fix = zeros(numel(t), 3);
for iv = 1:3
    q = vv(iv)/v0;
    D = pid_clpoly(ga); Dc = conv([1 -1], [1 -ga.pf]); Dq = conv(Dc, ga.full_den); B = q*conv(ga.Nc, ga.full_num);
    Dq(end-numel(B)+1:end) = Dq(end-numel(B)+1:end) + B; pq = roots(Dq);
    gu = ga; gu.umin = -Inf; gu.umax = Inf;
    y = pid_linsim(q*ga.full_num, ga.full_den, gu, V.lat.rf); Yrob_fix(:, iv) = y; mq = pid_stepinfo(t, y, 1);
    % scheduled: gains scaled by 1/q -> loop transfer function identical -> same polynomial
    gs = pid_sched(ga, vv(iv), 1); Ds = conv(Dc, ga.full_den); Bs = (gs.Kp/ga.Kp)*q*conv(ga.Nc, ga.full_num);
    Ds(end-numel(Bs)+1:end) = Ds(end-numel(Bs)+1:end) + Bs;
    Rv(iv).v = vv(iv); Rv(iv).fix_maxp = max(abs(pq)); Rv(iv).fix_zd = pid_domzeta(pq, T);
    Rv(iv).fix_os = mq.overshoot; Rv(iv).fix_ts = mq.ts; Rv(iv).sch_diff = max(abs(Ds - pid_clpoly(ga)));
    fprintf('[lat, fixed gains] v = %.0f m/s: max|p| = %.4f, zeta_dom = %.3f, OS = %.2f %%, ts = %.2f s | scheduled: char. poly diff %.1e\n', ...
        vv(iv), Rv(iv).fix_maxp, Rv(iv).fix_zd, Rv(iv).fix_os, Rv(iv).fix_ts, Rv(iv).sch_diff);
end
% lon: plant lag time-constant variation (DC gain kept) and plant gain variation
tauf = [0.7 1.0 1.4]; Rl = struct('tau', {}, 'zd', {}, 'os', {}, 'ts', {}); Yrob_lon = zeros(numel(t), 3);
a0 = -mdl.Gv.den(2); b0 = mdl.Gv.num(2);
for it = 1:3
    ap = exp(log(a0)/tauf(it)); bp = (b0/(1 - a0))*(1 - ap);
    nump = bp*T; denp = conv([1 -ap], [1 -1]);
    Dc = conv([1 -1], [1 -gl.pf]); Dq = conv(Dc, denp); B = conv(gl.Nc, nump); Dq(end-numel(B)+1:end) = Dq(end-numel(B)+1:end) + B;
    gu = gl; gu.umin = -Inf; gu.umax = Inf; y = pid_linsim(nump, denp, gu, V.lon.rf); Yrob_lon(:, it) = y; mq = pid_stepinfo(t, y, 1);
    Rl(it).tau = tauf(it); Rl(it).zd = pid_domzeta(roots(Dq), T); Rl(it).os = mq.overshoot; Rl(it).ts = mq.ts;
    fprintf('[lon] lag time constant x%.1f (tau = %.3f s): zeta_dom = %.3f, OS = %.2f %%, ts = %.2f s\n', tauf(it), -T/log(ap), Rl(it).zd, Rl(it).os, Rl(it).ts);
end
% lat: plant gain +-30 % at v0 (scheduled gains)
gfs = [0.7 1.0 1.3]; Rg = struct('gfac', {}, 'zd', {}, 'os', {}, 'ts', {}); Yrob_gain = zeros(numel(t), 3);
for ig = 1:3
    Dc = conv([1 -1], [1 -ga.pf]); Dq = conv(Dc, ga.full_den); B = gfs(ig)*conv(ga.Nc, ga.full_num); Dq(end-numel(B)+1:end) = Dq(end-numel(B)+1:end) + B;
    gu = ga; gu.umin = -Inf; gu.umax = Inf; y = pid_linsim(gfs(ig)*ga.full_num, ga.full_den, gu, V.lat.rf); Yrob_gain(:, ig) = y; mq = pid_stepinfo(t, y, 1);
    Rg(ig).gfac = gfs(ig); Rg(ig).zd = pid_domzeta(roots(Dq), T); Rg(ig).os = mq.overshoot; Rg(ig).ts = mq.ts;
    fprintf('[lat] plant gain x%.1f: zeta_dom = %.3f, OS = %.2f %%, ts = %.2f s\n', gfs(ig), Rg(ig).zd, Rg(ig).os, Rg(ig).ts);
end

%% ------------------------------------------------------------------ 6. anti-windup demonstrations (large steps)
fprintf('\n--- anti-windup ---\n');
awn = {'none', 'back-calculation', 'clamping'}; AW = struct();
Nl = 2001; tl = (0:Nl-1)'*T; step_lon = 20; refl = [0; step_lon*ones(Nl-1, 1)];     % 40 s, 20 m step
for ia = 0:2
    g = gl; g.aw = ia; S = pid_sim_lon(mdl, g, refl, zeros(Nl, 1), struct('prefilt', 0, 's0', 0));
    AW.lon(ia+1).S = S; AW.lon(ia+1).os = (max(S.s) - step_lon)/step_lon*100;
    mq = pid_stepinfo(tl, S.s/step_lon, 1); AW.lon(ia+1).ts = mq.ts; AW.lon(ia+1).maxI = max(abs(S.uun - S.vcmd)); AW.lon(ia+1).sat_time = sum(S.uun > g.umax + 2e-2)*T;
    fprintf('[lon] 20 m step, anti-windup %-17s: overshoot %6.2f %%, ts(2%%) = %.2f s, at upper limit %.2f s, peak |u_unsat - u| = %.1f m/s\n', awn{ia+1}, AW.lon(ia+1).os, AW.lon(ia+1).ts, AW.lon(ia+1).sat_time, AW.lon(ia+1).maxI);
end
Na = 1001; ta = (0:Na-1)'*T; step_lat = 2.0; refa = [0; step_lat*ones(Na-1, 1)];    % 20 s, 2 m offset step, u_lat limit = steering limit at v0
for ia = 0:2
    g = ga; g.aw = ia; [y, u, uun] = pid_linsim(g.full_num, g.full_den, g, refa);
    AW.lat(ia+1).S = struct('ey', y, 'u', u, 'uun', uun); AW.lat(ia+1).os = (max(y) - step_lat)/step_lat*100;
    mq = pid_stepinfo(ta, y/step_lat, 1); AW.lat(ia+1).ts = mq.ts; AW.lat(ia+1).sat_time = sum(abs(uun - u) > 1e-2)*T;
    fprintf('[lat] 2 m step, u_lat limit +-%.3f (|delta| <= %.2f at %.0f m/s), anti-windup %-17s: overshoot %6.2f %%, ts(2%%) = %.2f s, saturated %.2f s\n', ...
        g.umax, par.delta_max, v0, awn{ia+1}, AW.lat(ia+1).os, AW.lat(ia+1).ts, AW.lat(ia+1).sat_time);
end
% same step on the non-linear plant (1.5 m, gains not scheduled): without anti-windup the car circles (heading winds up)
Nb = 801; tb = (0:Nb-1)'*T; refb = [0; 1.5*ones(Nb-1, 1)];
for ia = 0:2
    g = ga; g.aw = ia; S = pid_sim_lat(mdl, g, v0, refb, 0, struct('prefilt', 0, 'sched', 0));
    AW.lat_nl(ia+1).os = (max(S.ey) - 1.5)/1.5*100; AW.lat_nl(ia+1).psimax = max(abs(S.psi)); mq = pid_stepinfo(tb, S.ey/1.5, 1); AW.lat_nl(ia+1).ts = mq.ts;
    fprintf('[lat, non-linear] 1.5 m step @ 2 m/s, anti-windup %-17s: overshoot %7.2f %%, ts = %.2f s, max heading error %.2f rad\n', awn{ia+1}, AW.lat_nl(ia+1).os, AW.lat_nl(ia+1).ts, AW.lat_nl(ia+1).psimax);
end

%% ------------------------------------------------------------------ 7. non-linear plant verification
fprintf('\n--- non-linear plant (identified Gv/Gr + lat_input/lat_inverse + limits) ---\n');
Nn = 601; tn = (0:Nn-1)'*T;                                % 12 s
% (a) longitudinal position step 2 m from standstill, prefilter + anti-windup (limits 0..3 m/s)
NL.lon_step = pid_sim_lon(mdl, gl, [0; 2*ones(Nn-1, 1)], zeros(Nn, 1), struct('prefilt', 1, 's0', 0));
NL.lon_step_m = pid_stepinfo(tn, NL.lon_step.s/2, 1); NL.lon_step_m.vmax = max(NL.lon_step.vcmd); NL.lon_step_m.sat_time = sum(NL.lon_step.uun > gl.umax + 2e-2)*T;
fprintf('[lon] 2 m step: OS = %.2f %%, rise = %.2f s, ts(2%%) = %.2f s, peak vcmd %.2f m/s, saturated %.2f s\n', ...
    NL.lon_step_m.overshoot, NL.lon_step_m.rise, NL.lon_step_m.ts, NL.lon_step_m.vmax, NL.lon_step_m.sat_time);
% (a2) 0.3 m step (no saturation) -> equals linear model
NL.lon_small = pid_sim_lon(mdl, gl, [0; 0.3*ones(Nn-1, 1)], zeros(Nn, 1), struct('prefilt', 1, 's0', 0));
NL.lon_small_m = pid_stepinfo(tn, NL.lon_small.s/0.3, 1);
fprintf('[lon] 0.3 m step (unsaturated): OS = %.2f %%, ts = %.2f s\n', NL.lon_small_m.overshoot, NL.lon_small_m.ts);
% (b) trapezoidal speed profile tracking (reference s_ref(t), v_ref): accel 1 m/s^2 to 2.5 m/s
Nt = 1501; tt = (0:Nt-1)'*T; vr = min(1.0*tt, 2.5); sr = cumsum([0; vr(1:end-1)])*T;
NL.trap_ff = pid_sim_lon(mdl, gl, sr, vr, struct('prefilt', 0, 's0', 0));
NL.trap_nf = pid_sim_lon(mdl, gl, sr, zeros(Nt, 1), struct('prefilt', 0, 's0', 0));
e_ff = sr - NL.trap_ff.s; e_nf = sr - NL.trap_nf.s; iss = tt > 8;
NL.trap.ff_max = max(abs(e_ff)); NL.trap.nf_max = max(abs(e_nf)); NL.trap.ff_ss = max(abs(e_ff(iss))); NL.trap.nf_ss = max(abs(e_nf(iss)));
NL.trap.ff_rmse = sqrt(mean(e_ff.^2)); NL.trap.nf_rmse = sqrt(mean(e_nf.^2));
fprintf('[lon] trapezoid tracking: with v_ref feedforward max|e| = %.4f m (steady %.1e m, RMSE %.4f); without feedforward max|e| = %.4f m (steady %.1e m)\n', ...
    NL.trap.ff_max, NL.trap.ff_ss, NL.trap.ff_rmse, NL.trap.nf_max, NL.trap.nf_ss);
% (c) lateral: 0.5 m path-offset step at v = 2 m/s (reference through prefilter), scheduled gains
refs = [0; 0.5*ones(Nn-1, 1)];
NL.lat_step = pid_sim_lat(mdl, ga, v0, refs, 0, struct('prefilt', 1));
NL.lat_step_m = pid_stepinfo(tn, NL.lat_step.ey/0.5, 1); NL.lat_step_m.dmax = max(abs(NL.lat_step.delta)); NL.lat_step_m.sat_time = sum(abs(NL.lat_step.uun - NL.lat_step.u_cmd) > 1e-2)*T;
fprintf('[lat] 0.5 m reference step @ 2 m/s: OS = %.2f %%, rise = %.2f s, ts(2%%) = %.2f s, max|delta| = %.3f rad (limit %.2f), saturated %.2f s\n', ...
    NL.lat_step_m.overshoot, NL.lat_step_m.rise, NL.lat_step_m.ts, NL.lat_step_m.dmax, par.delta_max, NL.lat_step_m.sat_time);
% (d) lateral: initial offset 0.5 m, reference 0: (i) plain 1-DOF regulation, (ii) prefilter state initialised at the offset
NL.lat_init = pid_sim_lat(mdl, ga, v0, zeros(Nn, 1), 0, struct('prefilt', 0, 'ey0', 0.5));
NL.lat_init2 = pid_sim_lat(mdl, ga, v0, zeros(Nn, 1), 0, struct('prefilt', 1, 'ey0', 0.5, 'pre0', 0.5));
initLabels = {'1-DOF regulation', 'with prefilter (state at offset)'};
for ii = 1:2
    if ii == 1, ey = NL.lat_init.ey; dd = NL.lat_init.delta; else, ey = NL.lat_init2.ey; dd = NL.lat_init2.delta; end
    cross_ov = max(0, -min(ey))/0.5*100; outb = find(abs(ey) > 0.02*0.5); ts_init = tn(outb(end)+1);
    NL.(sprintf('lat_init_m%d', ii)) = struct('undershoot', cross_ov, 'ts', ts_init, 'dmax', max(abs(dd)));
    fprintf('[lat] initial offset 0.5 m, %s: zero-crossing %.2f %% of offset, ts(2%% of offset) = %.2f s, max|delta| = %.3f rad\n', ...
        initLabels{ii}, cross_ov, ts_init, max(abs(dd)));
end
NL.lat_init_m = NL.lat_init_m1;
% (e) constant-curvature path (kappa = 0.25 1/m, R = 4 m) at 2 m/s, with / without curvature feedforward
NL.arc_ff = pid_sim_lat(mdl, ga, v0, zeros(Nn, 1), 0.25, struct('prefilt', 0, 'ff', 1));
NL.arc_nf = pid_sim_lat(mdl, ga, v0, zeros(Nn, 1), 0.25, struct('prefilt', 0, 'ff', 0));
NL.arc.ff_max = max(abs(NL.arc_ff.ey)); NL.arc.nf_max = max(abs(NL.arc_nf.ey));
NL.arc.ff_rmse = sqrt(mean(NL.arc_ff.ey.^2)); NL.arc.nf_rmse = sqrt(mean(NL.arc_nf.ey.^2));
NL.arc.dmax = max(abs(NL.arc_ff.delta));
fprintf('[lat] arc kappa = 0.25 1/m @ 2 m/s: CTE max / RMSE with feedforward %.4f / %.4f m, without %.4f / %.4f m; delta_ss = %.3f rad\n', ...
    NL.arc.ff_max, NL.arc.ff_rmse, NL.arc.nf_max, NL.arc.nf_rmse, NL.arc_ff.delta(end));
% (f) lateral 0.5 m step at v = 1, 2, 3 m/s with scheduled gains
for iv = 1:3
    S = pid_sim_lat(mdl, ga, vv(iv), refs, 0, struct('prefilt', 1, 'sched', 1)); NL.lat_v(iv).S = S;
    NL.lat_v(iv).m = pid_stepinfo(tn, S.ey/0.5, 1); NL.lat_v(iv).dmax = max(abs(S.delta));
    fprintf('[lat, scheduled] v = %.0f m/s, 0.5 m step: OS = %.2f %%, ts = %.2f s, max|delta| = %.3f rad\n', vv(iv), NL.lat_v(iv).m.overshoot, NL.lat_v(iv).m.ts, NL.lat_v(iv).dmax);
end

%% ------------------------------------------------------------------ 8. pass/fail table
chk.lon = struct('zeta', V.lon.zeta_dom, 'os', V.lon.m2.overshoot, 'ts', V.lon.m2.ts, 'rise', V.lon.m2.rise);
chk.lat = struct('zeta', V.lat.zeta_dom, 'os', V.lat.m2.overshoot, 'ts', V.lat.m2.ts, 'rise', V.lat.m2.rise);
fprintf('\n--- targets: zeta >= %.1f, ts <= %.1f s, OS < %.0f %% ---\n', spec_targets.zeta, spec_targets.ts, spec_targets.os);
for ii = 1:2
    c = chk.(names{ii}); pf_ = [c.zeta >= spec_targets.zeta, c.ts <= spec_targets.ts, c.os < spec_targets.os];
    fprintf('%s: zeta %.3f [%d] ts %.2f s [%d] OS %.2f %% [%d]\n', names{ii}, c.zeta, pf_(1), c.ts, pf_(2), c.os, pf_(3));
    if ~all(pf_), error('spec not met for %s', names{ii}); end
end

%% ------------------------------------------------------------------ 9. figures (IEEE column width)
fw = 3.5; set(0, 'defaultaxesfontsize', 7, 'defaulttextfontsize', 7, 'defaultlinelinewidth', 0.9);
c_lon = [0 0.35 0.75]; c_lat = [0.85 0.33 0.1]; c_g = [0.5 0.5 0.5]; c_g2 = [0.15 0.6 0.25];
th = linspace(0, 2*pi, 400);

% --- 9.1 pole-zero map
h = figure('visible', 'off'); set(h, 'paperunits', 'inches', 'papersize', [fw 4.6], 'paperposition', [0 0 fw 4.6]);
for ip = 1:2
    subplot(2, 1, ip); hold on; hh = zeros(1, 6);
    plot(cos(th), sin(th), 'k-', 'linewidth', 0.5);
    ms = 5 + 2*(ip == 2);
    hh(1) = plot(real(V.lon.poles), imag(V.lon.poles), 'o', 'color', c_lon, 'markersize', ms);
    hh(2) = plot(real(gl.desired_poles), imag(gl.desired_poles), 'x', 'color', c_lon, 'markersize', ms + 3);
    hh(3) = plot(real(V.lat.poles), imag(V.lat.poles), 's', 'color', c_lat, 'markersize', ms);
    hh(4) = plot(real(ga.desired_poles), imag(ga.desired_poles), '+', 'color', c_lat, 'markersize', ms + 3);
    hh(5) = plot(real(V.lon.zeros), imag(V.lon.zeros), 'd', 'color', c_lon, 'markersize', 4);
    hh(6) = plot(real(V.lat.zeros), imag(V.lat.zeros), 'd', 'color', c_lat, 'markersize', 4);
    grid on; xlabel('Re(z)'); ylabel('Im(z)');
    if ip == 1
        axis([-1.1 1.1 -1.1 1.1]); axis square; title('Closed-loop poles, full order, unit circle');
        pid_legend(hh, {'lon achieved', 'lon desired', 'lat achieved', 'lat desired', 'lon PID zeros', 'lat PID zeros'}, 'southwest');
    else
        axis([0.78 1.02 -0.08 0.08]); title('Zoom near z = 1 (dominant poles)');
    end
end
print(h, '-dpng', '-r200', fullfile('fig', 'fig_pz_map.png')); close(h);

% --- 9.2 step responses with spec bands
colsLL = {c_lon, c_lat}; ymaxLL = {1.3, 1.7}; ylabLL = {'position [m]', 'cross-track [m]'}; titLL = {'Longitudinal', 'Lateral'};
h = figure('visible', 'off'); set(h, 'paperunits', 'inches', 'papersize', [fw 4.6], 'paperposition', [0 0 fw 4.6]);
for ii = 1:2
    nm = names{ii}; subplot(2, 1, ii); hold on; hh = zeros(1, 5);
    hh(1) = fill([0 10 10 0], [0.98 0.98 1.02 1.02], [0.8 0.9 0.8], 'edgecolor', 'none');
    hh(2) = plot(t, V.(nm).y1, '-', 'color', c_g);
    hh(3) = plot(t, V.(nm).y2, '-', 'color', colsLL{ii}, 'linewidth', 1.2);
    hh(4) = plot([0 10], [1.1 1.1], 'r:', 'linewidth', 0.9); hh(5) = plot([2.5 2.5], [0 1.8], 'r-.', 'linewidth', 0.9);
    axis([0 5 0 ymaxLL{ii}]); grid on; xlabel('time [s]'); ylabel(ylabLL{ii});
    title(sprintf('%s: unit step (OS %.1f%%, t_{ss} %.2f s, \\zeta_{dom} %.2f)', titLL{ii}, V.(nm).m2.overshoot, V.(nm).m2.ts, V.(nm).zeta_dom));
    pid_legend(hh, {'\pm2 % band', '1-DOF PID', 'PID + prefilter', 'OS 10 %', 't = 2.5 s'}, 'southeast');
end
print(h, '-dpng', '-r200', fullfile('fig', 'fig_steps.png')); close(h);

% --- 9.3 anti-windup comparison
h = figure('visible', 'off'); set(h, 'paperunits', 'inches', 'papersize', [fw 5.4], 'paperposition', [0 0 fw 5.4]);
cc = {c_g, c_lon, c_g2}; lab = {'no AW', 'back-calc.', 'clamping'}; lsty = {'-', '-', '-'};
subplot(2, 2, 1); hold on; hh = zeros(1, 4);
for ia = 1:3, hh(ia) = plot(tl, AW.lon(ia).S.s, lsty{ia}, 'color', cc{ia}); end
hh(4) = plot([0 40], [20 20], 'k:'); axis([0 20 0 45]); grid on; xlabel('time [s]'); ylabel('position s [m]'); title('Lon: 20 m step');
pid_legend(hh, [lab {'ref'}], 'northwest');
subplot(2, 2, 3); hold on; hh = zeros(1, 4);
for ia = 1:3, hh(ia) = plot(tl, AW.lon(ia).S.vcmd, lsty{ia}, 'color', cc{ia}); end
hh(4) = plot([0 40], [3 3], 'r:'); plot([0 40], [0 0], 'r:'); axis([0 20 -0.3 3.6]); grid on; xlabel('time [s]'); ylabel('v_{cmd} [m/s]');
pid_legend(hh(4), {'limits'}, 'southeast');
subplot(2, 2, 2); hold on; hh = zeros(1, 4);
for ia = 1:3, hh(ia) = plot(ta, AW.lat(ia).S.ey, lsty{ia}, 'color', cc{ia}); end
hh(4) = plot([0 20], [2 2], 'k:'); axis([0 10 -0.5 4.5]); grid on; xlabel('time [s]'); ylabel('cross-track e_y [m]'); title('Lat: 2 m step, 2 m/s');
pid_legend(hh, [lab {'ref'}], 'northeast');
subplot(2, 2, 4); hold on; hh = zeros(1, 4);
for ia = 1:3, hh(ia) = plot(ta, AW.lat(ia).S.u, lsty{ia}, 'color', cc{ia}); end
hh(4) = plot([0 20], ga.umax*[1 1], 'r:'); plot([0 20], -ga.umax*[1 1], 'r:'); axis([0 10 -1.8 1.8]); grid on; xlabel('time [s]'); ylabel('u_{lat} [rad/s]');
pid_legend(hh(4), {'limits'}, 'southeast');
print(h, '-dpng', '-r200', fullfile('fig', 'fig_antiwindup.png')); close(h);

% --- 9.4 non-linear verification, longitudinal
h = figure('visible', 'off'); set(h, 'paperunits', 'inches', 'papersize', [fw 5.6], 'paperposition', [0 0 fw 5.6]);
subplot(3, 1, 1); hold on; hh = zeros(1, 4);
hh(1) = plot(tn, 2*ones(Nn,1), 'k:'); hh(2) = plot(tn, NL.lon_step.rf, '-', 'color', c_g); hh(3) = plot(tn, NL.lon_step.s, 'color', c_lon);
hh(4) = plot(tn, 2*1.02*ones(Nn,1), 'r:', 'linewidth', 0.6); plot(tn, 2*0.98*ones(Nn,1), 'r:', 'linewidth', 0.6);
grid on; axis([0 5 0 2.4]); xlabel('time [s]'); ylabel('position [m]');
title(sprintf('Lon, 2 m step: OS %.1f%%, t_{ss} %.2f s', NL.lon_step_m.overshoot, NL.lon_step_m.ts));
pid_legend(hh, {'step ref', 'prefiltered ref', 's', '\pm2 % band'}, 'southeast');
subplot(3, 1, 2); hold on; hh = zeros(1, 2);
hh(1) = plot(tn, NL.lon_step.vcmd, 'color', c_lat); hh(2) = plot([0 12], [3 3], 'r:'); plot([0 12], [0 0], 'r:');
grid on; axis([0 5 -0.2 3.6]); xlabel('time [s]'); ylabel('v_{cmd} [m/s]'); pid_legend(hh, {'v_{cmd}', 'limits 0 ... 3 m/s'}, 'northeast');
subplot(3, 1, 3); hold on; hh = zeros(1, 2);
hh(1) = plot(tt, 100*e_nf, '-', 'color', c_g); hh(2) = plot(tt, 100*e_ff, '-', 'color', c_lon);
grid on; xlabel('time [s]'); ylabel('e_s = s_{ref} - s [cm]'); axis([0 12 -5 12]);
title(sprintf('Lon, trapezoid 2.5 m/s: max |e| %.1f cm (FF), %.1f cm (no FF)', 100*NL.trap.ff_max, 100*NL.trap.nf_max));
pid_legend(hh, {'PID only', 'PID + v_{ref} FF'}, 'northeast');
print(h, '-dpng', '-r200', fullfile('fig', 'fig_nonlinear_lon.png')); close(h);

% --- 9.5 non-linear verification, lateral
h = figure('visible', 'off'); set(h, 'paperunits', 'inches', 'papersize', [fw 6.0], 'paperposition', [0 0 fw 6.0]);
subplot(3, 1, 1); hold on; hh = zeros(1, 5);
hh(1) = plot(tn, NL.lat_step.rf, ':', 'color', c_g); hh(2) = plot(tn, NL.lat_step.ey, '-', 'color', c_lat);
hh(3) = plot(tn, NL.lat_init.ey, '-', 'color', c_lon); hh(4) = plot(tn, NL.lat_init2.ey, '-', 'color', c_g2);
hh(5) = plot([0 12], [0.51 0.51], 'r:', 'linewidth', 0.6); plot([0 12], [0.49 0.49], 'r:', 'linewidth', 0.6);
grid on; axis([0 6 -0.35 0.7]); xlabel('time [s]'); ylabel('cross-track e_y [m]');
title(sprintf('Lat @ 2 m/s: 0.5 m ref. step (OS %.1f%%, t_{ss} %.2f s)', NL.lat_step_m.overshoot, NL.lat_step_m.ts));
pid_legend(hh, {'prefiltered ref', 'ref. step', 'offset 0.5 m, 1-DOF', 'offset 0.5 m, prefilter', '\pm2 % band'}, 'southeast');
subplot(3, 1, 2); hold on; hh = zeros(1, 4);
hh(1) = plot(tn, NL.lat_step.delta, '-', 'color', c_lat); hh(2) = plot(tn, NL.lat_init.delta, '-', 'color', c_lon); hh(3) = plot(tn, NL.lat_init2.delta, '-', 'color', c_g2);
hh(4) = plot([0 12], par.delta_max*[1 1], 'r:'); plot([0 12], -par.delta_max*[1 1], 'r:');
grid on; axis([0 6 -0.4 0.4]); xlabel('time [s]'); ylabel('\delta [rad]'); pid_legend(hh, {'ref. step', 'offset, 1-DOF', 'offset, prefilter', '|\delta| limit'}, 'southeast');
subplot(3, 1, 3); hold on; hh = zeros(1, 2);
hh(1) = plot(tn, 100*NL.arc_nf.ey, '-', 'color', c_g); hh(2) = plot(tn, 100*NL.arc_ff.ey, '-', 'color', c_lat);
yl = 100*max(NL.arc.nf_max, NL.arc.ff_max)*1.15; grid on; axis([0 8 -yl yl]); xlabel('time [s]'); ylabel('e_y [cm]');
title(sprintf('Arc \\kappa = 0.25 1/m @ 2 m/s: max e_y %.1f cm (FF), %.1f cm (no FF)', 100*NL.arc.ff_max, 100*NL.arc.nf_max));
pid_legend(hh, {'PID only', 'PID + curvature FF'}, 'southeast');
print(h, '-dpng', '-r200', fullfile('fig', 'fig_nonlinear_lat.png')); close(h);

% --- 9.6 robustness
h = figure('visible', 'off'); set(h, 'paperunits', 'inches', 'papersize', [fw 5.4], 'paperposition', [0 0 fw 5.4]);
cv = {[0.1 0.45 0.8], [0.85 0.33 0.1], [0.2 0.6 0.25]}; lsv = {'-', '-', '-'};
subplot(2, 2, 1); hold on; hh = zeros(1, 3); for iv = 1:3, hh(iv) = plot(t, Yrob_fix(:, iv), lsv{iv}, 'color', cv{iv}); end
plot([0 10], [1.1 1.1], 'r:'); axis([0 8 0 1.5]); grid on; xlabel('time [s]'); ylabel('e_y [m]'); title('Lat, fixed gains (lin.)');
pid_legend(hh, {'v = 1 m/s', 'v = 2 m/s', 'v = 3 m/s'}, 'southeast');
subplot(2, 2, 2); hold on; for iv = 1:3, plot(tn, NL.lat_v(iv).S.ey/0.5, lsv{iv}, 'color', cv{iv}); end
plot([0 12], [1.1 1.1], 'r:'); axis([0 8 0 1.5]); grid on; xlabel('time [s]'); ylabel('e_y / step'); title('Lat, scheduled (non-lin.)');
subplot(2, 2, 3); hold on; hh = zeros(1, 3); for it = 1:3, hh(it) = plot(t, Yrob_lon(:, it), lsv{it}, 'color', cv{it}); end
plot([0 10], [1.1 1.1], 'r:'); axis([0 5 0 1.5]); grid on; xlabel('time [s]'); ylabel('s [m]'); title('Lon, plant lag \tau');
pid_legend(hh, {'\tau x0.7', '\tau x1.0', '\tau x1.4'}, 'southeast');
subplot(2, 2, 4); hold on; hh = zeros(1, 3); for ig = 1:3, hh(ig) = plot(t, Yrob_gain(:, ig), lsv{ig}, 'color', cv{ig}); end
plot([0 10], [1.1 1.1], 'r:'); axis([0 8 0 1.5]); grid on; xlabel('time [s]'); ylabel('e_y [m]'); title('Lat, plant gain');
pid_legend(hh, {'x0.7', 'x1.0', 'x1.3'}, 'southeast');
print(h, '-dpng', '-r200', fullfile('fig', 'fig_robust.png')); close(h);

%% ------------------------------------------------------------------ 10. outputs: pid_gains.mat
form = ['parallel PID, u(k) = Kp e(k) + I(k) + D(k); I(k) = I(k-1) + Ki T e(k) (backward Euler); ' ...
        'D(k) = pf D(k-1) + Kd (1-pf)/T (e(k)-e(k-1)), pf = 1/(1+N T) (filtered backward difference); ' ...
        'C(z) = Kp + Ki T z/(z-1) + Kd(1-pf)/T (z-1)/(z-pf); T = 0.02 s'];
pid_lon = pid_pack(gl, V.lon, chk.lon, form, 'lon');
pid_lat = pid_pack(ga, V.lat, chk.lat, form, 'lat');
pid_lon.feedforward = 'v_cmd = sat(v_ref + PID(s_ref - s)); v_ref = reference speed along the path (s_ref integrates v_ref)';
pid_lat.feedforward = 'u_lat = sat(v*kappa_ref/k_delta + PID(e_ref - e_y)); k_delta = Gr(1) (DC gain of the identified yaw-rate model); delta = lat_inverse(u_lat, v)';
pid_lat.k_delta = mdl.Gr.dcgain;
pid_lat.sched = 'Kp, Ki, Kd scaled by v0/max(v,1) (pid_sched); loop transfer function then independent of v';
pid_lat.delta_max = par.delta_max; pid_lat.ulat_limit_v0 = spec_lat.umax;
pid_lat.ulat_limit_note = 'u_lat limit = lat_input(v, delta_max, par) = pid_ulat_limit(v, par), symmetric';
pid_lon.robust = struct('gain_range', [R.lon.q_lo R.lon.q_hi], 'tau', Rl);
pid_lat.robust = struct('gain_range', [R.lat.q_lo R.lat.q_hi], 'fixed_gain_v', Rv, 'gain_var', Rg);
pid_lon.nonlinear = struct('step2m', NL.lon_step_m, 'trapezoid', NL.trap);
pid_lat.nonlinear = struct('step05', NL.lat_step_m, 'init_offset', NL.lat_init_m, 'arc', NL.arc);
spec = struct('targets', spec_targets, 'lon', spec_lon, 'lat', spec_lat); %#ok<NASGU>
design_space_lat = sweep; naive_static_reduction = naive; antiwindup = struct('lon', rmfield(AW.lon, 'S'), 'lat', rmfield(AW.lat, 'S')); %#ok<NASGU>
save('-v7', 'pid_gains.mat', 'pid_lon', 'pid_lat', 'spec', 'T', 'design_space_lat', 'naive_static_reduction', 'antiwindup');
fprintf('\nSaved pid_gains.mat\n');

%% ------------------------------------------------------------------ 11. RESULTS.md
pid_write_results(mdl, spec_targets, spec_lon, spec_lat, gl, ga, V, chk, sweep, naive, R, Rv, Rl, Rg, AW, NL, dchk);
fprintf('Saved RESULTS.md, figures in fig/. Done.\n');
