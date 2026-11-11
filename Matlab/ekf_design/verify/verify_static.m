function [R, out] = verify_static(cls, n_long)
%VERIFY_STATIC  t_ekf_static (§14.1, REQ-EKF-01/05/09) at the D-30 rate (50 Hz).
%
%   [R, out] = verify_static(cls, n_long)
%   cls     'single' (default) or 'double'
%   n_long  number of estimator steps of the long symmetry/PD test (default 1e6)
%
%   Part 1: 60 s stationary, true bias 0.5 deg/s, gyro white noise ekf_gyro_nd at
%   p.imu_odr_hz (833 Hz) through the real yaw-integral ring, quantised to the LSB, EKF at 50 Hz
%   with ZUPT. Pass: sigma_bg < 0.02 deg/s after 3 s; heading drift < 0.01 deg.
%   Run three times: as designed, with the literal spec start (first 0.2 s
%   counted as motion, finding F-2) and with the literal spec F(3,4) = -dt
%   while still (finding F-1).
%   Part 2: n_long steps of predict(still) + ZUPT every second in the chosen
%   precision: P exactly symmetric, positive definite (Cholesky), finite.
if nargin < 1, cls = 'single'; end
if nargin < 2, n_long = 1e6; end
p0 = ekf_params();
R = [];
variants = {'design', 'spec start (F-2)', 'spec F still (F-1)'};
for v = 1:3
    p = p0;
    if v == 2, p.mx_still_init = 0; end
    if v == 3, p.mx_still_F_bg = 1; end
    o = static_run(ekf_param_cast(p, cls), 60, 0.5*pi/180, 11);
    out.var(v) = o;
    if v == 1
        R = [R, vres('t_ekf_static sigma_bg after 3 s', double(o.sig_bg_3s < 0.02), ...
            sprintf('%.4f deg/s', o.sig_bg_3s), '< 0.02 deg/s')]; %#ok<AGROW>
        R = [R, vres('t_ekf_static heading drift 60 s', double(o.drift < 0.01), ...
            sprintf('%.5f deg', o.drift), '< 0.01 deg')]; %#ok<AGROW>
        R = [R, vres('t_ekf_static bias error after 60 s', double(abs(o.bg_err) < 3*o.sig_bg_end), ...
            sprintf('%.2e deg/s (%.1f sigma)', o.bg_err, o.bg_err/o.sig_bg_end), '< 3 sigma')]; %#ok<AGROW>
    else
        R = [R, vres(['heading drift, ' variants{v}], -1, sprintf('%.5f deg', o.drift), ...
            'informational (literal spec text)')]; %#ok<AGROW>
    end
    fprintf('verify_static [%s, %s]: sigma_bg(3 s) %.4f deg/s, drift %.5f deg, bias err %.2e deg/s\n', ...
        variants{v}, cls, o.sig_bg_3s, o.drift, o.bg_err);
end
% part 2: long run
p  = ekf_param_cast(p0, cls);
e  = ekf_init(p, 0, 0, 0);
rs = sim_rng_init(77, 1);
c  = class(p.ekf_kd);
dt = cast(0.02, c);
bg = 0.5*pi/180;
minev = Inf;  asym = 0;  ok = true;
for k = 1:n_long
    [rs, z] = sim_randn(rs);
    dpsi = cast(bg*0.02 + p0.ekf_gyro_nd*sqrt(0.02)*z, c);
    [e, zd, rm, T] = ekf_still_update(e, p, 0, 1, dpsi, 20000);
    e = ekf_predict(e, p, cast(0, c), cast(0, c), dpsi, dt, k*20000, 0);
    if zd > 0
        e = ekf_update_zupt(e, p, rm, T);
    end
    if mod(k, 50000) == 0 || k == n_long
        asym = max(asym, max(max(abs(e.P - e.P'))));
        [~, f] = chol(double(e.P));
        ev = eig(double(e.P));
        minev = min(minev, min(ev));
        ok = ok && f == 0 && all(isfinite(e.P(:))) && e.fault == 0;
    end
end
out.long_minev = minev;  out.long_asym = asym;  out.long_P = e.P;
R = [R, vres(sprintf('t_ekf_static P sym+PD after %g steps (%s)', n_long, cls), ...
    double(ok && asym == 0), sprintf('asym %.1g, min eig %.3g', asym, minev), 'symmetric, chol OK')];
fprintf('verify_static long run (%s, %g steps): asym %.3g, min eig %.3g, Pbb %.3g, faults %d\n', ...
    cls, n_long, asym, minev, double(e.P(4,4)), e.n_fault);
end

function o = static_run(p, Tend, bg, seed)
% gyro at p.imu_odr_hz -> yaw ring -> EKF at 50 Hz, stationary
c   = class(p.ekf_kd);
e   = ekf_init(p, 0, 0, 0);
r   = imu_yaw_init(p);
rs  = sim_rng_init(seed, 1);
nd  = double(p.ekf_gyro_nd);
lsb = double(p.imu_gyro_lsb);
fi  = double(p.imu_odr_hz);  t0 = 1/(2*lcm(1000, round(fi)));  i = 0;
K   = round(Tend*1000);
psi_prev = cast(0, c);  have = 0;
o.sig_bg_3s = NaN;
th0 = double(e.x(3));
for k = 0:K
    t = k*1e-3;
    while t0 + i/fi < t
        [rs, z] = sim_randn(rs);
        w = round((bg + nd*sqrt(fi)*z)/lsb)*lsb;
        r = imu_yaw_push(r, p, round((t0 + i/fi)*1e6), 0, w, 0, 0);
        i = i + 1;
    end
    if mod(k - 1, 20) == 0 && k >= 1
        tu = k*1000;
        [psi, st] = imu_yaw_integral_at(r, p, tu);
        if st == 0 && have
            dpsi = psi - psi_prev;
            [e, zd, rm, T] = ekf_still_update(e, p, 0, 1, dpsi, 20000);
            e = ekf_predict(e, p, cast(0, c), cast(0, c), dpsi, cast(0.02, c), tu, 0);
            if zd > 0, e = ekf_update_zupt(e, p, rm, T); end
        end
        if st == 0, psi_prev = psi; have = 1; end
        if isnan(o.sig_bg_3s) && t >= 3.0
            o.sig_bg_3s = sqrt(double(e.P(4,4)))*180/pi;
        end
    end
end
o.drift = abs(double(e.x(3)) - th0)*180/pi;
o.bg_err = (double(e.x(4)) - bg)*180/pi;
o.sig_bg_end = sqrt(double(e.P(4,4)))*180/pi;
o.nis_zupt = e.nis_sum(1)/max(e.n_acc(1), 1);
end
