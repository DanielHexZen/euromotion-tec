function [R, out] = verify_float32(seed)
%VERIFY_FLOAT32  float32 (firmware) versus float64 filter on identical inputs.
%   1. maneuver (open loop, identical sensor data): max state difference single vs double;
%      line40 and rect10 closed loop: final errors of both precisions; P positive definite
%   2. float32 yaw integral at large |Psi| (finding F-7): stationary ZUPT bias
%      estimate when the accumulated Psi starts at 0, 100 and 1000 rad
%   The 1e6-step symmetry/PD test is part of verify_static.
if nargin < 1, seed = 3; end
R = [];
scen = {'maneuver', 'line40', 'rect10'};
for i = 1:3
    switch i
        case 1
            a = sim_config('maneuver', seed, 'cls', 'single');
            b = sim_config('maneuver', seed, 'cls', 'double');
        case 2
            a = sim_config('line40', seed, 'truth', 'spec_line', 'gnd', 2, 'cls', 'single');
            b = sim_config('line40', seed, 'truth', 'spec_line', 'gnd', 2, 'cls', 'double');
        otherwise
            a = sim_config('rect10', seed, 'cls', 'single');
            b = sim_config('rect10', seed, 'cls', 'double');
    end
    La = sim_run(a);  Lb = sim_run(b);
    n = min(La.n, Lb.n);
    d = La.est(1:n, 1:5) - Lb.est(1:n, 1:5);
    d(:,3) = d(:,3) - 2*pi*round(d(:,3)/(2*pi));
    dp = max(hypot(d(:,1), d(:,2)));
    pdok = true;
    for k = 1:50:n
        [~, f] = chol(reshape(La.est(k, 34:58), 5, 5));
        pdok = pdok && f == 0;
    end
    Ma = sim_metrics(La);  Mb = sim_metrics(Lb);
    ea = hypot(Ma.e(end,1), Ma.e(end,2));  eb = hypot(Mb.e(end,1), Mb.e(end,2));
    out.(scen{i}) = struct('dpos', dp, 'dth', max(abs(d(:,3))), 'dbg', max(abs(d(:,4))), ...
        'ds', max(abs(d(:,5))), 'err_single', ea, 'err_double', eb);
    if i == 1
        % open loop: identical sensor data, so the difference is the float32 effect alone
        R = [R, vres('float32 vs float64, identical inputs (maneuver)', double(dp < 1e-3 && pdok), ...
            sprintf('max dpos %.3g mm, dth %.3g mdeg, P PD %d', 1e3*dp, 1e3*max(abs(d(:,3)))*180/pi, pdok), ...
            '< 1 mm, P PD')]; %#ok<AGROW>
    else
        % closed loop: guidance acts on different estimates, the paths separate a little
        R = [R, vres(['float32 vs float64 closed loop ' scen{i}], double(abs(ea - eb) < 0.01 && pdok), ...
            sprintf('final err %.1f / %.1f mm, max dpos %.3g mm', 1e3*ea, 1e3*eb, 1e3*dp), ...
            'final errors within 10 mm, P PD')]; %#ok<AGROW>
    end
    fprintf('verify_float32 %s: max dpos %.3g mm, dth %.3g deg, dbg %.3g, ds %.3g; final err single %.4f double %.4f m\n', ...
        scen{i}, 1e3*dp, max(abs(d(:,3)))*180/pi, max(abs(d(:,4))), max(abs(d(:,5))), ea, eb);
end
% 2. yaw integral precision
offs = [0 100 1000];
for i = 1:3
    for cl = {'single', 'double'}
        p = ekf_param_cast(ekf_params(), cl{1});
        be = zupt_bias(p, offs(i), 20, 31);
        out.psi(i).(cl{1}) = be;
    end
    R = [R, vres(sprintf('float32 Psi offset %g rad: ZUPT bias error', offs(i)), ...
        -1 + 2*double(offs(i) == 0)*double(abs(out.psi(i).single) < 1e-4), ...
        sprintf('%.2e rad/s (double %.2e)', out.psi(i).single, out.psi(i).double), ...
        'informational except 0 rad (< 1e-4)')]; %#ok<AGROW>
    fprintf('Psi offset %6g rad: bias error single %.3e, double %.3e rad/s\n', offs(i), ...
        out.psi(i).single, out.psi(i).double);
end
end

function be = zupt_bias(p, off, Tend, seed)
c  = class(p.ekf_kd);
e  = ekf_init(p, 0, 0, 0);
r  = imu_yaw_init(p);
rs = sim_rng_init(seed, 1);
bg = 0.5*pi/180;  nd = 1e-4;  fi = double(p.imu_odr_hz);  t0 = 1/(2*lcm(1000, round(fi)));  i = 0;
first = true;  have = 0;  psi_prev = cast(0, c);
for k = 0:round(Tend*1000)
    t = k*1e-3;
    while t0 + i/fi < t
        [rs, z] = sim_randn(rs);
        r = imu_yaw_push(r, p, round((t0 + i/fi)*1e6), 0, bg + nd*sqrt(fi)*z, 0, 0);
        if first
            r.psi(r.head) = cast(off, c);   % accumulated Psi of a long session
            first = false;
        end
        i = i + 1;
    end
    if mod(k - 1, 20) == 0 && k > 1
        [psi, st] = imu_yaw_integral_at(r, p, k*1000);
        if st == 0 && have
            dpsi = psi - psi_prev;
            [e, zd, rm, T] = ekf_still_update(e, p, 0, 1, dpsi, 20000);
            e = ekf_predict(e, p, cast(0, c), cast(0, c), dpsi, cast(0.02, c), k*1000, 0);
            if zd > 0, e = ekf_update_zupt(e, p, rm, T); end
        end
        if st == 0, psi_prev = psi; have = 1; end
    end
end
be = double(e.x(4)) - bg;
end
