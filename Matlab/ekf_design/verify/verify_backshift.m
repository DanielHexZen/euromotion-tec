function R = verify_backshift(cls)
%VERIFY_BACKSHIFT  Pose history and back-shift (REQ-EKF-02, §7.17.1).
%   1. Constant-rate arc, history at 20 ms (50 Hz): the interpolated back-shift
%      for ages 10..100 ms matches the analytic motion.
%   2. A correction applied after the measurement time must NOT appear in the
%      back-shift (finding F-6): with mx_hist_corr = 1 it does not; with the
%      raw-pose history (literal reading) it does.
%   3. Ages > ekf_meas_max_age give ST_RANGE.
if nargin < 1, cls = 'single'; end
R = [];
for corr = [1 0]
    pd = ekf_params();  pd.mx_hist_corr = corr;
    p = ekf_param_cast(pd, cls);
    c = class(p.ekf_kd);
    e = ekf_init(p, 0, 0, 0);
    e.still = 0;  e.t_nochg_us = 0;
    v = 0.3;  w = 1.0;  dt = 0.02;  b = double(p.odo_wheelbase);
    for k = 1:100
        dsl = cast((v - w*b/2)*dt, c);  dsr = cast((v + w*b/2)*dt, c);
        e = ekf_predict(e, p, dsl, dsr, cast(w*dt, c), cast(dt, c), k*20000, 0);
    end
    % 1. interpolation accuracy against the exact arc
    errp = 0;  errt = 0;
    for age = 0.010:0.010:0.100
        [bk, st] = ekf_back_at(e, p, 100*20000 - age*1e6);
        % exact motion over 'age' on the circle the filter integrates (midpoint rule = exact chord heading)
        dth = w*age;
        th1 = double(e.x(3));  th0 = th1 - dth;
        chord = 2*(v/w)*sin(dth/2);
        dx = chord*cos(th0 + dth/2);  dy = chord*sin(th0 + dth/2);
        errp = max(errp, hypot(double(bk(1)) - dx, double(bk(2)) - dy));
        errt = max(errt, abs(double(bk(3)) - dth));
        if st ~= 0, errp = Inf; end
    end
    % 2. correction after the measurement time
    [bk0, ~] = ekf_back_at(e, p, 100*20000 - 50000);
    [e2, ~] = ekf_update_scalar(e, p, cast(0.02, c), cast([0 1 0 0 0], c), cast(1e-6, c), 2);
    [bk1, ~] = ekf_back_at(e2, p, 100*20000 - 50000);
    leak = double(norm(bk1(1:2) - bk0(1:2)));
    if corr == 1
        R = [R, vres('back-shift interpolation, 20 ms history, arc 1 rad/s', double(errp < 1e-4 && errt < 1e-5), ...
            sprintf('%.2g m, %.2g rad', errp, errt), '< 0.1 mm, 1e-5 rad')]; %#ok<AGROW>
        R = [R, vres('back-shift excludes later corrections (F-6)', double(leak < 1e-6), ...
            sprintf('leak %.3g m of a %.3g m correction', leak, double(e2.x(2) - e.x(2))), 'leak 0')]; %#ok<AGROW>
    else
        R = [R, vres('raw-pose history (literal): correction leak', -1, ...
            sprintf('leak %.3g m of %.3g m', leak, double(e2.x(2) - e.x(2))), 'informational')]; %#ok<AGROW>
    end
    fprintf('verify_backshift (hist_corr=%d): interp err %.3g m / %.3g rad, correction leak %.3g m\n', ...
        corr, errp, errt, leak);
end
[~, st] = ekf_back_at(e, p, 100*20000 - 1.2e6);
R = [R, vres('measurement older than 1.0 s discarded', double(st == 2), sprintf('st = %d', st), 'ST_RANGE')];
end
