function D = load_decimated(matfile, T)
%LOAD_DECIMATED  Load the 100 Hz open-loop log, clean NaNs and decimate to T.
%   D = load_decimated(matfile, T) with T an integer multiple of the 0.01 s grid
%   (T = 0.02 s -> keep every 2nd sample).
%   NaN handling:
%     * command / actuator signals (cmd_speed, servo_steering): zero-order hold
%       (a NaN means "no new message", the last command stays active);
%     * sensor signals (vesc_speed, imu_*, pf_*): the gaps are all very short
%       (<= 7 samples at 100 Hz = 70 ms) and are linearly interpolated BEFORE
%       decimation; D.gap flags the samples that were interpolated.
%   Output fields (column vectors, decimated): t, cmd_speed, servo_steering,
%   vesc_speed, imu_ax, imu_ay, imu_wz, imu_yaw, pf_x, pf_y, pf_yaw, gap,
%   plus scalars T, L, t_start, t_end (driving period from the file segments).
S = load(matfile);
dt0 = 0.01;
dec = round(T/dt0);
if abs(dec*dt0 - T) > 1e-9, error('T must be a multiple of 0.01 s'); end
cmdNames = {'cmd_speed','servo_steering'};
senNames = {'vesc_speed','imu_ax','imu_ay','imu_wz','imu_yaw','pf_x','pf_y','pf_yaw'};
n = numel(S.t);
idx = (1:n).';
gap = false(n,1);
for i = 1:numel(cmdNames)
    x = S.(cmdNames{i}); x = x(:);
    last = NaN;
    for k = 1:n
        if isnan(x(k)), x(k) = last; else, last = x(k); end
    end
    x(isnan(x)) = 0;                         % leading NaNs (none in this log)
    D.(cmdNames{i}) = x;
end
for i = 1:numel(senNames)
    x = S.(senNames{i}); x = x(:);
    bad = isnan(x);
    gap = gap | bad;
    if any(bad) && sum(~bad) > 2
        x(bad) = interp1(idx(~bad), x(~bad), idx(bad), 'linear', 'extrap');
    end
    D.(senNames{i}) = x;
end
D.t = S.t(:);
D.gap = gap;
% decimation: take every dec-th sample (as prescribed in SPEC.md)
sel = (1:dec:n).';
f = [{'t','gap'}, cmdNames, senNames];
for i = 1:numel(f)
    D.(f{i}) = D.(f{i})(sel);
end
D.t = D.t - D.t(1);
D.T = T;
D.L = S.gains.wheelbase;
D.t_start = S.segments.t_start;
D.t_end   = S.segments.t_end;
end
