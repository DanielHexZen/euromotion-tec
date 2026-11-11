function e = ekf_predict_imu(e, p, ds_l, ds_r, dpsi, dt, sat)
%EKF_PREDICT_IMU  Prediction step, IMU mode (spec §7.17.2, REQ-EKF-03/04).
%
%   e = ekf_predict_imu(e, p, ds_l, ds_r, dpsi, dt, sat)
%     ds_l, ds_r  wheel distances over the step, m (counts * odo_m_per_cnt * odo_scale)
%     dpsi        yaw-integral difference over the step, rad (imu_yaw_integral_at)
%     dt          step length from the encoder timestamps, s
%     sat         1 if a gyro sample of this step was saturated (REQ-IMU-07)
%   e.still must be set before the call (ekf_still_update).
%
%   C order of operations:
%     1. slip detector (uses b_g before the step)
%     2. x+, F             (ekf_model_imu)
%     3. Q                 (distance, gyro white, gyro scale factor, b_g RW, s RW)
%                          scale-factor term: p.mx_sf_mode 0 literal spec, 1 per radian
%                          (default), 2 net-angle (uses and updates e.th_net)
%     4. P = F P F' + Q;  P = (P + P')/2
%#codegen
c     = class(e.x);
still = e.still;

% 1. slip detector (REQ-EKF-04), only while moving
if still == 0
    e = ekf_slip_detect(e, p, ds_l, ds_r, dpsi, dt);
end
if dt > 0
    e.w_meas = dpsi/dt - e.x(4);
end

% 2. state and Jacobian
[xn, F, dse, dth, phi] = ekf_model_imu(e.x, ds_l, ds_r, dpsi, dt, still, p);
s  = e.x(5);
cp = cos(phi);
sp = sin(phi);

% 3. process noise (table of REQ-EKF-03)
Q = zeros(5, 5, c);
infl = cast(1, c);
if e.slip > 0
    infl = p.ekf_slip_infl;
end
d_abs = abs(dse);
if e.slip > 0 && p.mx_slip_abs > 0
    % finding F-11: in an in-place turn dse ~ 0, so the literal form adds nothing
    % although a slipping wheel makes the vehicle pivot about it (translate)
    d_abs = cast(0.5, c)*(abs(ds_l) + abs(ds_r));
end
sig2_ds = p.ekf_kd*d_abs*infl;                    % distance noise
g = [s*cp; s*sp; 0; 0; 0];
Q = Q + g*sig2_ds*g';
if still == 0
    qth = p.ekf_gyro_nd*p.ekf_gyro_nd*dt;          % gyro white noise
    if sat > 0
        qth = qth*p.ekf_sat_infl;                  % REQ-IMU-07
    end
    Q(3,3) = Q(3,3) + qth;
end
if p.mx_sf_mode == 0
    Q(3,3) = Q(3,3) + (p.ekf_gyro_sf*dth)^2;       % gyro scale factor, spec form
elseif p.mx_sf_mode == 1
    % proposal (see README, finding F-3): variance per radian turned, so a
    % 90 deg turn gets (sf*pi/2)^2 whatever the step rate
    Q(3,3) = Q(3,3) + p.ekf_gyro_sf*p.ekf_gyro_sf*cast(p.mx_sf_corr, c)*abs(dth);
else
    % mode 2 (see README, finding F-9): a CONSTANT scale error gives the heading
    % error sf*Theta, Theta = net angle turned since the pose was (re)set. Add the
    % increment of (sf*Theta)^2, never negative: exact for turning on in one
    % direction, conservative after a reversal or after heading corrections.
    th_new = e.th_net + dth;
    dq = p.ekf_gyro_sf*p.ekf_gyro_sf*(th_new*th_new - e.th_net*e.th_net);
    if dq > 0
        Q(3,3) = Q(3,3) + dq;
    end
    e.th_net = th_new;
end
Q(4,4) = Q(4,4) + p.ekf_q_bg*dt;                   % bias random walk
if p.mx_freeze_s == 0
    Q(5,5) = Q(5,5) + p.ekf_q_s*dt;                % scale random walk
end

% 4. covariance
e.x = xn;
FP  = F*e.P;
e.P = FP*F' + Q;
e.P = cast(0.5, c)*(e.P + e.P');
end
