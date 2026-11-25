function D = ekf_load_data(dataFile, Ts)
%EKF_LOAD_DATA  Load the synced ROS-bag export and prepare it for the discrete EKF.
%   D = ekf_load_data(dataFile, Ts)
%   dataFile : path to <bag>_synced.mat (100 Hz grid)
%   Ts       : filter sample time [s] (default 0.02 s = 50 Hz = VESC/IMU rate)
%
%   The synced file is resampled to Ts (decimation), commands are held
%   (zero-order hold) across gaps, and every sensor gets a validity flag
%   so the filter can skip measurement updates where data is missing (NaN).

if nargin < 2, Ts = 0.02; end
S = load(dataFile);

t100 = S.t(:);
dec  = round(Ts / (t100(2) - t100(1)));       % 100 Hz -> 1/Ts
idx  = (1:dec:numel(t100)).';

D.Ts = Ts;
D.t  = t100(idx);
D.N  = numel(D.t);
D.L  = S.gains.wheelbase;                       % 0.324 m

% ---- control inputs (commands, zero-order hold over NaN) -----------------
D.v_cmd     = ffill(S.cmd_speed(idx));          % commanded speed [m/s]
D.delta_cmd = ffill(S.servo_steering(idx));     % steering actually sent to servo [rad]

% ---- on-board sensors -----------------------------------------------------
D.v_meas  = S.vesc_speed(idx);                  % wheel speed from VESC ERPM [m/s]
D.wz_meas = S.imu_wz(idx);                      % IMU yaw rate [rad/s]

% ---- reference / absolute pose (particle filter on LiDAR map) -------------
D.x_pf   = S.pf_x(idx);
D.y_pf   = S.pf_y(idx);
D.psi_pf = S.pf_yaw(idx);                       % unwrapped

% ---- wheel odometry as published by the car (for comparison) -------------
D.x_odo   = S.odom_x(idx);
D.y_odo   = S.odom_y(idx);
D.psi_odo = S.odom_yaw(idx);

% ---- validity flags -------------------------------------------------------
D.ok_v  = ~isnan(D.v_meas);
D.ok_wz = ~isnan(D.wz_meas);
D.ok_pf = ~isnan(D.x_pf) & ~isnan(D.y_pf) & ~isnan(D.psi_pf);

D.k0 = find(D.ok_pf, 1, 'first');               % first sample with a reference pose
end

function a = ffill(a)
a = a(:);
if isnan(a(1)), a(1) = 0; end
for k = 2:numel(a)
    if isnan(a(k)), a(k) = a(k-1); end
end
end
