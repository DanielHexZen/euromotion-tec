%% MAIN_KALMAN  Discrete-time Extended Kalman Filter for autonomous navigation
%  Robot   : F1TENTH Ackermann car "McQueen56" (open-loop test openloop_test01)
%  Inputs  : executed Ackermann commands  (speed, steering)
%  Sensors : VESC wheel speed (50 Hz), IMU yaw rate (50 Hz),
%            particle-filter pose on LiDAR map (~30 Hz, used as reference
%            and - optionally, at a reduced rate - as absolute measurement)
%
%  Run this script from its folder. It
%    1) loads and resamples the data (Ts = 20 ms)
%    2) identifies the model and noise parameters
%    3) runs four filter configurations and compares them with the reference
%    4) saves the parameters to ekf_params.mat (used by the Simulink model)
clear; close all; clc;

dataFile = fullfile('..', 'openloop_test01', 'openloop_test01_synced.mat');
if ~exist(dataFile, 'file'), dataFile = 'openloop_test01_synced.mat'; end

Ts = 0.02;                                   % 50 Hz = rate of VESC and IMU
D  = ekf_load_data(dataFile, Ts);
p  = ekf_identify(D, true);

%% ------------------------------------------------------------- Tuning
% Process noise Q (per step) - diag for [px py psi v delta b_g]
p.q = [1e-6;  1e-6;               % position: model is exact up to slip
       1e-6;                      % heading
       (0.05)^2 * Ts;             % speed: unmodelled acceleration
       (0.05)^2;                  % steering: servo / tyre slip
       1e-8];                     % gyro bias random walk
% Measurement noise R (variances) - from the standstill statistics
p.R_v   = max(p.sigma_v, 0.02)^2; % wheel speed (inflated: wheel slip)
p.R_g   = p.sigma_g^2;            % gyro
p.R_xy  = 0.05^2;                 % absolute position (LiDAR localisation)
p.R_psi = 0.03^2;                 % absolute heading
p.pf_rate_hz = 1;                 % rate of absolute updates in configs C/D

pv = ekf_param_vector(p);

% initial state at the first reference pose, robot at rest
x0 = [D.x_pf(D.k0); D.y_pf(D.k0); D.psi_pf(D.k0); 0; 0; p.k_g * p.b_g];
P0 = diag([0.05^2, 0.05^2, 0.05^2, 0.01^2, 0.01^2, 0.005^2]);
save('ekf_params.mat', 'p', 'pv', 'x0', 'P0', 'Ts');

%% ------------------------------------------------------------- Scenarios
pfEvery = round(1 / (p.pf_rate_hz * Ts));
k_pf    = false(D.N, 1); k_pf(D.k0:pfEvery:end) = true;
outage  = D.t > 40 & D.t < 60;               % 20 s loss of localisation

cfg(1) = struct('name', 'A: model only (open loop)',       'odo', 0, 'gyro', 0, 'pf', false(D.N,1));
cfg(2) = struct('name', 'B: EKF wheel speed + gyro',        'odo', 1, 'gyro', 1, 'pf', false(D.N,1));
cfg(3) = struct('name', 'C: EKF + pose 1 Hz',               'odo', 1, 'gyro', 1, 'pf', k_pf);
cfg(4) = struct('name', 'D: EKF + pose 1 Hz, 20 s outage',  'odo', 1, 'gyro', 1, 'pf', k_pf & ~outage);

for c = 1:numel(cfg)
    R(c) = ekf_run(D, cfg(c), x0, P0, pv); %#ok<SAGROW>
end

%% ------------------------------------------------------------- Metrics
% Reference: particle filter pose. For comparison also the car's own wheel
% odometry (rotated/translated onto the reference start pose).
odo = ekf_align_odometry(D);
fprintf('\n=== Position / heading error w.r.t. reference (t = %.1f ... %.1f s) ===\n', D.t(D.k0), D.t(end));
fprintf('%-36s %10s %10s %12s\n', 'configuration', 'mean [m]', 'max [m]', 'heading [deg]');
fprintf('%-36s %10.3f %10.3f %12.2f\n', 'wheel odometry (/odom topic)', odo.e_mean, odo.e_max, odo.epsi_mean);
for c = 1:numel(cfg)
    fprintf('%-36s %10.3f %10.3f %12.2f\n', cfg(c).name, R(c).e_mean, R(c).e_max, R(c).epsi_mean);
end

%% ------------------------------------------------------------- Plots
col = lines(5);
figure('Name', 'Trajectory', 'Color', 'w');
plot(D.x_pf, D.y_pf, 'k', 'LineWidth', 2); hold on;
plot(odo.x, odo.y, ':', 'Color', [0.5 0.5 0.5]);
for c = 1:numel(cfg), plot(R(c).X(:,1), R(c).X(:,2), 'Color', col(c,:)); end
idx = find(cfg(4).pf); plot(D.x_pf(idx), D.y_pf(idx), 'k.', 'MarkerSize', 8);
axis equal; grid on; xlabel('x [m]'); ylabel('y [m]');
legend(['reference (PF)', 'wheel odometry', {cfg.name}, 'pose updates (D)'], 'Location', 'bestoutside');
title('Estimated trajectories in the map frame');

figure('Name', 'Errors', 'Color', 'w');
subplot(2,1,1); hold on;
for c = 1:numel(cfg), plot(D.t, R(c).e, 'Color', col(c,:)); end
yl = ylim; patch([40 60 60 40], [yl(1) yl(1) yl(2) yl(2)], 'r', 'FaceAlpha', 0.07, 'EdgeColor', 'none');
grid on; ylabel('position error [m]'); legend({cfg.name}, 'Location', 'northwest');
subplot(2,1,2); hold on;
for c = 1:numel(cfg), plot(D.t, rad2deg(R(c).epsi), 'Color', col(c,:)); end
grid on; ylabel('heading error [deg]'); xlabel('t [s]');

% consistency of configuration D: error vs. 3-sigma bound
r = R(4);
figure('Name', 'Consistency (config D)', 'Color', 'w');
lab = {'x [m]', 'y [m]', '\psi [rad]'}; ref = [D.x_pf D.y_pf D.psi_pf];
for i = 1:3
    subplot(3,1,i);
    s3 = 3 * sqrt(r.Pd(:,i));
    plot(D.t, r.X(:,i) - ref(:,i), 'b', D.t, s3, 'r--', D.t, -s3, 'r--'); grid on;
    ylabel(['e_', lab{i}]); if i == 1, title('Config D: estimation error and \pm3\sigma'); legend('error', '\pm3\sigma'); end
end
xlabel('t [s]');

figure('Name', 'Estimated states', 'Color', 'w');
subplot(3,1,1); plot(D.t, D.v_meas, '.', D.t, r.X(:,4), 'LineWidth', 1); grid on; ylabel('v [m/s]'); legend('VESC', 'EKF');
subplot(3,1,2); plot(D.t, D.delta_cmd, D.t, r.X(:,5)); grid on; ylabel('\delta [rad]'); legend('command', 'EKF');
subplot(3,1,3); plot(D.t, r.X(:,6)); grid on; ylabel('b_g [rad/s]'); xlabel('t [s]');

save('ekf_results.mat', 'R', 'cfg', 'D', 'odo');
