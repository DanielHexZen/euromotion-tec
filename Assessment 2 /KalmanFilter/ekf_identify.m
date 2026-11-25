function p = ekf_identify(D, doPlot)
%EKF_IDENTIFY  Identify the model and noise parameters of the discrete EKF
%   from the open-loop experiment (commands + sensor data).
%
%   Model (kinematic bicycle with actuator dynamics, sample time Ts):
%     v(k+1)     = a_v * v(k)     + (1-a_v) * v_cmd(k)      a_v = exp(-Ts/tau_v)
%     delta(k+1) = a_d * delta(k) + (1-a_d) * delta_cmd(k)  a_d = exp(-Ts/tau_d)
%     yaw rate   r = k_delta * v / L * tan(delta)
%
%   Identified here:
%     tau_v    time constant of the drive (command speed -> wheel speed)
%     tau_d    time constant of the steering servo
%     k_delta  effective steering gain (servo calibration / tyre effects)
%     b_g, sigma_g   gyro bias and noise (from standstill)
%     sigma_v        wheel-speed noise (from standstill)
%     k_g      gyro scale factor (heading change of the reference / integrated gyro)

if nargin < 2, doPlot = true; end
Ts = D.Ts; L = D.L;
p.Ts = Ts; p.L = L;

% ------------------------------------------------- 1) drive dynamics tau_v
okv  = D.ok_v;
cost_v = @(tau) rms_nan(sim_lag(D.v_cmd, Ts, tau) - D.v_meas, okv);
p.tau_v = fminbnd(cost_v, 0.02, 2.0);

% ------------------------------------- 2) steering lag tau_d and gain k_delta
mv = okv & D.ok_wz & D.v_meas > 0.3;           % only while driving
cost_d = @(tau) steer_fit(D, tau, mv);
p.tau_d = fminbnd(cost_d, 0.005, 0.5);
[~, p.k_delta] = steer_fit(D, p.tau_d, mv);

% -------------------------------------------------- 3) noise at standstill
still = D.v_cmd == 0 & okv & D.ok_wz & abs(D.v_meas) < 0.01;
p.b_g     = mean(D.wz_meas(still));
p.sigma_g = std(D.wz_meas(still));
p.sigma_v = max(std(D.v_meas(still)), 0.01);    % floor: standstill hides slip

% ----------------------------------------------------- 4) gyro scale factor
k0 = D.k0; k1 = find(D.ok_pf, 1, 'last');
wz = D.wz_meas; wz(~D.ok_wz) = 0;
p.k_g = (D.psi_pf(k1) - D.psi_pf(k0)) / sum((wz(k0:k1-1) - p.b_g) * Ts);

% ------------------------------------------------------------------- report
fprintf('\n=== Identified parameters (Ts = %.3f s) ===\n', Ts);
fprintf('  drive time constant      tau_v   = %.3f s  (RMS %.3f m/s)\n', p.tau_v, cost_v(p.tau_v));
fprintf('  steering time constant   tau_d   = %.3f s\n', p.tau_d);
fprintf('  steering gain            k_delta = %.3f    (RMS yaw rate %.3f rad/s)\n', p.k_delta, cost_d(p.tau_d));
fprintf('  gyro bias                b_g     = %.2e rad/s\n', p.b_g);
fprintf('  gyro noise (1 sigma)     sigma_g = %.4f rad/s\n', p.sigma_g);
fprintf('  wheel-speed noise        sigma_v = %.4f m/s\n', p.sigma_v);
fprintf('  gyro scale factor        k_g     = %.4f\n', p.k_g);

if doPlot
    figure('Name', 'Identification', 'Color', 'w');
    subplot(2,1,1);
    plot(D.t, D.v_cmd, D.t, D.v_meas, D.t, sim_lag(D.v_cmd, Ts, p.tau_v), '--');
    grid on; ylabel('v [m/s]'); legend('v_{cmd}', 'v_{VESC}', 'model'); title('Drive dynamics');
    subplot(2,1,2);
    dl = sim_lag(D.delta_cmd, Ts, p.tau_d);
    plot(D.t, D.wz_meas, D.t, p.k_delta * D.v_meas / L .* tan(dl), '--');
    grid on; ylabel('r [rad/s]'); xlabel('t [s]'); legend('IMU', 'model'); title('Yaw-rate model');
end
end

% ---------------------------------------------------------------- helpers
function y = sim_lag(u, Ts, tau)
a = exp(-Ts / tau); y = zeros(size(u));
for k = 2:numel(u), y(k) = a * y(k-1) + (1 - a) * u(k-1); end
end

function r = rms_nan(e, ok)
r = sqrt(mean(e(ok).^2));
end

function [J, kd] = steer_fit(D, tau, m)
dl = sim_lag(D.delta_cmd, D.Ts, tau);
phi = D.v_meas(m) / D.L .* tan(dl(m));
kd  = (phi' * D.wz_meas(m)) / (phi' * phi);     % least squares gain
J   = sqrt(mean((D.wz_meas(m) - kd * phi).^2));
end
