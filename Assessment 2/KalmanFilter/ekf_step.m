function [x, P] = ekf_step(x, P, u, z, valid, pv)
%EKF_STEP  One discrete-time step (prediction + correction) of the
%   Extended Kalman Filter for the Ackermann robot (F1TENTH "McQueen56").
%
%   State  x = [px; py; psi; v; delta; b_g]   (6x1)
%            px, py  position in map frame [m]
%            psi     heading [rad] (unwrapped)
%            v       longitudinal speed [m/s]
%            delta   effective steering angle [rad]
%            b_g     gyro bias [rad/s]
%   Input  u = [v_cmd; delta_cmd]              (control commands)
%   Meas.  z = [v_wheel; r_gyro; px_pf; py_pf; psi_pf]
%   valid  = [use wheel speed; use gyro; use absolute pose]  (0/1)
%   pv     = parameter vector, see ekf_param_vector.m
%
%   The function is code-generation compatible and is used unchanged in
%   MATLAB (main_kalman.m) and in the Simulink MATLAB Function block.

%#codegen
Ts = pv(1);  L = pv(2);
av = exp(-Ts / pv(3));          % drive pole
ad = exp(-Ts / pv(4));          % steering pole
kd = pv(5);  kg = pv(6);
Q  = diag(pv(7:12));
Rv = pv(13); Rg = pv(14); Rxy = pv(15); Rpsi = pv(16);

% ============================== PREDICTION  x(k+1|k) = f(x(k|k), u(k))
px = x(1); py = x(2); ps = x(3); v = x(4); dl = x(5); bg = x(6);
r  = kd * v / L * tan(dl);

xp = [ px + Ts * v * cos(ps);
       py + Ts * v * sin(ps);
       ps + Ts * r;
       av * v  + (1 - av) * u(1);
       ad * dl + (1 - ad) * u(2);
       bg ];

F = eye(6);                     % Jacobian df/dx
F(1,3) = -Ts * v * sin(ps);   F(1,4) = Ts * cos(ps);
F(2,3) =  Ts * v * cos(ps);   F(2,4) = Ts * sin(ps);
F(3,4) =  Ts * kd / L * tan(dl);
F(3,5) =  Ts * kd * v / (L * cos(dl)^2);
F(4,4) = av;  F(5,5) = ad;

x = xp;
P = F * P * F' + Q;

% ============================== CORRECTION 1: wheel speed (VESC)
if valid(1) > 0.5
    H = [0 0 0 1 0 0];
    [x, P] = kf_update(x, P, z(1) - x(4), H, Rv);
end

% ============================== CORRECTION 2: gyro yaw rate (IMU)
if valid(2) > 0.5
    v = x(4); dl = x(5);
    h = kd * v / L * tan(dl) + x(6);
    H = [0 0 0, kd / L * tan(dl), kd * v / (L * cos(dl)^2), 1];
    [x, P] = kf_update(x, P, kg * z(2) - h, H, Rg);
end

% ============================== CORRECTION 3: absolute pose (localisation)
if valid(3) > 0.5
    H = [eye(3), zeros(3,3)];
    e_psi = atan2(sin(z(5) - x(3)), cos(z(5) - x(3)));   % wrapped heading error
    y = [z(3) - x(1); z(4) - x(2); e_psi];
    [x, P] = kf_update(x, P, y, H, diag([Rxy Rxy Rpsi]));
end
end

function [x, P] = kf_update(x, P, y, H, R)
% Joseph-form measurement update (numerically robust, keeps P symmetric)
S = H * P * H' + R;
K = (P * H') / S;
x = x + K * y;
IKH = eye(numel(x)) - K * H;
P = IKH * P * IKH' + K * R * K';
P = 0.5 * (P + P');
end
