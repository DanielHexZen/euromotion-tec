function R = verify_jacobians()
%VERIFY_JACOBIANS  t_ekf_jacobians (§14.1): analytic F and every H against
%   central finite differences, 1e-3 relative, in double precision.
%
%   Checked: F (IMU mode moving, IMU mode still, encoder mode), H of edge,
%   face range, heading, point, ZUPT, at 200 random states and inputs each,
%   including a non-zero back-shift. Relative error per matrix:
%   max|H_a - H_fd| / max(max|H_fd|, 1e-9).
%   Also: the literal spec F while still (F(3,4) = -dt) is compared with the
%   finite-difference Jacobian of the still model (finding F-1).

p  = ekf_params();
rs = sim_rng_init(4242, 1);
h  = 1e-6;
tol = 1e-3;
N = 200;
worst = zeros(1, 8);
names = {'F IMU moving', 'F IMU still', 'F encoder', 'H edge', 'H face', 'H heading', 'H point', 'H ZUPT'};
spec_still_err = 0;
for it = 1:N
    [rs, u] = rnd(rs, 12);
    x  = [10*(u(1)-0.5); 10*(u(2)-0.5); 2*pi*(u(3)-0.5); 0.02*(u(4)-0.5); 1 + 0.02*(u(5)-0.5)];
    dsl = 0.01*u(6);  dsr = 0.01*u(7);  dt = 0.02;  dpsi = 0.05*(u(8)-0.5);
    bk = [0.02*(u(9)-0.5); 0.02*(u(10)-0.5); 0.05*(u(11)-0.5)];
    % F, IMU moving / still, encoder
    f1 = @(z) ekf_model_imu(z, dsl, dsr, dpsi, dt, 0, p);
    [~, Fa] = ekf_model_imu(x, dsl, dsr, dpsi, dt, 0, p);
    worst(1) = max(worst(1), relerr(Fa, fd_vec(f1, x, h, 3)));
    f2 = @(z) ekf_model_imu(z, 0, 0, dpsi, dt, 1, p);
    [~, Fa] = ekf_model_imu(x, 0, 0, dpsi, dt, 1, p);
    Ffd = fd_vec(f2, x, h, 3);
    worst(2) = max(worst(2), relerr(Fa, Ffd));
    ps = p;  ps.mx_still_F_bg = 1;
    [~, Fs] = ekf_model_imu(x, 0, 0, dpsi, dt, 1, ps);
    spec_still_err = max(spec_still_err, max(abs(Fs(:) - Ffd(:))));
    f3 = @(z) ekf_model_enc(z, dsl, dsr, dt, 0, p);
    [~, Fa] = ekf_model_enc(x, dsl, dsr, dt, 0, p);
    worst(3) = max(worst(3), relerr(Fa, fd_vec(f3, x, h, 3)));
    % edge
    th = 2*pi*u(12);
    ln = [cos(th); sin(th); 3*(u(1)-0.5)];
    rsb = [0.15; 0.25*(2*(u(2) > 0.5) - 1)];
    [~, Ha] = ekf_meas_edge(x, ln, rsb, bk);
    worst(4) = max(worst(4), relerr(Ha, fd_vec(@(z) ekf_meas_edge(z, ln, rsb, bk), x, h, 0)));
    % face (keep |g| >= 0.3)
    us = [0; 1];
    for tries = 1:10
        [~, ~, g] = ekf_meas_face(x, ln, [0; 0.175], us, bk);
        if abs(g) >= 0.3, break; end
        ln(1:2) = [cos(th + tries); sin(th + tries)];
    end
    [~, Ha] = ekf_meas_face(x, ln, [0; 0.175], us, bk);
    worst(5) = max(worst(5), relerr(Ha, fd_vec(@(z) ekf_meas_face(z, ln, [0; 0.175], us, bk), x, h, 0)));
    % heading: h = theta
    Hh = [0 0 1 0 0];
    worst(6) = max(worst(6), relerr(Hh, fd_vec(@(z) z(3), x, h, 0)));
    % point
    q = [0; 0.6*(2*(u(3) > 0.5) - 1)];
    [~, Ha] = ekf_meas_point(x, q, bk);
    worst(7) = max(worst(7), relerr(Ha, fd_vec(@(z) ekf_meas_point(z, q, bk), x, h, 0)));
    % ZUPT: h = b_g
    worst(8) = max(worst(8), relerr([0 0 0 1 0], fd_vec(@(z) z(4), x, h, 0)));
end
R = [];
for k = 1:8
    R = [R, vres(['t_ekf_jacobians ' names{k}], double(worst(k) < tol), ...
        sprintf('%.2e', worst(k)), '< 1e-3 rel')]; %#ok<AGROW>
end
R = [R, vres('F-1 spec F(3,4)=-dt while still', -1, sprintf('%.3g abs', spec_still_err), ...
    'FD Jacobian of the still model', 'spec matrix differs from the exact Jacobian at standstill')];
fprintf('verify_jacobians: worst relative errors\n');
for k = 1:8, fprintf('  %-14s %.3e\n', names{k}, worst(k)); end
fprintf('  literal spec F while still vs FD: max abs diff %.3g (= dt)\n', spec_still_err);
end

function J = fd_vec(f, x, h, iwrap)
% central differences of a vector function; component iwrap is an angle
y0 = f(x);
J = zeros(numel(y0), numel(x));
for i = 1:numel(x)
    xp = x; xm = x; xp(i) = xp(i) + h; xm(i) = xm(i) - h;
    d = f(xp) - f(xm);
    if iwrap > 0
        d(iwrap) = d(iwrap) - 2*pi*round(d(iwrap)/(2*pi));
    end
    J(:, i) = d/(2*h);
end
end

function e = relerr(A, B)
e = max(abs(A(:) - B(:)))/max(max(abs(B(:))), 1e-9);
end

function [s, u] = rnd(s, n)
u = zeros(n, 1);
for i = 1:n, [s, u(i)] = sim_rand(s); end
end
