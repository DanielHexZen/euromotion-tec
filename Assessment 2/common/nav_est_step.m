function [xs, est] = nav_est_step(xs, y, k, P) %#codegen
%NAV_EST_STEP  Estimator at sample k: bias calibration at standstill, complementary heading
%   filter, rotation of the accelerations to the global frame, 6-state LKF (lkf_step).
%   y = [ax ay wz zx zy psi_fix fixvalid vwheel]  (nav_sensor_step).
%   est = [px py vx vy psi ux uy] (position, velocity, heading, global calibrated acceleration).
T = P.T;
if k <= P.nc
    xs.cal = xs.cal + y(1:3);
    xs.cb = xs.cal/k;                    % running mean at standstill, then frozen
end
acx = y(1) - xs.cb(1); acy = y(2) - xs.cb(2); wzc = y(3) - xs.cb(3);
if k == 1
    psi = xs.psi;
else
    psi = xs.psi + xs.wzc*T;
    if y(7) ~= 0
        psi = psi + P.Kc*(y(6) - psi);
    end
end
c = cos(psi); s = sin(psi);
u = [c*acx - s*acy; s*acx + c*acy];
if k > 1
    ubar = 0.5*(xs.u + u);
    lp = P.lkf;
    pm = 0.5*(xs.psi + psi);
    lp.F(1:4,5:6) = -lp.G(1:4,:)*[cos(pm) -sin(pm); sin(pm) cos(pm)];
    zm = y(4:5);
    if size(lp.H,1) == 4
        zm = [y(4); y(5); y(8)*cos(psi); y(8)*sin(psi)];   % optional wheel-speed velocity measurement
    end
    [x, Pm] = lkf_step(xs.x, xs.Pm, ubar, zm, y(7), lp);
    xs.x = x; xs.Pm = Pm;
end
xs.psi = psi; xs.wzc = wzc; xs.u = u;
est = [xs.x(1); xs.x(3); xs.x(2); xs.x(4); psi; u(1); u(2)];
end
