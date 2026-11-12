function [xn, F, dse, dth, phi] = ekf_model_imu(x, ds_l, ds_r, dpsi, dt, still, p)
%EKF_MODEL_IMU  Process model and Jacobian, IMU mode (spec §7.17.2).
%
%   [xn, F, dse, dth, phi] = ekf_model_imu(x, ds_l, ds_r, dpsi, dt, still, p)
%
%     dse = (ds_l + ds_r)/2
%     dth = still ? 0 : dpsi - b_g*dt
%     ds  = still ? 0 : s*dse
%     phi = theta + dth/2
%     x  += ds*cos(phi),  y += ds*sin(phi),  theta = wrap(theta + dth)
%
%   F is the exact Jacobian of this map (verified against central finite
%   differences in verify_jacobians.m).
%
%   SPEC FINDING (F-1): while `still`, dth does not depend on b_g, so
%   dtheta+/db_g = 0. The spec matrix of §7.17.2 lists -dt unconditionally.
%   With -dt the heading variance grows by dt^2*P_bb per step at standstill and
%   a theta-b_g correlation builds up, so every ZUPT then moves the frozen
%   heading (contradicts D-12). p.mx_still_F_bg = 1 reproduces the literal spec.
%#codegen
c   = class(x);
one = cast(1, c);
dse = (ds_l + ds_r)*cast(0.5, c);
if still > 0
    dth = cast(0, c);
    ds  = cast(0, c);
else
    dth = dpsi - x(4)*dt;
    ds  = x(5)*dse;
end
phi = x(3) + cast(0.5, c)*dth;
cp  = cos(phi);
sp  = sin(phi);
xn    = x;
xn(1) = x(1) + ds*cp;
xn(2) = x(2) + ds*sp;
xn(3) = ekf_wrap(x(3) + dth);

F = eye(5, c);
if still > 0
    if p.mx_still_F_bg > 0
        F(3,4) = -dt;                     % literal spec (see F-1 above)
    end
else
    F(1,3) = -ds*sp;
    F(1,4) =  cast(0.5, c)*ds*sp*dt;
    F(1,5) =  dse*cp;
    F(2,3) =  ds*cp;
    F(2,4) = -cast(0.5, c)*ds*cp*dt;
    F(2,5) =  dse*sp;
    F(3,4) = -dt*one;
end
end
