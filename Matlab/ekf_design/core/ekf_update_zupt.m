function [e, st] = ekf_update_zupt(e, p, rate_mean, T)
%EKF_UPDATE_ZUPT  Zero-velocity update of the gyro bias (spec §7.17.3/§7.17.4).
%
%   z = mean of dpsi/dt over the standstill window of length T,  h = b_g,  H_bg = 1
%   R = ekf_gyro_nd^2/T + ekf_zupt_floor^2
%   (white rate noise of density nd averaged over T has variance nd^2/T;
%   the floor covers gyro quantisation, temperature drift inside the window and
%   residual vibration. Checked by Monte Carlo in verify_static.m.)
%#codegen
c = class(e.x);
H = zeros(1, 5, c);
H(4) = 1;
R  = p.ekf_gyro_nd*p.ekf_gyro_nd/T + p.ekf_zupt_floor*p.ekf_zupt_floor;
nu = rate_mean - e.x(4);
[e, st] = ekf_update_scalar(e, p, nu, H, R, 1);
end
