function [e, st] = ekf_update_heading(e, p, th_meas, var)
%EKF_UPDATE_HEADING  Absolute heading from a face fit (spec §7.17.3): h = theta.
%
%   Note: the API of §7.17.1 has no back-shift for this update. The face fit
%   reports the heading over a >= 1 m window that ends up to one estimator step
%   before the update; while driving straight the heading does not change over
%   that delay, so no back-shift is applied (the lat module requires a heading
%   change < 0.5 deg over the window).
%#codegen
c = class(e.x);
H = zeros(1, 5, c);
H(3) = 1;
nu = ekf_wrap(th_meas - e.x(3));
[e, st] = ekf_update_scalar(e, p, nu, H, var, 4);
end
