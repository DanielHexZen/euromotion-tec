function [h, H] = ekf_meas_edge(x, ln, rs, bk)
%EKF_MEAS_EDGE  Edge-crossing model (spec §7.17.3 table, row "Edge").
%
%   ln = [nx; ny; d]  boundary line n'q = d, n unit, pointing into the path
%   rs = [rx; ry]     body position of the ground-sensor footprint
%   bk = [dx; dy; dth] back-shift (motion between measurement time and now)
%
%   Evaluated at the past pose p - dp, theta - dth (REQ-EKF-02, dp exact):
%     f = p + R(theta) rs,   h = n'f - d,   z = 0
%     H_p = n',  H_theta = n' R'(theta) rs
%#codegen
c  = class(x);
th = x(3) - bk(3);
ct = cos(th);  st = sin(th);
fx = x(1) - bk(1) + ct*rs(1) - st*rs(2);
fy = x(2) - bk(2) + st*rs(1) + ct*rs(2);
h  = ln(1)*fx + ln(2)*fy - ln(3);
dRx = -st*rs(1) - ct*rs(2);         % R'(theta) rs
dRy =  ct*rs(1) - st*rs(2);
H  = zeros(1, 5, c);
H(1) = ln(1);
H(2) = ln(2);
H(3) = ln(1)*dRx + ln(2)*dRy;
end
