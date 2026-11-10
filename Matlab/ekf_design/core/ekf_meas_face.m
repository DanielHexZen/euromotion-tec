function [h, H, g] = ekf_meas_face(x, ln, rs, us, bk)
%EKF_MEAS_FACE  Range to a mapped straight face (spec §7.17.3 table, row "Face range").
%
%   Sensor ray f + r R(theta) us meets the line n'q = d:
%     g = n' R(theta) us,  D = d - n'f,  h = D/g,  z = r_meas
%     H_p = -n'/g,  H_theta = (-n' R' rs * g - D * n' R' us)/g^2
%   (the spec expression is the quotient rule of D/g and is correct; checked
%   against finite differences in verify_jacobians.m). The sign of n does not
%   matter: D and g change sign together.
%#codegen
c  = class(x);
th = x(3) - bk(3);
ct = cos(th);  st = sin(th);
fx = x(1) - bk(1) + ct*rs(1) - st*rs(2);
fy = x(2) - bk(2) + st*rs(1) + ct*rs(2);
gx = ct*us(1) - st*us(2);           % R us
gy = st*us(1) + ct*us(2);
g  = ln(1)*gx + ln(2)*gy;
D  = ln(3) - (ln(1)*fx + ln(2)*fy);
h  = D/g;
nRrs = ln(1)*(-st*rs(1) - ct*rs(2)) + ln(2)*(ct*rs(1) - st*rs(2));   % n' R' rs
nRus = ln(1)*(-st*us(1) - ct*us(2)) + ln(2)*(ct*us(1) - st*us(2));   % n' R' us
H  = zeros(1, 5, c);
H(1) = -ln(1)/g;
H(2) = -ln(2)/g;
H(3) = (-nRrs*g - D*nRus)/(g*g);
end
