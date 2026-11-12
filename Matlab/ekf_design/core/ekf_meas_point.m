function [h, H] = ekf_meas_point(x, q, bk)
%EKF_MEAS_POINT  Point landmark (post) model (spec §7.17.3 table, row "Point").
%
%   h = p + R(theta) q, q = body offset of the post centre;  z = (l_x, l_y) surveyed
%   H = [I2 | R'(theta) q | 0 | 0]
%#codegen
c  = class(x);
th = x(3) - bk(3);
ct = cos(th);  st = sin(th);
h  = [x(1) - bk(1) + ct*q(1) - st*q(2);
      x(2) - bk(2) + st*q(1) + ct*q(2)];
H  = zeros(2, 5, c);
H(1,1) = 1;
H(2,2) = 1;
H(1,3) = -st*q(1) - ct*q(2);
H(2,3) =  ct*q(1) - st*q(2);
end
