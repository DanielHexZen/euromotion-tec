function a = ekf_wrap(a)
%EKF_WRAP  Wrap an angle to [-pi, pi). Class-preserving (single stays single).
%   C reference:  a -= 2*PI_F * floorf((a + PI_F) / (2*PI_F));
%#codegen
c   = class(a);
pi_ = cast(pi, c);
a   = a - 2*pi_*floor((a + pi_)/(2*pi_));
end
