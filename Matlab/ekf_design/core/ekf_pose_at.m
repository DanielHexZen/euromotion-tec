function [x, y, th, st] = ekf_pose_at(e, p, t_us)
%EKF_POSE_AT  Estimated pose at time t_us (spec §7.17.1): current pose minus back-shift.
%#codegen
[bk, st] = ekf_back_at(e, p, t_us);
x  = e.x(1) - bk(1);
y  = e.x(2) - bk(2);
th = ekf_wrap(e.x(3) - bk(3));
end
