function e = ekf_reset_pose(e, p, x, y, th)
%EKF_RESET_POSE  Reset the pose to (x, y, th), keep b_g and s (spec §7.17.1).
%   The pose covariance returns to P0, the cross-covariances with the pose are
%   cleared, the b_g and s blocks are kept. The pose history is emptied.
%#codegen
c = class(e.x);
e.x(1) = cast(x, c);  e.x(2) = cast(y, c);  e.x(3) = ekf_wrap(cast(th, c));
e.P(1:3, :) = 0;  e.P(:, 1:3) = 0;
e.P(1,1) = p.ekf_p0_xy^2;  e.P(2,2) = p.ekf_p0_xy^2;  e.P(3,3) = p.ekf_p0_th^2;
e.hist_n = 0;  e.hist_head = 0;
e.corr(:) = 0;
e.th_net(:) = 0;
end
