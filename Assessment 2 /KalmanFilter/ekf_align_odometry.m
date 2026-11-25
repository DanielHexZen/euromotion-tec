function o = ekf_align_odometry(D)
%EKF_ALIGN_ODOMETRY  Move the car's own wheel odometry (/odom, odom frame) onto
%   the reference start pose in the map frame so it can be compared.
k = find(D.ok_pf & ~isnan(D.x_odo), 1);
dp = D.psi_pf(k) - D.psi_odo(k);
Rz = [cos(dp) -sin(dp); sin(dp) cos(dp)];
xy = (Rz * [D.x_odo' - D.x_odo(k); D.y_odo' - D.y_odo(k)])';
o.x = xy(:,1) + D.x_pf(k); o.y = xy(:,2) + D.y_pf(k);
e = hypot(o.x - D.x_pf, o.y - D.y_pf); m = ~isnan(e);
ep = atan2(sin(D.psi_odo + dp - D.psi_pf), cos(D.psi_odo + dp - D.psi_pf));
o.e_mean = mean(e(m)); o.e_max = max(e(m)); o.epsi_mean = rad2deg(mean(abs(ep(m))));
end
