function out = ekf_run(D, c, x, P, pv)
%EKF_RUN  Run the discrete EKF over a whole data set for one configuration c
%   (fields odo, gyro: 0/1 use of the sensor; pf: logical vector of the
%   samples at which an absolute pose update is applied).
N = D.N; X = nan(N, 6); Pd = nan(N, 6);
for k = D.k0:N
    u = [D.v_cmd(k); D.delta_cmd(k)];
    z = [D.v_meas(k); D.wz_meas(k); D.x_pf(k); D.y_pf(k); D.psi_pf(k)];
    valid = [c.odo * D.ok_v(k); c.gyro * D.ok_wz(k); c.pf(k) * D.ok_pf(k)];
    z(isnan(z)) = 0;
    [x, P] = ekf_step(x, P, u, z, valid, pv);
    X(k,:) = x'; Pd(k,:) = diag(P)';
end
out.X = X; out.Pd = Pd;
out.e    = hypot(X(:,1) - D.x_pf, X(:,2) - D.y_pf);
out.epsi = atan2(sin(X(:,3) - D.psi_pf), cos(X(:,3) - D.psi_pf));
m = ~isnan(out.e);
out.e_mean = mean(out.e(m)); out.e_max = max(out.e(m));
out.epsi_mean = rad2deg(mean(abs(out.epsi(m))));
end
