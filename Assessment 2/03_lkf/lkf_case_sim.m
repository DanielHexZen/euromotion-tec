function C = lkf_case_sim(csvfile, S)
%LKF_CASE_SIM  Evaluation case (b): simulated lap on the 1:10 Mexico City track.
%   Sensors use the statistics of the real data (S): accelerometer noise and bias
%   (bias additionally random-walks with the intensity q_bias observed between
%   the two real standstills), gyro noise/bias, 10 Hz position and
%   heading fixes. The filter only knows a bias estimated from a 10 s standstill.
T = S.T;
t_stand = 10;
tr = lkf_track_sim(csvfile, T, t_stand, 0.1, 2.0, 2.0, 1.5);
N = numel(tr.t);
sqb = sqrt(S.q_bias*T);                      % random-walk step std from the real standstills
bx = S.bias_ax + sqb*[0; cumsum(nav_randn(N-1,1))];
by = S.bias_ay + sqb*[0; cumsum(nav_randn(N-1,1))];
ax_m = tr.at   + bx + S.sigma_ax*nav_randn(N,1);
ay_m = tr.ay_b + by + S.sigma_ay*nav_randn(N,1);
wz_m = tr.r + S.bias_wz + S.sigma_wz*nav_randn(N,1);
nc = round(t_stand/T);                      % calibration window (standstill)
cb.ax = mean(ax_m(1:nc)); cb.ay = mean(ay_m(1:nc)); cb.wz = mean(wz_m(1:nc));
ref = [tr.px'; tr.py'];
step = round(1/(S.fix_rate*T));
zv = false(1,N); zv(1:step:N) = true;
z  = ref + S.sigma_z*nav_randn(2,N);
yfix = tr.psi + S.sigma_psi*nav_randn(N,1);
psi = lkf_heading(wz_m, yfix, zv, tr.psi(1), cb.wz, T, 0.1);
C.name = 'sim';
C.t = tr.t; C.ref = ref; C.vref = [tr.vx'; tr.vy']; C.psi = psi; C.psi_ref = tr.psi;
C.z = z; C.zv = zv;
C.u    = lkf_rotate(ax_m - cb.ax, ay_m - cb.ay, psi);
C.u_raw = lkf_rotate(ax_m, ay_m, psi);
C.u_true = lkf_rotate(tr.at, tr.ay_b, tr.psi);
vw = tr.v + S.sigma_v_hf*nav_randn(N,1);                         % wheel speed (noise as measured in motion)
C.zvel = [vw'.*cos(psi'); vw'.*sin(psi')];
C.zvv = zv;
C.x0 = [ref(1,1); 0; ref(2,1); 0];
C.has_vel_truth = true;
C.tr = tr; C.calib = cb; C.bias_true = [bx by];
end
