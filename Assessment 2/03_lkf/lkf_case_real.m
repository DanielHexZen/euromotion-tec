function C = lkf_case_real(D, S)
%LKF_CASE_REAL  Evaluation case (a): real openloop_test01 run.
%   Reference = particle-filter pose (pf_x, pf_y). Accelerometers = imu_ax/ay
%   rotated with the heading. Position and heading fixes = PF pose subsampled to
%   10 Hz plus Gaussian noise with the std documented in S (stream set by caller).
T = D.T;
k0 = find(~isnan(D.px) & ~isnan(D.py) & ~isnan(D.yaw), 1);
sl = k0:numel(D.t);
t = D.t(sl) - D.t(k0);
N = numel(t);
ax = D.ax(sl); ay = D.ay(sl); wz = D.wz(sl);
ax(isnan(ax)) = S.bias_ax; ay(isnan(ay)) = S.bias_ay; wz(isnan(wz)) = S.bias_wz;  % gaps: no information
px = D.px(sl); py = D.py(sl); yw = D.yaw(sl);
vld = ~isnan(px) & ~isnan(py) & ~isnan(yw);
ref = [interp1(find(vld), px(vld), (1:N)', 'linear', 'extrap')'; ...
       interp1(find(vld), py(vld), (1:N)', 'linear', 'extrap')'];
yref = interp1(find(vld), yw(vld), (1:N)', 'linear', 'extrap');
step = round(1/(S.fix_rate*T));
zv = false(1,N); zv(1:step:N) = true; zv = zv & vld';
z  = ref + S.sigma_z*nav_randn(2,N);                 % noisy position fixes
yfix = yref + S.sigma_psi*nav_randn(N,1);            % noisy heading fixes
psi = lkf_heading(wz, yfix, zv, yref(1), S.bias_wz, T, 0.1);
C.name = 'real';
C.t = t; C.ref = ref; C.psi = psi; C.psi_ref = yref;
C.z = z; C.zv = zv;
C.u    = lkf_rotate(ax - S.bias_ax, ay - S.bias_ay, psi);   % bias-corrected
C.u_raw = lkf_rotate(ax, ay, psi);                          % plain accelerometer
vs = D.vs(sl); vv = ~isnan(vs);
C.zvel = [vs(:)'.*cos(psi(:)'); vs(:)'.*sin(psi(:)')]; C.zvel(:,~vv) = 0;
C.zvv = zv & vv';                                           % wheel speed at the fix instants
C.x0 = [ref(1,1); 0; ref(2,1); 0];                          % car starts at standstill
C.has_vel_truth = false;
C.k0 = k0;
end
