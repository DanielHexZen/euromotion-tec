function S = lkf_noise_stats(D)
%LKF_NOISE_STATS  Sensor statistics from the real run (standstill segments).
%   D : struct from lkf_load_data. Standstill = cmd_speed==0 & |vesc_speed|<0.01.
%   Accelerometer / gyro noise is the within-segment std (around each segment's
%   mean); bias = pooled mean; the spread of the segment means is the bias
%   repeatability (sigma_bres).
T = D.T; fs = 1/T;
ok = ~isnan(D.ax) & ~isnan(D.ay) & ~isnan(D.wz) & ~isnan(D.vs);
st = (D.cmd == 0) & (abs(D.vs) < 0.01) & ok;
dd = diff([0; st(:); 0]);
a = find(dd == 1); b = find(dd == -1) - 1;
len = b - a + 1;
keep = len >= 100;                        % >= 2 s
a = a(keep); b = b(keep); len = len(keep);
nseg = numel(a);
mx = zeros(nseg,1); my = mx; mz = mx; vx = mx; vy = mx; vz = mx;
for s = 1:nseg
    r = a(s):b(s);
    mx(s) = mean(D.ax(r)); my(s) = mean(D.ay(r)); mz(s) = mean(D.wz(r));
    vx(s) = var(D.ax(r)); vy(s) = var(D.ay(r)); vz(s) = var(D.wz(r));
end
w = len/sum(len);
S.T = T;
S.seg_start_t = D.t(a); S.seg_end_t = D.t(b); S.seg_len = len;
S.n_standstill = sum(len);
S.seg_mean_ax = mx; S.seg_mean_ay = my; S.seg_mean_wz = mz;
S.bias_ax = sum(w.*mx); S.bias_ay = sum(w.*my); S.bias_wz = sum(w.*mz);
S.sigma_ax = sqrt(sum(w.*vx)); S.sigma_ay = sqrt(sum(w.*vy)); S.sigma_wz = sqrt(sum(w.*vz));
% residual bias repeatability between standstills (RMS deviation of segment means)
S.sigma_bres_x = sqrt(mean((mx - S.bias_ax).^2));
S.sigma_bres_y = sqrt(mean((my - S.bias_ay).^2));
% isotropic effective accelerometer noise for Q: white noise + residual bias
S.sigma_a_white = sqrt(0.5*(S.sigma_ax^2 + S.sigma_ay^2));
S.sigma_bres    = sqrt(0.5*(S.sigma_bres_x^2 + S.sigma_bres_y^2));
S.sigma_a       = sqrt(S.sigma_a_white^2 + S.sigma_bres^2);
% bias random-walk intensity from the change of the segment means between the
% first and the last standstill: E[(b2-b1)^2] = q_b * dt  (indicative: 2 points only)
if nseg >= 2
    dtb = 0.5*(D.t(b(end))+D.t(a(end))) - 0.5*(D.t(b(1))+D.t(a(1)));
    S.q_bias_x = (mx(end)-mx(1))^2/dtb; S.q_bias_y = (my(end)-my(1))^2/dtb;
    S.q_bias = 0.5*(S.q_bias_x + S.q_bias_y);       % (m/s^2)^2/s
    S.dt_bias = dtb;
else
    S.q_bias = 0; S.q_bias_x = 0; S.q_bias_y = 0; S.dt_bias = 0;
end
% PSD (Welch) of the longest standstill segment per axis
[~, k] = max(len); r = a(k):b(k);
[f, Px] = lkf_psd(D.ax(r), fs, 256);
[~, Py] = lkf_psd(D.ay(r), fs, 256);
S.psd_f = f; S.psd_ax = Px; S.psd_ay = Py;
band = f > 1 & f < 0.9*fs/2;
S.psd_level_ax = mean(Px(band)); S.psd_level_ay = mean(Py(band));
S.noise_density_ax = sqrt(S.psd_level_ax);      % (m/s^2)/sqrt(Hz)
S.noise_density_ay = sqrt(S.psd_level_ay);
S.psd_white_expect_ax = 2*S.sigma_ax^2*T;       % flat level implied by sigma
S.psd_white_expect_ay = 2*S.sigma_ay^2*T;
S.vrw_x = S.noise_density_ax; S.vrw_y = S.noise_density_ay;  % m/s/sqrt(s)
% Position fix of the particle-filter pose: jitter from 2nd difference
% (white noise of std s gives std(diff2) = sqrt(6) s; true motion adds only a*T^2)
v = ~isnan(D.px) & ~isnan(D.py);
d2x = diff(D.px,2); d2y = diff(D.py,2);
S.pf_jitter_x = std(d2x(~isnan(d2x)))/sqrt(6);
S.pf_jitter_y = std(d2y(~isnan(d2y)))/sqrt(6);
S.pf_jitter = sqrt(0.5*(S.pf_jitter_x^2 + S.pf_jitter_y^2));
% Design decision (see RESULTS.md): the PF pose is smooth (mm-level jitter) and
% its absolute error is unobservable here, so the emulated fix sensor uses a
% GNSS/LiDAR-localisation-like accuracy at 1:10 scale.
S.sigma_z   = 0.05;          % m, per axis
S.fix_rate  = 10;            % Hz
S.sigma_psi = 0.02;          % rad, heading fix noise (~1.1 deg)
S.vesc_std_standstill = std(D.vs(st));
% In-motion noise (cmd_speed > 0.5 m/s): high-pass residual of a 5-sample moving
% mean (white noise of std s gives std s*sqrt(1-1/5)); includes vibration.
mv = (D.cmd > 0.5) & ok;
ax0 = D.ax; ay0 = D.ay; vs0 = D.vs; ax0(isnan(ax0)) = 0; ay0(isnan(ay0)) = 0;
vs0(isnan(vs0)) = 0;
hx = ax0 - movmean(ax0,5); hy = ay0 - movmean(ay0,5); hv = vs0 - movmean(vs0,5);
S.sigma_ax_mot = std(hx(mv))/sqrt(0.8); S.sigma_ay_mot = std(hy(mv))/sqrt(0.8);
S.sigma_a_mot = sqrt(0.5*(S.sigma_ax_mot^2 + S.sigma_ay_mot^2));
S.sigma_v_hf = std(hv(mv))/sqrt(0.8);
S.sigma_v = max(0.05, 2*S.sigma_v_hf);   % design: wheel-speed projection error (heading, slip)
end
