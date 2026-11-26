% LKF_MAIN  Work package 03 - discrete Linear Kalman Filter state estimation.
%   Runs from its own folder: reads ../data, writes lkf_params.mat, fig/*.png and
%   RESULTS.md. Octave and MATLAB compatible (no local functions in this script).
%
%   Pipeline: (1) sensor statistics from the real standstill segments,
%   (2) LKF design (Q,R), (3) case (a) real data, (4) case (b) simulated lap on the
%   1:10 Mexico City track, (5) sensitivity + consistency, (6) figures, report.
clear; close all; clc;
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
cd(here); addpath(here);
if ~exist('fig','dir'), mkdir('fig'); end
datafile = fullfile('..','data','openloop_test01_synced.mat');
csvfile  = fullfile('..','data','MexicoCity.csv');
T = 0.02;
addpath(fullfile(here,'..','common'));
nav_randn(0,0,1);                 % portable N(0,1) stream (identical in MATLAB and Octave)

%% 1) Sensor statistics from the real data -----------------------------------
D = lkf_load_data(datafile, T);
S = lkf_noise_stats(D);

%% 2) Design ------------------------------------------------------------------
% Simulation: sensor model = standstill statistics. Real run: the in-motion
% accelerometer noise (vibration) measured on the same run is used for Q.
sw_sim  = S.sigma_a_white;            % white accel. noise std, standstill
sw_real = S.sigma_a_mot;              % white accel. noise std, in motion
par6   = lkf_design(S, 1, 1, 1, 0, sw_sim);    % 6 states (bias states), main design
par4   = lkf_design(S, 1, 1, 0, 0, sw_sim);    % 4 states (no bias states), baseline
par6v  = lkf_design(S, 1, 1, 1, 1, sw_sim);    % + wheel-speed velocity measurement
par6m  = lkf_design(S, 1, 1, 1, 0, sw_real);   % 6 states, in-motion Q (real run)
par4m  = lkf_design(S, 1, 1, 0, 0, sw_real);

%% 3) Evaluation cases (fixed random stream: real case first, then simulation) ---
Cr = lkf_case_real(D, S);
Cs = lkf_case_sim(csvfile, S);
fxr = find(Cr.zv); fxs = find(Cs.zv);
Rs6 = lkf_eval(par6, Cs);  Rs4 = lkf_eval(par4, Cs);  Rs6v = lkf_eval(par6v, Cs);
Rr6 = lkf_eval(par6m, Cr); Rr4 = lkf_eval(par4m, Cr); Rr6n = lkf_eval(par6, Cr);  % Rr6n: standstill Q
Rr6v = lkf_eval(lkf_design(S,1,1,1,1,sw_real), Cr);
cs6  = lkf_consistency(Rs6.out, Rs6.err_lkf, Rs6.out.Pd, fxs);
cs6v = lkf_consistency(Rs6v.out, Rs6v.err_lkf, Rs6v.out.Pd, fxs);
cr6  = lkf_consistency(Rr6.out, Rr6.err_lkf, Rr6.out.Pd, fxr);
crn  = lkf_consistency(Rr6n.out, Rr6n.err_lkf, Rr6n.out.Pd, fxr);

%% 4) Sensitivity (Q and R scaled around the design values) ----------------------
sc_q = 10.^(-1:1/3:1);             % 0.1 ... 10
sc_r = sc_q;
rq_s = zeros(size(sc_q)); rq_r = rq_s; nq_s = rq_s; nq_r = rq_s;
rr_s = rq_s; rr_r = rq_s; nr_s = rq_s; nr_r = rq_s;
for i = 1:numel(sc_q)
    pq = lkf_design(S, sc_q(i), 1, 1, 0, sw_sim);  pr = lkf_design(S, sc_q(i), 1, 1, 0, sw_real);
    a = lkf_eval(pq, Cs); b = lkf_eval(pr, Cr);
    rq_s(i) = a.rmse_lkf; rq_r(i) = b.rmse_lkf;
    c1 = lkf_consistency(a.out, a.err_lkf, a.out.Pd, fxs); nq_s(i) = c1.nis_mean;
    c2 = lkf_consistency(b.out, b.err_lkf, b.out.Pd, fxr); nq_r(i) = c2.nis_mean;
    pq = lkf_design(S, 1, sc_r(i), 1, 0, sw_sim);  pr = lkf_design(S, 1, sc_r(i), 1, 0, sw_real);
    a = lkf_eval(pq, Cs); b = lkf_eval(pr, Cr);
    rr_s(i) = a.rmse_lkf; rr_r(i) = b.rmse_lkf;
    c1 = lkf_consistency(a.out, a.err_lkf, a.out.Pd, fxs); nr_s(i) = c1.nis_mean;
    c2 = lkf_consistency(b.out, b.err_lkf, b.out.Pd, fxr); nr_r(i) = c2.nis_mean;
end
g = [0.1 10^-0.5 1 10^0.5 10];     % 2-D grid (rows: R scale, columns: Q scale)
H_s = zeros(5); H_r = zeros(5);
for iq = 1:5
    for ir = 1:5
        H_s(ir,iq) = lkf_eval(lkf_design(S, g(iq), g(ir), 1, 0, sw_sim), Cs).rmse_lkf;
        H_r(ir,iq) = lkf_eval(lkf_design(S, g(iq), g(ir), 1, 0, sw_real), Cr).rmse_lkf;
    end
end
[~, ibest_r] = min(rq_r); [~, ibest_s] = min(rq_s);
% Q/R ratio dependent only -> best ratio over both sweeps

%% 5) Save parameters --------------------------------------------------------
stats = rmfield(S, {'psd_f','psd_ax','psd_ay'});
F = par6.F; G = par6.G; H = par6.H; Q = par6.Q; R = par6.R; P0 = par6.P0;      % F(1:4,5:6) = -G*Rot(psi_k)
F4 = par4.F; G4 = par4.G; H4 = par4.H; Q4 = par4.Q; R4 = par4.R;
Q_real = par6m.Q;                      % in-motion Q used for the real run
Hv = par6v.H; Rv = par6v.R;            % optional wheel-speed variant
state_names = 'x=[pX vX pY vY bx by], u=[aX aY]=Rot(psi)(a_meas-bias_cal)';
if exist('OCTAVE_VERSION','builtin')
    save('-v7','lkf_params.mat','T','F','G','H','Q','R','P0','F4','G4','H4','Q4','R4','Q_real','Hv','Rv','stats','state_names');
else
    save('lkf_params.mat','T','F','G','H','Q','R','P0','F4','G4','H4','Q4','R4','Q_real','Hv','Rv','stats','state_names','-v7');
end

%% 6) Figures ----------------------------------------------------------------
col.true = [0 0 0]; col.raw = [0.85 0.33 0.10]; col.lkf = [0 0.45 0.74];
col.fix = [0.6 0.6 0.6]; col.l4 = [0.47 0.67 0.19];

% --- noise PSD
h = lkf_newfig(3.5, 2.4);
loglog(S.psd_f(2:end), S.psd_ax(2:end), 'color', col.raw); hold on
loglog(S.psd_f(2:end), S.psd_ay(2:end), 'color', col.lkf);
loglog([S.psd_f(2) S.psd_f(end)], S.psd_white_expect_ax*[1 1], '--', 'color', col.raw);
loglog([S.psd_f(2) S.psd_f(end)], S.psd_white_expect_ay*[1 1], '--', 'color', col.lkf);
xlabel('Frequency [Hz]'); ylabel('PSD [(m/s^2)^2/Hz]'); grid on
legend('a_x','a_y','white level of \sigma_{a_x}','white level of \sigma_{a_y}','location','southwest');
title('Accelerometer PSD, longest standstill');
lkf_savefig(h, fullfile('fig','lkf_noise_psd.png'));

% --- trajectories, simulation
Xs = Rs6.out.X;
h = lkf_newfig(3.5, 5.2);
lkf_subp(2,1);
plot(Cs.ref(1,:), Cs.ref(2,:), '-', 'color', col.true, 'linewidth', 1.5); hold on
plot(Rs6.Xraw(1,:), Rs6.Xraw(3,:), '-', 'color', col.raw);
plot(Xs(1,:), Xs(3,:), '-', 'color', col.lkf);
grid on; xlabel('X [m]'); ylabel('Y [m]'); xlim([-100 900]); ylim([-120 400]);
legend('truth','raw double integration','LKF','location','southeast'); title('(a) Simulated lap, full view');
lkf_subp(2,2);
[~, ik] = max(abs(Cs.tr.kappa)); cx = Cs.ref(1,ik); cy = Cs.ref(2,ik);
w = abs(Cs.ref(1,:)-cx) < 6 & abs(Cs.ref(2,:)-cy) < 6;
plot(Cs.ref(1,w), Cs.ref(2,w), '-', 'color', col.true, 'linewidth', 1.5); hold on
fz = find(Cs.zv & w);
plot(Cs.z(1,fz), Cs.z(2,fz), '.', 'color', col.fix, 'markersize', 8);
plot(Xs(1,w), Xs(3,w), '-', 'color', col.lkf);
grid on; xlabel('X [m]'); ylabel('Y [m]'); xlim([cx-5 cx+11]); ylim([cy-5 cy+5]);
legend('truth','position fixes (10 Hz)','LKF','location','northeast'); title('(b) Zoom on the sharpest curve');
lkf_savefig(h, fullfile('fig','lkf_traj_sim.png'));

% --- trajectories, real
Xr = Rr6.out.X;
h = lkf_newfig(3.5, 5.2);
lkf_subp(2,1);
plot(Cr.ref(1,:), Cr.ref(2,:), '-', 'color', col.true, 'linewidth', 1.5); hold on
plot(Rr6.Xraw(1,:), Rr6.Xraw(3,:), '-', 'color', col.raw);
plot(Xr(1,:), Xr(3,:), '-', 'color', col.lkf);
grid on; xlabel('X [m]'); ylabel('Y [m]'); xlim([-50 450]); ylim([-20 180]);
legend('PF reference','raw double integration','LKF','location','southeast'); title('(a) Real run, full view');
lkf_subp(2,2);
plot(Cr.ref(1,:), Cr.ref(2,:), '-', 'color', col.true, 'linewidth', 1.5); hold on
fz = find(Cr.zv);
plot(Cr.z(1,fz), Cr.z(2,fz), '.', 'color', col.fix, 'markersize', 4);
plot(Xr(1,:), Xr(3,:), '-', 'color', col.lkf);
grid on; xlabel('X [m]'); ylabel('Y [m]'); xlim([-1 13.5]); ylim([-0.7 5.5]);
legend('PF reference','position fixes (10 Hz)','LKF','location','east'); title('(b) Real run, LKF vs reference');
lkf_savefig(h, fullfile('fig','lkf_traj_real.png'));

% --- position error with +-3 sigma
xyl = 'XY';
h = lkf_newfig(3.5, 4.2);
for a = 1:2
    lkf_subp(2,a);
    ix = 1 + 2*(a-1);
    sg = 3*sqrt(Rs6.out.Pd(ix,:));
    plot(Cs.t, Rs6.err_lkf(a,:), '-', 'color', col.lkf); hold on
    plot(Cs.t, sg, 'k--'); plot(Cs.t, -sg, 'k--');
    grid on; ylabel(['Error ' xyl(a) ' [m]']); ylim([-0.2 0.2]);
    if a == 1, title('Simulation: LKF position error and \pm3\sigma'); end
end
xlabel('Time [s]'); legend('error','\pm3\sigma bound','location','northeast');
lkf_savefig(h, fullfile('fig','lkf_err_sim.png'));
h = lkf_newfig(3.5, 4.2);
for a = 1:2
    lkf_subp(2,a);
    ix = 1 + 2*(a-1);
    sg = 3*sqrt(Rr6.out.Pd(ix,:));
    plot(Cr.t, Rr6.err_lkf(a,:), '-', 'color', col.lkf); hold on
    plot(Cr.t, sg, 'k--'); plot(Cr.t, -sg, 'k--');
    grid on; ylabel(['Error ' xyl(a) ' [m]']); ylim([-0.6 0.6]);
    if a == 1, title('Real run: LKF error vs PF and \pm3\sigma'); end
end
xlabel('Time [s]'); legend('error','\pm3\sigma bound','location','northeast');
lkf_savefig(h, fullfile('fig','lkf_err_real.png'));

% --- RMSE bar chart (log scale)
vals = [Rs6.rmse_raw Rs6.rmse_rawbc Rs6.rmse_zoh Rs4.rmse_lkf Rs6.rmse_lkf;
        Rr6.rmse_raw Rr6.rmse_rawbc Rr6.rmse_zoh Rr4.rmse_lkf Rr6.rmse_lkf];
h = lkf_newfig(3.5, 3.0);
hb = bar(vals, 'grouped'); set(gca, 'yscale', 'log'); try, set(hb, 'basevalue', 1e-3); catch, end
set(gca, 'xticklabel', {'Simulation','Real run'});
ylabel('Position RMSE [m]'); grid on; ylim([1e-3 1e5]);
legend('raw integration','raw, bias-corrected','raw fixes (held)','LKF 4 states','LKF 6 states','location','northeast');
lkf_savefig(h, fullfile('fig','lkf_rmse_bar.png'));

% --- sensitivity 1-D
h = lkf_newfig(3.5, 4.6);
lkf_subp(2,1);
loglog(sc_q, rq_s, '-o', 'color', col.lkf, 'markersize', 3); hold on
loglog(sc_r, rr_s, '--s', 'color', col.lkf, 'markersize', 3);
loglog(sc_q, rq_r, '-o', 'color', col.raw, 'markersize', 3);
loglog(sc_r, rr_r, '--s', 'color', col.raw, 'markersize', 3);
grid on; xlabel('Scale factor (nominal = 1)'); ylabel('Position RMSE [m]');
legend('sim: Q scaled','sim: R scaled','real: Q scaled','real: R scaled','location','northwest');
title('(a) RMSE sensitivity');
lkf_subp(2,2);
[nlo, nhi] = lkf_chi2_bounds(2*cs6.n_fix, cs6.n_fix);
loglog(sc_q, nq_s, '-o', 'color', col.lkf, 'markersize', 3); hold on
loglog(sc_q, nq_r, '-o', 'color', col.raw, 'markersize', 3);
loglog([sc_q(1) sc_q(end)], [2 2], 'k-');
loglog([sc_q(1) sc_q(end)], [nlo nlo], 'k:'); loglog([sc_q(1) sc_q(end)], [nhi nhi], 'k:');
grid on; xlabel('Q scale factor (R fixed)'); ylabel('mean NIS [-]');
legend('sim','real','expected (2)','95 % bounds','location','northeast');
title('(b) Innovation consistency');
lkf_savefig(h, fullfile('fig','lkf_sensitivity.png'));

% --- sensitivity 2-D
h = lkf_newfig(3.5, 5.6);
for k = 1:2
    lkf_subp(2,k,0.60);
    if k == 1, M = H_s; tt = '(a) Simulation: RMSE [cm]'; else, M = H_r; tt = '(b) Real run: RMSE [cm]'; end
    imagesc(log10(g), log10(g), 100*M); axis xy; colorbar
    set(gca, 'xtick', log10(g), 'xticklabel', {'0.1','0.32','1','3.2','10'}, ...
             'ytick', log10(g), 'yticklabel', {'0.1','0.32','1','3.2','10'});
    for iq = 1:5
        for ir = 1:5
            text(log10(g(iq)), log10(g(ir)), sprintf('%.1f', 100*M(ir,iq)), 'horizontalalignment','center','fontsize',7,'color','w');
        end
    end
    xlabel('Q scale'); ylabel('R scale'); title(tt);
end
lkf_savefig(h, fullfile('fig','lkf_sensitivity_2d.png'));

%% 7) RESULTS.md ----------------------------------------------------------------
fid = fopen('RESULTS.md','w');
fp = @(varargin) fprintf(fid, varargin{:});
fp('# Work package 03 - Linear Kalman Filter state estimation\n\n');
fp('All numbers are generated by `lkf_main.m` (Octave/MATLAB, portable `nav_randn` stream, seed 1), T = %.3f s.\n\n', T);
fp('## 1. Sensor statistics (real data `openloop_test01_synced.mat`, decimated to T = 0.02 s)\n\n');
fp('Standstill = `cmd_speed==0 & |vesc_speed|<0.01`, segments >= 2 s: %d segments, %d samples (%.1f s): ', numel(S.seg_len), S.n_standstill, S.n_standstill*T);
for i = 1:numel(S.seg_len), fp('[%.1f, %.1f] s  ', S.seg_start_t(i), S.seg_end_t(i)); end
fp('\n\n| quantity | a_x | a_y | gyro w_z |\n|---|---|---|---|\n');
fp('| noise std, standstill (within segment) | %.4f m/s^2 | %.4f m/s^2 | %.5f rad/s |\n', S.sigma_ax, S.sigma_ay, S.sigma_wz);
fp('| bias (pooled mean) | %.4f m/s^2 | %.4f m/s^2 | %.2e rad/s |\n', S.bias_ax, S.bias_ay, S.bias_wz);
fp('| segment means (first / last standstill) | %.4f / %.4f | %.4f / %.4f | %.2e / %.2e |\n', S.seg_mean_ax(1), S.seg_mean_ax(end), S.seg_mean_ay(1), S.seg_mean_ay(end), S.seg_mean_wz(1), S.seg_mean_wz(end));
fp('| bias repeatability sigma_bres (RMS of segment means) | %.4f | %.4f | - |\n', S.sigma_bres_x, S.sigma_bres_y);
fp('| noise density sqrt(PSD), 1-22 Hz, standstill | %.2e (m/s^2)/sqrt(Hz) | %.2e | - |\n', S.noise_density_ax, S.noise_density_ay);
fp('| white level implied by sigma: sqrt(2 sigma^2 T) | %.2e | %.2e | - |\n', sqrt(S.psd_white_expect_ax), sqrt(S.psd_white_expect_ay));
fp('| **in-motion** noise std (cmd_speed > 0.5 m/s, high-pass residual, vibration) | %.3f m/s^2 | %.3f m/s^2 | - |\n\n', S.sigma_ax_mot, S.sigma_ay_mot);
fp('Notes. imu_az carries gravity and is not used. The standstill PSD in the 1-22 Hz band is below the flat level implied by the sample std (see `fig/lkf_noise_psd.png`) because the standstill variance is dominated by low-frequency content (< 1 Hz drift/settling), so the sampled std used for Q is a conservative white-noise equivalent; velocity random walk equivalent = noise density = %.2e / %.2e m/s/sqrt(s). Bias random walk (change of the segment means %.0f s apart): q_b = %.2e (m/s^2)^2/s, sigma_bw = %.2e m/s^2/sqrt(s) (indicative, two standstills only). **In motion the accelerometer noise is about %.0f times larger than at standstill** (chassis/motor vibration), which matters for the real run.\n\n', ...
    S.vrw_x, S.vrw_y, S.dt_bias, S.q_bias, sqrt(S.q_bias), S.sigma_a_mot/S.sigma_a_white);
fp('Particle-filter pose (pf_x, pf_y): sample-to-sample jitter (2nd difference / sqrt(6)) = %.1f mm (x), %.1f mm (y). The PF pose is smooth and its absolute error cannot be observed from this data set, so this is only a lower bound. **Design decision:** the emulated 10 Hz position fix (GNSS / LiDAR-localisation-like, 0.5 m at full scale) has sigma_z = %.3f m per axis; heading fix sigma_psi = %.3f rad. Wheel speed: standstill std %.1e m/s, in-motion HF std %.4f m/s (only used in the optional speed variant, sigma_v = %.3f m/s incl. projection error).\n\n', ...
    1e3*S.pf_jitter_x, 1e3*S.pf_jitter_y, S.sigma_z, S.sigma_psi, S.vesc_std_standstill, S.sigma_v_hf, S.sigma_v);
fp('## 2. Filter formulation\n\n');
fp('* State x = [p_X v_X p_Y v_Y b_x b_y]^T: position/velocity in the global (map/track) frame, b = residual accelerometer bias in the body frame (random walk). Input u = [a_X a_Y] = Rot(psi)(a_meas - bias_cal): the measured body-frame a_x, a_y are rotated into the global frame by the heading psi **outside** the filter (heading = complementary filter of bias-corrected gyro and the 10 Hz heading fix, gain 0.1), so the filter is linear. psi is a known exogenous signal; it only enters the bias columns F(1:4,5:6) = -G Rot(psi_k) (linear time-varying, set by `lkf_fk`). The 4-state variant (no bias states, constant F) is kept as baseline.\n');
fp('* Per axis F_a = [1 T; 0 1], G_a = [T^2/2; T]; F = blkdiag(F_a,F_a), G = blkdiag(G_a,G_a), H = [1 0 0 0 0 0; 0 0 1 0 0 0]. Predict x- = F x + G u_bar, P- = F P F^T + Q; update (only when a fix is available) K = P- H^T (H P- H^T + R)^-1, x = x- + K (z - H x-), Joseph form P = (I-KH)P-(I-KH)^T + K R K^T. u_bar = (u(k-1)+u(k))/2 (causal interval mean; removes the half-sample bias of a left-Riemann input).\n');
fp('* Vehicle-frame formulation rejected: position would not be a linear state (heading enters F).\n\n');
fp('## 3. Q and R\n\n');
fp('* White accel. noise (isotropic so that Q is constant while body noise is rotated): sigma_w = sqrt((sigma_ax^2+sigma_ay^2)/2) = %.4f m/s^2 (standstill, used in simulation) or %.4f m/s^2 (in motion, used on the real run). Q_axis = sigma_w^2 [T^4/4 T^3/2; T^3/2 T^2]; standstill value = [%.3e %.3e; %.3e %.3e], in-motion value = [%.3e %.3e; %.3e %.3e].\n', ...
    sw_sim, sw_real, par6.Q(1,1), par6.Q(1,2), par6.Q(2,1), par6.Q(2,2), par6m.Q(1,1), par6m.Q(1,2), par6m.Q(2,1), par6m.Q(2,2));
fp('* Bias states: Q_b = q_b T I2 = %.3e (m/s^2)^2 per sample.\n', S.q_bias*T);
fp('* R = sigma_z^2 I2 = %.4e m^2 (sigma_z = %.3f m). P0 = diag(%.1e, %.1e, %.1e, %.1e, %.1e, %.1e).\n', par6.R(1,1), S.sigma_z, diag(par6.P0));
fp('* 4-state baseline: sigma_a^2 = sigma_w^2 + sigma_bres^2 (residual bias as extra white noise): sigma_a = %.4f m/s^2 (sim).\n\n', par4.sigma_a);
fp('Justification. The accelerometer error is integrated twice, so it enters the kinematic model as input noise through G (discrete white-noise-acceleration model); its std is measured on the same IMU (standstill for the simulation sensor model, in motion for the real run because vibration dominates there). The slow drift between the two standstills (bias change %.3f / %.3f m/s^2 over %.0f s) is not white, so it is represented by bias random-walk states with the measured intensity rather than by inflating Q. R is the variance of the position fix. The estimate depends only on the ratio Q/R, which the sensitivity study (Sec. 6) and the NIS test confirm to be at the optimum in simulation.\n\n', ...
    abs(S.seg_mean_ax(end)-S.seg_mean_ax(1)), abs(S.seg_mean_ay(end)-S.seg_mean_ay(1)), S.dt_bias);
fp('## 4. Results\n\nRMSE = sqrt(mean(e_X^2+e_Y^2)) over the whole record (all 50 Hz samples); reduction = 100 (1 - RMSE_LKF/RMSE_ref). "raw fixes (held)" = zero-order hold of the noisy 10 Hz fixes (50 Hz estimate without a filter); "raw fixes @fix" = error of the fixes at their own instants (= sigma_z sqrt 2).\n\n');
fp('| case | raw integration | raw, bias-corrected | raw fixes @fix | raw fixes (held) | LKF 4 st. | LKF 6 st. |\n|---|---|---|---|---|---|---|\n');
fp('| (b) simulation [m] | %.2f | %.2f | %.4f | %.4f | %.4f | **%.4f** |\n', Rs6.rmse_raw, Rs6.rmse_rawbc, Rs6.rmse_fix, Rs6.rmse_zoh, Rs4.rmse_lkf, Rs6.rmse_lkf);
fp('| (a) real run [m] (in-motion Q) | %.2f | %.2f | %.4f | %.4f | %.4f | **%.4f** |\n\n', Rr6.rmse_raw, Rr6.rmse_rawbc, Rr6.rmse_fix, Rr6.rmse_zoh, Rr4.rmse_lkf, Rr6.rmse_lkf);
fp('| reduction by LKF 6 st. | vs raw integration | vs bias-corrected raw | vs raw fixes @fix | vs held fixes |\n|---|---|---|---|---|\n');
fp('| (b) simulation | **%.2f %%** | %.2f %% | %.1f %% | %.1f %% |\n', Rs6.red_raw, Rs6.red_rawbc, Rs6.red_fix, Rs6.red_zoh);
fp('| (a) real run | **%.2f %%** | %.2f %% | %.1f %% | %.1f %% |\n\n', Rr6.red_raw, Rr6.red_rawbc, Rr6.red_fix, Rr6.red_zoh);
fp('* Real run with the standstill Q instead of the in-motion Q: RMSE %.4f m (%.2f %% vs raw integration) - over-confident (mean NIS %.1f instead of 2).\n', Rr6n.rmse_lkf, Rr6n.red_raw, crn.nis_mean);
fp('* Max position error: sim LKF %.3f m (raw %.0f m); real LKF %.3f m (raw %.0f m). RMSE at the fix instants: sim LKF %.4f m vs fix %.4f m; real LKF %.4f m vs fix %.4f m. Simulation: lap %.1f s, length %.1f m, max |a_y| = %.2f m/s^2, v_max %.2f m/s, LKF velocity RMSE %.3f m/s.\n', ...
    Rs6.max_lkf, Rs6.max_raw, Rr6.max_lkf, Rr6.max_raw, Rs6.rmse_lkf_atfix, Rs6.rmse_fix, Rr6.rmse_lkf_atfix, Rr6.rmse_fix, Cs.tr.lap_time, Cs.tr.length, Cs.tr.max_alat, max(Cs.tr.v), Rs6.rmse_vel);
fp('* Optional wheel-speed velocity measurement (v_wheel [cos psi; sin psi], R_v = sigma_v^2): simulation RMSE %.4f m (vs %.4f m without). On the real run it is **worse** (%.3f m) because the projection assumes no side slip (the car slides in the corners), so it is not used there.\n', Rs6v.rmse_lkf, Rs6.rmse_lkf, Rr6v.rmse_lkf);
fp('* Simulation set-up: 1:10 Mexico City centreline, periodic spline + Gaussian smoothing, curvature-limited speed (|a_lat| <= 2 design, realised max %.2f m/s^2), 10 s standstill for bias calibration (residual bias error ax %.4f, ay %.4f m/s^2), accelerometer noise/bias, gyro noise/bias, bias random walk q_b from the real data.\n', Cs.tr.max_alat, Cs.calib.ax - Cs.bias_true(1,1), Cs.calib.ay - Cs.bias_true(1,2));
fp('* Real case: accelerometer, gyro from the real run; position and heading fixes = PF pose at 10 Hz + Gaussian noise (sigma_z, sigma_psi); reference = noise-free PF pose; raw integration uses the same heading and initial state.\n');
fp('* **Caveat (real data):** in motion the IMU does not agree with the acceleration derived from the PF pose (lateral channel correlates ~0.96 but is about 1.3-1.4x larger by in-sample regression; longitudinal only ~0.5 correlated; roll/pitch gravity leakage, vibration), so the accelerometer error in motion is coloured (mean NIS stays above 2, innovations are not white) and the LKF is only on par with the held fixes (%.1f %% lower RMSE) and not below the fix noise at the fix instants. The >= 70 %% reduction versus raw integration holds with a huge margin in both cases.\n\n', Rr6.red_zoh);
fp('## 5. Consistency (design Q)\n\n');
fp('| case | n fixes | mean NIS (expected 2, 95 %% band) | norm. innovation autocorr. x lag 1/2/3 (band +-%.3f) | y lag 1 | mean pos. NEES (2 dof) | 3-sigma coverage |\n|---|---|---|---|---|---|---|\n', cs6.ac_bound);
fp('| sim | %d | %.2f (%.2f..%.2f) | %.2f / %.2f / %.2f | %.2f | %.2f | %.1f %% |\n', cs6.n_fix, cs6.nis_mean, cs6.nis_lo, cs6.nis_hi, cs6.ac(1,:), cs6.ac(2,1), cs6.nees_mean, 100*Rs6.cover3);
fp('| real, in-motion Q | %d | %.2f (%.2f..%.2f) | %.2f / %.2f / %.2f | %.2f | %.2f | %.1f %% |\n', cr6.n_fix, cr6.nis_mean, cr6.nis_lo, cr6.nis_hi, cr6.ac(1,:), cr6.ac(2,1), cr6.nees_mean, 100*Rr6.cover3);
fp('| real, standstill Q | %d | %.2f (%.2f..%.2f) | %.2f / %.2f / %.2f | %.2f | %.2f | %.1f %% |\n\n', crn.n_fix, crn.nis_mean, crn.nis_lo, crn.nis_hi, crn.ac(1,:), crn.ac(2,1), crn.nees_mean, 100*Rr6n.cover3);
fp('## 6. Sensitivity (Q and R scaled around the design values, nominal = 1)\n\n');
fp('The estimate depends on Q/R only, so scaling Q by k equals scaling R by 1/k. RMSE [cm] (rows R scale 0.1, 0.32, 1, 3.2, 10; columns Q scale 0.1 ... 10).\n\nSimulation (standstill Q):\n\n');
for ir = 1:5, fp('    %s\n', sprintf('%7.2f', 100*H_s(ir,:))); end
fp('\nReal run (in-motion Q):\n\n');
for ir = 1:5, fp('    %s\n', sprintf('%7.2f', 100*H_r(ir,:))); end
fp('\nQ sweep (R fixed), RMSE [cm] / mean NIS: scales %s\n\n    sim  RMSE %s\n    sim  NIS  %s\n    real RMSE %s\n    real NIS  %s\n\n', ...
    sprintf('%.2f ', sc_q), sprintf('%.2f ', 100*rq_s), sprintf('%.2f ', nq_s), sprintf('%.2f ', 100*rq_r), sprintf('%.2f ', nq_r));
fp('Simulation: nominal %.2f cm; best of the Q sweep %.2f cm at Q scale %.2f, i.e. the justified tuning is optimal within %.1f %% and NIS = %.2f (expected 2). Real run: nominal %.2f cm; best of the sweep %.2f cm at Q scale %.2f (NIS %.2f).\n\n', ...
    100*Rs6.rmse_lkf, 100*rq_s(ibest_s), sc_q(ibest_s), 100*(Rs6.rmse_lkf/rq_s(ibest_s)-1), cs6.nis_mean, 100*Rr6.rmse_lkf, 100*rq_r(ibest_r), sc_q(ibest_r), nq_r(ibest_r));
fp('## 7. Files\n\n`lkf_main.m`, `lkf_design.m`, `lkf_step.m` (code-gen compatible, `[x,P,nu,S]=lkf_step(x,P,u,z,zvalid,par)`), `lkf_fk.m` (heading-dependent bias columns of F: call `par=lkf_fk(par,psi_mid)` before `lkf_step` for the 6-state model), helpers `lkf_*.m`, `lkf_params.mat`, `fig/*.png`.\n');
fclose(fid);
fprintf('Done. Sim: LKF %.4f m vs raw %.2f m (%.2f %%). Real: LKF %.4f m vs raw %.2f m (%.2f %%).\n', ...
    Rs6.rmse_lkf, Rs6.rmse_raw, Rs6.red_raw, Rr6.rmse_lkf, Rr6.rmse_raw, Rr6.red_raw);
