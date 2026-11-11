function p = ekf_params()
%EKF_PARAMS  Default parameter set of the MR2006B estimator and its surroundings.
%
%   p = ekf_params() returns a flat struct whose field names and default values
%   are copied from FIRMWARE_SPEC.md §10 (parameter catalogue) and the numbers
%   given in the text of §7.11, §7.14, §7.15, §7.17, §7.19 and §7.21.
%
%   NOTHING HERE IS MEASURED unless §10 says so. Every value tagged [MEASURE],
%   [VERIFY] or [DECIDE] in the spec is a placeholder and is marked below with
%   "% [MEASURE]" etc. Do not read these numbers as properties of the vehicle.
%
%   Groups:
%     - spec parameters (IDs in §10)                    -> same name as par_table.h
%     - spec constants from the text (no §10 ID)        -> prefix kept, comment "text"
%     - MATLAB-only switches and proposals (not in spec) -> prefix  mx_
%
%   The struct is double precision. ekf_param_cast(p, 'single') makes the
%   float32 variant that the firmware uses (one switch, see run_all.m).
%
%   See also EKF_PARAM_CAST, EKF_INIT.

% ------------------------------------------------------------- odometry (§10.2)
p.odo_m_per_cnt   = 6.3638e-5;   % 200  m      pi*0.0726/3584, measured 8 Oct (M)
p.odo_scale_l     = 1.0;         % 201  -      [MEASURE] UMBmark
p.odo_scale_r     = 1.0;         % 202  -      [MEASURE] UMBmark
p.odo_wheelbase   = 0.22325;     % 203  m      geometric; effective [MEASURE] UMBmark

% ------------------------------------------------------------- IMU (§10.2, §7.11)
p.imu_timeout     = 0.020;       % 403  s
p.imu_extrap_max  = 0.002;       % 404  s      extrapolation limit of the yaw integral
p.imu_odr_hz      = 833;         % gyro sample rate, Hz. LSM6DS3 (replaces the LSM6DSV): gyro ODRs are
                                 % 12.5/26/52/104/208/416/833/1666 Hz, there is no 960 Hz. 833 Hz is
                                 % [VERIFY] pending the firmware decision (§7.11.1). Must be an integer
                                 % below 1000 here (at most one sample per 1 kHz tick, sim_yaw_step).
                                 % The white-noise level is ekf_gyro_nd (503) below; it stays the spec
                                 % placeholder [MEASURE]. The LSM6DS3 datasheet rate noise density must
                                 % be looked up and compared with it [VERIFY]; no datasheet number is
                                 % assumed in this folder.
p.imu_gyro_lsb    = 17.50e-3*pi/180; % text §7.11.1, rad/s per LSB at +-500 dps [VERIFY] for the LSM6DS3
p.imu_ring_n      = 32;          % text §7.11.3 ring length of the yaw integral (32/833 Hz = 38.4 ms,
                                 % covers one 20 ms EKF period; checked in verify_static)

% ------------------------------------------------------------- estimator (§10.3)
p.ekf_kd          = 1.0e-4;      % 500  m           distance noise [MEASURE] repeated D5000
p.ekf_slip_infl   = 100;         % 501  -
p.ekf_slip_thr    = 0.15;        % 502  rad/s
p.ekf_gyro_nd     = 1.0e-4;      % 503  rad/s/sqrt(Hz) [MEASURE] Allan variance
p.ekf_gyro_sf     = 0.003;       % 504  -           gyro scale-factor uncertainty (E)
p.ekf_q_bg        = 1.0e-10;     % 505  (rad/s)^2/s [MEASURE] Allan variance
p.ekf_q_s         = 1.0e-9;      % 506  1/s
p.ekf_q_th_enc    = 1.0e-4;      % 507  rad^2/m     encoder-heading mode
p.ekf_zupt_floor  = 3.5e-5;      % 508  rad/s
p.ekf_zupt_T      = 1.0;         % 509  s
p.ekf_still_s     = 0.2;         % 510  s
p.ekf_gate1       = 9.0;         % 511  -           chi2_1 99.7 %
p.ekf_gate2       = 9.21;        % 512  -           chi2_2 99 %
p.ekf_sig_edge    = 0.02;        % 513  m           [MEASURE] outdoor
p.ekf_sig_post    = 0.03;        % 514  m
p.ekf_edge_skip   = 0.25;        % 515  m
p.ekf_bg_conv     = 3.5e-4;      % 516  rad/s       0.02 deg/s
p.ekf_p0_xy       = 0.01;        % 517  m (sigma)
p.ekf_p0_th       = 0.0087;      % 518  rad (sigma)  0.5 deg
p.ekf_p0_bg       = 0.0087;      % 519  rad/s (sigma) 0.5 deg/s
p.ekf_p0_s        = 0.005;       % 520  - (sigma)

% constants from the text of §7.17 (no parameter ID)
p.ekf_hist_n      = 256;         % §7.17.1 pose-history ring length (1.28 s at 200 Hz, 5.12 s at 50 Hz)
p.ekf_meas_max_age = 1.0;        % §7.17.1 measurements older than this are discarded, s
p.ekf_slip_n      = 3;           % §7.17.2 consecutive steps over threshold
p.ekf_face_g_min  = 0.3;         % REQ-EKF-06
p.ekf_zupt_min_T  = 0.5;         % §7.17.4 shortest partial ZUPT window that is still applied, s
p.ekf_pxy_max     = 25;          % REQ-EKF-05 step 5, m^2
p.ekf_pth_max     = 1;           % REQ-EKF-05 step 5, rad^2
p.ekf_sat_infl    = 100;         % REQ-IMU-07 q_theta inflation for a saturated step
p.ekf_lm_y_off_max = 0.1;        % REQ-EKF-08, m

% ------------------------------------------------------------- sensors and geometry (§10.4)
p.vl_tb_ms        = 20;          % 600  ms   timing budget
p.vl_imp_ms       = 20;          % 601  ms   inter-measurement [VERIFY] >= TB rule
p.vl_poll_ms      = 5;           % 602  ms
p.gnd_x           = 0.15;        % 610  m    [MEASURE]
p.gnd_y_in        = 0.15;        % 611  m    [DECIDE] D-4
p.gnd_y_out       = 0.25;        % 612  m    [DECIDE] D-4
p.gnd_h0          = 0.12;        % 613  m    [MEASURE]
p.gnd_n_deb       = 2;           % 619  -
p.lat_x           = 0.0;         % 650  m    [MEASURE]
p.lat_y           = 0.175;       % 651  m    [MEASURE]
p.lat_r_min       = 0.05;        % 652  m
p.lat_r_max       = 1.2;         % 653  m
p.lat_n_min       = 50;          % 654  -
p.lat_n_max       = 400;         % 655  -
p.lat_span_min    = 1.0;         % 656  m
p.lat_rms_max     = 0.03;        % 657  m
p.lat_notch_depth = 0.15;        % 658  m
p.lat_sig_face_min = 8.7e-4;     % 659  rad  0.05 deg
p.lat_dth_max     = 0.5*pi/180;  % text §7.15: heading change over the window < 0.5 deg

% ------------------------------------------------------------- drive limits (§10.1)
p.drv_v_max       = 0.45;        % 154  m/s
p.drv_w_max       = 2.5;         % 155  rad/s
p.drv_a_max       = 0.3;         % 156  m/s^2
p.drv_a_stop      = 0.5;         % 157  m/s^2
p.drv_alpha_max   = 3.0;         % 158  rad/s^2
p.drv_v_whl_max   = 0.45;        % 159  m/s
p.drv_pos_tol     = 0.002;       % 150  m

% ------------------------------------------------------------- guidance (§10.5, §7.19)
p.nav_wn          = 1.5;         % 700  rad/s
p.nav_zeta        = 0.7;         % 701  -
p.nav_v_min       = 0.05;        % 702  m/s
p.nav_a_acc       = 0.3;         % 703  m/s^2
p.nav_a_dec       = 0.3;         % 704  m/s^2
p.nav_w_max       = 1.0;         % 705  rad/s
p.nav_final_dist  = 0.30;        % 706  m
p.nav_turn_w      = 0.8;         % 707  rad/s
p.nav_turn_ta     = 0.4;         % 708  s
p.nav_turn_k      = 2.0;         % 709  1/s
p.nav_turn_tol    = 0.0035;      % 710  rad (0.2 deg)

% ------------------------------------------------------------- mission (§10.5)
p.msn_init_min_s  = 3.0;         % 800  s
p.msn_init_max_s  = 30;          % 801  s
p.msn_preturn_s   = 1.0;         % 802  s
p.msn_wp_hold     = 10.5;        % 814  s

% ------------------------------------------------------------- MATLAB-only switches (NOT in the spec)
% Each one selects between the literal spec text (value given first) and the
% variant this design work found to be correct. See README.md "Spec findings".
p.mx_still_init   = 1;   % 1: ekf_init starts with the standstill timer full (vehicle is
                         %    stationary at AUTO_INIT); 0: literal (first 0.2 s counted as moving)
p.mx_still_F_bg   = 0;   % 0: F(3,4)=0 while still (heading frozen, D-12); 1: literal F(3,4)=-dt
p.mx_hist_corr    = 1;   % 1: pose history excludes measurement corrections (back-shift = motion
                         %    only); 0: raw poses (corrections leak into the back-shift)
p.mx_sf_mode      = 1;   % 1: per-radian model Q_tt += sf^2*mx_sf_corr*|dth| (rate-independent,
                         %    finding F-3); 0: literal spec Q_tt += (sf*dth)^2 per step;
                         % 2: net-angle model Q_tt += sf^2*max(0, (Th+dth)^2 - Th^2), Th = net
                         %    angle turned since init/reset (finding F-9; consistent for any
                         %    number of turns in one direction, see verify_turn_nees)
p.mx_sf_corr      = 2*pi;% correlation angle of the per-radian model, rad [DECIDE]: one full
                         %    circuit of turning, so (sf*Theta)^2 is matched for Theta = 2 pi
p.mx_slip_abs     = 0;   % 0: literal spec, slip inflates kd*|ds_e| (nothing in an in-place turn,
                         %    where ds_e ~ 0); 1: while slip, use (|ds_l| + |ds_r|)/2 instead
                         %    (finding F-11, see verify_slip)
p.mx_freeze_s     = 0;   % 1: odometry scale state frozen (no Q_ss, P_ss = 0) for the observability study
end
