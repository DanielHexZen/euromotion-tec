function cfg = sim_config(scenario, seed, varargin)
%SIM_CONFIG  Scenario, truth model and run settings for sim_run.
%
%   cfg = sim_config(scenario, seed, 'name', value, ...)
%
%   scenario  'rect10' | 'rect10_lm' | 'rect10_all' | 'line40' | 'maneuver' | 'static'
%   seed      integer; every random quantity of the run derives from it
%
%   Options (name/value):
%     'cls'        'single' (default, as the firmware) or 'double'
%     'ekf_rate'   50 (default, D-30: EKF inside S_CTRL) or 200 (S_EST design)
%     'truth'      'prior' (default): uncertain truth quantities are DRAWN from
%                  the filter's own prior/noise model (start pose from P0,
%                  initial bias from P0_bg, scale from P0_s, scale-factor error
%                  from ekf_gyro_sf, noise = ekf_gyro_nd, ekf_q_bg) so that NEES
%                  tests the filter equations, not the parameter guesses;
%                  'spec_line': the fixed numbers of t_ekf_line (scale +0.5 %, ...)
%     'gnd'        0 off, 1 physical boundary crossings (default), 2 periodic
%                  synthetic edge events every tr.gnd_period_m (t_ekf_line)
%     'p'          parameter struct to use instead of ekf_params()
%     'T_end'      override of the run length, s
%
%   cfg.tr holds the TRUTH. Everything in it is a simulation assumption, not a
%   vehicle property; see README "What must be measured".

opt = struct('cls', 'single', 'ekf_rate', 50, 'truth', 'prior', 'gnd', 1, 'p', [], 'T_end', 0);
for k = 1:2:numel(varargin)
    opt.(varargin{k}) = varargin{k+1};
end
if isempty(opt.p), p = ekf_params(); else, p = opt.p; end

cfg.scenario = scenario;
cfg.seed     = seed;
cfg.cls      = opt.cls;
cfg.ekf_rate = opt.ekf_rate;
cfg.p        = p;
course_name  = scenario;
if strcmp(scenario, 'static'), course_name = 'maneuver'; end
cfg.course   = sim_course(course_name);

% ------------------------------------------------------------ truth defaults
rs = sim_rng_init(seed, 99);
tr.seed     = seed;
tr.b_true   = p.odo_wheelbase*1.02;      % effective wheelbase 2 % larger (casters) [assumption]
tr.tau_w    = 1/6.8;                     % closed speed loop, crossover 6.8 rad/s (§7.19 text)
tr.eps_lr   = 0.0064;                    % D_R/D_L - 1 hand-turn value (tech. ref. §4.7)
tr.n_slip   = 0;
tr.slip_t0  = zeros(1,4); tr.slip_t1 = zeros(1,4); tr.slip_side = zeros(1,4); tr.slip_frac = zeros(1,4);
% gyro
tr.imu_on   = 1;
tr.f_imu    = p.imu_odr_hz;
tr.imu_t0   = 1/(2*lcm(1000, round(p.imu_odr_hz)));  % DRDY phase: half the common grid of 1 kHz and
                                         % f_imu, so a sample never coincides with a 1 kHz tick
                                         % (833 Hz: 1/1666000 s; 960 Hz: 1/48000 s)
tr.nd       = p.ekf_gyro_nd;
tr.q_bg     = p.ekf_q_bg;
tr.bg_ramp  = 0;
tr.lsb      = p.imu_gyro_lsb;
% ground sensors
tr.gnd_mode     = opt.gnd;
tr.gnd_period_m = 0.5;
tr.sig_edge_true = p.ekf_sig_edge;       % periodic mode: footprint-on-line error
tr.sig_spot     = 0.010;                 % physical mode: spot-position noise per sample [assumption]
tr.sig_wobble   = 0.015;                 % physical mode: raggedness of the true boundary [assumption]
tr.wob_lmin     = 0.3;  tr.wob_lmax = 3.0;  % wavelengths of the raggedness, m
tr.bnd_offset   = 0;                     % survey error of the boundary line, m
tr.gnd_outlier_p = 0;                    % periodic mode: probability of a false event (gate study)
tr.gnd_outlier_m = 0.20;                 % its offset magnitude, m
% lateral sensors
tr.lat_on   = 0;
tr.sig_lat_r = 0.02;                     % pessimistic sigma_r of tech. ref. §4.10
tr.lm_err   = zeros(4, 2);               % true minus surveyed landmark position, m

switch opt.truth
    case 'prior'
        [rs, n1] = sim_randn(rs);  [rs, n2] = sim_randn(rs);  [rs, n3] = sim_randn(rs);
        tr.start_err = [n1*p.ekf_p0_xy, n2*p.ekf_p0_xy, n3*p.ekf_p0_th];
        [rs, n4] = sim_randn(rs);  tr.bg0 = n4*p.ekf_p0_bg;
        [rs, n5] = sim_randn(rs);  tr.scale = 1 + n5*p.ekf_p0_s;
        [rs, n6] = sim_randn(rs);  tr.sf = n6*p.ekf_gyro_sf;
    case 'spec_line'
        tr.start_err = [0 0 0];
        tr.bg0   = 0.5*pi/180;               % 0.5 deg/s, removed by the INIT ZUPT
        tr.scale = 1.005;                    % scale error 0.5 %
        tr.sf    = 0.001;
        tr.bg_ramp = 1e-6;                   % "gyro bias drift": 1e-6 rad/s^2 on top of the random walk
    otherwise
        error('sim_config: truth %s', opt.truth);
end
for k = 1:4                               % survey error of the landmarks (truth - map)
    [rs, n7] = sim_randn(rs);  [rs, n8] = sim_randn(rs);
    tr.lm_err(k, :) = [n7 n8]*0.02;
end
tr.kL = tr.scale*(1 - tr.eps_lr/2);
tr.kR = tr.scale*(1 + tr.eps_lr/2);

switch scenario
    case 'line40',    T = 160;
    case 'maneuver',  T = cfg.course.ol_t(cfg.course.n_ol);
    case 'static',    T = 60;
    otherwise,        T = 240;
end
if strcmp(scenario, 'rect10_lm'), tr.lat_on = 1; end
if strcmp(scenario, 'maneuver') || strcmp(scenario, 'static'), tr.gnd_mode = 0; end
if opt.T_end > 0, T = opt.T_end; end
cfg.T_end = T;
cfg.tr = tr;
cfg.bk_query = strcmp(scenario, 'maneuver');
cfg.log_rate = 50;                       % logging rate of est outputs at 200 Hz runs
end
