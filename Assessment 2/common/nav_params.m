function P = nav_params(cfg, root)
%NAV_PARAMS  Collect all numeric parameters of the navigation loop in one struct (no strings,
%   fixed-size fields, usable as Simulink parameter data).
M = load(fullfile(root,'01_sysid','model_id.mat'));
G = load(fullfile(root,'02_pid','pid_gains.mat'));
K = load(fullfile(root,'03_lkf','lkf_params.mat'));
P.T = M.T;
P.par = struct('L',M.par.L,'delta_max',M.par.delta_max,'Kus',M.par.Kus,'cs',M.par.cs,'k_delta',M.par.k_delta);
% identified plant (difference equations)
P.av = -M.Gv.den(2); P.bv = M.Gv.num(2);                 % v(k+1) = av v(k) + bv v_cmd(k)
P.a1r = -M.Gr.den(2); P.a2r = -M.Gr.den(3); P.br = M.Gr.num(4);  % r(k) = a1 r(k-1)+a2 r(k-2)+br u(k-3)
P.kv_scale = cfg.kv_scale; P.kr_scale = cfg.kr_scale; P.vcmd_max = 3.0;
% PID structs (numeric fields only)
f = {'Kp','Ki','Kd','pf','T','umin','umax','aw','Kt','Nc','Nc1','v0'};
P.lon = nav_pick(G.pid_lon, f); P.lon.v0 = 2;
P.lat = nav_pick(G.pid_lat, f);
% LKF: noise level and 6-state design
S = K.stats;
if strcmp(cfg.noise, 'motion')
    sax = S.sigma_ax_mot; say = S.sigma_ay_mot; sw = S.sigma_a_mot;
else
    sax = S.sigma_ax; say = S.sigma_ay; sw = S.sigma_a_white;
end
lp = lkf_design(S, 1, 1, 1, cfg.usev, sw);
P.lkf = struct('F',lp.F,'G',lp.G,'H',lp.H,'Q',lp.Q,'R',lp.R,'P0',lp.P0);
P.sens = struct('sax',sax,'say',say,'swz',S.sigma_wz,'sz',S.sigma_z,'spsi',S.sigma_psi, ...
                'svw',S.sigma_v_hf,'bax',S.bias_ax,'bay',S.bias_ay,'bwz',S.bias_wz,'qb',S.q_bias,'sw',sw);
P.fix_step = round(1/(S.fix_rate*P.T));      % 10 Hz fixes on the 50 Hz grid
P.Kc = cfg.Kc;
P.nst = round(cfg.t_stand/P.T);              % standstill samples (calibration, controllers idle)
P.nc = P.nst;
P.tauc = cfg.tauc; P.usev = cfg.usev; P.servo_rate = cfg.servo_rate; P.dmode = cfg.dmode;
P.tla = cfg.tla; P.ffmode = cfg.ffmode; P.vlat_min = cfg.vlat_min; P.W = cfg.W;
end
