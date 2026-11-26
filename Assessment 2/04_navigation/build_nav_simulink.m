% BUILD_NAV_SIMULINK  Build nav_closed_loop.slx programmatically, simulate it and compare with nav_sim.
%   The model is a fixed-step discrete (T = 0.02 s) closed loop made of MATLAB Function blocks that
%   call exactly the same .m functions as nav_main (pid_step, lkf_step, nav_project, ...):
%     Clock -> StepIdx (k) ; Reference (track lookup) ; Sensors (noise from From Workspace) ;
%     Estimator (bias calibration + heading filter + LKF) ; Projection (estimated) ;
%     Longitudinal PID ; Lateral PID ; Vehicle plant (identified Gv/Gr + kinematic bicycle) ;
%     Unit Delay on the plant state (breaks the loop), Projection (true, for evaluation).
%   Constant parameters are Parameter-scope data of the MATLAB Function blocks taken from base
%   workspace structs P, trk, REFF. Sensor noise sequences E_in = [t E] (7 error columns) come from
%   the portable generator nav_randn (identical in MATLAB and Octave, seed 1, same as nav_main run 1)
%   and are fed through a From Workspace block.
%   Fallback if struct parameters are rejected: replace the parameter data by
%   'S = coder.load(''nav_sl_data.mat''); P = S.P;' inside the scripts.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'common'), fullfile(root,'02_pid'), fullfile(root,'03_lkf'));
here = fileparts(mfilename('fullpath'));
mdl = 'nav_closed_loop';

% ---- data (identical to the nominal run of nav_main) ----
TR = load(fullfile(here,'track.mat'));
cfg = nav_cfg_default();
P = nav_params(cfg, root);
vper = nav_speed_profile(TR.trk.kap, TR.trk.ds, cfg.vmax, cfg.alat, cfg.aacc, cfg.adec, P.par);
ref = nav_reference(TR.trk, vper, P.T, cfg.aacc, cfg.runout);
E = nav_noise(P, P.nst + numel(ref.s) + 5, cfg.seed);
N = P.nst + numel(ref.s);
Ts = P.T;
xp0 = nav_plant_init(TR.trk, cfg.offset0);
P.xp0 = xp0;
trk = struct('N', TR.trk.N, 'ds', TR.trk.ds, 'L', TR.trk.L, 'x', TR.trk.x, 'y', TR.trk.y, ...
             'psi', TR.trk.psi, 'kap', TR.trk.kap);
REFF = struct('s', [zeros(P.nst,1); ref.s(:); ref.s(end)], 'v', [zeros(P.nst,1); ref.v(:); ref.v(end)]);
E_in = [(0:N-1)'*Ts, E(1:N,:)];
assignin('base','P',P); assignin('base','trk',trk); assignin('base','REFF',REFF);
assignin('base','Ts',Ts); assignin('base','xp0',xp0); assignin('base','E_in',E_in);
save(fullfile(here,'nav_sl_data.mat'), 'P', 'trk', 'REFF', 'Ts', 'xp0', 'E_in');

% ---- model ----
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl); open_system(mdl);
set_param(mdl, 'SolverType','Fixed-step', 'Solver','FixedStepDiscrete', 'FixedStep','Ts', ...
    'StartTime','0', 'StopTime', num2str((N-1)*Ts, '%.10g'));

sc_k   = sprintf('function k = nav_blk_k(t)\nk = round(t/P.T) + 1;\n');
sc_ref = sprintf(['function [sr, vr, vrn] = nav_blk_ref(k)\nn = numel(REFF.s);\ni = min(max(round(k),1), n);\n' ...
    'sr = REFF.s(i); vr = REFF.v(i); vrn = REFF.v(min(i+1,n));\n']);
sc_sen = sprintf(['function y = nav_blk_sens(xp, e, k)\npersistent vprev\nif isempty(vprev), vprev = 0; end\n' ...
    '[y, vprev] = nav_sensor_step(xp(:), vprev, e(:)'', k, P);\n']);
sc_est = sprintf(['function est = nav_blk_est(y, k)\npersistent xs\nif isempty(xs)\n    xs = nav_est_init(P, P.xp0(1:2), P.xp0(3));\nend\n' ...
    '[xs, est] = nav_est_step(xs, y(:), k, P);\n']);
sc_prj = sprintf(['function [s, ey, psip] = nav_blk_proj(xp)\npersistent ic\nif isempty(ic), ic = 1; end\n' ...
    '[s, ey, psip, ~, ic] = nav_project(xp(1), xp(2), ic, trk, P.W);\n']);
sc_lon = sprintf(['function vcmd = nav_blk_lon(sh, sr, vr, vrn, k)\npersistent ls\nif isempty(ls), ls = nav_lon_init(); end\n' ...
    '[vcmd, ls] = nav_lon_step(ls, sh, sr, vr, vrn, double(k > P.nst), P);\n']);
sc_lat = sprintf(['function [delta, ulat] = nav_blk_lat(est, sh, eyh, psip)\npersistent ls\nif isempty(ls), ls = nav_lat_init(); end\n' ...
    'vh = hypot(est(3), est(4));\n[delta, ls, ulat] = nav_lat_step(ls, eyh, vh, sh, psip, est(3:4), trk, P);\n']);
sc_pla = sprintf(['function xn = nav_blk_plant(xp, vcmd, delta)\nxn = nav_plant_step(xp(:), vcmd, delta, P);\n']);

add_block('simulink/Sources/Digital Clock', [mdl '/Clock'], 'SampleTime', 'Ts', 'Position', [20 20 60 50]);
nav_sl_block(mdl, 'StepIdx',    sc_k,   {'k','1'}, {'P'}, [100 20 180 50]);
nav_sl_block(mdl, 'Reference',  sc_ref, {'sr','1'; 'vr','1'; 'vrn','1'}, {'REFF'}, [240 20 340 90]);
add_block('simulink/Sources/From Workspace', [mdl '/Noise'], 'VariableName','E_in', 'SampleTime','Ts', ...
    'Interpolate','off', 'OutputAfterFinalValue','Holding final value', 'Position', [20 120 80 150]);
nav_sl_block(mdl, 'Sensors',    sc_sen, {'y','8'}, {'P'}, [240 120 340 190]);
nav_sl_block(mdl, 'Estimator',  sc_est, {'est','7'}, {'P'}, [400 120 500 170]);
nav_sl_block(mdl, 'ProjEst',    sc_prj, {'s','1'; 'ey','1'; 'psip','1'}, {'P','trk'}, [560 120 660 190]);
nav_sl_block(mdl, 'LonPID',     sc_lon, {'vcmd','1'}, {'P'}, [740 20 840 90]);
nav_sl_block(mdl, 'LatPID',     sc_lat, {'delta','1'; 'ulat','1'}, {'P','trk'}, [740 120 840 190]);
nav_sl_block(mdl, 'Plant',      sc_pla, {'xn','9'}, {'P'}, [920 60 1020 130]);
add_block('simulink/Discrete/Unit Delay', [mdl '/PlantState'], 'InitialCondition','xp0', 'SampleTime','Ts', ...
    'Position', [1060 60 1100 100]);
nav_sl_block(mdl, 'ProjTrue',   sc_prj, {'s','1'; 'ey','1'; 'psip','1'}, {'P','trk'}, [240 230 340 300]);

% logging
logs = {'log_xp','xp'; 'log_est','est'; 'log_vcmd','vcmd'; 'log_delta','delta'; 'log_ulat','ulat'; ...
        'log_ptrue','ptrue'; 'log_pest','pest'};
for i = 1:size(logs,1)
    add_block('simulink/Sinks/To Workspace', [mdl '/' logs{i,1}], 'VariableName', logs{i,1}, ...
        'SaveFormat','Array', 'SampleTime','Ts', 'MaxDataPoints','inf', 'Position', [1160 20+45*i 1230 50+45*i]);
end
add_block('simulink/Signal Routing/Mux', [mdl '/MuxTrue'], 'Inputs','3', 'Position', [400 230 405 300]);
add_block('simulink/Signal Routing/Mux', [mdl '/MuxEst'],  'Inputs','3', 'Position', [720 230 725 300]);

% wiring
a = @(s,d) add_line(mdl, s, d, 'autorouting', 'on');
a('Clock/1','StepIdx/1');
a('StepIdx/1','Reference/1'); a('StepIdx/1','Sensors/3'); a('StepIdx/1','Estimator/2'); a('StepIdx/1','LonPID/5');
a('PlantState/1','Sensors/1'); a('PlantState/1','ProjTrue/1'); a('PlantState/1','Plant/1'); a('PlantState/1','log_xp/1');
a('Noise/1','Sensors/2');
a('Sensors/1','Estimator/1');
a('Estimator/1','ProjEst/1'); a('Estimator/1','LatPID/1'); a('Estimator/1','log_est/1');
a('ProjEst/1','LonPID/1'); a('ProjEst/1','LatPID/2'); a('ProjEst/2','LatPID/3'); a('ProjEst/3','LatPID/4');
a('Reference/1','LonPID/2'); a('Reference/2','LonPID/3'); a('Reference/3','LonPID/4');
a('LonPID/1','Plant/2'); a('LatPID/1','Plant/3');
a('Plant/1','PlantState/1');
a('LonPID/1','log_vcmd/1'); a('LatPID/1','log_delta/1'); a('LatPID/2','log_ulat/1');
a('ProjTrue/1','MuxTrue/1'); a('ProjTrue/2','MuxTrue/2'); a('ProjTrue/3','MuxTrue/3'); a('MuxTrue/1','log_ptrue/1');
a('ProjEst/1','MuxEst/1'); a('ProjEst/2','MuxEst/2'); a('ProjEst/3','MuxEst/3'); a('MuxEst/1','log_pest/1');
save_system(mdl, fullfile(here, [mdl '.slx']));

% ---- simulate and compare with nav_sim ----
so = sim(mdl);
xp = nav_sl_unlog(so.get('log_xp')); est = nav_sl_unlog(so.get('log_est'));
dl = nav_sl_unlog(so.get('log_delta')); vc = nav_sl_unlog(so.get('log_vcmd'));
pt = nav_sl_unlog(so.get('log_ptrue'));
ms = nav_sim(cfg, TR.trk, ref, P, E);
n = min(size(xp,1), size(ms.X,1));
fprintf('Simulink samples: %d, nav_sim samples: %d\n', size(xp,1), size(ms.X,1));
fprintf('max |position diff|  (true plant state) : %.3e m\n', max(max(abs(xp(1:n,1:2) - ms.X(1:n,1:2)))));
fprintf('max |estimate diff|  (LKF position)       : %.3e m\n', max(max(abs(est(1:n,1:2) - ms.est(1:n,1:2)))));
fprintf('max |steering diff|                       : %.3e rad\n', max(abs(dl(1:n) - ms.cmd(1:n,2))));
fprintf('max |v_cmd diff|                          : %.3e m/s\n', max(abs(vc(1:n) - ms.cmd(1:n,1))));
w = (P.nst+1):n;
fprintf('CTE (true): Simulink max %.4f rms %.4f | nav_sim max %.4f rms %.4f m\n', max(abs(pt(w,2))), sqrt(mean(pt(w,2).^2)), ...
    max(abs(ms.proj(w,5))), sqrt(mean(ms.proj(w,5).^2)));
