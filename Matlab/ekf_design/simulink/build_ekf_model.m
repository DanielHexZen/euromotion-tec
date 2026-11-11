function mdl = build_ekf_model(cfg, mdl)
%BUILD_EKF_MODEL  Create the multirate Simulink model of vehicle, sensors, EKF and guidance.
%
%   mdl = build_ekf_model(cfg)            cfg from sim_config (default rect10, seed 1)
%   mdl = build_ekf_model(cfg, 'name')
%
%   NOT EXECUTED BY THE AUTHOR: no MATLAB/Simulink was available when this
%   file was written. It uses only documented APIs (new_system, add_block,
%   add_line, set_param, sfroot / Stateflow.EMChart.Script) and targets
%   MATLAB/Simulink R2020b or newer (no Stateflow licence needed for MATLAB
%   Function blocks). Expect small fixes on the first run.
%
%   One source of truth: every MATLAB Function block contains only a
%   persistent state and ONE call of the same step function that the script
%   reference sim_run.m calls (sim_plant_step, sim_imu_step, sim_yaw_step,
%   sim_enc_step, sim_gnd_step, sim_lat_step, sim_est_step -> ekf_*, sim_gdn_step).
%
%   Structure (mirrors the firmware, D-30):
%     Plant            1 kHz          truth: first-order wheels, mismatch, wheelbase, slip
%     Sensors/Encoders 1 kHz          enc_snapshot_isr: counts + t_us
%     Sensors/IMU      960 Hz, offset 1/48000 s   gyro with bias, RW, noise, SF, LSB
%     Sensors/Ground   1 kHz          G1..G4 ranging 50 Hz, polling, debounce, edge events
%     Sensors/Lateral  1 kHz          S1/S2 ranging 50 Hz (EXTI timestamps)
%     Estimator/YawIntegral 1 kHz     imu_drain: yaw-integral ring (960 Hz samples)
%     Estimator/EKF    50 Hz, offset 1 ms   app_est in S_CTRL: predict, ZUPT, aiding
%     Guidance         50 Hz, offset 1 ms   msn + nav + drv_vel, after the EKF
%     CmdDelay         Unit Delay 1 kHz: drv output acts from the next tick (no algebraic loop)
%     StatusDelay      Unit Delay 50 Hz: EKF uses the drv status of the previous control tick
%   Solver: fixed-step discrete, base step 1/48000 s (the 960 Hz samples never
%   coincide with a 1 kHz tick, so the execution order equals sim_run's),
%   single-tasking, algebraic loops = error.
%   Logged (To Workspace, Array): sl_est (70), sl_upd (56), sl_gdn (10), sl_plant (13);
%   the named signal lines are also marked for signal logging.
if nargin < 1 || isempty(cfg), cfg = sim_config('rect10', 1); end
if nargin < 2, mdl = 'ekf_mr2006b'; end
here = fileparts(mfilename('fullpath'));
root = fileparts(here);

% ------------------------------------------------------------ workspace parameters
assignin('base', 'ekf_p',      cfg.p);
assignin('base', 'ekf_pc',     ekf_param_cast(cfg.p, cfg.cls));
assignin('base', 'ekf_tr',     cfg.tr);
assignin('base', 'ekf_course', cfg.course);
gst0 = [1; 1; 0; 1 + 7*(cfg.course.mode == 1)];
if cfg.ekf_rate ~= 50
    warning('build_ekf_model: the model implements D-30 (EKF at 50 Hz); cfg.ekf_rate ignored');
end

% ------------------------------------------------------------ fresh model
if bdIsLoaded(mdl), close_system(mdl, 0); end
f = fullfile(here, [mdl '.slx']);
if exist(f, 'file'), delete(f); end
new_system(mdl);
load_system(mdl);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '1/48000', 'StopTime', sprintf('%.6f', cfg.T_end), ...
    'SampleTimeConstraint', 'Unconstrained', 'AlgebraicLoopMsg', 'error', ...
    'ReturnWorkspaceOutputs', 'on', 'SignalLogging', 'on');
trysp(mdl, 'EnableMultiTasking', 'off');          % R2019a+
trysp(mdl, 'SolverMode', 'SingleTasking');        % older releases
trysp(mdl, 'SingleTaskRateTransMsg', 'none');
trysp(mdl, 'PreLoadFcn', sprintf('addpath(genpath(''%s''));', root));
trysp(mdl, 'InitFcn', sprintf('addpath(genpath(''%s''));', root));

T1k = '1e-3';  T960 = '[1/960 1/48000]';  T50 = '[0.02 0.001]';

% ------------------------------------------------------------ blocks
% columns: x = 30 (plant), 260 (sensors), 520 (estimator), 800 (guidance)
mfb(mdl, 'Plant', T1k, {'u_cmd'}, {'y_plant'}, ...
    sprintf(['function y_plant = fcn(u_cmd, ekf_tr, ekf_course)\n%%#codegen\n' ...
    'persistent st\nif isempty(st)\n    st = sim_plant_init(ekf_tr, ekf_course);\nend\n' ...
    '[st, y_plant] = sim_plant_step(st, u_cmd, ekf_tr);\n']), {'ekf_tr', 'ekf_course'}, [60 180 200 240]);

add_block('built-in/SubSystem', [mdl '/Sensors'], 'Position', [300 60 460 360]);
S = [mdl '/Sensors'];
delete_default(S);
add_block('built-in/Inport', [S '/y_plant'], 'Position', [20 200 50 214]);
mfb(S, 'Encoders', T1k, {'y_plant'}, {'y_enc'}, ...
    sprintf('function y_enc = fcn(y_plant, ekf_p)\n%%#codegen\ny_enc = sim_enc_step(y_plant, ekf_p);\n'), ...
    {'ekf_p'}, [150 30 290 80]);
mfb(S, 'IMU', T960, {'y_plant'}, {'y_imu'}, ...
    sprintf(['function y_imu = fcn(y_plant, ekf_tr)\n%%#codegen\npersistent st\nif isempty(st)\n' ...
    '    st = sim_imu_init(ekf_tr);\nend\n[st, y_imu] = sim_imu_step(st, y_plant, ekf_tr);\n']), ...
    {'ekf_tr'}, [150 110 290 160]);
mfb(S, 'Ground', T1k, {'y_plant'}, {'y_gnd'}, ...
    sprintf(['function y_gnd = fcn(y_plant, ekf_tr, ekf_course, ekf_p)\n%%#codegen\npersistent st\n' ...
    'if isempty(st)\n    st = sim_gnd_init(ekf_tr, ekf_course, ekf_p);\nend\n' ...
    '[st, y_gnd] = sim_gnd_step(st, y_plant, ekf_tr, ekf_course, ekf_p);\n']), ...
    {'ekf_tr', 'ekf_course', 'ekf_p'}, [150 190 290 240]);
mfb(S, 'Lateral', T1k, {'y_plant'}, {'y_lat'}, ...
    sprintf(['function y_lat = fcn(y_plant, ekf_tr, ekf_course, ekf_p)\n%%#codegen\npersistent st\n' ...
    'if isempty(st)\n    st = sim_lat_init(ekf_tr, ekf_p);\nend\n' ...
    '[st, y_lat] = sim_lat_step(st, y_plant, ekf_tr, ekf_course, ekf_p);\n']), ...
    {'ekf_tr', 'ekf_course', 'ekf_p'}, [150 270 290 320]);
outs = {'y_enc', 'y_imu', 'y_gnd', 'y_lat'};
blks = {'Encoders', 'IMU', 'Ground', 'Lateral'};
for i = 1:4
    lin(S, 'y_plant/1', [blks{i} '/1'], 'y_plant');
    add_block('built-in/Outport', [S '/' outs{i}], 'Position', [360 45 + 80*(i-1) 390 59 + 80*(i-1)]);
    lin(S, [blks{i} '/1'], [outs{i} '/1'], outs{i});
end

add_block('built-in/SubSystem', [mdl '/Estimator'], 'Position', [540 60 700 360]);
E = [mdl '/Estimator'];
delete_default(E);
ins = {'y_enc', 'y_imu', 'y_gnd', 'y_lat', 'y_gst'};
for i = 1:5
    add_block('built-in/Inport', [E '/' ins{i}], 'Position', [20 40 + 60*(i-1) 50 54 + 60*(i-1)]);
end
mfb(E, 'YawIntegral', T1k, {'y_imu'}, {'y_yaw'}, ...
    sprintf(['function y_yaw = fcn(y_imu, ekf_pc)\n%%#codegen\npersistent st\nif isempty(st)\n' ...
    '    st = sim_yaw_init(ekf_pc);\nend\n[st, y_yaw] = sim_yaw_step(st, y_imu, ekf_pc);\n']), ...
    {'ekf_pc'}, [120 90 260 140]);
mfb(E, 'EKF', T50, {'y_enc', 'y_yaw', 'y_gnd', 'y_lat', 'y_gst'}, {'y_est', 'y_upd'}, ...
    sprintf(['function [y_est, y_upd] = fcn(y_enc, y_yaw, y_gnd, y_lat, y_gst, ekf_pc, ekf_course)\n' ...
    '%%#codegen\npersistent es\nif isempty(es)\n    es = sim_est_init(ekf_pc, ekf_course);\nend\n' ...
    '[es, y_est, y_upd] = sim_est_step(es, y_enc, y_yaw, y_gnd, y_lat, y_gst, ekf_pc, ekf_course);\n']), ...
    {'ekf_pc', 'ekf_course'}, [330 40 480 300]);
lin(E, 'y_imu/1', 'YawIntegral/1', 'y_imu');
lin(E, 'y_enc/1', 'EKF/1', 'y_enc');
lin(E, 'YawIntegral/1', 'EKF/2', 'y_yaw');
lin(E, 'y_gnd/1', 'EKF/3', 'y_gnd');
lin(E, 'y_lat/1', 'EKF/4', 'y_lat');
lin(E, 'y_gst/1', 'EKF/5', 'y_gst');
add_block('built-in/Outport', [E '/y_est'], 'Position', [560 100 590 114]);
add_block('built-in/Outport', [E '/y_upd'], 'Position', [560 220 590 234]);
lin(E, 'EKF/1', 'y_est/1', 'y_est');
lin(E, 'EKF/2', 'y_upd/1', 'y_upd');

mfb(mdl, 'Guidance', T50, {'y_est'}, {'u_cmd', 'y_gst', 'y_gdn'}, ...
    sprintf(['function [u_cmd, y_gst, y_gdn] = fcn(y_est, ekf_p, ekf_course)\n%%#codegen\n' ...
    'persistent gs\nif isempty(gs)\n    gs = sim_gdn_init(ekf_p, ekf_course);\nend\n' ...
    '[gs, y_gdn] = sim_gdn_step(gs, y_est, ekf_p, ekf_course);\n' ...
    'u_cmd = y_gdn(1:2);\ny_gst = y_gdn(5:8);\n']), {'ekf_p', 'ekf_course'}, [780 120 930 260]);

add_block('simulink/Discrete/Unit Delay', [mdl '/CmdDelay'], 'Position', [520 440 560 480], ...
    'InitialCondition', '[0;0]', 'SampleTime', T1k);
add_block('simulink/Discrete/Unit Delay', [mdl '/StatusDelay'], 'Position', [700 400 740 440], ...
    'InitialCondition', mat2str(gst0), 'SampleTime', T50);

% ------------------------------------------------------------ connections
lin(mdl, 'Plant/1', 'Sensors/1', 'y_plant');
for i = 1:4
    lin(mdl, sprintf('Sensors/%d', i), sprintf('Estimator/%d', i), outs{i});
end
lin(mdl, 'Estimator/1', 'Guidance/1', 'y_est');
lin(mdl, 'Guidance/1', 'CmdDelay/1', 'u_cmd');
lin(mdl, 'CmdDelay/1', 'Plant/1', 'u_cmd_applied');
lin(mdl, 'Guidance/2', 'StatusDelay/1', 'y_gst');
lin(mdl, 'StatusDelay/1', 'Estimator/5', 'y_gst_prev');

% ------------------------------------------------------------ logging
logs = {'sl_est', 'Estimator/1', [760 20 830 50]; 'sl_upd', 'Estimator/2', [760 320 830 350]; ...
        'sl_gdn', 'Guidance/3', [980 180 1050 210]; 'sl_plant', 'Plant/1', [300 420 370 450]};
for i = 1:size(logs, 1)
    b = [mdl '/log_' logs{i, 1}];
    add_block('simulink/Sinks/To Workspace', b, 'Position', logs{i, 3}, ...
        'VariableName', logs{i, 1}, 'SaveFormat', 'Array', 'MaxDataPoints', 'inf', 'SampleTime', '-1');
    lin(mdl, logs{i, 2}, ['log_' logs{i, 1} '/1'], '');
end
save_system(mdl, f);
fprintf('build_ekf_model: saved %s\n', f);
end

% =========================================================================
function mfb(sys, name, Ts, ins, outs, code, params, pos)
% atomic subsystem with explicit sample time holding one MATLAB Function block
sub = [sys '/' name];
add_block('built-in/SubSystem', sub, 'Position', pos);
delete_default(sub);
set_param(sub, 'TreatAsAtomicUnit', 'on', 'SystemSampleTime', Ts);
fb = [sub '/' name '_fcn'];
add_block('simulink/User-Defined Functions/MATLAB Function', fb, 'Position', [150 40 330 40 + 40*max(numel(ins), numel(outs))]);
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', fb);
ch.Script = code;
for k = 1:numel(params)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', params{k});
    d.Scope = 'Parameter';
    try, d.Tunable = false; catch, end %#ok<NOCOM>
end
for k = 1:numel(ins)
    add_block('built-in/Inport', [sub '/' ins{k}], 'Position', [30 40 + 40*(k-1) 60 54 + 40*(k-1)]);
    add_line(sub, [ins{k} '/1'], sprintf('%s_fcn/%d', name, k));
end
for k = 1:numel(outs)
    add_block('built-in/Outport', [sub '/' outs{k}], 'Position', [420 40 + 40*(k-1) 450 54 + 40*(k-1)]);
    h = add_line(sub, sprintf('%s_fcn/%d', name, k), [outs{k} '/1']);
    set_param(h, 'Name', outs{k});
end
end

function lin(sys, from, to, name)
try
    h = add_line(sys, from, to, 'autorouting', 'on');
catch
    h = add_line(sys, from, to);
end
if ~isempty(name)
    try
        set_param(h, 'Name', name);
        ph = get_param(h, 'SrcPortHandle');
        set_param(ph, 'DataLogging', 'on');
    catch
    end
end
end

function delete_default(sub)
% a new built-in SubSystem may contain a default In1 -> Out1 pair
try, delete_line(sub, 'In1/1', 'Out1/1'); catch, end %#ok<NOCOM>
try, delete_block([sub '/In1']); catch, end %#ok<NOCOM>
try, delete_block([sub '/Out1']); catch, end %#ok<NOCOM>
end

function trysp(mdl, name, value)
try
    set_param(mdl, name, value);
catch
end
end
