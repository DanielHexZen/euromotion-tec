%% BUILD_EKF_SIMULINK  Build and run the Simulink model of the discrete EKF
%  Creates ekf_navigation.slx programmatically:
%
%   [From Workspace: u]  ----------------------\
%   [From Workspace: z]  ---------------------> [MATLAB Function: EKF] --> x_hat --> To Workspace / Scopes
%   [From Workspace: valid] -> [sensor mask] -------/     ^   ^         |
%   [Constant: pv] ---------------------------------/   |         |
%                 [Unit Delay x(k-1)] <------------------+---------+
%                 [Unit Delay P(k-1)] <------------------+
%
%  The block calls the same ekf_step.m as the MATLAB script, so the
%  Simulink result must match main_kalman.m (checked at the end).
%  Requires: MATLAB + Simulink (R2021a or newer). Run main_kalman.m first.
clear; clc;
if ~exist('ekf_params.mat', 'file'), main_kalman; close all; end
load('ekf_params.mat', 'p', 'pv', 'x0', 'P0', 'Ts');

dataFile = fullfile('..', 'openloop_test01', 'openloop_test01_synced.mat');
if ~exist(dataFile, 'file'), dataFile = 'openloop_test01_synced.mat'; end
D = ekf_load_data(dataFile, Ts);

%% ---------------------------------------------- signals for From Workspace
% start at the first reference pose (filter initialisation)
k  = (D.k0:D.N).';
tt = D.t(k) - D.t(D.k0);
pfEvery = round(1 / (p.pf_rate_hz * Ts));
pfTick  = false(D.N, 1); pfTick(D.k0:pfEvery:end) = true;

z = [D.v_meas D.wz_meas D.x_pf D.y_pf D.psi_pf];  z(isnan(z)) = 0;
valid = double([D.ok_v D.ok_wz (D.ok_pf & pfTick)]);

sim_u     = [tt, D.v_cmd(k), D.delta_cmd(k)];
sim_z     = [tt, z(k,:)];
sim_valid = [tt, valid(k,:)];
sim_ref   = [tt, D.x_pf(k), D.y_pf(k), D.psi_pf(k)];
use_mask  = [1 1 1];          % [wheel speed; gyro; absolute pose]  -> set 0 to switch a sensor off
Tend      = tt(end);

%% ---------------------------------------------------------- build model
mdl = 'ekf_navigation';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl); open_system(mdl);
set_param(mdl, 'Solver', 'FixedStepDiscrete', 'FixedStep', 'Ts', ...
               'StopTime', 'Tend', 'SaveOutput', 'on', 'ReturnWorkspaceOutputs', 'on');

% --- sources
fw = {'u',     'sim_u',     [ 30  40 130  70];
      'z',     'sim_z',     [ 30 110 130 140];
      'valid', 'sim_valid', [ 30 180 130 210]};
for i = 1:size(fw,1)
    b = [mdl '/' fw{i,1}];
    add_block('simulink/Sources/From Workspace', b, 'VariableName', fw{i,2}, ...
        'SampleTime', 'Ts', 'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value', ...
        'Position', fw{i,3});
end
add_block('simulink/Sources/Constant', [mdl '/sensor mask'], 'Value', 'use_mask', ...
    'SampleTime', 'Ts', 'Position', [30 240 130 270]);
add_block('simulink/Math Operations/Product', [mdl '/apply mask'], ...
    'Multiplication', 'Element-wise(.*)', 'Position', [180 190 210 250]);
add_block('simulink/Sources/Constant', [mdl '/parameters pv'], 'Value', 'pv', ...
    'SampleTime', 'Ts', 'Position', [30 300 130 330]);

% --- state memory  x(k-1), P(k-1)
add_block('simulink/Discrete/Unit Delay', [mdl '/x(k-1)'], 'InitialCondition', 'x0', ...
    'SampleTime', 'Ts', 'Orientation', 'left', 'Position', [330 380 370 410]);
add_block('simulink/Discrete/Unit Delay', [mdl '/P(k-1)'], 'InitialCondition', 'P0', ...
    'SampleTime', 'Ts', 'Orientation', 'left', 'Position', [330 440 370 470]);

% --- EKF (MATLAB Function block calling ekf_step.m)
ekf = [mdl '/EKF'];
add_block('simulink/User-Defined Functions/MATLAB Function', ekf, 'Position', [300 60 460 340]);
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', ekf);
ch.Script = sprintf([ ...
    'function [x, P] = EKF(x_prev, P_prev, u, z, valid, pv)\n' ...
    '%%#codegen\n' ...
    '%% Discrete EKF: prediction with u(k), correction with available measurements\n' ...
    '[x, P] = ekf_step(x_prev, P_prev, u, z, valid, pv);\n']);
% inputs: inherited size; outputs: fixed size (needed for the delay feedback loop)
for nm = {'x', 'P'}
    d = ch.find('-isa', 'Stateflow.Data', 'Name', nm{1});
    if strcmp(nm{1}, 'x'), d.Props.Array.Size = '[6 1]'; else, d.Props.Array.Size = '[6 6]'; end
end

% --- outputs
add_block('simulink/Sinks/To Workspace', [mdl '/x_hat'], 'VariableName', 'x_hat', ...
    'SaveFormat', 'Array', 'SampleTime', 'Ts', 'Position', [620 40 690 70]);
add_block('simulink/Signal Routing/Demux', [mdl '/Demux'], 'Outputs', '6', 'Position', [540 110 545 330]);
add_block('simulink/Sinks/Scope', [mdl '/Scope position'], 'NumInputPorts', '2', 'Position', [720 110 760 170]);
add_block('simulink/Sinks/Scope', [mdl '/Scope heading, speed, bias'], 'NumInputPorts', '3', 'Position', [720 200 760 280]);
add_block('simulink/Sinks/Terminator', [mdl '/T5'], 'Position', [620 290 640 310]);
add_block('simulink/Sources/From Workspace', [mdl '/reference'], 'VariableName', 'sim_ref', ...
    'SampleTime', 'Ts', 'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value', ...
    'Position', [470 380 570 410]);
add_block('simulink/Sinks/To Workspace', [mdl '/ref'], 'VariableName', 'ref', ...
    'SaveFormat', 'Array', 'SampleTime', 'Ts', 'Position', [620 380 690 410]);

% --- lines
al = @(a, b) add_line(mdl, a, b, 'autorouting', 'on');
al('u/1', 'EKF/3');  al('z/1', 'EKF/4');
al('valid/1', 'apply mask/1'); al('sensor mask/1', 'apply mask/2'); al('apply mask/1', 'EKF/5');
al('parameters pv/1', 'EKF/6');
al('x(k-1)/1', 'EKF/1');  al('P(k-1)/1', 'EKF/2');
al('EKF/1', 'x(k-1)/1');  al('EKF/2', 'P(k-1)/1');
al('EKF/1', 'x_hat/1');   al('EKF/1', 'Demux/1');
al('Demux/1', 'Scope position/1'); al('Demux/2', 'Scope position/2');
al('Demux/3', 'Scope heading, speed, bias/1'); al('Demux/4', 'Scope heading, speed, bias/2');
al('Demux/6', 'Scope heading, speed, bias/3'); al('Demux/5', 'T5/1');
al('reference/1', 'ref/1');
set_param(mdl, 'SimulationCommand', 'update');
Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl);
fprintf('Model %s.slx created.\n', mdl);

%% ---------------------------------------------------------- simulate
out = sim(mdl);
X  = to_rows(out.x_hat, 6);   % To Workspace stores a [6x1] signal as 6x1xN
Rf = to_rows(out.ref, 3);
e = hypot(X(:,1) - Rf(:,1), X(:,2) - Rf(:,2));
fprintf('Simulink EKF: mean position error %.3f m, max %.3f m\n', mean(e), max(e));

% cross-check against the MATLAB implementation (config C: pose at 1 Hz)
c = struct('odo', 1, 'gyro', 1, 'pf', pfTick);
Rm = ekf_run(D, c, x0, P0, pv);
Xm = Rm.X(D.k0:end, :);
n  = min(size(X,1), size(Xm,1));
fprintf('max |x_Simulink - x_MATLAB| = %.2e\n', max(abs(X(1:n,:) - Xm(1:n,:)), [], 'all'));

figure('Name', 'Simulink EKF', 'Color', 'w');
plot(Rf(:,1), Rf(:,2), 'k', 'LineWidth', 2); hold on; plot(X(:,1), X(:,2), 'b');
axis equal; grid on; xlabel('x [m]'); ylabel('y [m]'); legend('reference (PF)', 'EKF (Simulink)');
title('Simulink: EKF with wheel speed, gyro and 1 Hz pose');

%% ===================================================================
function A = to_rows(A, n)
%TO_ROWS  Convert To Workspace 'Array' output to N x n (time along rows),
%   independent of whether the signal was logged as N x n or n x 1 x N.
if ndims(A) == 3
    A = reshape(A, n, []).';
elseif size(A, 2) ~= n && size(A, 1) == n
    A = A.';
end
end
