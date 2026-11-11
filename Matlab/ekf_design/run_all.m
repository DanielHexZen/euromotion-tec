function S = run_all(varargin)
%RUN_ALL  Design, verification and Simulink comparison of the MR2006B EKF.
%
%   S = run_all                       full run (MATLAB: ~10-20 min; Octave: hours)
%   S = run_all('quick', true)        shorter: MC N = 3, 1e5-step long test, 1 seed
%   S = run_all('name', value, ...)   options:
%       'mc_N'      Monte Carlo runs of rect10 (default 10)
%       'n_long'    steps of the symmetry/PD test (default 1e6)
%       'seeds'     seeds of the 50/200 Hz comparison (default 1:2)
%       'sens'      run the sensitivity sweep (default true)
%       'simulink'  build and compare the Simulink model (default true; skipped
%                   automatically where Simulink is absent, e.g. Octave)
%       'plots'     save figures to results/ (default true where possible)
%
%   Prints a pass/fail table (1 pass, 0 FAIL, - informational) and saves
%   results/summary.txt and results/summary.mat.
opt = struct('quick', false, 'mc_N', 10, 'n_long', 1e6, 'seeds', 1:2, 'sens', true, ...
             'simulink', true, 'plots', true);
for k = 1:2:numel(varargin), opt.(varargin{k}) = varargin{k+1}; end
if opt.quick
    opt.mc_N = 3;  opt.n_long = 1e5;  opt.seeds = 1;  opt.sens = false;
end
root = fileparts(mfilename('fullpath'));
addpath(genpath(root));
rdir = fullfile(root, 'results');
if ~exist(rdir, 'dir'), mkdir(rdir); end
plots = opt.plots && can_plot();
t0 = tic;
R = [];
S = struct();

step('design: noise models / Allan');
try
    [r, S.noise] = design_noise_models(plots);
    R = [R, r];
catch err
    R = [R, step_error('design: noise models / Allan', err)];
end
step('design: observability');
try
    [r, S.obs]   = design_observability(true, 2);
    R = [R, r];
catch err
    R = [R, step_error('design: observability', err)];
end
step('design: gates');
try
    [r, S.gates] = design_gates(true);
    R = [R, r];
catch err
    R = [R, step_error('design: gates', err)];
end
if opt.sens
step('design: sensitivity');
try
    [r, S.sens]  = design_sensitivity();
    R = [R, r];
catch err
    R = [R, step_error('design: sensitivity', err)];
end
end
step('verify: Jacobians');
try
    r = verify_jacobians();
    R = [R, r];
catch err
    R = [R, step_error('verify: Jacobians', err)];
end
step('verify: static (float32)');
try
    [r, S.static] = verify_static('single', opt.n_long);
    R = [R, r];
catch err
    R = [R, step_error('verify: static (float32)', err)];
end
step('verify: gate');
try
    r = verify_gate('single');
    R = [R, r];
catch err
    R = [R, step_error('verify: gate', err)];
end
step('verify: back-shift');
try
    r = verify_backshift('single');
    R = [R, r];
catch err
    R = [R, step_error('verify: back-shift', err)];
end
step('verify: t_ekf_line');
try
    [r, Ll] = verify_line('single', 1);
    R = [R, r];
catch err
    R = [R, step_error('verify: t_ekf_line', err)];
end
if plots && exist('Ll', 'var'), plot_run(Ll, 'line40'); end
step('verify: float32 vs float64');
try
    [r, S.f32] = verify_float32(3);
    R = [R, r];
catch err
    R = [R, step_error('verify: float32 vs float64', err)];
end
step('verify: 50 Hz vs 200 Hz (D-30)');
try
    [r, S.rate] = verify_rate_50_200(opt.seeds, true);
    R = [R, r];
catch err
    R = [R, step_error('verify: 50 Hz vs 200 Hz (D-30)', err)];
end
step(sprintf('verify: Monte Carlo rect10, N = %d', opt.mc_N));
try
    [r, S.mc] = verify_monte_carlo(opt.mc_N, 'rect10');
    R = [R, r];
catch err
    R = [R, step_error('Monte Carlo', err)];
end
step('verify: Monte Carlo rect10, literal spec scale-factor Q (mx_sf_mode = 0)');
try
    ps = ekf_params();  ps.mx_sf_mode = 0;
    [r, S.mc_spec_sf] = verify_monte_carlo(max(2, round(opt.mc_N/3)), 'rect10', 'p', ps);
    for i = 1:numel(r)
        r(i).name = ['[spec SF] ' r(i).name];
        if r(i).pass >= 0, r(i).pass = -1; end      % comparison only
    end
    R = [R, r];
catch err
    R = [R, step_error('Monte Carlo spec SF', err)];
end
if plots
    try
        L = sim_run(sim_config('rect10', 101));  plot_run(L, 'rect10');
    catch err
        step_error('plots', err);
    end
end
if opt.simulink
step('Simulink: build and compare');
try
    [r, S.sl] = run_ekf_model('rect10', 60, 1);
    R = [R, r];
catch err
    R = [R, step_error('Simulink: build and compare', err)];
end
end

% ------------------------------------------------------------ summary
lines = {};
lines{end+1} = sprintf('%-58s %-6s %-46s %s', 'check', 'result', 'value', 'criterion');
lines{end+1} = repmat('-', 1, 140);
for i = 1:numel(R)
    switch R(i).pass
        case 1,  pf = 'pass';
        case 0,  pf = 'FAIL';
        otherwise, pf = '-';
    end
    lines{end+1} = sprintf('%-58s %-6s %-46s %s', R(i).name, pf, R(i).value, R(i).crit); %#ok<AGROW>
end
np = sum([R.pass] == 1);  nf = sum([R.pass] == 0);
lines{end+1} = sprintf('%d pass, %d FAIL, %d informational; %.1f min; %s', np, nf, ...
    sum([R.pass] < 0), toc(t0)/60, ternary(exist('OCTAVE_VERSION', 'builtin') > 0, 'Octave', 'MATLAB'));
fprintf('\n');
fid = fopen(fullfile(rdir, 'summary.txt'), 'w');
for i = 1:numel(lines)
    fprintf('%s\n', lines{i});
    fprintf(fid, '%s\n', lines{i});
end
fclose(fid);
S.R = R;  S.opt = opt;
try
    Rsave = R; optsave = opt; %#ok<NASGU>
    save(fullfile(rdir, 'summary.mat'), 'Rsave', 'optsave');
catch
end
end

function r = step_error(label, err)
fprintf('!!! %s failed: %s\n', label, err.message);
for k = 1:min(3, numel(err.stack))
    fprintf('    at %s line %d\n', err.stack(k).name, err.stack(k).line);
end
r = vres(['ERROR in ' label], 0, err.message, 'must run');
end

function step(s)
fprintf('\n##### %s\n', s);
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
