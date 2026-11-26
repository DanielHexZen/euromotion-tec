% NAV_MAIN  Closed-loop navigation of the 1:10 Mexico City circuit with LKF feedback.
%   Runs: nominal (LKF feedback, in-motion sensor noise, wheel-speed fusion, D on the LKF velocity,
%   6.98 rad/s servo rate limit (0.15 s/60 deg)), ideal (true-state feedback), standstill-level noise, speed
%   sensitivity, noise seeds 1-10, plant mismatch. Design ablation over 10 seeds: nav_ablation.m. Saves nav_results.mat.
%   Set nav_quick = 1 before running to skip the additional runs.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'common'), fullfile(root,'02_pid'), fullfile(root,'03_lkf'));
here = fileparts(mfilename('fullpath'));
if ~exist(fullfile(here,'fig'),'dir'), mkdir(fullfile(here,'fig')); end
if ~exist('nav_quick','var'), nav_quick = 0; end
TR = load(fullfile(here,'track.mat'));
trk = TR.trk;

names = {'nominal (LKF)'; 'ideal (true state)'};
over  = {struct(); struct('fb','true')};
if ~nav_quick
    sn = cell(9,1); so = cell(9,1);
    for i = 1:9, sn{i} = sprintf('LKF, seed %d', i+1); so{i} = struct('seed', i+1); end
    names = [names; {'LKF, standstill-level noise'; 'LKF, v_max 2.0'; 'LKF, v_max 3.0'}; sn; ...
        {'LKF, Gr gain x0.85'; 'LKF, Gr gain x1.15'; 'LKF, Gv gain x0.85'; 'LKF, Gv gain x1.15'}];
    over = [over; {struct('noise','standstill'); struct('vmax',2.0); struct('vmax',3.0)}; so; ...
        {struct('kr_scale',0.85); struct('kr_scale',1.15); struct('kv_scale',0.85); struct('kv_scale',1.15)}];
end
nr = numel(names);
M = cell(nr,1); CFG = cell(nr,1); OUT = [];
for q = 1:nr
    cfg = nav_cfg_default();
    fn = fieldnames(over{q});
    for i = 1:numel(fn), cfg.(fn{i}) = over{q}.(fn{i}); end
    P = nav_params(cfg, root);
    vper = nav_speed_profile(trk.kap, trk.ds, cfg.vmax, cfg.alat, cfg.aacc, cfg.adec, P.par);
    ref = nav_reference(trk, vper, P.T, cfg.aacc, cfg.runout);
    E = nav_noise(P, P.nst + numel(ref.s) + 5, cfg.seed);
    tic; out = nav_sim(cfg, trk, ref, P, E); tt = toc;
    m = nav_metrics(out, trk, P);
    M{q} = m; CFG{q} = cfg;
fprintf('%-28s lap %.1f s | CTE max %.3f rms %.3f IAE %.2f | ATE rms %.3f max %.3f | est RMSE %.3f (raw %.1f, red %.2f %%) | margin %.3f geo %.3f | sat %.1f %% | %.0fs\n', ...
        names{q}, m.lap_time, m.cte_max, m.cte_rmse, m.iae_cte, m.ate_rmse, m.ate_max, m.est_pos_rmse, m.raw_pos_rmse, m.red_vs_raw, m.margin_min, m.margin_geo_min, m.sat_delta_pct, tt);
    if q <= 2
        OUT.(sprintf('run%d', q)) = out;
        if q == 1, REF = ref; PP = P; end
    end
end
save(fullfile(here,'nav_results.mat'), 'names', 'M', 'CFG', 'OUT', 'REF', 'PP', '-v7');

% ---- targets (scaled 1:10) ----
m = M{1};
fprintf('\nTargets: CTE_max %.3f <= 0.15 m : %s | RMSE_CTE %.3f <= 0.05 m : %s | track limits: %s\n', m.cte_max, nav_pf(m.cte_max <= 0.15), ...
    m.cte_rmse, nav_pf(m.cte_rmse <= 0.05), nav_pf(~m.violation));
fprintf('Full-scale equivalent: CTE_max %.2f m, RMSE_CTE %.2f m\n', 10*m.cte_max, 10*m.cte_rmse);
nav_figures;
