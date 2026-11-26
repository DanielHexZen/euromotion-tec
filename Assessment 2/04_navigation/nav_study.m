% NAV_STUDY  Multi-seed study for a configuration override struct cfgover and a seed list seeds.
%   Prints one line per seed (CTE_max, RMSE_CTE, steering statistics). Helper for tuning.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'common'), fullfile(root,'02_pid'), fullfile(root,'03_lkf'));
here = fileparts(mfilename('fullpath'));
TR = load(fullfile(here,'track.mat'));
if ~exist('cfgover','var'), cfgover = struct(); end
if ~exist('seeds','var'), seeds = 1:10; end
for sd = seeds
    cfg = nav_cfg_default(); cfg.seed = sd;
    fn = fieldnames(cfgover);
    for i = 1:numel(fn), cfg.(fn{i}) = cfgover.(fn{i}); end
    P = nav_params(cfg, root);
    vper = nav_speed_profile(TR.trk.kap, TR.trk.ds, cfg.vmax, cfg.alat, cfg.aacc, cfg.adec, P.par);
    ref = nav_reference(TR.trk, vper, P.T, cfg.aacc, cfg.runout);
    E = nav_noise(P, P.nst + numel(ref.s) + 5, cfg.seed);
    out = nav_sim(cfg, TR.trk, ref, P, E);
    m = nav_metrics(out, TR.trk, P);
    fprintf('seed %2d: CTEmax %.3f rms %.3f | lap %.1f | est %.3f | delta rms %.3f rate cmd %.2f app %.2f lim %.1f%% | margin %.3f\n', sd, m.cte_max, m.cte_rmse, m.lap_time, m.est_pos_rmse, m.delta_rms, m.delta_rate_rms, m.delta_app_rate_rms, m.rate_limited_pct, m.margin_min);
end
