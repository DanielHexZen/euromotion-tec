% NAV_ABLATION  Design ablation over 10 noise seeds (rng-free, portable noise): which change of the
%   iteration-1 design reduces the CTE / steering noise. Saves nav_ablation.mat.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'common'), fullfile(root,'02_pid'), fullfile(root,'03_lkf'));
here = fileparts(mfilename('fullpath'));
TR = load(fullfile(here,'track.mat')); trk = TR.trk;
abl_names = {'A: iteration-1 design (differenced D, no servo limit, LKF without wheel speed)'; ...
             'B: A + servo rate limit 6.98 rad/s (0.15 s/60 deg)'; ...
             'C: B + D on the LKF velocity (no wheel speed)'; ...
             'D: B + wheel-speed fusion in the LKF (differenced D)'};
abl_over = {struct('dmode',1,'servo_rate',1e3,'usev',0); struct('dmode',1,'usev',0); ...
            struct('dmode',2,'usev',0); struct('dmode',1,'usev',1)};
seeds = 1:10;
ABL = cell(numel(abl_names), numel(seeds));
for c = 1:numel(abl_names)
    for q = 1:numel(seeds)
        cfg = nav_cfg_default(); cfg.seed = seeds(q);
        fn = fieldnames(abl_over{c});
        for i = 1:numel(fn), cfg.(fn{i}) = abl_over{c}.(fn{i}); end
        P = nav_params(cfg, root);
        vper = nav_speed_profile(trk.kap, trk.ds, cfg.vmax, cfg.alat, cfg.aacc, cfg.adec, P.par);
        ref = nav_reference(trk, vper, P.T, cfg.aacc, cfg.runout);
        E = nav_noise(P, P.nst + numel(ref.s) + 5, cfg.seed);
        out = nav_sim(cfg, trk, ref, P, E);
        ABL{c,q} = nav_metrics(out, trk, P);
        fprintf('%s seed %d: CTE_max %.3f RMSE %.3f rate %.2f\n', abl_names{c}(1), seeds(q), ABL{c,q}.cte_max, ABL{c,q}.cte_rmse, ABL{c,q}.delta_rate_rms);
    end
end
save(fullfile(here,'nav_ablation.mat'), 'abl_names', 'abl_over', 'ABL', 'seeds', '-v7');
