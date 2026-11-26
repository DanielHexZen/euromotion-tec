% NAV_TABLES  Print the markdown tables used in RESULTS.md from nav_results.mat.
here = fileparts(mfilename('fullpath')); addpath(fullfile(fileparts(here),'common'));
R = load(fullfile(here,'nav_results.mat'));
M = R.M; names = R.names; sc = 10;
m = M{1}; mi = M{2};
pf = @(c) nav_pf(c);
fprintf('| metric | target (1:10) | nominal LKF | status | full-scale equiv. (x10) | ideal feedback |\n|---|---|---|---|---|---|\n');
fprintf('| CTE_max [m] | <= 0.15 | %.3f | %s | %.2f m (<= 1.5) | %.3f |\n', m.cte_max, pf(m.cte_max<=0.15), sc*m.cte_max, mi.cte_max);
fprintf('| RMSE_CTE [m] | <= 0.05 | %.3f | %s | %.2f m (<= 0.5) | %.3f |\n', m.cte_rmse, pf(m.cte_rmse<=0.05), sc*m.cte_rmse, mi.cte_rmse);
fprintf('| IAE_CTE [m s] | - | %.2f | - | %.1f | %.2f |\n', m.iae_cte, sc*m.iae_cte, mi.iae_cte);
fprintf('| mean abs CTE [m] | - | %.3f | - | %.2f | %.3f |\n', m.cte_mean_abs, sc*m.cte_mean_abs, mi.cte_mean_abs);
fprintf('| ATE RMSE [m] | - | %.3f | - | %.2f | %.3f |\n', m.ate_rmse, sc*m.ate_rmse, mi.ate_rmse);
fprintf('| ATE max [m] | - | %.3f | - | %.2f | %.3f |\n', m.ate_max, sc*m.ate_max, mi.ate_max);
fprintf('| position RMSE vs time-param. reference [m] | - | %.3f | - | %.2f | %.3f |\n', m.pos_rmse, sc*m.pos_rmse, mi.pos_rmse);
fprintf('| lap time [s] | - | %.1f | - | - | %.1f |\n', m.lap_time, mi.lap_time);
fprintf('| min margin to boundary - 0.15 m (lateral coord.) [m] | > 0 | %.3f | %s | %.2f | %.3f |\n', m.margin_min, pf(m.margin_min>0), sc*m.margin_min, mi.margin_min);
fprintf('| min margin, geometric distance to boundary polyline [m] | > 0 | %.3f | %s | - | %.3f |\n', m.margin_geo_min, pf(m.margin_geo_min>0), mi.margin_geo_min);
fprintf('| min margin, 0.54 x 0.30 m footprint corners [m] | > 0 (info) | %.3f | %s | - | %.3f |\n', m.margin_footprint_min, pf(m.margin_footprint_min>0), mi.margin_footprint_min);
fprintf('| max abs delta [rad] / delta at limit [%% of lap] | <= 0.34 | %.3f / %.2f | - | - | %.3f / %.2f |\n', m.delta_max, m.sat_delta_pct, mi.delta_max, mi.sat_delta_pct);
fprintf('| delta RMS [rad] / commanded rate RMS [rad/s] | - | %.3f / %.2f | - | - | %.3f / %.2f |\n', m.delta_rms, m.delta_rate_rms, mi.delta_rms, mi.delta_rate_rms);
fprintf('| applied (rate-limited) steering rate RMS [rad/s] / samples rate-limited [%%] | - | %.2f / %.2f | - | - | %.2f / %.2f |\n', m.delta_app_rate_rms, m.rate_limited_pct, mi.delta_app_rate_rms, mi.rate_limited_pct);
fprintf('| u_lat saturated [%% of lap] / v_cmd at 0 or 3 m/s [%%] | - | %.2f / %.2f | - | - | %.2f / %.2f |\n', m.sat_ulat_pct, m.sat_vcmd_pct, mi.sat_ulat_pct, mi.sat_vcmd_pct);
fprintf('| mean / max speed [m/s] | - | %.2f / %.2f | - | - | %.2f / %.2f |\n', m.v_mean, m.v_max, mi.v_mean, mi.v_max);
fprintf('\nLKF estimation (nominal): position RMSE %.3f m, max %.3f m; velocity RMSE %.3f m/s; heading RMSE %.4f rad\n', m.est_pos_rmse, m.est_pos_max, m.est_vel_rmse, m.est_psi_rmse);
fprintf('raw double integration RMSE %.1f m -> reduction %.3f %%; raw 10 Hz fixes RMSE %.3f m -> reduction %.1f %%\n', m.raw_pos_rmse, m.red_vs_raw, m.fix_rmse, m.red_vs_fix);
fprintf('\n| run | lap [s] | CTE_max | RMSE_CTE | IAE | ATE RMSE | ATE max | LKF RMSE | min margin | delta lim %% | delta RMS | targets |\n|---|---|---|---|---|---|---|---|---|---|---|---|\n');
for q = 1:numel(M)
    x = M{q};
    fprintf('| %s | %.1f | %.3f | %.3f | %.2f | %.3f | %.3f | %.3f | %.3f | %.2f | %.3f | %s |\n', names{q}, x.lap_time, x.cte_max, x.cte_rmse, x.iae_cte, x.ate_rmse, x.ate_max, x.est_pos_rmse, x.margin_min, x.sat_delta_pct, x.delta_rms, pf(x.cte_max<=0.15 && x.cte_rmse<=0.05 && ~x.violation));
end

% ---- 10-seed study: ablation of the design changes ----
fprintf('\n### SEEDS\n');
A = load(fullfile(here,'nav_ablation.mat'));
nc = numel(A.abl_names);
rows = cell(nc+1,1); lab = [A.abl_names; {'E: nominal = D + D action on the LKF velocity (nav_main)'}];
for c = 1:nc+1
    if c <= nc
        X = A.ABL(c,:);
    else
        X = M([1, 6:14])';
    end
    f = @(fn) cellfun(@(q) q.(fn), X);
    cm = f('cte_max'); cr = f('cte_rmse'); dr = f('delta_rate_rms'); da = f('delta_app_rate_rms'); er = f('est_pos_rmse'); dd = f('delta_rms');
    ok = all(cm <= 0.13) && all(cr <= 0.045);
    fprintf('| %s | %.3f / %.3f / %.3f | %.3f / %.3f / %.3f | %.2f / %.2f | %.3f | %.3f | %s |\n', lab{c}, mean(cm), std(cm), max(cm), mean(cr), std(cr), max(cr), mean(dr), mean(da), mean(dd), mean(er), nav_pf(ok));
end
fprintf('\nper-seed CTE_max / RMSE_CTE of the nominal design (seeds 1..10):\n');
X = M([1, 6:14]);
fprintf('| seed | %s |\n|---|---|---|---|---|---|---|---|---|---|---|\n', strjoin(arrayfun(@(q) sprintf('%d', q), 1:10, 'UniformOutput', false), ' | '));
fprintf('| CTE_max [m] | %s |\n', strjoin(cellfun(@(q) sprintf('%.3f', q.cte_max), X', 'UniformOutput', false), ' | '));
fprintf('| RMSE_CTE [m] | %s |\n', strjoin(cellfun(@(q) sprintf('%.3f', q.cte_rmse), X', 'UniformOutput', false), ' | '));
fprintf('| LKF pos. RMSE [m] | %s |\n', strjoin(cellfun(@(q) sprintf('%.3f', q.est_pos_rmse), X', 'UniformOutput', false), ' | '));
