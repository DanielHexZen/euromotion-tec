% NAV_FIGURES  Figures of the navigation study (run after nav_main, or standalone).
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'common'), fullfile(root,'02_pid'), fullfile(root,'03_lkf'));
here = fileparts(mfilename('fullpath'));
if ~exist('M','var') || ~exist('OUT','var')
    R = load(fullfile(here,'nav_results.mat')); M = R.M; OUT = R.OUT; REF = R.REF; PP = R.PP; names = R.names;
end
if ~exist('trk','var'), TR = load(fullfile(here,'track.mat')); trk = TR.trk; end
fd = fullfile(here,'fig');
o = OUT.run1; m = M{1}; P = PP;
w = m.k0:m.k1; t = o.t(w) - o.t(m.k0);
s_tr = o.proj(w,4); cte = o.proj(w,5);
fs = 7;
% 1) full track
f = figure('visible','off');
plot(trk.xl, trk.yl, 'k-', trk.xr, trk.yr, 'k-', 'linewidth', 0.5); hold on;
h1 = plot(o.X(w,1), o.X(w,2), 'b-', 'linewidth', 0.8);
h2 = plot(o.est(w,1), o.est(w,2), 'r-', 'linewidth', 0.3);
h3 = plot(trk.x(1), trk.y(1), 'gs', 'markerfacecolor', 'g', 'markersize', 5);
axis equal; grid on; set(gca,'fontsize',fs); xlabel('x [m]'); ylabel('y [m]');
legend([h1 h2 h3], {'true path','LKF estimate','start/finish'}, 'location', 'southwest');
title('1:10 Mexico City circuit, one lap', 'fontsize', fs+1);
nav_savefig(f, fullfile(fd,'fig_nav_track.png'), 3.5, 3.2);
% 2) zoomed hairpin sections (two windows with the largest curvature density)
win = {[345 395], [172 215]}; ttl = {'Peraltada sector', 'Foro Sol (stadium) sector'};
f = figure('visible','off');
for q = 1:2
    subplot(1,2,q);
    ii = find(trk.s >= win{q}(1) & trk.s <= win{q}(2));
    plot(trk.xl(ii), trk.yl(ii), 'k-', trk.xr(ii), trk.yr(ii), 'k-', trk.x(ii), trk.y(ii), 'k:'); hold on;
    jj = find(s_tr >= win{q}(1) & s_tr <= win{q}(2));
    plot(o.X(w(jj),1), o.X(w(jj),2), 'b-', 'linewidth', 1);
    plot(o.est(w(jj),1), o.est(w(jj),2), 'r-', 'linewidth', 0.4);
    axis equal; grid on; xlabel('x [m]'); if q == 1, ylabel('y [m]'); end
    title(sprintf('%s (s = %d-%d m)', ttl{q}, win{q}(1), win{q}(2)));
end
nav_savefig(f, fullfile(fd,'fig_nav_zoom.png'), 7.0, 4.0);
% 3) CTE vs s with limits and boundary margin
f = figure('visible','off');
subplot(2,1,1); set(gca,'position',[0.19 0.57 0.77 0.34]);
h1 = plot(s_tr, cte, 'b-'); hold on; h2 = plot(o.proj(w,1), o.proj(w,2), 'r-', 'linewidth', 0.3);
h3 = plot([0 trk.L], 0.15*[1 1], 'k--'); plot([0 trk.L], -0.15*[1 1], 'k--');
set(gca,'fontsize',fs); grid on; ylabel('CTE [m]'); xlim([0 trk.L]); ylim([-0.2 0.32]);
legend([h1 h2 h3], {'true CTE', 'estimated CTE', '\pm0.15 m target'}, 'location', 'northeast');
title(sprintf('CTE: max %.3f m, RMSE %.3f m', m.cte_max, m.cte_rmse), 'fontsize', fs+1);
subplot(2,1,2); set(gca,'position',[0.19 0.12 0.77 0.34]);
ml = nav_lookup(trk.wl, trk.ds, s_tr) - cte - 0.15; mr = nav_lookup(trk.wr, trk.ds, s_tr) + cte - 0.15;
plot(s_tr, min(ml, mr), 'b-'); hold on; plot([0 trk.L], [0 0], 'r--');
set(gca,'fontsize',fs); grid on; xlabel('s [m]'); ylabel('margin to boundary [m]'); xlim([0 trk.L]);
nav_savefig(f, fullfile(fd,'fig_nav_cte.png'), 3.5, 3.6);
% 4) speed vs reference
f = figure('visible','off');
subplot(2,1,1); set(gca,'position',[0.19 0.57 0.77 0.34]);
plot(t, o.vref(w), 'r-', t, o.X(w,4), 'b-', 'linewidth', 0.6); grid on; set(gca,'fontsize',fs);
ylabel('v [m/s]'); legend('v_{ref}', 'v (true)', 'location', 'south');
title(sprintf('Speed tracking (lap time %.1f s)', m.lap_time), 'fontsize', fs+1);
subplot(2,1,2); set(gca,'position',[0.19 0.12 0.77 0.34]);
plot(t, o.proj(w,4) - o.sref(w), 'b-'); grid on; set(gca,'fontsize',fs);
xlabel('t [s]'); ylabel('along-track error [m]');
nav_savefig(f, fullfile(fd,'fig_nav_speed.png'), 3.5, 3.6);
% 5) steering command
f = figure('visible','off');
subplot(2,1,1); set(gca,'position',[0.19 0.57 0.77 0.34]);
plot(t, o.cmd(w,2), 'b-', 'linewidth', 0.5); hold on;
plot([0 t(end)], 0.34*[1 1], 'r--', [0 t(end)], -0.34*[1 1], 'r--');
grid on; set(gca,'fontsize',fs); ylabel('\delta_{cmd} [rad]'); ylim([-0.42 0.42]);
title(sprintf('Steering command, |\\delta| >= limit for %.1f %% of the lap', m.sat_delta_pct), 'fontsize', fs+1);
subplot(2,1,2); set(gca,'position',[0.19 0.12 0.77 0.34]);
plot(t, o.cmd(w,1), 'b-', 'linewidth', 0.5); hold on; plot([0 t(end)], [3 3], 'r--');
grid on; set(gca,'fontsize',fs); xlabel('t [s]'); ylabel('v_{cmd} [m/s]');
nav_savefig(f, fullfile(fd,'fig_nav_steering.png'), 3.5, 3.6);
% 6) estimation error
f = figure('visible','off');
pe = o.est(w,1:2) - o.X(w,1:2);
subplot(2,1,1); set(gca,'position',[0.19 0.57 0.77 0.34]);
plot(t, pe(:,1), 'b-', t, pe(:,2), 'r-', 'linewidth', 0.4); grid on; set(gca,'fontsize',fs);
ylabel('position error [m]'); legend('x', 'y', 'location', 'northeast');
title(sprintf('LKF error: RMSE %.3f m (raw double integration %.0f m)', m.est_pos_rmse, m.raw_pos_rmse), 'fontsize', fs+1);
subplot(2,1,2); set(gca,'position',[0.19 0.12 0.77 0.34]);
ev = [o.est(w,3) - o.X(w,4).*cos(o.X(w,3)), o.est(w,4) - o.X(w,4).*sin(o.X(w,3))];
plot(t, ev(:,1), 'b-', t, ev(:,2), 'r-', 'linewidth', 0.4); grid on; set(gca,'fontsize',fs);
xlabel('t [s]'); ylabel('velocity error [m/s]');
nav_savefig(f, fullfile(fd,'fig_nav_lkf_error.png'), 3.5, 3.6);
% 7) heading error and sensitivity bar
f = figure('visible','off');
if numel(M) > 3
    vm = [2.0 2.5 3.0]; ix = [4 1 5];
    subplot(1,2,1);
    plot(vm, cellfun(@(q) q.cte_rmse, M(ix)), 'bo-', vm, cellfun(@(q) q.cte_max, M(ix)), 'rs-'); hold on;
    plot([1.9 3.1], 0.05*[1 1], 'b:', [1.9 3.1], 0.15*[1 1], 'r:');
    grid on; set(gca,'fontsize',fs); xlabel('v_{max,track} [m/s]'); ylabel('CTE [m]'); legend('RMSE','max','location','northwest');
    subplot(1,2,2);
    plot(vm, cellfun(@(q) q.lap_time, M(ix)), 'ko-'); grid on; set(gca,'fontsize',fs);
    xlabel('v_{max,track} [m/s]'); ylabel('lap time [s]');
end
if numel(M) > 3
    nav_savefig(f, fullfile(fd,'fig_nav_speed_sens.png'), 3.5, 2.2);
else
    close(f);
end

% 8) seed study: CTE_max and steering rate per seed for the ablation steps
if exist(fullfile(here,'nav_ablation.mat'),'file') && numel(M) >= 14
    A = load(fullfile(here,'nav_ablation.mat'));
    f = figure('visible','off');
    subplot(2,1,1); set(gca,'position',[0.19 0.57 0.77 0.34]);
    en = M([1, 6:14]);
    h = plot(1:10, cellfun(@(q) q.cte_max, A.ABL(1,:)), 'ro-', 1:10, cellfun(@(q) q.cte_max, A.ABL(4,:)), 'gs-', 1:10, cellfun(@(q) q.cte_max, en), 'bd-'); hold on;
    h2 = plot([0.5 10.5], 0.13*[1 1], 'k--'); plot([0.5 10.5], 0.15*[1 1], 'k:');
    set(gca,'fontsize',fs); grid on; ylabel('CTE_{max} [m]'); xlim([0.5 10.5]); ylim([0.08 0.2]);
    legend(h, {'A: iteration 1', 'D: + wheel speed', 'E: nominal'}, 'location', 'northeast');
    title('Noise-seed study (10 seeds)', 'fontsize', fs+1);
    subplot(2,1,2); set(gca,'position',[0.19 0.12 0.77 0.34]);
    plot(1:10, cellfun(@(q) q.delta_rate_rms, A.ABL(1,:)), 'ro-', 1:10, cellfun(@(q) q.delta_rate_rms, A.ABL(4,:)), 'gs-', 1:10, cellfun(@(q) q.delta_rate_rms, en), 'bd-');
    set(gca,'fontsize',fs,'yscale','log'); grid on; xlabel('seed'); ylabel('steering rate RMS [rad/s]'); xlim([0.5 10.5]);
    nav_savefig(f, fullfile(fd,'fig_nav_seeds.png'), 3.5, 3.6);
end
