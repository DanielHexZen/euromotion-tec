% TRACK_PREPARE  Scaled (1:10) Mexico City circuit: smooth centreline, curvature, boundaries,
% speed profile and time-parametrised reference. Writes track.mat and fig/fig_track.png.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'common'), fullfile(root,'02_pid'), fullfile(root,'03_lkf'));
here = fileparts(mfilename('fullpath'));
if ~exist(fullfile(here,'fig'),'dir'), mkdir(fullfile(here,'fig')); end

scale = 0.1; ds = 0.05; sig_pos = 0.4; sig_kap = 0.3;
T = 0.02;
cfg0 = nav_cfg_default();
vmax_track = cfg0.vmax; a_lat_max = cfg0.alat; a_acc = cfg0.aacc; a_dec = cfg0.adec; runout = cfg0.runout;

M = load(fullfile(root,'01_sysid','model_id.mat'));
par = M.par;
trk = nav_track_build(fullfile(root,'data','MexicoCity.csv'), scale, ds, sig_pos, sig_kap);
[vper, vlim] = nav_speed_profile(trk.kap, trk.ds, vmax_track, a_lat_max, a_acc, a_dec, par);
ref = nav_reference(trk, vper, T, a_acc, runout);

fprintf('Track: N = %d points, ds = %.4f m, length L = %.2f m (full scale %.0f m)\n', trk.N, trk.ds, trk.L, trk.L/scale);
fprintf('Width (scaled): %.2f ... %.2f m; max |kappa| = %.3f 1/m (min radius %.2f m)\n', min(trk.wl+trk.wr), max(trk.wl+trk.wr), max(abs(trk.kap)), 1/max(abs(trk.kap)));
fprintf('Speed profile: vmax_track %.1f, a_lat %.1f, a_x +%.1f/-%.1f -> mean %.2f m/s, min %.2f m/s\n', vmax_track, a_lat_max, a_acc, a_dec, mean(vper), min(vper));
fprintf('Reference lap time (s_ref reaches L): %.1f s\n', ref.t_lap);
fprintf('Closure: |p(N+1)-p(1)| = %.4f m (ds = %.3f)\n', hypot(trk.x(1)-trk.x(end), trk.y(1)-trk.y(end)), trk.ds);

prof = struct('vmax_track',vmax_track,'a_lat_max',a_lat_max,'a_acc',a_acc,'a_dec',a_dec,'runout',runout,'scale',scale);
sref = ref.s; vref = ref.v; tref = (0:numel(sref)-1)'*T; t_lap_ref = ref.t_lap;
save(fullfile(here,'track.mat'), 'trk','vper','vlim','ref','prof','T','sref','vref','tref','t_lap_ref', '-v7');

% ---- figure ----
f = figure('visible','off','position',[100 100 900 900]);
subplot(2,2,[1 2]);
plot(trk.xl, trk.yl, 'k-', trk.xr, trk.yr, 'k-', trk.x, trk.y, 'b--'); hold on;
plot(trk.x(1), trk.y(1), 'rs', 'markerfacecolor','r', 'markersize', 7);
ia = 1:round(trk.N/12):trk.N;
quiver(trk.x(ia), trk.y(ia), cos(trk.psi(ia)), sin(trk.psi(ia)), 0.4, 'color',[0.2 0.5 0.2]);
axis equal; grid on; xlabel('x [m]'); ylabel('y [m]');
title(sprintf('Mexico City 1:10 (L = %.0f m); red = start/finish, direction of travel', trk.L));
subplot(2,2,3);
plot(trk.s, trk.kap, 'b-'); grid on; xlabel('s [m]'); ylabel('\kappa [1/m]'); title('Curvature (smoothed)');
subplot(2,2,4);
plot(trk.s, vlim, 'c-', trk.s, vper, 'r-', 'linewidth', 1.2); grid on; xlabel('s [m]'); ylabel('v_{ref} [m/s]');
legend('curvature/steering limit', 'with accel. limits', 'location','south'); title('Speed profile');
print(f, fullfile(here,'fig','fig_track.png'), '-dpng', '-r200');
close(f);
