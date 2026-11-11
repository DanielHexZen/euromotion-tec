function plot_run(L, name)
%PLOT_RUN  Standard figures of one sim_run log: path, errors with 3-sigma, NEES, NIS.
%   Saved as results/<name>_*.png (only where figures can be rendered, see can_plot).
if ~can_plot(), return; end
M = sim_metrics(L);
t = L.t;
f = figure('Visible', 'off');
plot(L.truth(:,1), L.truth(:,2), 'k-', L.est(:,1), L.est(:,2), 'b--');
axis equal; grid on; xlabel('x [m]'); ylabel('y [m]'); legend('truth', 'estimate');
title(['path: ' name]);
save_fig(f, [name '_path']);
f = figure('Visible', 'off');
lab = {'x [m]', 'y [m]', '\theta [deg]', 'b_g [deg/s]', 's [-]'};
sc  = [1 1 180/pi 180/pi 1];
for i = 1:5
    subplot(5, 1, i);
    sg = 3*sqrt(max(L.est(:, 5+i), 0))*sc(i);
    plot(t, M.e(:, i)*sc(i), 'b', t, sg, 'r:', t, -sg, 'r:');
    grid on; ylabel(lab{i});
end
xlabel('t [s]'); subplot(5, 1, 1); title(['estimation error and 3\sigma: ' name]);
save_fig(f, [name '_errors']);
f = figure('Visible', 'off');
b = M.nees3_band;
semilogy(t, max(M.nees3, 1e-6), 'b.', t([1 end]), [b(1) b(1)], 'r--', t([1 end]), [b(2) b(2)], 'r--');
grid on; xlabel('t [s]'); ylabel('NEES (x, y, \theta)');
title(sprintf('NEES, %.1f %% in the 95 %% band', 100*M.f_nees3));
save_fig(f, [name '_nees']);
if ~isempty(M.nis)
    f = figure('Visible', 'off');
    semilogy(max(M.nis(:,3), 1e-6), '.'); grid on; hold on;
    plot([1 size(M.nis,1)], M.nis_band1(2)*[1 1], 'r--', [1 size(M.nis,1)], M.nis_band1(1)*[1 1], 'r--');
    xlabel('update #'); ylabel('NIS');
    title(sprintf('NIS, %.1f %% in the 95 %% band', 100*M.f_nis));
    save_fig(f, [name '_nis']);
end
end
