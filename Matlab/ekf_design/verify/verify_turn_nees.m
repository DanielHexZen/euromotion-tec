function [R, out] = verify_turn_nees(seeds)
%VERIFY_TURN_NEES  Why the filter is overconfident in the in-place turns of the
%   'maneuver' course (NEES <= upper bound in 0.74 of the steps, 50 and 200 Hz).
%
%   [R, out] = verify_turn_nees(seeds)      seeds default 1 (as verify_rate_50_200)
%
%   The same manoeuvre (straight, two circles, 2 + 2 in-place turns, all in the
%   same direction: 12 pi of net turning) is run with one truth quantity
%   removed at a time and with the three scale-factor Q models:
%     base       default truth and filter (mx_sf_mode = 1, mx_sf_corr = 2 pi)
%     sf = 0     plant gyro without scale-factor error
%     b = b_odo  plant wheelbase equal to odo_wheelbase
%     eps = 0    no wheel-diameter mismatch (no translation while spinning)
%     corr 12pi  filter mx_sf_corr = 12 pi (matched to THIS manoeuvre)
%     mode 2     filter mx_sf_mode = 2 (net-angle model)
%     mode 0     filter mx_sf_mode = 0 (literal spec)
%   Printed for the in-place-turn phase: fraction of steps with NEES(x,y,theta)
%   <= upper 95 % bound, mean heading-only NEES e_th^2/P_tt, mean position-only
%   NEES, and at the end of the run: heading error, sigma_theta, their ratio,
%   the analytic values sf*Theta and sf*sqrt(c*Theta).
if nargin < 1 || isempty(seeds), seeds = 1; end
names = {'base', 'sf = 0', 'b = b_odo', 'eps = 0', 'corr 12pi', 'mode 2', 'mode 0'};
nv = numel(names);
out.names = names;
b3 = chi2_quantile(0.975, 3);
fprintf('\nIn-place-turn consistency, maneuver course (50 Hz)\n');
fprintf('%-4s %-10s %8s | %7s %8s %8s | %9s %9s %6s | %9s %9s\n', 'seed', 'variant', 'sf true', ...
    'NEES<=hi', 'NEES_th', 'NEES_xy', 'e_th mrad', 'sig mrad', 'ratio', 'sf*Th', 'model sig');
acc = zeros(nv, 3);
for s = seeds
    for v = 1:nv
        p = ekf_params();
        if v == 5, p.mx_sf_corr = 12*pi; end
        if v == 6, p.mx_sf_mode = 2; end
        if v == 7, p.mx_sf_mode = 0; end
        cfg = sim_config('maneuver', s, 'p', p);
        if v == 2, cfg.tr.sf = 0; end
        if v == 3, cfg.tr.b_true = p.odo_wheelbase; end
        if v == 4, cfg.tr.eps_lr = 0;  cfg.tr.kL = cfg.tr.scale;  cfg.tr.kR = cfg.tr.scale; end
        L = sim_run(cfg);
        M = sim_metrics(L);
        m = abs(L.gdn(:, 3)) < 1e-6 & abs(L.gdn(:, 4)) >= 1e-6;      % in-place turn
        m(1:M.i0) = false;
        nth = M.e(:, 3).^2 ./ L.est(:, 8);
        nxy = zeros(L.n, 1);
        for i = find(m)'
            P = reshape(L.est(i, 34:58), 5, 5);
            nxy(i) = M.e(i, 1:2)*(P(1:2,1:2)\M.e(i, 1:2)');
        end
        Th = L.truth(end, 3) - L.truth(1, 3);                         % net true turn
        f_hi = mean(M.nees3(m) <= b3);
        eth = M.e(end, 3);  sg = sqrt(L.est(end, 8));
        fprintf('%-4d %-10s %8.5f | %7.2f %8.2f %8.2f | %9.2f %9.2f %6.2f | %9.2f %9.2f\n', s, names{v}, ...
            cfg.tr.sf, f_hi, mean(nth(m)), mean(nxy(m)), 1e3*eth, 1e3*sg, abs(eth)/sg, ...
            1e3*cfg.tr.sf*Th, 1e3*p.ekf_gyro_sf*sqrt(p.mx_sf_corr*abs(Th)));
        out.f_hi(s, v) = f_hi;  out.nees_th(s, v) = mean(nth(m));  out.nees_xy(s, v) = mean(nxy(m));
        out.eth(s, v) = eth;  out.sig(s, v) = sg;  out.sf(s) = cfg.tr.sf;  out.Th(s) = Th;
        acc(v, :) = acc(v, :) + [f_hi, mean(nth(m)), 1];
    end
end
R = [];
for v = [1 2 5 6 7]
    R = [R, vres(['in-place turn NEES<=upper, ' names{v}], -1, ...
        sprintf('%.2f (mean heading NEES %.2f)', acc(v,1)/acc(v,3), acc(v,2)/acc(v,3)), ...
        '0.975 expected; informational')]; %#ok<AGROW>
end
end
