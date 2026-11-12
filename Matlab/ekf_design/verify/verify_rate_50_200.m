function [R, out] = verify_rate_50_200(seeds, do_rect)
%VERIFY_RATE_50_200  D-30 check: EKF at 50 Hz inside S_CTRL versus 200 Hz in S_EST.
%
%   [R, out] = verify_rate_50_200(seeds, do_rect)
%   For every seed, the same truth and the same sensor noise streams are run
%   with ekf_rate = 50 and 200:
%     - 'maneuver' (open loop, so both rates see IDENTICAL sensor data):
%       straight, arcs at nav_w_max = 1.0 rad/s (R = 0.3 m), in-place turns at
%       0.8 and 1.5 rad/s; per phase: RMS/max position and heading error,
%       NEES(x,y,theta) consistency, P_theta growth, and the back-shift error of
%       delayed-measurement queries 10..100 ms old against the true motion
%     - 'rect10' (closed loop; trajectories differ slightly because guidance
%       acts on different estimates): closure, waypoint estimate errors, NEES, NIS
%   The verdict compares the two rates; it does not judge the absolute numbers.
if nargin < 1 || isempty(seeds), seeds = 1:2; end
if nargin < 2, do_rect = true; end
rates = [50 200];
out = struct();
ph_names = {'standing', 'straight', 'arc 1.0 rad/s', 'in-place turn'};
for ir = 1:2
    acc = zeros(4, 6);  nacc = zeros(4, 1);  bkq = zeros(0, 4);
    for s = seeds
        cfg = sim_config('maneuver', s, 'ekf_rate', rates(ir));
        L = sim_run(cfg);
        M = sim_metrics(L);
        ph = arrayfun(@(i) phase_from_cmd(L.gdn(i, 3), L.gdn(i, 4)), (1:L.n)') + 1;
        for q = 1:4
            m = ph == q;  m(1:M.i0) = false;
            if ~any(m), continue; end
            ep = hypot(M.e(m,1), M.e(m,2));
            et = abs(M.e(m,3));
            acc(q, :) = acc(q, :) + [sum(ep.^2), max(ep), sum(et.^2), max(et), ...
                sum(M.nees3(m) <= M.nees3_band(2)), sum(m)];
            nacc(q) = nacc(q) + sum(m);
        end
        bkq = [bkq; L.bkq]; %#ok<AGROW>
        out.man(ir, s).final_pos_err = hypot(M.e(end,1), M.e(end,2)); %#ok<*AGROW>
        out.man(ir, s).final_th_err  = M.e(end,3);
        out.man(ir, s).Ptt_end = L.est(end, 8);
        out.man(ir, s).nees3_mean = mean(M.nees3(M.i0:end));
    end
    for q = 1:4
        n = max(acc(q, 6), 1);
        out.phase(ir, q, :) = [sqrt(acc(q,1)/n), acc(q,2), sqrt(acc(q,3)/n), acc(q,4), acc(q,5)/n];
    end
    for q = 1:4
        m = bkq(:, 4) == q - 1;
        if any(m)
            out.bk(ir, q, :) = [sqrt(mean(bkq(m,2).^2)), max(bkq(m,2)), sqrt(mean(bkq(m,3).^2)), max(bkq(m,3))];
        else
            out.bk(ir, q, :) = nan(1, 4);
        end
    end
    if do_rect
        for s = seeds
            cfg = sim_config('rect10', s, 'ekf_rate', rates(ir));
            L = sim_run(cfg);
            M = sim_metrics(L);
            out.rect(ir, s).closure = M.closure;
            out.rect(ir, s).wp_est = max([M.wp(:,2); 0]);
            out.rect(ir, s).wp_true = max([M.wp(M.wp(:,1) < 99, 3); 0]);
            out.rect(ir, s).f_nees3 = M.f_nees3;
            out.rect(ir, s).f_nis = M.f_nis;
            out.rect(ir, s).finished = M.finished;
        end
    end
end
% ---- report
fprintf('\n50 Hz vs 200 Hz, maneuver scenario, seeds %s (identical sensor data)\n', mat2str(seeds));
fprintf('%-16s | %-34s | %-34s\n', 'phase', '50 Hz: pos rms/max mm, th rms/max mdeg, NEES<=hi', '200 Hz');
for q = 2:4
    a = squeeze(out.phase(1, q, :));  b = squeeze(out.phase(2, q, :));
    fprintf('%-16s | %5.2f/%5.2f  %6.1f/%6.1f  %4.2f | %5.2f/%5.2f  %6.1f/%6.1f  %4.2f\n', ph_names{q}, ...
        1e3*a(1), 1e3*a(2), 1e3*a(3)*180/pi, 1e3*a(4)*180/pi, a(5), 1e3*b(1), 1e3*b(2), 1e3*b(3)*180/pi, 1e3*b(4)*180/pi, b(5));
end
fprintf('back-shift error of 10..100 ms old queries (pos rms/max mm, th rms/max mrad):\n');
for q = 1:4
    a = squeeze(out.bk(1, q, :));  b = squeeze(out.bk(2, q, :));
    fprintf('  %-16s 50 Hz %6.3f/%6.3f  %6.3f/%6.3f   200 Hz %6.3f/%6.3f  %6.3f/%6.3f\n', ph_names{q}, ...
        1e3*a(1), 1e3*a(2), 1e3*a(3), 1e3*a(4), 1e3*b(1), 1e3*b(2), 1e3*b(3), 1e3*b(4));
end
pe50 = [out.man(1, seeds).final_pos_err];  pe200 = [out.man(2, seeds).final_pos_err];
fprintf('final position error (m): 50 Hz %s, 200 Hz %s\n', mat2str(pe50, 3), mat2str(pe200, 3));
fprintf('final P_tt (rad^2):       50 Hz %s, 200 Hz %s\n', mat2str([out.man(1, seeds).Ptt_end], 3), ...
    mat2str([out.man(2, seeds).Ptt_end], 3));
R = [];
% verdict: position error difference small against the error itself
dpos = max(abs(squeeze(out.phase(1, 2:4, 1)) - squeeze(out.phase(2, 2:4, 1))));
dbk  = max(max(abs(squeeze(out.bk(1, :, 2)) - squeeze(out.bk(2, :, 2)))));
R = [R, vres('D-30 pos RMS difference 50 vs 200 Hz', double(dpos < 0.002), ...
    sprintf('%.2f mm', 1e3*dpos), '< 2 mm (vs. 20 cm waypoint goal)')];
R = [R, vres('D-30 back-shift max error 50 Hz', double(max(out.bk(1, :, 2)) < 0.002), ...
    sprintf('%.3f mm (200 Hz: %.3f mm)', 1e3*max(out.bk(1, :, 2)), 1e3*max(out.bk(2, :, 2))), '< 2 mm')];
R = [R, vres('D-30 NEES<=upper fraction 50 / 200 Hz', -1, ...
    sprintf('%.2f / %.2f (straight) %.2f / %.2f (turns)', out.phase(1,2,5), out.phase(2,2,5), ...
    out.phase(1,4,5), out.phase(2,4,5)), 'informational')];
R = [R, vres('D-30 P_tt after turns 50 / 200 Hz (SF term, F-3)', -1, ...
    sprintf('%.3g / %.3g rad^2', mean([out.man(1, seeds).Ptt_end]), mean([out.man(2, seeds).Ptt_end])), ...
    'informational')];
if do_rect
    c50 = [out.rect(1, seeds).closure];  c200 = [out.rect(2, seeds).closure];
    w50 = [out.rect(1, seeds).wp_est];   w200 = [out.rect(2, seeds).wp_est];
    fprintf('rect10 closure (m): 50 Hz %s, 200 Hz %s\n', mat2str(c50, 3), mat2str(c200, 3));
    fprintf('rect10 max waypoint estimate error (m): 50 Hz %s, 200 Hz %s\n', mat2str(w50, 3), mat2str(w200, 3));
    fprintf('rect10 NEES in band: 50 Hz %s, 200 Hz %s\n', mat2str([out.rect(1, seeds).f_nees3], 3), ...
        mat2str([out.rect(2, seeds).f_nees3], 3));
    R = [R, vres('D-30 rect10 closure 50 / 200 Hz', -1, sprintf('%.1f / %.1f cm (mean)', ...
        100*mean(c50), 100*mean(c200)), 'informational')];
    R = [R, vres('D-30 rect10 wp estimate err 50 / 200 Hz', -1, sprintf('%.1f / %.1f cm (max)', ...
        100*max(w50), 100*max(w200)), 'informational')];
end
end

function ph = phase_from_cmd(v, w)
if abs(v) < 1e-6 && abs(w) < 1e-6, ph = 0;
elseif abs(w) < 1e-6, ph = 1;
elseif abs(v) < 1e-6, ph = 3;
else, ph = 2; end
end
