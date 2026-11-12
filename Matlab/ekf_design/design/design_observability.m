function [R, out] = design_observability(run_sims, n_seeds)
%DESIGN_OBSERVABILITY  Which of the 5 states are observable from which update.
%
%   [R, out] = design_observability(run_sims, n_seeds)
%
%   Part A (fast, deterministic): covariance analysis along NOMINAL trajectories
%   with the filter's own predict/update code (P only: the updates are applied
%   with zero innovation, so the state stays on the nominal path). Each case is
%   run with the default Q and with Q = 0; with Q = 0 a state whose variance
%   stays at its prior is unobservable in that scenario. For the straight with
%   edges, the eigen-decomposition of P0^-1/2 P P0^-1/2 names the unobservable
%   combination.
%     A1 dead reckoning only            A2 + ZUPT at the start
%     A3 40 m straight, edges / 0.5 m   A4 rect10 (leg 1 without boundary)
%     A5 rect10, boundary on all legs   A6 rect10 + face heading + post on leg 2
%   Part B (run_sims = true): closed-loop rect10 runs with the scale state
%   estimated (default) and frozen (mx_freeze_s = 1, P_ss = 0): NEES and errors.
if nargin < 1, run_sims = true; end
if nargin < 2, n_seeds = 2; end
p = ekf_params();
names = {'A1 dead reckoning', 'A2 + ZUPT', 'A3 straight + edges', 'A4 rect10 (leg1 no edge)', ...
         'A5 rect10 all edges', 'A6 rect10 + face + post'};
sig0 = [p.ekf_p0_xy p.ekf_p0_xy p.ekf_p0_th p.ekf_p0_bg p.ekf_p0_s];
fprintf('\n=== Observability: final sigma / prior sigma (Q = default | Q = 0) ===\n');
fprintf('%-26s %28s   %28s\n', '', 'x     y     th    bg    s', 'x     y     th    bg    s');
R = [];
for k = 1:6
    for q0 = [0 1]
        P = cov_run(p, k, q0);
        out.P{k, q0+1} = P;
        out.ratio(k, :, q0+1) = sqrt(diag(P))'./sig0;
    end
    fprintf('%-26s %s   %s\n', names{k}, sprintf('%5.2f ', out.ratio(k,:,1)), sprintf('%5.2f ', out.ratio(k,:,2)));
end
% unobservable combination on the straight with edges (Q = 0)
S = diag(1./sig0);
[V, D] = eig(S*out.P{3, 2}*S);
[dv, i] = sort(diag(D), 'descend');
fprintf('A3, Q = 0: normalised eigenvalues %s\n', mat2str(dv', 3));
fprintf('  direction of the largest one (x y th bg s, normalised by the prior): %s\n', mat2str(V(:, i(1))', 2));
out.A3_eig = dv;  out.A3_vec = V(:, i(1));
rs3 = out.ratio(3, 5, 2);  rs4 = out.ratio(4, 5, 2);  rs6 = out.ratio(6, 5, 2);
R = [R, vres('scale s from edges on one straight (Q=0)', -1, sprintf('sigma_s/prior %.3f', rs3), ...
    'informational: ~1 = unobservable')];
R = [R, vres('scale s from edges on rect10 (Q=0)', -1, sprintf('sigma_s/prior %.3f', rs4), 'informational')];
R = [R, vres('scale s with face + post (Q=0)', -1, sprintf('sigma_s/prior %.3f', rs6), 'informational')];
if run_sims
    for fz = [0 1]
        pp = p;  pp.mx_freeze_s = fz;
        for s = 1:n_seeds
            L = sim_run(sim_config('rect10', 200 + s, 'p', pp));
            M = sim_metrics(L);
            w = M.wp(M.wp(:,1) < 99, :);
            out.sim(fz+1, s) = struct('f_nees', M.f_nees3, 'nees_mean', mean(M.nees3(M.i0:end)), ...
                'wp_est', max(w(:,2)), 'clos', M.closure, 's_err', L.est(end,5) - L.truth(end,5), ...
                'sig_s', sqrt(max(L.est(end,10), 0)), 's_true', L.truth(end,5), 'L', []);
            fprintf('  rect10 seed %d, freeze_s=%d: NEES in band %.2f (mean %.2f), wp est max %.3f m, closure %.3f m, s err %.4f (sigma %.4f)\n', ...
                s, fz, M.f_nees3, mean(M.nees3(M.i0:end)), max(w(:,2)), M.closure, ...
                L.est(end,5) - L.truth(end,5), sqrt(max(L.est(end,10), 0)));
        end
    end
    a = out.sim(1, :);  b = out.sim(2, :);
    R = [R, vres('rect10 s estimated vs frozen: NEES in band', -1, ...
        sprintf('%.2f vs %.2f', mean([a.f_nees]), mean([b.f_nees])), 'informational')];
    R = [R, vres('rect10 s estimated vs frozen: max wp est err', -1, ...
        sprintf('%.1f vs %.1f cm', 100*max([a.wp_est]), 100*max([b.wp_est])), 'informational')];
end
end

% -------------------------------------------------------------------------
function P = cov_run(p, scen, q0)
if q0
    p.ekf_kd = 0;  p.ekf_gyro_nd = 0;  p.ekf_gyro_sf = 0;  p.ekf_q_bg = 0;  p.ekf_q_s = 0;
    p.ekf_zupt_floor = 1e-6;
end
e = ekf_init(p, 0, 0, 0);
dt = 0.02;  v = 0.3;  w = p.nav_turn_w;  b = p.odo_wheelbase;
c = sim_course('rect10');
if scen == 5, c = sim_course('rect10_all'); end
switch scen
    case 1, plan = [2 40 0];                 % [type value edge]  type 1 still, 2 line, 3 turn
    case 2, plan = [1 3 0; 2 40 0];
    case 3, plan = [1 3 0; 2 40 1];
    otherwise
        plan = [1 3 0; 2 10 (scen == 5); 1 10.5 0; 3 pi/2 0; 2 10 1; 1 10.5 0; 3 pi/2 0; ...
                2 10 1; 1 10.5 0; 3 pi/2 0; 2 10 1; 1 3 0];
end
x = zeros(5, 1);  x(5) = 1;
leg = 0;
for r = 1:size(plan, 1)
    ty = plan(r, 1);
    if ty == 2, leg = leg + 1; end
    if ty == 1
        n = round(plan(r, 2)/dt);
        e.still = 1;  zs = 0;
        for k = 1:n
            e.x = x;  e = ekf_predict_imu(e, p, 0, 0, x(4)*dt, dt, 0);
            zs = zs + dt;
            if zs >= p.ekf_zupt_T - 1e-9
                [e, ~] = ekf_update_zupt(e, p, x(4), zs);  zs = 0;
            end
        end
    elseif ty == 2
        n = round(plan(r, 2)/(v*dt));
        e.still = 0;  snext = 0.5;  s = 0;
        for k = 1:n
            e.x = x;
            e = ekf_predict_imu(e, p, v*dt, v*dt, x(4)*dt, dt, 0);
            x(1) = x(1) + v*dt*cos(x(3));  x(2) = x(2) + v*dt*sin(x(3));  s = s + v*dt;
            e.x = x;
            if plan(r, 3) > 0 && s >= snext
                snext = snext + 0.5;
                if scen == 3
                    ln = [0; 1; -0.2];
                else
                    kseg = find(c.type == 1, leg);  kseg = kseg(end);
                    ln = map_boundary(c, kseg, -1);
                end
                [~, H] = ekf_meas_edge(x, ln, [p.gnd_x; -p.gnd_y_out], zeros(3,1));
                [e, ~] = ekf_update_scalar(e, p, 0, H, p.ekf_sig_edge^2, 2);
            end
            if scen == 6 && leg == 2
                if abs(s - 5.0) < v*dt/2           % face fit at the window end
                    [e, ~] = ekf_update_scalar(e, p, 0, [0 0 1 0 0], (0.13*pi/180)^2, 4);
                end
                if abs(s - 7.1) < v*dt/2           % post abeam
                    [~, H] = ekf_meas_point(x, [0; -0.6], zeros(3,1));
                    [e, ~] = ekf_update_2d(e, p, [0; 0], H, (p.ekf_sig_post^2 + 0.03^2)*[1; 1], 5);
                end
            end
        end
    else
        n = round(plan(r, 2)/(w*dt));
        e.still = 0;
        for k = 1:n
            e.x = x;
            e = ekf_predict_imu(e, p, -w*dt*b/2, w*dt*b/2, (w + x(4))*dt, dt, 0);
            x(3) = x(3) + w*dt;
        end
    end
end
P = e.P;
end
