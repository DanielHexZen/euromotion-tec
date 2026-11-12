function [R, out] = verify_slip(seed, do_rect)
%VERIFY_SLIP  Slip detector (REQ-EKF-04) at the 50 Hz EKF step of D-30 against 200 Hz.
%
%   [R, out] = verify_slip(seed, do_rect)       seed default 1, do_rect default true
%
%   The spec fires the detector after ekf_slip_n = 3 consecutive steps with
%   |r_slip| > ekf_slip_thr (0.15 rad/s): 15 ms at 200 Hz, 60 ms at 50 Hz
%   ([VERIFY] in D-30). This function measures what that costs.
%
%   Part 1, slip events (open loop, default noise, truth from sim_config
%   'maneuver'): after 3 s standstill and 2.5 s of steady motion ONE wheel
%   loses a fraction f of its ground motion for a time T (f = 1: the wheel
%   spins, no ground motion), on
%     straight      v = 0.30 m/s                         (left wheel slips)
%     arc           v = 0.30 m/s, w = 1.0 rad/s          (outer = right wheel slips)
%     in-place turn w = 0.8 rad/s                        (right wheel slips)
%   The SAME sensor data go through five estimators (sim_est_step, prediction
%   only): 50 Hz with ekf_slip_n = 3, 2, 1 and 200 Hz with ekf_slip_n = 3, all
%   with the adopted slip inflation (mx_slip_abs = 1, spec D-45: while slip is
%   set the distance noise uses (|ds_l| + |ds_r|)/2 instead of |ds_e|), and for
%   comparison 50 Hz with ekf_slip_n = 2 and mx_slip_abs = 0 ("lit": the literal
%   form of spec version <= 1.4, kd*|ds_e|*ekf_slip_infl, finding F-11).
%   At 50 Hz a time threshold "|r_slip| > thr for >= T_slip" is the count
%   ceil(T_slip/20 ms), so n = 1, 2, 3 are the time thresholds <= 20, 40, 60 ms.
%   Reported per case and estimator:
%     det      1 if the flag was set
%     delay    slip onset -> first step with slip = 1, ms
%   All errors are differences to a run WITHOUT the slip (same seed and noise):
%     e@det    position error caused by the slip up to the detection step, mm
%              (this part gets NO covariance inflation)
%     e_end    position error caused by the whole event, mm (0.5 s after its end)
%     th_end   heading error caused by the event, mdeg (IMU heading: ~0)
%     sig_add  sqrt of the position variance the slip flag added, mm
%     ratio    e_end/sig_add (> 3: the event is not covered by the inflation;
%              Inf: never flagged, nothing added)
%   For the detected in-place events the error is also split along the principal
%   axes of the added position covariance dP (2x2): the distance noise is mapped
%   along the heading only (g = [s*c, s*n, 0, 0, 0]), so this shows whether the
%   error is covered in both directions.
%
%   Part 2, false detections (no slip): |r_slip| statistics of the whole
%   'maneuver' course (straights, arcs, in-place turns, all transients) at 50
%   and 200 Hz, with the number of episodes each count would have produced;
%   with do_rect, one closed-loop rect10 mission per rate with ekf_slip_n = 1
%   (any single step over the threshold is an episode).
if nargin < 1 || isempty(seed), seed = 1; end
if nargin < 2, do_rect = true; end
p0 = ekf_params();
% estimators: [rate, ekf_slip_n, mx_slip_abs]
EST = [50 3 1; 50 2 1; 50 1 1; 200 3 1; 50 2 0];
enames = {'50Hz n=3', '50Hz n=2', '50Hz n=1', '200Hz n=3', '50Hz n=2 lit'};
man   = {'straight', 'arc', 'in-place'};
mv    = [0.30 0.30 0.0];
mw    = [0.0  1.0  0.8];
mside = [1 2 2];
fracs = [0.15 0.3 1.0];
durs  = [0.030 0.050 0.070 0.100 0.300];
t_slip = 5.5;  T_end = 6.4;

fprintf('\n=== Slip detector: events (thr %.2f rad/s; one wheel loses fraction f of its ground motion for T) ===\n', p0.ekf_slip_thr);
fprintf('%-9s %4s %5s %6s | %-12s %3s %6s %6s %6s %7s %7s %6s\n', 'manoeuvre', 'f', 'T ms', 'r nom', ...
    'estimator', 'det', 'delay', 'e@det', 'e_end', 'th_end', 'sig_add', 'ratio');
rows = zeros(0, 17);
for im = 1:3
    % reference without slip: same seed, so identical noise; the differences
    % below are the effect of the slip event alone
    o0 = run_open(p0, seed, EST, [3 0 0; T_end - 3, mv(im), mw(im)], T_end, []);
    for f = fracs
        for T = durs
            sl = struct('t0', t_slip, 't1', t_slip + T, 'side', mside(im), 'frac', f);
            % phase of the onset inside the 20 ms step is fixed by t_slip (5.5 s
            % = 0.5 ms before the control tick at 5.501 s: the first 50 Hz step
            % after the onset contains almost no slip, which is the worst case)
            o = run_open(p0, seed, EST, [3 0 0; T_end - 3, mv(im), mw(im)], T_end, sl);
            wheel_v = abs(mv(im) + (2*(mside(im) == 2) - 1)*mw(im)*p0.odo_wheelbase/2);
            r_nom = f*wheel_v/p0.odo_wheelbase;
            for ie = 1:size(EST, 1)
                E = o.est{ie};
                i0 = find(E.t <= t_slip, 1, 'last');                 % last step before the onset
                i1 = find(E.t >= t_slip + T + 0.5, 1);
                if isempty(i1), i1 = numel(E.t); end
                id = find(E.slip(i0+1:i1) > 0, 1) + i0;
                E0 = o0.est{ie};
                de = E.e(:, 1:2) - E0.e(:, 1:2);
                dth = E.e(:, 3) - E0.e(:, 3);
                det = ~isempty(id);
                if det
                    delay = E.t(id) - t_slip;  e_det = norm(de(id, :));
                else
                    delay = NaN;  e_det = NaN;
                end
                e_end = norm(de(i1, :));
                sig_add = sqrt(max(E.trP(i1) - E0.trP(i1), 0));
                dP = E.P(i1, :) - E0.P(i1, :);                       % added [Pxx Pyy Pxy]
                fprintf('%-9s %4.2f %5.0f %6.2f | %-12s %3d %6.0f %6.2f %6.2f %7.1f %7.2f %6.2f\n', man{im}, f, 1e3*T, ...
                    r_nom, enames{ie}, det, 1e3*delay, 1e3*e_det, 1e3*e_end, 1e3*dth(i1)*180/pi, 1e3*sig_add, e_end/sig_add);
                rows(end+1, :) = [im, f, T, r_nom, ie, det, delay, e_det, e_end, dth(i1), sig_add, e_end/sig_add, de(i1, :), dP]; %#ok<AGROW>
            end
        end
    end
end
out.rows = rows;
out.cols = 'manoeuvre f T r_nom estimator det delay e_det e_end th_end sig_add ratio de_x de_y dPxx dPyy dPxy';

% ---- summary of part 1
fprintf('\nSummary over the %d events (r nom = f*v_wheel/b is the residual the slip produces):\n', size(rows, 1)/size(EST, 1));
fprintf('%-12s %9s %14s %14s %16s %14s\n', 'estimator', 'detected', 'delay mean ms', 'delay max ms', 'e@det max mm', 'missed: e_end max mm');
S = zeros(size(EST, 1), 5);
for ie = 1:size(EST, 1)
    m  = rows(:, 5) == ie & rows(:, 4) > p0.ekf_slip_thr;            % events above the threshold
    md = m & rows(:, 6) > 0;
    mm = m & rows(:, 6) == 0;
    S(ie, :) = [sum(md)/max(sum(m), 1), mean(rows(md, 7)), max([rows(md, 7); 0]), max([rows(md, 8); 0]), max([rows(mm, 9); 0])];
    fprintf('%-12s %4d/%-4d %14.1f %14.1f %16.2f %14.2f\n', enames{ie}, sum(md), sum(m), 1e3*S(ie, 2), 1e3*S(ie, 3), ...
        1e3*S(ie, 4), 1e3*S(ie, 5));
end
out.summary = S;
% shortest detected duration per estimator, full slip on the straight
for ie = 1:size(EST, 1)
    m = rows(:, 5) == ie & rows(:, 1) == 1 & rows(:, 2) == 1 & rows(:, 6) > 0;
    out.T_min_detected(ie) = min([rows(m, 3); Inf]);
end
fprintf('shortest detected full slip on the straight (of %s ms): %s ms\n', mat2str(1e3*durs), mat2str(1e3*out.T_min_detected));

% coverage of the detected in-place events (the literal inflation adds nothing there)
m1 = rows(:, 1) == 3 & rows(:, 6) > 0 & rows(:, 5) == 5;             % literal form
m2 = rows(:, 1) == 3 & rows(:, 6) > 0 & rows(:, 5) == 2;             % adopted form (D-45)
out.inplace = [max([rows(m2, 9); 0]), max([rows(m1, 11); 0]), max([rows(m2, 11); 0]), max([rows(m2, 12); 0])];
fprintf('in-place turn, detected events, 50 Hz n=2: error up to %.1f mm; variance added: literal (mx_slip_abs = 0) sigma %.2f mm, adopted (D-45) %.2f mm (worst ratio %.2f)\n', ...
    1e3*out.inplace(1), 1e3*out.inplace(2), 1e3*out.inplace(3), out.inplace(4));
% direction: error and added sigma along the principal axes of the added
% position covariance (major = mean heading direction while the flag was set)
fprintf('same events by direction (principal axes of the added covariance), mm:\n');
fprintf('%5s %6s | %8s %8s %6s | %8s %8s %6s | %6s\n', 'f', 'T ms', 'e major', 'sigma', 'ratio', 'e minor', 'sigma', 'ratio', 'd_M');
out.inplace_dir = zeros(0, 9);
for i = find(m2)'
    dP = [rows(i, 15), rows(i, 17); rows(i, 17), rows(i, 16)];
    [V, D] = eig(dP);
    [dd, k] = sort(diag(D), 'descend');  V = V(:, k);
    dd = max(dd, 0);
    ea = abs(V'*rows(i, 13:14)');
    sg = sqrt(dd);
    if sg(2) > 1e-5
        r2 = ea(2)/sg(2);
        dM = sqrt(sum(ea.^2./dd));                                    % Mahalanobis distance of the error
    else
        % flag set for one step only: the added covariance has rank 1 (minor
        % sigma below 0.01 mm); no ratio, d_M from the major axis alone
        r2 = NaN;
        dM = ea(1)/sg(1);
    end
    out.inplace_dir(end+1, :) = [rows(i, 2), rows(i, 3), ea(1), sg(1), ea(1)/sg(1), ea(2), sg(2), r2, dM];
    fprintf('%5.2f %6.0f | %8.3f %8.3f %6.2f | %8.3f %8.3f %6.2f | %6.2f\n', rows(i, 2), 1e3*rows(i, 3), 1e3*ea(1), 1e3*sg(1), ...
        ea(1)/sg(1), 1e3*ea(2), 1e3*sg(2), r2, dM);
end

% ---- part 2: false detections without slip
fprintf('\n=== Slip detector: residual without slip (maneuver course, seed %d) ===\n', seed);
c = sim_course('maneuver');
tab = [diff([0 c.ol_t(1:c.n_ol)])', c.ol_v(1:c.n_ol)', c.ol_w(1:c.n_ol)'];
o = run_open(p0, seed, EST([1 4], :), tab, c.ol_t(c.n_ol), []);
fprintf('%-6s %10s %10s %10s %12s | episodes for n = 1, 2, 3\n', 'rate', 'max |r|', 'p99.9 |r|', 'rms r', 'steps > thr');
rate_names = {'50 Hz', '200 Hz'};
for ie = 1:2
    E = o.est{ie};
    r = abs(E.r(E.still == 0));
    rs = sort(r);
    ep = zeros(1, 3);
    for n = 1:3
        ep(n) = episodes(abs(E.r) > p0.ekf_slip_thr & E.still == 0, n);
    end
    fprintf('%-6s %10.4f %10.4f %10.4f %8d/%-6d | %d, %d, %d\n', rate_names{ie}, max(r), rs(ceil(0.999*numel(rs))), ...
        sqrt(mean(r.^2)), sum(r > p0.ekf_slip_thr), numel(r), ep(1), ep(2), ep(3));
    out.noslip(ie) = struct('rmax', max(r), 'rms', sqrt(mean(r.^2)), 'n_over', sum(r > p0.ekf_slip_thr), 'ep', ep);
end
q50  = p0.odo_m_per_cnt/(p0.odo_wheelbase*0.02);
q200 = p0.odo_m_per_cnt/(p0.odo_wheelbase*0.005);
fprintf('one encoder count of wheel difference = %.4f rad/s at 50 Hz, %.4f rad/s at 200 Hz (threshold %.2f)\n', ...
    q50, q200, p0.ekf_slip_thr);
out.q_count = [q50 q200];

R = [];
R = [R, vres('slip: false episodes, maneuver, 50 Hz (n = 1/2/3)', double(out.noslip(1).ep(3) == 0), ...
    sprintf('%d / %d / %d, max |r| %.3f rad/s', out.noslip(1).ep, out.noslip(1).rmax), 'none at n = 3')];
R = [R, vres('slip: false episodes, maneuver, 200 Hz (n = 1/2/3)', -1, ...
    sprintf('%d / %d / %d, max |r| %.3f rad/s', out.noslip(2).ep, out.noslip(2).rmax), 'informational (former S_EST)')];
if do_rect
    for rate = [50 200]
        p = p0;  p.ekf_slip_n = 1;
        L = sim_run(sim_config('rect10', 100 + seed, 'ekf_rate', rate, 'p', p));
        ne = L.est(end, 14);
        fprintf('rect10 closed loop, %d Hz, ekf_slip_n = 1: %d episodes (steps with slip flag %d of %d)\n', rate, ne, ...
            sum(L.est(:, 13) > 0), L.n);
        out.rect_ep(1 + (rate == 200)) = ne;
        R = [R, vres(sprintf('slip: false episodes, rect10, %d Hz, n = 1', rate), double(ne == 0) - 2*(rate == 200)*double(ne ~= 0), ...
            sprintf('%d', ne), 'none (any single step over thr counts)')]; %#ok<AGROW>
    end
end
R = [R, vres('slip: in-place turn, sigma added by the slip flag, literal / adopted (D-45)', -1, ...
    sprintf('%.2f / %.2f mm for up to %.1f mm error', 1e3*out.inplace(2), 1e3*out.inplace(3), 1e3*out.inplace(1)), 'informational (F-11)')];
R = [R, vres('slip: detected events above thr, 50 Hz n=3 / n=2 / 200 Hz n=3', -1, ...
    sprintf('%.0f %% / %.0f %% / %.0f %%', 100*S(1,1), 100*S(2,1), 100*S(4,1)), 'informational')];
R = [R, vres('slip: detection delay max, 50 Hz n=3 / n=2 / 200 Hz n=3', -1, ...
    sprintf('%.0f / %.0f / %.0f ms', 1e3*S(1,3), 1e3*S(2,3), 1e3*S(4,3)), 'informational')];
R = [R, vres('slip: uninflated error at detection, max, 50 Hz n=3 / n=2 / 200 Hz n=3', -1, ...
    sprintf('%.1f / %.1f / %.1f mm', 1e3*S(1,4), 1e3*S(2,4), 1e3*S(4,4)), 'informational')];
R = [R, vres('slip: error of undetected events, max, 50 Hz n=3 / n=2 / 200 Hz n=3', -1, ...
    sprintf('%.1f / %.1f / %.1f mm', 1e3*S(1,5), 1e3*S(2,5), 1e3*S(4,5)), 'informational')];
end

function n = episodes(over, cnt)
% number of rising edges of "cnt consecutive steps over the threshold"
run = 0;  n = 0;  on = false;
for i = 1:numel(over)
    if over(i), run = run + 1; else, run = 0; end
    now = run >= cnt;
    if now && ~on, n = n + 1; end
    on = now;
end
end

function o = run_open(p0, seed, EST, tab, T_end, sl)
% Open-loop run with the scheduler of sim_run (plant, IMU, yaw ring, encoder at
% 1 kHz; guidance at 50 Hz) and several estimators on the same sensor data.
% tab = [duration v w] rows; sl = slip episode struct or [].
cfg = sim_config('maneuver', seed, 'p', p0);
c = cfg.course;
n = size(tab, 1);
c.n_ol = n;
c.ol_t(:) = 0;  c.ol_v(:) = 0;  c.ol_w(:) = 0;
c.ol_t(1:n) = cumsum(tab(:, 1))';  c.ol_v(1:n) = tab(:, 2)';  c.ol_w(1:n) = tab(:, 3)';
tr = cfg.tr;
if ~isempty(sl)
    tr.n_slip = 1;
    tr.slip_t0(1) = sl.t0;  tr.slip_t1(1) = sl.t1;  tr.slip_side(1) = sl.side;  tr.slip_frac(1) = sl.frac;
end
ne = size(EST, 1);
pc = cell(1, ne);  es = cell(1, ne);  div = zeros(1, ne);  ph = zeros(1, ne);
K = round(T_end*1000);
for i = 1:ne
    p = p0;  p.ekf_slip_n = EST(i, 2);  p.mx_slip_abs = EST(i, 3);
    pc{i} = ekf_param_cast(p, cfg.cls);
    es{i} = sim_est_init(pc{i}, c);
    div(i) = round(1000/EST(i, 1));
    ph(i) = double(EST(i, 1) == 50);                 % as sim_run: 50 Hz at phase 1, 200 Hz at phase 0
    nmax = ceil(K/div(i)) + 2;
    o.est{i} = struct('t', zeros(nmax, 1), 'e', zeros(nmax, 3), 'slip', zeros(nmax, 1), 'r', zeros(nmax, 1), ...
        'still', zeros(nmax, 1), 'trP', zeros(nmax, 1), 'P', zeros(nmax, 3), 'n', 0);
end
plant = sim_plant_init(tr, c);
imu   = sim_imu_init(tr);
ys    = sim_yaw_init(pc{1});
gs    = sim_gdn_init(p0, c);
cmd = [0; 0];  y_imu = zeros(5, 1);  y_gst = [1; 1; 0; gs.state];  y_gdn = zeros(10, 1);
y_est1 = zeros(70, 1);
y_gnd = zeros(97, 1);  y_lat = zeros(81, 1);
t_imu_next = tr.imu_t0;
for k = 0:K
    t = k*1e-3;
    while t_imu_next < t
        [imu, y_imu] = sim_imu_step(imu, plant.ylast, tr);
        t_imu_next = tr.imu_t0 + imu.i/tr.f_imu;
    end
    [plant, yp] = sim_plant_step(plant, cmd, tr);
    y_enc = sim_enc_step(yp, p0);
    [ys, y_yaw] = sim_yaw_step(ys, y_imu, pc{1});
    for i = 1:ne
        if mod(k - ph(i), div(i)) == 0 && k >= ph(i)
            [es{i}, y_est] = sim_est_step(es{i}, y_enc, y_yaw, y_gnd, y_lat, y_gst, pc{i}, c);
            if i == 1, y_est1 = y_est; end
            E = o.est{i};
            j = E.n + 1;
            E.n = j;  E.t(j) = t;
            E.e(j, :) = [y_est(1) - yp(2), y_est(2) - yp(3), y_est(3) - yp(4)];
            E.e(j, 3) = E.e(j, 3) - 2*pi*round(E.e(j, 3)/(2*pi));
            E.slip(j) = y_est(13);
            E.r(j) = double(es{i}.ekf.r_slip);
            E.still(j) = y_est(12);
            E.trP(j) = y_est(6) + y_est(7);
            E.P(j, :) = [y_est(6), y_est(7), y_est(11)];
            o.est{i} = E;
        end
    end
    if mod(k - 1, 20) == 0 && k >= 1
        [gs, y_gdn] = sim_gdn_step(gs, y_est1, p0, c);
        y_gst = y_gdn(5:8);
    end
    cmd = y_gdn(1:2);
end
for i = 1:ne
    E = o.est{i};  j = E.n;
    E.t = E.t(1:j);  E.e = E.e(1:j, :);  E.slip = E.slip(1:j);  E.r = E.r(1:j);  E.still = E.still(1:j);  E.trP = E.trP(1:j);  E.P = E.P(1:j, :);
    o.est{i} = E;
end
end
