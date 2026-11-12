function M = sim_metrics(L, varargin)
%SIM_METRICS  Errors, NEES and NIS statistics of one sim_run log.
%
%   M = sim_metrics(L)
%   M.e        estimation error [ex ey eth ebg es] per estimator step (est - truth)
%   M.e_ct     cross-track estimation error w.r.t. the current LINE segment
%   M.e_at     along-track estimation error
%   M.nees3    NEES of (x, y, theta), M.nees5 of all five states
%   M.f_nees3  fraction of steps with nees3 inside the two-sided 95 % chi2_3 band
%   M.nis      [type d2 st] of every update; M.f_nis fraction inside the
%              two-sided 95 % band of its dof (rejected updates count as outside)
%   M.f_nis_type  same per type (ZUPT EDGE FACE HEADING POINT), NaN if none
%   M.wp       waypoints [id |p_est - p_true| |p_true - p_wp| ...]
%   M.closure  |p_true(end) - p_start|
%   Steps before the first motion (INIT) are excluded from NEES (P0 there is
%   the start uncertainty and is checked separately).
c  = L.cfg.course;
X  = L.est(:, 1:5);
T  = L.truth;
e  = X - T(:, 1:5);
e(:, 3) = wrap_d(e(:, 3));
M.e = e;
n = size(X, 1);
seg = L.gdn(:, 6);
M.e_ct = nan(n, 1);  M.e_at = nan(n, 1);
for i = 1:n
    k = seg(i);
    if k >= 1 && k <= c.n_seg && c.type(k) == 1
        dx = c.x1(k) - c.x0(k);  dy = c.y1(k) - c.y0(k);  Ls = hypot(dx, dy);
        tx = dx/Ls;  ty = dy/Ls;
        M.e_at(i) =  e(i,1)*tx + e(i,2)*ty;
        M.e_ct(i) = -e(i,1)*ty + e(i,2)*tx;
    end
end
M.nees3 = zeros(n, 1);  M.nees5 = zeros(n, 1);
for i = 1:n
    P = reshape(L.est(i, 34:58), 5, 5);
    P = 0.5*(P + P');
    v3 = e(i, 1:3)';
    M.nees3(i) = v3'*(P(1:3,1:3)\v3);
    if P(5,5) > 0
        M.nees5(i) = e(i,:)*(P\e(i,:)');
    else
        M.nees5(i) = e(i,1:4)*(P(1:4,1:4)\e(i,1:4)');
    end
end
moving = find(L.gdn(:, 8) ~= 1, 1);
if isempty(moving), moving = 1; end
M.i0 = moving;
b3 = chi2_quantile([0.025 0.975], 3);
M.nees3_band = b3;
M.f_nees3 = mean(M.nees3(moving:end) >= b3(1) & M.nees3(moving:end) <= b3(2));
M.f_nees3_upper = mean(M.nees3(moving:end) <= b3(2));
% NIS
R = [];
for i = 1:n
    rec = reshape(L.upd(i, :), 8, 7);
    m = rec(:, 1) > 0;
    R = [R; rec(m, :)]; %#ok<AGROW>
end
M.nis = R;
b1 = chi2_quantile([0.025 0.975], 1);
b2 = chi2_quantile([0.025 0.975], 2);
M.nis_band1 = b1;  M.nis_band2 = b2;
if isempty(R)
    M.f_nis = NaN;  M.f_nis_type = nan(1, 5);
else
    dof = 1 + (R(:,1) == 5);
    lo = b1(1)*(dof == 1) + b2(1)*(dof == 2);
    hi = b1(2)*(dof == 1) + b2(2)*(dof == 2);
    inb = R(:,3) >= lo & R(:,3) <= hi & R(:,2) == 0;
    M.f_nis = mean(inb);
    M.f_nis_type = nan(1, 5);
    M.n_nis_type = zeros(1, 5);
    for ty = 1:5
        s = R(:,1) == ty;
        M.n_nis_type(ty) = sum(s);
        if any(s), M.f_nis_type(ty) = mean(inb(s)); end
    end
end
% waypoints and closure
M.wp = zeros(0, 4);
for i = 1:size(L.wp, 1)
    id = L.wp(i, 1);
    if id == 99
        pw = c.start(1:2);
    else
        k = find(c.type == 4 & c.wp_id == id, 1);
        pw = [c.x1(k-1), c.y1(k-1)];
    end
    M.wp(end+1, :) = [id, hypot(L.wp(i,3) - L.wp(i,5), L.wp(i,4) - L.wp(i,6)), ...
                      hypot(L.wp(i,5) - pw(1), L.wp(i,6) - pw(2)), L.wp(i,2)];
end
M.closure = hypot(T(end,1) - c.start(1), T(end,2) - c.start(2));
M.finished = any(L.gdn(:, 8) == 7);
M.n_edge_acc = L.est(end, 18);  M.n_edge_rej = L.est(end, 23);
end

function a = wrap_d(a)
a = a - 2*pi*floor((a + pi)/(2*pi));
end
