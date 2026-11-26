function m = nav_metrics(out, trk, P)
%NAV_METRICS  Performance indices of one run (TRUE position vs. reference), lap window only.
%   Lap window: from the start of the reference (sample nst+1) until the true along-track
%   position reaches the lap length L (run-out excluded).
T = P.T; nst = out.nst; N = size(out.X,1);
s_tr = out.proj(:,4);
k1 = find(s_tr >= trk.L, 1, 'first');
if isempty(k1), k1 = N; m.finished = false; else, m.finished = true; end
w = (nst+1):k1;
m.k0 = nst+1; m.k1 = k1; m.lap_time = (k1 - nst - 1)*T;
cte = out.proj(w,5);
m.cte_max = max(abs(cte)); m.cte_rmse = sqrt(mean(cte.^2)); m.iae_cte = sum(abs(cte))*T;
m.cte_mean_abs = mean(abs(cte));
ate = s_tr(w) - out.sref(w);
m.ate_rmse = sqrt(mean(ate.^2)); m.ate_max = max(abs(ate));
xr = nav_lookup(trk.x, trk.ds, out.sref(w)); yr = nav_lookup(trk.y, trk.ds, out.sref(w));
pe = hypot(out.X(w,1) - xr, out.X(w,2) - yr);
m.pos_rmse = sqrt(mean(pe.^2)); m.pos_max = max(pe);
hw = 0.15;
sw = s_tr(w);
ml = nav_lookup(trk.wl, trk.ds, sw) - cte - hw;
mr = nav_lookup(trk.wr, trk.ds, sw) + cte - hw;
m.margin_min = min(min(ml), min(mr));
% geometric check: distance to the boundary polylines (vertices, spacing ds)
nw = numel(w); dmin = zeros(nw,1);
for q = 1:nw
    i0 = round(sw(q)/trk.ds);
    idx = mod((i0-150):(i0+150), trk.N) + 1;
    px = out.X(w(q),1); py = out.X(w(q),2);
    d1 = min((trk.xl(idx)-px).^2 + (trk.yl(idx)-py).^2);
    d2 = min((trk.xr(idx)-px).^2 + (trk.yr(idx)-py).^2);
    dmin(q) = sqrt(min(d1,d2));
end
m.margin_geo_min = min(dmin) - hw;
% footprint (0.54 x 0.30 m about the reference point): lateral coordinate of the 4 corners
cmin = inf; ic = ones(4,1)*(round(sw(1)/trk.ds)+1);
for q = 1:10:nw
    psi = out.X(w(q),3); c = cos(psi); s = sin(psi);
    for j = 1:4
        a = 0.27*(2*(j<=2)-1); b = 0.15*(2*mod(j,2)-1);
        cx = out.X(w(q),1) + a*c - b*s; cy = out.X(w(q),2) + a*s + b*c;
        [sj, eyj, ~, ~, ic(j)] = nav_project(cx, cy, ic(j), trk, 120);
        cmin = min(cmin, min(nav_lookup(trk.wl, trk.ds, sj) - eyj, nav_lookup(trk.wr, trk.ds, sj) + eyj));
    end
end
m.margin_footprint_min = cmin;
m.violation = (m.margin_min <= 0) || (m.margin_geo_min <= 0);
% actuators
dl = out.cmd(w,2);
m.delta_max = max(abs(dl));
m.delta_rms = sqrt(mean(dl.^2));
m.delta_rate_rms = sqrt(mean(diff(dl).^2))/T;          % commanded steering rate
da = out.X(w,9);                                        % applied (rate-limited) steering
m.delta_app_rate_rms = sqrt(mean(diff(da).^2))/T;
m.rate_limited_pct = 100*mean(abs(min(max(dl, -P.par.delta_max), P.par.delta_max) - da) > P.servo_rate*T + 1e-12);
m.sat_delta_pct = 100*mean(abs(dl) >= P.par.delta_max - 1e-4);
m.sat_ulat_pct = 100*mean(out.cmd(w,4) > 0);
vc = out.cmd(w,1);
m.sat_vcmd_pct = 100*mean(vc <= 1e-9 | vc >= P.vcmd_max - 1e-9);
m.v_mean = mean(out.X(w,4)); m.v_max = max(out.X(w,4));
ev = out.X(w,4) - out.vref(w);
m.v_rmse = sqrt(mean(ev.^2));
% estimation
pe_est = out.est(w,1:2) - out.X(w,1:2);
m.est_pos_rmse = sqrt(mean(sum(pe_est.^2,2))); m.est_pos_max = max(sqrt(sum(pe_est.^2,2)));
ve = [out.est(w,3) - out.X(w,4).*cos(out.X(w,3)), out.est(w,4) - out.X(w,4).*sin(out.X(w,3))];
m.est_vel_rmse = sqrt(mean(sum(ve.^2,2)));
pse = out.est(w,5) - out.X(w,3);
m.est_psi_rmse = sqrt(mean(pse.^2));
% raw double integration of the calibrated, rotated accelerometers from the true initial state
u = out.est(:,6:7); n = size(u,1);
vr = zeros(n,2); pr = zeros(n,2); pr(1,:) = out.X(1,1:2);
for k = 2:n
    vr(k,:) = vr(k-1,:) + T*0.5*(u(k-1,:) + u(k,:));
    pr(k,:) = pr(k-1,:) + T*0.5*(vr(k-1,:) + vr(k,:));
end
er = pr(w,:) - out.X(w,1:2);
m.raw_pos_rmse = sqrt(mean(sum(er.^2,2)));
m.red_vs_raw = 100*(1 - m.est_pos_rmse/m.raw_pos_rmse);
% raw position fixes at their own instants
fv = find(out.y(w,7) ~= 0);
ef = out.y(w(fv),4:5) - out.X(w(fv),1:2);
m.fix_rmse = sqrt(mean(sum(ef.^2,2)));
m.red_vs_fix = 100*(1 - m.est_pos_rmse/m.fix_rmse);
end
