function out = nav_sim(cfg, trk, ref, P, E)
%NAV_SIM  Closed-loop simulation of the standstill phase, one lap and the run-out.
%   cfg : nav_cfg_default; trk, ref : track and time-parametrised reference (lap starts at
%   sample P.nst+1); P : nav_params; E : nav_noise sequences. Returns logged signals (N rows).
T = P.T; nst = P.nst; nl = numel(ref.s);
N = nst + nl;
sref = [zeros(nst,1); ref.s(:); ref.s(end)];  vref = [zeros(nst,1); ref.v(:); ref.v(end)];
xp = nav_plant_init(trk, cfg.offset0);
xs = nav_est_init(P, xp(1:2), xp(3));
lon = nav_lon_init(); lat = nav_lat_init();
vprev = 0; iest = 1; itru = 1;
useTrue = strcmp(cfg.fb, 'true');
L.X = zeros(N,9); L.est = zeros(N,7); L.y = zeros(N,8);
L.cmd = zeros(N,4);    % v_cmd, delta, u_lat, sat flag lat
L.proj = zeros(N,8);   % s_hat, ey_hat, epsi_hat, s_true, ey_true, psip_true, kap_true, e_s
for k = 1:N
    y = nav_sensor_step(xp, vprev, E(k,:), k, P);
    vprev = xp(4);
    [xs, est] = nav_est_step(xs, y, k, P);
    if useTrue
        ph = xp(1:2); vh = xp(4); psih = xp(3);
        vxy = [xp(4)*cos(xp(3)); xp(4)*sin(xp(3))];
    else
        ph = est(1:2); vh = hypot(est(3), est(4)); psih = est(5);
        vxy = est(3:4);
    end
    [sh, eyh, psip, ~, iest] = nav_project(ph(1), ph(2), iest, trk, P.W);
    [st_, eyt, psipt, kapt, itru] = nav_project(xp(1), xp(2), itru, trk, P.W);
    epsi = atan2(sin(psih - psip), cos(psih - psip));
    [vcmd, lon, es] = nav_lon_step(lon, sh, sref(k), vref(k), vref(k+1), k > nst, P);
    [delta, lat, ulat] = nav_lat_step(lat, eyh, vh, sh, psip, vxy, trk, P);
    L.X(k,:) = xp'; L.est(k,:) = est'; L.y(k,:) = y';
    L.cmd(k,:) = [vcmd delta ulat lat.st.sat];
    L.proj(k,:) = [sh eyh epsi st_ eyt psipt kapt es];
    xp = nav_plant_step(xp, vcmd, delta, P);
end
L.t = (0:N-1)'*T; L.sref = sref(1:N); L.vref = vref(1:N);
L.nst = nst; L.cfg = cfg;
out = L;
end
