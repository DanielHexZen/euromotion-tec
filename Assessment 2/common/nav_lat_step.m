function [delta, ls, ulat, e] = nav_lat_step(ls, ey_hat, v_hat, s_hat, psip, vxy, trk, P) %#codegen
%NAV_LAT_STEP  Lateral PID on the estimated cross-track error (reference 0), gain-scheduled
%   (pid_sched, v0/v), curvature feedforward with look-ahead:
%     u_ff = v_hat * kappa(s_hat + v_hat*tla) / k_delta ,   u_lat = sat(u_ff + PID(e_ref - ey_hat))
%     delta = lat_inverse(u_lat, v_hat).
%   (the measurement-D variants are used only at v_hat >= 1 m/s, the lower end of the identified
%   speed range; below, i.e. during the start-up, the designed differenced D is used).
%   D action (P.dmode): 1 = as designed (filtered difference of e inside pid_step); 2 = on the
%   MEASUREMENT only: Kd(v0/v) is moved out of pid_step and u_D = -Kd(v0/v) d(ey)/dt with
%   d(ey)/dt = vxy . n(psip) (cross-track component of the LKF velocity estimate, psip = path
%   heading at the foot point) -> same D action in steady tracking, no kick from the 10 Hz
%   position-fix jumps (dmode 3: velocity rate + low-passed difference correction, removes the
%   velocity bias of the LKF). Kp, Ki, Kd values are those of the pole-placement design.
%   The zero-cancelling prefilter (pid_prefilt) shapes the decay of the initial offset
%   (reference steps from the offset at engagement to 0). Inactive below P.vlat_min.
if v_hat < P.vlat_min
    ls.act = 0; ls.st = pid_init(0); ulat = 0; delta = 0; e = 0;
    return;
end
if ls.act == 0
    ls.act = 1;
    ls.st = pid_init(-ey_hat);
    ls.pf = pid_prefilt_init(ey_hat);
    ls.eyp = ey_hat; ls.c = 0;
end
[eref, ls.pf] = pid_prefilt(0, ls.pf, P.lat);
e = eref - ey_hat;
g = pid_sched(P.lat, v_hat, 1);
ul = pid_ulat_limit(v_hat, P.par);
g.umin = -ul; g.umax = ul;
kf = nav_lookup(trk.kap, trk.ds, s_hat + v_hat*P.tla);
uff = v_hat*kf/P.par.k_delta;
if P.dmode >= 2 && v_hat >= 1.0
    dey = -sin(psip)*vxy(1) + cos(psip)*vxy(2);
    if P.dmode == 3
        % complementary rate: velocity-based rate + low-passed (difference of the estimated ey - velocity rate)
        al = P.T/(P.tauc + P.T);
        ls.c = ls.c + al*(((ey_hat - ls.eyp)/P.T - dey) - ls.c);
        dey = dey + ls.c;
    end
    uff = uff - g.Kd*dey;
    g.Kd = 0;
end
ls.eyp = ey_hat;
[ulat, ls.st] = pid_step(e, ls.st, g, uff);
delta = lat_inverse(ulat, v_hat, P.par);
end
