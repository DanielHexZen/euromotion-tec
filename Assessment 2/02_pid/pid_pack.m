function p = pid_pack(g, Vl, chk, form, nm)
%PID_PACK  Collect the interface struct (SPEC) for pid_gains.mat of one loop.
%   The struct contains every field pid_step needs (Kp, Ki, Kd, pf, T, umin, umax, aw, Kt),
%   so  [u, st] = pid_step(e, st, pid_lon)  works directly on the saved struct.
p = struct();
p.loop = nm; p.T = g.T; p.form = form;
p.Kp = g.Kp; p.Ki = g.Ki; p.Kd = g.Kd; p.N = g.N; p.pf = g.pf; p.Kdf = g.Kd*(1 - g.pf)/g.T;
p.Nc = g.Nc; p.Nc1 = g.Nc1; p.Ti = g.Ti; p.Td = g.Td; p.Tt = g.Tt; p.Kt = g.Kt;
p.umin = g.umin; p.umax = g.umax; p.aw = g.aw;
p.sat_limits = [g.umin g.umax];
p.antiwindup = 'back-calculation: I(k) = I(k-1) + Ki*T*e(k) + T*Kt*(u_sat - u_unsat), Kt = 1/sqrt(Ti*Td) (g.aw = 1); g.aw = 2: conditional integration (clamping); g.aw = 0: none';
p.zeta_design = g.zeta; p.wn_design = g.wn; p.ts_dom_design = g.ts_dom; p.kappa = g.kappa;
p.v0 = g.v0;
p.desired_poles = g.desired_poles; p.desired_poles_s = log(g.desired_poles)/g.T;
p.achieved_poles = Vl.poles; p.residual_poles = g.R_poles;
p.zeta_dominant_achieved = Vl.zeta_dom; p.wn_dominant_achieved = Vl.wn_dom;
p.pid_zeros = Vl.zeros;
p.prefilter = 'F(z) = Nc(1) z^2 / Nc(z) on the reference (pid_prefilt); cancels the two PID zeros, T_r = Nc(1) num z^2 / Delta';
p.plant_num = g.full_num; p.plant_den = g.full_den;
p.step_metrics_1dof = struct('overshoot_pct', Vl.m1.overshoot, 'rise_10_90_s', Vl.m1.rise, 'ts_2pct_s', Vl.m1.ts);
p.step_metrics = struct('overshoot_pct', Vl.m2.overshoot, 'rise_10_90_s', Vl.m2.rise, 'ts_2pct_s', Vl.m2.ts, ...
    'zeta_dominant', chk.zeta, 'note', 'with reference prefilter (2-DOF); linear full-order model, unit step');
end
