function g = pid_design(loop, mdl, spec)
%PID_DESIGN  Analytic pole-placement design of the discrete PID for the 'lon' or 'lat' loop.
%   g = pid_design('lon'|'lat', mdl, spec)
%   mdl  : struct with T, Gv, Gr (fields num, den in z, descending, equal length; dcgain), par
%          (from 01_sysid/model_id.mat)
%   spec : zeta, ts (settling time of the dominant pair, 2 %), kappa (1x2 or scalar: real extra
%          poles at -kappa*zeta*wn); lat: v0 [m/s]; optional umin, umax, aw (0/1/2)
%   Design plants (T = sampling time, forward-Euler integrators T/(z-1)):
%     lon: Gs(z) = Gv(z)*T/(z-1)                       -> order 2, closed loop order 4
%     lat: G_ey(z) = v0*T^2*Gr(z)/(z-1)^2 (FULL model) -> order 5, closed loop order 7
%   The 4 controller unknowns (pf, c2, c1, c0) place the dominant pair and two real poles
%   EXACTLY on the full model via the Diophantine equation (pid_diophantine); for lat the
%   other n-2 = 3 poles are the roots of the residual polynomial R(z) (reported, verified).
T = mdl.T;
zeta = spec.zeta; wn = 4/(zeta*spec.ts); kappa = spec.kappa;
dp = pid_dpoles(zeta, wn, kappa, T);
switch loop
    case 'lon'
        a = -mdl.Gv.den(2); b = mdl.Gv.num(2);
        full_num = b*T; full_den = conv([1 -a], [1 -1]);
        v0 = NaN;
    case 'lat'
        v0 = spec.v0;
        full_num = v0*T^2*mdl.Gr.num(:).'; full_num = full_num(find(full_num ~= 0, 1):end);
        full_den = conv(mdl.Gr.den(:).', conv([1 -1], [1 -1]));   % z (z^2+a1 z+a2)(z-1)^2
    otherwise
        error('loop must be lon or lat');
end
d = pid_diophantine(full_num, full_den, dp, T);
g.loop = loop; g.T = T;
g.Kp = d.Kp; g.Ki = d.Ki; g.Kd = d.Kd; g.pf = d.pf; g.N = d.N;
g.Nc = d.Nc; g.Nc1 = d.Nc1;
g.zeta = zeta; g.wn = wn; g.ts_dom = spec.ts; g.kappa = kappa;
g.desired_poles = dp; g.desired_poly = d.Ad;
g.R = d.R; g.R_poles = d.Rpoles; g.dioph_cond = d.cond; g.dioph_resid = d.resid;
g.full_num = full_num; g.full_den = full_den; g.v0 = v0;
if isfield(spec, 'umin'), g.umin = spec.umin; else, g.umin = -Inf; end
if isfield(spec, 'umax'), g.umax = spec.umax; else, g.umax = Inf; end
if isfield(spec, 'aw'),   g.aw = spec.aw;     else, g.aw = 1; end
% back-calculation tracking gain Kt = 1/Tt, Tt = sqrt(Ti*Td) (Astrom/Hagglund), Ti = Kp/Ki, Td = Kd/Kp
g.Ti = g.Kp/g.Ki; g.Td = g.Kd/g.Kp; g.Tt = sqrt(g.Ti*g.Td); g.Kt = 1/g.Tt;
end
