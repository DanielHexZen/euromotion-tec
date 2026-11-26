function S = pid_sim_lon(mdl, g, sref, vff, opt)
%PID_SIM_LON  Non-linear/saturated longitudinal simulation of the position loop.
%   Plant (identified Gv, difference equation): v(k+1) = a*v(k) + b*vcmd(k),
%   position s(k+1) = s(k) + T*v(k) (forward Euler), actuator limits g.umin..g.umax on vcmd.
%   sref : reference position sequence [m] (column);  vff : velocity feedforward sequence [m/s]
%   opt  : fields  prefilt (0/1), v0 (initial speed), s0, tau_fac (lag time-constant factor), gfac
%   Controller: e = sref - s ; vcmd = sat( vff + PID(e) ) with pid_step (anti-windup per g.aw).
if nargin < 5, opt = struct(); end
if ~isfield(opt, 'prefilt'), opt.prefilt = 0; end
if ~isfield(opt, 'v0'), opt.v0 = 0; end
if ~isfield(opt, 's0'), opt.s0 = sref(1); end
if ~isfield(opt, 'tau_fac'), opt.tau_fac = 1; end
if ~isfield(opt, 'gfac'), opt.gfac = 1; end
T = mdl.T; N = numel(sref);
a = -mdl.Gv.den(2); b = mdl.Gv.num(2);
if opt.tau_fac ~= 1                       % plant-pole variation, DC gain kept
    dc = b/(1 - a); a = exp(log(a)/opt.tau_fac); b = dc*(1 - a);
end
b = b*opt.gfac;
s = zeros(N,1); v = zeros(N,1); vc = zeros(N,1); un = zeros(N,1); sat = zeros(N,1); rf = zeros(N,1);
s(1) = opt.s0; v(1) = opt.v0;
st = pid_init(sref(1) - s(1)); pst = pid_prefilt_init(sref(1));
for k = 1:N
    if opt.prefilt, [rf(k), pst] = pid_prefilt(sref(k), pst, g); else, rf(k) = sref(k); end
    e = rf(k) - s(k);
    [vc(k), st] = pid_step(e, st, g, vff(k));
    un(k) = st.uun; sat(k) = st.sat;
    if k < N
        v(k+1) = a*v(k) + b*vc(k);
        s(k+1) = s(k) + T*v(k);
    end
end
S.t = (0:N-1)'*T; S.s = s; S.v = v; S.vcmd = vc; S.uun = un; S.sat = sat; S.rf = rf;
end
