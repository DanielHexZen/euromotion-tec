function S = pid_sim_lat(mdl, g, v, ey_ref, kappa, opt)
%PID_SIM_LAT  Non-linear lateral path-following simulation (Frenet error dynamics).
%   Plant: identified Gr (u_lat -> r, difference equation incl. 3-sample delay) with the
%   static front end lat_input/lat_inverse and steering limit |delta| <= par.delta_max,
%   heading error  psi_e(k+1) = psi_e(k) + T*(r(k) - v*kappa(k))        (forward Euler)
%   cross-track    e_y(k+1)   = e_y(k) + T*v*sin(psi_e(k))              (non-linear)
%   v      : speed [m/s], scalar or sequence;  ey_ref : cross-track reference sequence [m]
%   kappa  : path curvature sequence [1/m] (0 = straight)
%   opt    : prefilt (0/1), pre0 (initial prefilter output / assumed previous reference, default ey_ref(1)), ey0, psi0, sched (0/1: gains scaled by v0/v), ff (0/1 curvature
%            feedforward u_ff = v*kappa/k_delta), gfac (plant gain factor), dly (extra delay samples)
%   Controller: e = ey_ref - e_y ; u_lat = sat(u_ff + PID(e), +-u_lim(v)) ; delta = lat_inverse(u_lat, v).
if nargin < 6, opt = struct(); end
if ~isfield(opt, 'prefilt'), opt.prefilt = 0; end
if ~isfield(opt, 'ey0'), opt.ey0 = 0; end
if ~isfield(opt, 'psi0'), opt.psi0 = 0; end
if ~isfield(opt, 'sched'), opt.sched = 1; end
if ~isfield(opt, 'ff'), opt.ff = 0; end
if ~isfield(opt, 'gfac'), opt.gfac = 1; end
if ~isfield(opt, 'dly'), opt.dly = 0; end
if ~isfield(opt, 'nolimit'), opt.nolimit = 0; end
if ~isfield(opt, 'pre0'), opt.pre0 = ey_ref(1); end
T = mdl.T; par = mdl.par; N = numel(ey_ref);
if isscalar(v), v = v*ones(N,1); end
if isscalar(kappa), kappa = kappa*ones(N,1); end
a1 = mdl.Gr.den(2); a2 = mdl.Gr.den(3); b = mdl.Gr.num(end)*opt.gfac; nk = 3 + opt.dly;
ey = zeros(N,1); psi = zeros(N,1); r = zeros(N,1); ul = zeros(N,1); dl = zeros(N,1);
uc = zeros(N,1); sat = zeros(N,1); un = zeros(N,1); rf = zeros(N,1);
ey(1) = opt.ey0; psi(1) = opt.psi0;
st = pid_init(opt.pre0 - ey(1)); pst = pid_prefilt_init(opt.pre0);
for k = 1:N
    % yaw-rate response of the identified model
    r1 = 0; r2 = 0; uu = 0;
    if k > 1, r1 = r(k-1); end
    if k > 2, r2 = r(k-2); end
    if k - nk >= 1, uu = ul(k-nk); end
    r(k) = -a1*r1 - a2*r2 + b*uu;
    % controller
    if opt.prefilt, [rf(k), pst] = pid_prefilt(ey_ref(k), pst, g); else, rf(k) = ey_ref(k); end
    e = rf(k) - ey(k);
    gk = g;
    if opt.sched, gk = pid_sched(g, v(k), 1); end
    if ~opt.nolimit
        lim = pid_ulat_limit(v(k), par); gk.umax = lim; gk.umin = -lim;
    end
    uff = 0; if opt.ff, uff = v(k)*kappa(k)/mdl.Gr.dcgain; end
    [uc(k), st] = pid_step(e, st, gk, uff);
    un(k) = st.uun; sat(k) = st.sat;
    dl(k) = lat_inverse(uc(k), v(k), par);
    ul(k) = lat_input(v(k), dl(k), par);           % what the vehicle actually receives
    if k < N
        psi(k+1) = psi(k) + T*(r(k) - v(k)*kappa(k));
        ey(k+1)  = ey(k) + T*v(k)*sin(psi(k));
    end
end
S.t = (0:N-1)'*T; S.ey = ey; S.psi = psi; S.r = r; S.u_cmd = uc; S.u_act = ul; S.uun = un;
S.delta = dl; S.sat = sat; S.rf = rf;
end
