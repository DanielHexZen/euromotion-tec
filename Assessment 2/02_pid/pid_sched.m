function gv = pid_sched(g, v, vmin)
%PID_SCHED  Speed scheduling of the lateral PID: gains scale with v0/v.
%   The path-following plant gain is proportional to v (e_y' = v*psi_e), so the loop gain
%   C(z)*G_ey(z) -- and hence the closed-loop poles of the reduced model -- stay unchanged if
%   Kp, Ki, Kd are multiplied by v0/v. v is limited below by vmin (default 1 m/s).
%#codegen
if nargin < 3, vmin = 1; end
s = g.v0/max(v, vmin);
gv = g; gv.Kp = g.Kp*s; gv.Ki = g.Ki*s; gv.Kd = g.Kd*s;
gv.Kt = g.Kt;
end
