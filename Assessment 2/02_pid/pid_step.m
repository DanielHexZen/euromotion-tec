function [u, st] = pid_step(e, st, g, uff)
%PID_STEP  One sample of the discrete PID with saturation and anti-windup.
%   Form (parallel, backward-Euler integrator, filtered backward-difference derivative):
%     ud(k) = pf*ud(k-1) + Kd*(1-pf)/T * (e(k) - e(k-1))        pf = 1/(1+N*T)
%     I(k)  = I(k-1) + Ki*T*e(k)
%     u_un  = uff + Kp*e(k) + I(k) + ud(k)
%     u     = sat(u_un, umin, umax)
%   Anti-windup (g.aw):  0 none | 1 back-calculation  I <- I + T*Kt*(u - u_un)
%                        | 2 conditional integration (clamping: integrate only if not driving
%                          further into saturation)
%   Inputs : e (scalar), st (struct from pid_init), g (struct: Kp,Ki,Kd,pf,T,umin,umax,aw,Kt),
%            uff (optional feedforward added before the saturation, default 0)
%   Outputs: u saturated command, st updated state (st.sat = 1 when the actuator saturates, st.uun = unsaturated command).
%#codegen
if nargin < 4, uff = 0; end
ud   = g.pf*st.ud + g.Kd*(1 - g.pf)/g.T*(e - st.eprev);
Inew = st.I + g.Ki*g.T*e;
uun  = uff + g.Kp*e + Inew + ud;
u    = min(max(uun, g.umin), g.umax);
st.ud = ud; st.eprev = e; st.uun = uun;
if u ~= uun, st.sat = 1; else, st.sat = 0; end
if g.aw == 1
    st.I = Inew + g.T*g.Kt*(u - uun);
elseif g.aw == 2
    if (uun > g.umax && e > 0) || (uun < g.umin && e < 0)
        % keep the old integrator value (clamp)
    else
        st.I = Inew;
    end
else
    st.I = Inew;
end
end
