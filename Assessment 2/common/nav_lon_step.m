function [vcmd, ls, e] = nav_lon_step(ls, s_hat, s_ref, v_ref, v_ref_next, active, P) %#codegen
%NAV_LON_STEP  Longitudinal PID on the along-track error e_s = s_ref - s_hat with velocity
%   feedforward. ffmode 1: uff = v_ref; ffmode 2: plant inverse of Gv, uff = (v_ref(k+1) - av v_ref(k))/bv
%   (DC gain ~1, adds tau*a_ref during accelerations). Saturation 0..3 m/s + back-calculation in pid_step.
%   Idle (command 0, PID state reset) while active == 0 (standstill calibration phase).
e = s_ref - s_hat;
if active == 0
    vcmd = 0; ls.st = pid_init(e);
    return;
end
if P.ffmode == 2
    uff = (v_ref_next - P.av*v_ref)/P.bv;
else
    uff = v_ref;
end
[vcmd, ls.st] = pid_step(e, ls.st, P.lon, uff);
end
