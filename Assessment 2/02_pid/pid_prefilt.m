function [rf, pst] = pid_prefilt(r, pst, g)
%PID_PREFILT  Reference prefilter F(z) = Nc(1)*z^2/Nc(z) cancelling the two PID zeros.
%   Nc(z) = c2 z^2 + c1 z + c0 is the PID numerator (g.Nc). With F the closed-loop reference
%   transfer function becomes T_r(z) = n0*Nc(1)*z^2/Delta(z): all-pole, unit DC gain.
%   Difference equation:  c2*rf(k) = Nc(1)*r(k) - c1*rf(k-1) - c0*rf(k-2)
%   pst: struct with fields r1, r2 (past outputs), init with pid_prefilt_init(r0).
%#codegen
rf = (g.Nc1*r - g.Nc(2)*pst.r1 - g.Nc(3)*pst.r2)/g.Nc(1);
pst.r2 = pst.r1; pst.r1 = rf;
end
