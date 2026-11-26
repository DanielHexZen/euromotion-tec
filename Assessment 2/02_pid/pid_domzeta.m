function [zeta, wn, sdom] = pid_domzeta(zp, T)
%PID_DOMZETA  Damping ratio / natural frequency of the dominant (slowest) discrete poles zp.
%   s = ln(z)/T ; the dominant pole has the smallest |Re(s)|. Returns the damping ratio
%   zeta = -Re(s)/|s| and wn = |s| of that pole (real pole: zeta = 1).
s = log(zp(:))/T;
[~, i] = min(abs(real(s)));
sdom = s(i);
wn = abs(sdom); zeta = -real(sdom)/wn;
end
