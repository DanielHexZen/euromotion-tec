function [Delta, Dc] = pid_clpoly(g, num, den)
%PID_CLPOLY  Closed-loop characteristic polynomial Delta(z) = Dc(z)*den(z) + Nc(z)*num(z),
%   Dc = (z-1)(z-pf), Nc = g.Nc, for the plant num/den (default: g.full_num/g.full_den).
if nargin < 2, num = g.full_num; den = g.full_den; end
Dc = conv([1 -1], [1 -g.pf]);
A = conv(Dc, den); B = conv(g.Nc, num);
Delta = A; Delta(end-numel(B)+1:end) = Delta(end-numel(B)+1:end) + B;
end
