function d = pid_coeffmatch(p1, p0, n0, dp, T)
%PID_COEFFMATCH  Analytic pole placement of the filtered-derivative parallel PID
%   on a plant  G(z) = n0 / (z^2 + p1 z + p0)  (2nd order, constant numerator).
%
%   Controller (parallel, backward-Euler integrator, filtered backward-difference derivative):
%     C(z) = Kp + Ki*T*z/(z-1) + Kdf*(z-1)/(z-pf),   Kdf = Kd*(1-pf)/T,  pf = 1/(1+N*T)
%          = N_c(z) / ((z-1)(z-pf)),  N_c = c2 z^2 + c1 z + c0
%   Closed-loop characteristic polynomial (degree 4):
%     Delta(z) = (z-1)(z-pf)(z^2+p1 z+p0) + n0*N_c(z) = z^4 + d3 z^3 + d2 z^2 + d1 z + d0
%   Coefficient matching with the desired polynomial  z^4 + e3 z^3 + e2 z^2 + e1 z + e0 = prod(z - dp):
%     d3 = p1 - 1 - pf             -> pf = p1 - 1 - e3       (filter pole fixed by the sum of poles)
%     c2 = (e2 - p0 + (1+pf)p1 - pf)/n0
%     c1 = (e1 - pf p1 + (1+pf)p0)/n0
%     c0 = (e0 - pf p0)/n0
%   and the linear 3x3 system   [1 1 1; 1+pf pf 2; pf 0 1]*[Kp; Ki*T; Kdf] = [c2; -c1; c0]
%   with closed-form solution (det = -(1-pf)^2):
%     Kp  = (-c1 - pf*c2 - (2-pf)*c0)/(1-pf)^2
%     Kdf = c0 - pf*Kp ,   Ki*T = c2 - c0 - (1-pf)*Kp ,   Kd = Kdf*T/(1-pf).
ep = real(poly(dp(:)));                 % [1 e3 e2 e1 e0]
e3 = ep(2); e2 = ep(3); e1 = ep(4); e0 = ep(5);
pf = p1 - 1 - e3;
c2 = (e2 - p0 + (1+pf)*p1 - pf)/n0;
c1 = (e1 - pf*p1 + (1+pf)*p0)/n0;
c0 = (e0 - pf*p0)/n0;
Kp  = (-c1 - pf*c2 - (2-pf)*c0)/(1-pf)^2;
Kdf = c0 - pf*Kp;
KiT = c2 - c0 - (1-pf)*Kp;
% cross-check with the generic linear solve
M = [1 1 1; 1+pf pf 2; pf 0 1];
xs = M \ [c2; -c1; c0];
d.solve_check = max(abs(xs - [Kp; KiT; Kdf]));
d.Kp = Kp; d.Ki = KiT/T; d.Kd = Kdf*T/(1-pf);
d.pf = pf; d.N = (1/pf - 1)/T; d.Kdf = Kdf;
d.Nc = [c2 c1 c0]; d.Nc1 = c2 + c1 + c0;
d.coef_desired = ep; d.n0 = n0; d.p1 = p1; d.p0 = p0;
end
