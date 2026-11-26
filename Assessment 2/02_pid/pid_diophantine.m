function d = pid_diophantine(num, den, dp, T)
%PID_DIOPHANTINE  Exact placement of 4 closed-loop poles of a filtered-derivative PID on a
%   plant G(z) = num(z)/den(z) of ARBITRARY order n (den monic, num strictly proper).
%   PID:  C(z) = N_c(z) / ((z-1)(z-pf)),  N_c = c2 z^2 + c1 z + c0   (see pid_coeffmatch).
%   Closed-loop characteristic polynomial (degree n+2):
%       Delta(z) = (z-1)(z-pf) den(z) + N_c(z) num(z).
%   Diophantine equation with the desired 4-pole polynomial A_d(z) = prod(z - dp) and an
%   unknown residual polynomial R(z) (monic, degree n-2: the n-2 closed-loop poles that are
%   NOT assigned):
%       Delta(z) = A_d(z) * R(z).
%   Delta is affine in the controller unknowns (pf, c2, c1, c0) and R is affine in its own
%   coefficients, so matching the n+2 non-leading coefficients is a SQUARE LINEAR system
%   for  x = [pf; c2; c1; c0; r_{n-3}; ...; r_0]  (n+2 equations, n+2 unknowns).
%   For n = 2 (R = 1) this reduces to pid_coeffmatch. Then Kp, Ki, Kd follow in closed form.
den = den(:).'; num = num(:).'; n = numel(den) - 1;
Ad = real(poly(dp(:)));                       % degree 4
Lp = n + 3;                                   % length of degree n+2 coefficient vectors
pad = @(v) [zeros(1, Lp - numel(v)) v];       % right-align (descending powers)
base = pad(conv([1 -1 0], den));              % z(z-1)*den    (pf-independent part)
Bpf  = pad(conv([-1 1], den));                % coefficient of pf:  (1 - z)*den
Bc2  = pad(conv([1 0 0], num));
Bc1  = pad(conv([1 0],   num));
Bc0  = pad(num);
nr = n - 2;                                   % number of unknown R coefficients (monic R)
cols = [Bpf.' Bc2.' Bc1.' Bc0.'];
rhs  = -base.' + pad(conv(Ad, [1 zeros(1, nr)])).';   % A_d*z^(n-2) moves to the right
for j = 0:nr-1                                % R = z^(n-2) + sum_j r_j z^j
    cols = [cols, -pad(conv(Ad, [1 zeros(1, j)])).'];   %#ok<AGROW>
end
A = cols(2:end, :); b = rhs(2:end);           % drop the (identically cancelling) leading row
x = A \ b;
d.cond = cond(A); d.resid = norm(A*x - b);
pf = x(1); c2 = x(2); c1 = x(3); c0 = x(4);
d.R = [1 fliplr(x(5:end).')];                % residual polynomial (descending)
if nr > 0, d.Rpoles = roots(d.R); else, d.Rpoles = zeros(0,1); end
Kp  = (-c1 - pf*c2 - (2-pf)*c0)/(1-pf)^2;
Kdf = c0 - pf*Kp;
KiT = c2 - c0 - (1-pf)*Kp;
d.Kp = Kp; d.Ki = KiT/T; d.Kd = Kdf*T/(1-pf); d.pf = pf; d.N = (1/pf - 1)/T;
d.Kdf = Kdf; d.Nc = [c2 c1 c0]; d.Nc1 = c2 + c1 + c0; d.Ad = Ad;
end
