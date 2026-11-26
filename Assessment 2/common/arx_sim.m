function [yhat, yone] = arx_sim(a, b, u, y, rows)
%ARX_SIM  Free-run simulation and one-step-ahead prediction of an ARX model.
%   a = [1 a1 .. a_na], b = [0 .. 0 b1 .. b_nb] (polynomials in z^-1, b already
%   contains the nk leading zeros, b(1) multiplies u(k)).
%   rows : contiguous index range [k1 ... k2] of the segment to simulate.
%   yhat : free-run simulation. The first n0 = max(numel(a)-1, numel(b)-1)
%          outputs of the segment are initialised with the measured values
%          (initial condition), afterwards ONLY the input u and past simulated
%          outputs are used: yhat(k) = -a(2:end)*yhat(k-1..) + b*u(k..).
%   yone : one-step-ahead prediction using measured past outputs.
%   Both are returned on the same index vector rows (first n0 samples = measured).
rows = rows(:);
na = numel(a)-1; nb1 = numel(b);
n0 = max(na, nb1-1);
yhat = y(rows); yone = y(rows);
for i = n0+1:numel(rows)
    k = rows(i);
    accS = 0; accM = 0;
    for j = 1:na
        accS = accS - a(j+1)*yhat(i-j);
        accM = accM - a(j+1)*y(k-j);
    end
    ub = 0;
    for j = 1:nb1, ub = ub + b(j)*u(k-j+1); end
    yhat(i) = accS + ub;
    yone(i) = accM + ub;
end
end
