function [theta, Phi, Y, info] = arx_ls(y, u, na, nb, nk, rows)
%ARX_LS  Least-squares identification of an ARX model, explicit regression.
%   Model:  y(k) = -a1 y(k-1) - ... - a_na y(k-na) + b1 u(k-nk) + ... + b_nb u(k-nk-nb+1)
%   Regressor row:  phi(k) = [-y(k-1) ... -y(k-na)  u(k-nk) ... u(k-nk-nb+1)]
%   Stacked:        Y = Phi*theta + e,   theta = [a1..a_na  b1..b_nb]'
%   Solution:       theta = Phi\Y  (QR, numerically preferred) which is
%                   equivalent to the normal equations (Phi'Phi) theta = Phi'Y.
%   rows : optional vector of sample indices k to use (default: all k for which
%          every regressor exists). Only rows whose regressors all lie inside
%          the data are kept.
%   info : struct with cond(Phi'Phi), normal-equation solution, residual variance,
%          parameter covariance (std. errors), and the model polynomials
%          a = [1 a1..], b = [zeros(1,nk) b1..] (in z^-1).
y = y(:); u = u(:);
k0 = max(na, nk+nb-1) + 1;           % first sample with full regressor
if nargin < 6 || isempty(rows), rows = (k0:numel(y)).'; end
rows = rows(:); rows = rows(rows >= k0);
N = numel(rows);
Phi = zeros(N, na+nb);
for i = 1:na, Phi(:,i)    = -y(rows - i); end
for j = 1:nb, Phi(:,na+j) =  u(rows - nk - j + 1); end
Y = y(rows);
theta = Phi\Y;
res = Y - Phi*theta;
info.N = N;
info.rows = rows;
info.k0 = k0;
info.theta_ne = (Phi.'*Phi)\(Phi.'*Y);          % normal equations
info.cond_PtP = cond(Phi.'*Phi);
info.sigma2 = sum(res.^2)/max(N-numel(theta),1);
info.cov = info.sigma2*inv(Phi.'*Phi);
info.stderr = sqrt(diag(info.cov));
info.a = [1; theta(1:na)].';
info.b = [zeros(1,nk), theta(na+1:na+nb).'];
info.na = na; info.nb = nb; info.nk = nk;
% Akaike final prediction error / AIC on one-step residuals
info.fpe = info.sigma2*(1+numel(theta)/N)/(1-numel(theta)/N);
info.aic = N*log(sum(res.^2)/N) + 2*numel(theta);
end
