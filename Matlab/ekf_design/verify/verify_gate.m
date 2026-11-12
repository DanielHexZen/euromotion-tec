function R = verify_gate(cls)
%VERIFY_GATE  t_ekf_gate (§14.1, REQ-EKF-05) plus the chi-square helper and the
%   sequential 2-D update.
%   - an edge measurement whose innovation is 10 sqrt(S) is rejected, 2 sqrt(S) accepted
%   - chi2_quantile against tabulated values (A&S Table 26.8)
%   - the gate values: P(chi2_1 <= 9.0), P(chi2_2 <= 9.21)
%   - ekf_update_2d (joint gate, two sequential rows) equals the batch 2-D
%     Kalman update to rounding
if nargin < 1, cls = 'single'; end
p = ekf_param_cast(ekf_params(), cls);
c = class(p.ekf_kd);
R = [];
% ----- 10 sigma / 2 sigma edge outlier
e0 = ekf_init(p, 5, 0.1, 0.01);
e0.P = e0.P + diag(cast([1e-4 4e-4 1e-5 0 0], c));
ln = cast([0; 1; -0.2], c);             % right boundary y = -0.2, normal +y into the path
rsb = cast([0.15; -0.25], c);
bk = zeros(3, 1, c);
[h, H] = ekf_meas_edge(e0.x, ln, rsb, bk);
S = H*e0.P*H' + p.ekf_sig_edge^2;
for ks = [10 2]
    % shift the line so that the innovation becomes ks*sqrt(S)
    l2 = ln;  l2(3) = ln(3) + h + ks*sqrt(S);    % nu = -(h - ks sqrt S - h) ... = ks sqrt S
    [e1, st] = ekf_update_edge(e0, p, l2, rsb(1), rsb(2), bk);
    d2 = e1.rec(1, 3);
    want = double(ks == 2);
    R = [R, vres(sprintf('t_ekf_gate %d sigma edge %s', ks, ternary(want, 'accepted', 'rejected')), ...
        double((st == 0) == want), sprintf('d2 = %.2f, st = %d', d2, st), ...
        sprintf('gate1 = %.2f', double(p.ekf_gate1)))]; %#ok<AGROW>
end
% ----- chi-square helper
tab = [1 0.975 5.0239; 2 0.99 9.2103; 5 0.025 0.8312; 5 0.975 12.8325; 3 0.975 9.3484; 1 0.025 0.000982];
err = 0;
for i = 1:size(tab, 1)
    q = chi2_quantile(tab(i, 2), tab(i, 1));
    err = max(err, abs(q - tab(i, 3))/tab(i, 3));
end
R = [R, vres('chi2_quantile vs tables', double(err < 1e-3), sprintf('%.1e rel', err), '< 1e-3')];
fprintf('verify_gate: P(chi2_1 <= %.2f) = %.5f, P(chi2_2 <= %.2f) = %.5f\n', ...
    double(p.ekf_gate1), chi2_cdf(double(p.ekf_gate1), 1), double(p.ekf_gate2), chi2_cdf(double(p.ekf_gate2), 2));
R = [R, vres('gate1/gate2 confidence', -1, sprintf('%.4f / %.4f', chi2_cdf(double(p.ekf_gate1), 1), ...
    chi2_cdf(double(p.ekf_gate2), 2)), 'spec: 99.7 % / 99 %')];
% ----- sequential 2-D = batch
pd = ekf_params();
e  = ekf_init(pd, 9.8, 7.0, pi/2 + 0.01);
e.P = e.P + 1e-4*[2 0.5 0.1 0 0.3; 0.5 3 0.2 0 0.1; 0.1 0.2 1 0 0; 0 0 0 0 0; 0.3 0.1 0 0 0.5];
q = [0; -0.6];
[hh, Hp] = ekf_meas_point(e.x, q, [0; 0; 0]);
z = hh + [0.02; -0.03];
Rm = (pd.ekf_sig_post^2 + 0.03^2)*eye(2);
S  = Hp*e.P*Hp' + Rm;  K = e.P*Hp'/S;
xb = e.x + K*(z - hh);
Pb = (eye(5) - K*Hp)*e.P*(eye(5) - K*Hp)' + K*Rm*K';
[e2, st] = ekf_update_point(e, pd, z(1), z(2), q(1), q(2), 0.03^2, [0; 0; 0]);
dx = max(abs(e2.x - xb));  dP = max(abs(e2.P(:) - Pb(:)));
R = [R, vres('2-D sequential = batch update', double(st == 0 && dx < 1e-12 && dP < 1e-14), ...
    sprintf('dx %.1e, dP %.1e', dx, dP), 'rounding only')];
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
