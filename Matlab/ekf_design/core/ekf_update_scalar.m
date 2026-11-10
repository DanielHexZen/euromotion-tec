function [e, st, d2] = ekf_update_scalar(e, p, nu, H, R, type)
%EKF_UPDATE_SCALAR  Common update procedure for a scalar measurement (REQ-EKF-05).
%
%   [e, st, d2] = ekf_update_scalar(e, p, nu, H, R, type)
%     nu    innovation z - h(x) (angles already wrapped)
%     H     1x5 measurement Jacobian
%     R     measurement variance
%     type  1 ZUPT, 2 EDGE, 3 FACE, 4 HEADING, 5 POINT (counter index)
%   st = 0 ST_OK, 1 ST_REJECT.
%
%   C order of operations:
%     PHt = P*H';  S = H*PHt + R;  d2 = nu*nu/S
%     gate: d2 > ekf_gate1 (or not finite) -> n_rej[type]++, ST_REJECT
%     K = PHt/S;  x += K*nu;  theta = wrap(theta)
%     A = I - K*H;  P = A*P*A' + (K*R)*K';  P = (P + P')/2      (Joseph form)
%     health; n_acc[type]++; nis_sum[type] += d2
%#codegen
c   = class(e.x);
PHt = e.P*H';
S   = H*PHt + R;
d2  = nu*nu/S;
st  = 0;
if ~(d2 <= p.ekf_gate1)            % also rejects NaN
    st = 1;
    e.n_rej(type) = e.n_rej(type) + 1;
    e = rec_add(e, type, st, d2, nu, S);
    return;
end
K   = PHt/S;
dx  = K*nu;
e.x = e.x + dx;
e.x(3) = ekf_wrap(e.x(3));
e.corr = e.corr + dx(1:3);
A   = eye(5, c) - K*H;
e.P = A*e.P*A' + (K*R)*K';
e.P = cast(0.5, c)*(e.P + e.P');
e = ekf_health(e, p);
e.n_acc(type)   = e.n_acc(type) + 1;
e.nis_sum(type) = e.nis_sum(type) + double(d2);
e = rec_add(e, type, st, d2, nu, S);
end

function e = rec_add(e, type, st, d2, nu, S)
% MATLAB-only logging of this call's updates (not part of the C module).
if e.n_rec < size(e.rec, 1)
    e.n_rec = e.n_rec + 1;
    e.rec(e.n_rec, :) = [type, st, double(d2), double(nu), double(S), 0, 0];
end
end
