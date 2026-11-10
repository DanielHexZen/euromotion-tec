function [e, st, d2] = ekf_update_2d(e, p, nu, H, R, type)
%EKF_UPDATE_2D  Common update procedure for a 2-D measurement with diagonal R (REQ-EKF-05).
%
%   [e, st, d2] = ekf_update_2d(e, p, nu, H, R, type)
%     nu  2x1 innovation, H 2x5, R 2x1 (diagonal of R)
%
%   "2-D updates gate on the joint d2, then apply both rows sequentially."
%   C order of operations:
%     PHt = P*H' (5x2);  S = H*PHt + diag(R) (2x2, symmetrised)
%     d2 = nu' S^-1 nu with the explicit 2x2 inverse; d2 > ekf_gate2 -> ST_REJECT
%     row 1: scalar Joseph update with (nu1, H1, R1)          -> dx1
%     row 2: nu2 <- nu2 - H2*dx1  (linearised re-evaluation), scalar Joseph update
%     health once; n_acc/nis_sum once with the joint d2
%   With the re-evaluated nu2 the two sequential rows equal the joint 2-D update
%   exactly for a linear model and diagonal R (checked in verify_gate.m).
%#codegen
c   = class(e.x);
PHt = e.P*H';
S   = H*PHt + diag(R);
S12 = cast(0.5, c)*(S(1,2) + S(2,1));
det = S(1,1)*S(2,2) - S12*S12;
st  = 0;
if ~(det > 0)
    d2 = cast(Inf, c);
else
    d2 = (S(2,2)*nu(1)*nu(1) - cast(2, c)*S12*nu(1)*nu(2) + S(1,1)*nu(2)*nu(2))/det;
end
if ~(d2 <= p.ekf_gate2)
    st = 1;
    e.n_rej(type) = e.n_rej(type) + 1;
    e = rec_add2(e, type, st, d2, nu, S);
    return;
end
I5 = eye(5, c);
% row 1
H1  = H(1, :);
P1  = e.P*H1';
s1  = H1*P1 + R(1);
K1  = P1/s1;
dx1 = K1*nu(1);
e.x = e.x + dx1;
A   = I5 - K1*H1;
e.P = A*e.P*A' + (K1*R(1))*K1';
e.P = cast(0.5, c)*(e.P + e.P');
% row 2
H2  = H(2, :);
n2  = nu(2) - H2*dx1;
P2  = e.P*H2';
s2  = H2*P2 + R(2);
K2  = P2/s2;
dx2 = K2*n2;
e.x = e.x + dx2;
A   = I5 - K2*H2;
e.P = A*e.P*A' + (K2*R(2))*K2';
e.P = cast(0.5, c)*(e.P + e.P');
e.x(3) = ekf_wrap(e.x(3));
e.corr = e.corr + dx1(1:3) + dx2(1:3);
e = ekf_health(e, p);
e.n_acc(type)   = e.n_acc(type) + 1;
e.nis_sum(type) = e.nis_sum(type) + double(d2);
e = rec_add2(e, type, st, d2, nu, S);
end

function e = rec_add2(e, type, st, d2, nu, S)
if e.n_rec < size(e.rec, 1)
    e.n_rec = e.n_rec + 1;
    e.rec(e.n_rec, :) = [type, st, double(d2), double(nu(1)), double(S(1,1)), double(nu(2)), double(S(2,2))];
end
end
