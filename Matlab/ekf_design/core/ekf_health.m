function e = ekf_health(e, p)
%EKF_HEALTH  Covariance health check (REQ-EKF-05 step 5). Failure -> F_EKF in C.
%   All elements finite, diagonal > 0, P_xx, P_yy < ekf_pxy_max (25 m^2),
%   P_tt < ekf_pth_max (1 rad^2).
%#codegen
dg = diag(e.P);
if p.mx_freeze_s > 0
    dg(5) = 1;                     % frozen scale state has P_ss = 0 by construction
end
ok = all(isfinite(e.P(:))) && all(isfinite(e.x)) && all(dg > 0) ...
     && e.P(1,1) < p.ekf_pxy_max && e.P(2,2) < p.ekf_pxy_max && e.P(3,3) < p.ekf_pth_max;
if ~ok
    e.fault   = 1;
    e.n_fault = e.n_fault + 1;
end
end
