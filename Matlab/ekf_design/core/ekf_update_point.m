function [e, st] = ekf_update_point(e, p, lx, ly, qx, qy, var, bk)
%EKF_UPDATE_POINT  Post landmark (spec §7.17.3): 2-D, R = (ekf_sig_post^2 + var)*I2,
%   var = survey sigma^2 of the landmark. Joint gate on chi2_2 (ekf_gate2).
%#codegen
[h, H] = ekf_meas_point(e.x, [qx; qy], bk);
r  = p.ekf_sig_post*p.ekf_sig_post + var;
nu = [lx; ly] - h;
[e, st] = ekf_update_2d(e, p, nu, H, [r; r], 5);
end
