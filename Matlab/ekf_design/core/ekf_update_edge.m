function [e, st] = ekf_update_edge(e, p, ln, sx, sy, bk)
%EKF_UPDATE_EDGE  ekf_update_edge(e, ln, sx, sy, bk) of spec §7.17.1/§7.17.3.
%   z = 0 (footprint on the boundary), R = ekf_sig_edge^2.
%#codegen
[h, H] = ekf_meas_edge(e.x, ln, [sx; sy], bk);
R  = p.ekf_sig_edge*p.ekf_sig_edge;
nu = -h;
[e, st] = ekf_update_scalar(e, p, nu, H, R, 2);
end
