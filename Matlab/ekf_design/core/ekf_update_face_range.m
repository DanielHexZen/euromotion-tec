function [e, st] = ekf_update_face_range(e, p, ln, sx, sy, ux, uy, r_m, var, bk)
%EKF_UPDATE_FACE_RANGE  Range to a mapped face (spec §7.17.3, REQ-EKF-06).
%   st = 0 OK, 1 REJECT (gate), 2 RANGE (|g| < ekf_face_g_min: ray too oblique,
%   no update, not counted as a rejection).
%#codegen
[h, H, g] = ekf_meas_face(e.x, ln, [sx; sy], [ux; uy], bk);
if abs(g) < p.ekf_face_g_min
    st = 2;
    return;
end
nu = r_m - h;
[e, st] = ekf_update_scalar(e, p, nu, H, var, 3);
end
