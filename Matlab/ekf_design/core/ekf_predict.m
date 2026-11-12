function e = ekf_predict(e, p, ds_l, ds_r, dpsi, dt, t_us, sat)
%EKF_PREDICT  ekf_predict(e, &odo) of spec §7.17.1: dispatch, then pose history.
%
%   e = ekf_predict(e, p, ds_l, ds_r, dpsi, dt, t_us, sat)
%   dpsi = NaN selects the encoder-heading mode (FEAT_IMU = 0, §7.17.7).
%   e.still must have been updated for this step (ekf_still_update).
%#codegen
if isnan(dpsi)
    e = ekf_predict_enc(e, p, ds_l, ds_r, dt);
else
    e = ekf_predict_imu(e, p, ds_l, ds_r, dpsi, dt, sat);
end
e.t_us = t_us;
e = ekf_hist_push(e, p, t_us);
end
