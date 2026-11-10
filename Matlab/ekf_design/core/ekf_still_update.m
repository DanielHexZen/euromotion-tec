function [e, zupt_due, rate_mean, T] = ekf_still_update(e, p, cnt_changed, drv_idle, dpsi, dt)
%EKF_STILL_UPDATE  Standstill logic and ZUPT accumulator (REQ-EKF-09, §7.17.4).
%
%   [e, zupt_due, rate_mean, T] = ekf_still_update(e, p, cnt_changed, drv_idle, dpsi, dt)
%     cnt_changed  1 if either wheel count changed over this step
%     drv_idle     1 if drv is in HOLD, DISABLED or IDLE (no motion commanded)
%     dpsi, dt     yaw-integral difference and step length of this step
%   Call once per estimator step BEFORE ekf_predict (the prediction uses e.still).
%   If zupt_due, call ekf_update_zupt(e, p, rate_mean, T) after the prediction.
%
%   still = no count change for >= ekf_still_s AND drv idle.
%   While still: z_sum += dpsi, z_T += dt; every ekf_zupt_T one ZUPT with the
%   mean dpsi/dt, then restart. Leaving still applies a partial window of at
%   least ekf_zupt_min_T (0.5 s) and discards a shorter one.
%#codegen
c = class(e.x);
zupt_due  = 0;
rate_mean = cast(0, c);
T         = cast(0, c);
if cnt_changed > 0
    e.t_nochg = cast(0, c);
else
    e.t_nochg = e.t_nochg + dt;
end
was = e.still;
e.still = double(e.t_nochg >= p.ekf_still_s && drv_idle > 0);
if e.still > 0
    if ~isnan(dpsi)
        e.z_sum = e.z_sum + dpsi;
        e.z_T   = e.z_T + dt;
        if e.z_T >= p.ekf_zupt_T
            zupt_due  = 1;
            rate_mean = e.z_sum/e.z_T;
            T         = e.z_T;
            e.z_sum   = cast(0, c);
            e.z_T     = cast(0, c);
        end
    end
elseif was > 0
    if e.z_T >= p.ekf_zupt_min_T
        zupt_due  = 1;
        rate_mean = e.z_sum/e.z_T;
        T         = e.z_T;
    end
    e.z_sum = cast(0, c);
    e.z_T   = cast(0, c);
end
end
