function [e, zupt_due, rate_mean, T] = ekf_still_update(e, p, cnt_changed, drv_idle, dpsi, dt_us)
%EKF_STILL_UPDATE  Standstill logic and ZUPT accumulator (REQ-EKF-09, §7.17.4).
%
%   [e, zupt_due, rate_mean, T] = ekf_still_update(e, p, cnt_changed, drv_idle, dpsi, dt_us)
%     cnt_changed  1 if either wheel count changed over this step
%     drv_idle     1 if drv is in HOLD, DISABLED or IDLE (no motion commanded)
%     dpsi         yaw-integral difference of this step
%     dt_us        step length in INTEGER microseconds (t_now - t_prev of the encoder
%                  timestamps; uint32 arithmetic in C)
%
%   FINDING F-8: the standstill timer and the ZUPT window are accumulated in
%   integer microseconds. Accumulating float dt is precision-dependent:
%   50 x 0.02f = 0.99999958f < 1.0f, so in float32 the ZUPT window becomes 51
%   steps and 'still' is set one step later than in double, which shifts the
%   whole closed-loop mission timing between float32 and float64.
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
    e.t_nochg_us = 0;
else
    e.t_nochg_us = e.t_nochg_us + dt_us;
end
was = e.still;
e.still = double(e.t_nochg_us >= round(double(p.ekf_still_s)*1e6) && drv_idle > 0);
Tz  = round(double(p.ekf_zupt_T)*1e6);
Tz0 = round(double(p.ekf_zupt_min_T)*1e6);
if e.still > 0
    if ~isnan(dpsi)
        e.z_sum  = e.z_sum + dpsi;
        e.z_T_us = e.z_T_us + dt_us;
        if e.z_T_us >= Tz
            zupt_due  = 1;
            T         = cast(e.z_T_us*1e-6, c);
            rate_mean = e.z_sum/T;
            e.z_sum   = cast(0, c);
            e.z_T_us  = 0;
        end
    end
elseif was > 0
    if e.z_T_us >= Tz0
        zupt_due  = 1;
        T         = cast(e.z_T_us*1e-6, c);
        rate_mean = e.z_sum/T;
    end
    e.z_sum  = cast(0, c);
    e.z_T_us = 0;
end
end
