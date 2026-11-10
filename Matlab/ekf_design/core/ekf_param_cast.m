function q = ekf_param_cast(p, cls)
%EKF_PARAM_CAST  Cast the floating-point parameters to 'single' or 'double'.
%
%   q = ekf_param_cast(p, cls) returns a copy of p in which every real
%   parameter is of class cls. Integer-valued structural constants (ring
%   lengths, counts, millisecond settings and the mx_ switches) stay double
%   so that they can be used as sizes and loop bounds; in C they are integer
%   constants as well.
%
%   This is THE precision switch: the filter functions take their working
%   class from class(p.ekf_kd) and from the state, so passing a single-cast
%   parameter struct to ekf_init makes the whole filter path float32, as on
%   the STM32F446 (CLAUDE.md: float only).

keep = {'ekf_hist_n','ekf_slip_n','imu_ring_n','gnd_n_deb','lat_n_min','lat_n_max', ...
        'vl_tb_ms','vl_imp_ms','vl_poll_ms','imu_odr_hz', ...
        'mx_still_init','mx_still_F_bg','mx_hist_corr','mx_sf_mode','mx_freeze_s'};
q = p;
f = fieldnames(p);
for i = 1:numel(f)
    v = p.(f{i});
    if isnumeric(v) && ~any(strcmp(f{i}, keep))
        q.(f{i}) = cast(v, cls);
    end
end
end
