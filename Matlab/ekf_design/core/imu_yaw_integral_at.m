function [psi, st] = imu_yaw_integral_at(r, p, t_us)
%IMU_YAW_INTEGRAL_AT  Psi at an arbitrary time t (spec §7.11.3, REQ-IMU-05).
%
%   Linear interpolation inside the ring; beyond the newest sample,
%   extrapolation with the newest rate by at most imu_extrap_max; otherwise
%   st = 2 (ST_RANGE). st = 0 OK.
%#codegen
o   = r.psi(1)*0 + 1;            % 1 in the working class
psi = 0*o;
st  = 0;
M   = numel(r.t);
if r.n == 0
    st = 2;
    return;
end
t_new = r.t(r.head);
if t_us >= t_new
    dt = (t_us - t_new)*1e-6;
    if dt > p.imu_extrap_max
        st = 2;
        return;
    end
    psi = r.psi(r.head) + r.rate(r.head)*(dt*o);
    return;
end
j = r.head;
for k = 1:(r.n - 1)
    i = j - 1;
    if i < 1
        i = M;
    end
    if r.t(i) <= t_us
        a   = ((t_us - r.t(i))/(r.t(j) - r.t(i)))*o;
        psi = r.psi(i) + a*(r.psi(j) - r.psi(i));
        return;
    end
    j = i;
end
st = 2;                                   % older than the ring
end
