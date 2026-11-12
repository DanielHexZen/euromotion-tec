function r = imu_yaw_push(r, p, t_us, wq, wr, roll, pitch)
%IMU_YAW_PUSH  One IMU sample into the yaw integral (spec §7.11.3, in imu_drain).
%
%   psidot_i = (q sin(phi) + r cos(phi))/cos(theta)        (body rates q, r in rad/s)
%   Psi_i    = Psi_{i-1} + psidot_i*(t_i - t_{i-1})        (rectangle rule, as the spec)
%
%   PRECISION NOTE (finding F-7): Psi grows without bound (total turned
%   angle). In float32 its resolution at |Psi| = 100 rad is 7.6e-6 rad, and
%   the EKF consumes differences Psi(t1) - Psi(t0); see verify_float32.m for
%   the measured effect. Keep Psi in double, in int64 LSB-microseconds, or
%   re-base it (subtract a common offset from the ring) in the C code.
%#codegen
o  = r.psi(1)*0 + 1;               % 1 in the working class: v*o == cast(v, class)
M  = numel(r.t);
pd = ((wq*o)*sin(roll*o) + (wr*o)*cos(roll*o))/cos(pitch*o);
if r.n == 0
    psi = 0*o;
else
    dt  = ((t_us - r.t(r.head))*1e-6)*o;
    psi = r.psi(r.head) + pd*dt;
end
r.head = mod(r.head, M) + 1;
r.t(r.head)    = t_us;
r.psi(r.head)  = psi;
r.rate(r.head) = pd;
if r.n < M
    r.n = r.n + 1;
end
end
