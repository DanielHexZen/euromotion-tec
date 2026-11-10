function r = imu_yaw_init(p)
%IMU_YAW_INIT  Empty yaw-integral ring (spec §7.11.3). Class follows p.ekf_kd.
%   r.t (us, double), r.psi, r.rate (working class), r.head, r.n
%#codegen
c = class(p.ekf_kd);
M = p.imu_ring_n;
r.t    = zeros(M, 1);
r.psi  = zeros(M, 1, c);
r.rate = zeros(M, 1, c);
r.head = 0;
r.n    = 0;
end
