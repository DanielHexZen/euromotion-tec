function r = sim_yaw_unpack(y, p)
%SIM_YAW_UNPACK  Inverse of sim_yaw_pack in the working class of p.
%#codegen
r = imu_yaw_init(p);
M = numel(r.t);
c = class(r.psi);
r.t    = y(1:M);
r.psi  = cast(y(M+1:2*M), c);
r.rate = cast(y(2*M+1:3*M), c);
r.head = y(3*M+1);
r.n    = y(3*M+2);
end
