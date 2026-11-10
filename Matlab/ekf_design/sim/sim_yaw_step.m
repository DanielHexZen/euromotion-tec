function [st, y] = sim_yaw_step(st, y_imu, p)
%SIM_YAW_STEP  imu_drain in S_FAST (1 kHz): push a new DRDY sample into the yaw ring.
%   At most one sample arrives per tick (960 Hz < 1 kHz). Roll and pitch are 0
%   (level ground; att is not simulated). Output: packed ring (sim_yaw_pack).
%#codegen
if y_imu(1) > st.seq
    st.seq = y_imu(1);
    st.r = imu_yaw_push(st.r, p, y_imu(2), 0, y_imu(3), 0, 0);
end
y = sim_yaw_pack(st.r);
end
