function st = sim_yaw_init(p)
%SIM_YAW_INIT  State of the firmware IMU front end (imu_drain + yaw integral ring).
%#codegen
st.seq = 0;
st.r   = imu_yaw_init(p);
end
