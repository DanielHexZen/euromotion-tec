function st = sim_imu_init(tr)
%SIM_IMU_INIT  Gyro truth state (bias, PRNG streams).
%#codegen
st.i   = 0;                       % sample index; sample i is at imu_t0 + i/f_imu
st.seq = 0;
st.bg  = tr.bg0;
st.rng_w = sim_rng_init(tr.seed, 1);
st.rng_b = sim_rng_init(tr.seed, 2);
st.y   = zeros(5, 1);
end
