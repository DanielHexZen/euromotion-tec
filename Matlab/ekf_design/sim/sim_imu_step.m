function [st, y] = sim_imu_step(st, yp, tr)
%SIM_IMU_STEP  One gyro sample at tr.f_imu (833 Hz, LSM6DS3) (DRDY-timestamped, §7.11).
%
%   [st, y] = sim_imu_step(st, yp, tr), yp = latest plant output (1 kHz, held).
%
%   omega_meas = (1 + sf)*omega_true + b_g + n_w,   n_w ~ N(0, nd^2 * f_imu)
%   b_g       += sqrt(q_bg/f_imu)*N(0,1) + bg_ramp/f_imu          (random walk + ramp)
%   quantised to imu_gyro_lsb (17.5 mdps at +-500 dps).
%   omega_true is evaluated exactly at the sample time from the plant's
%   first-order wheel solution inside the current 1 ms tick. Level ground:
%   roll = pitch = 0, so the projected yaw rate equals the z rate.
%
%   y = [seq; t_us; omega_meas; b_g(true); omega_true]
%#codegen
t   = tr.imu_t0 + st.i/tr.f_imu;
tau = t - yp(1);
a   = exp(-tau/tr.tau_w);
vL  = yp(9)  + (yp(7) - yp(9))*a;
vR  = yp(10) + (yp(8) - yp(10))*a;
w_true = (yp(12)*vR - yp(11)*vL)/tr.b_true;
dt  = 1/tr.f_imu;
[st.rng_b, nb] = sim_randn(st.rng_b);
st.bg = st.bg + sqrt(tr.q_bg*dt)*nb + tr.bg_ramp*dt;
[st.rng_w, nw] = sim_randn(st.rng_w);
wm = (1 + tr.sf)*w_true + st.bg + tr.nd*sqrt(tr.f_imu)*nw;
if tr.lsb > 0
    wm = round(wm/tr.lsb)*tr.lsb;
end
st.i   = st.i + 1;
st.seq = st.seq + 1;
y = [st.seq; round(t*1e6); wm; st.bg; w_true];
st.y = y;
end
