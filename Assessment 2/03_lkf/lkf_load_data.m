function D = lkf_load_data(datafile, T)
%LKF_LOAD_DATA  Load the real 100 Hz run, decimate by 2 to T = 0.02 s, flag NaNs.
%   D.t, D.ax, D.ay, D.wz, D.cmd, D.vs (NaN kept), D.px, D.py, D.yaw (NaN kept)
d = load(datafile);
step = round(T/0.01);
i = 1:step:numel(d.t);
D.T   = T;
D.t   = d.t(i) - d.t(i(1));
D.ax  = d.imu_ax(i);  D.ay = d.imu_ay(i);  D.wz = d.imu_wz(i);
D.cmd = d.cmd_speed(i);  D.vs = d.vesc_speed(i);
D.px  = d.pf_x(i);  D.py = d.pf_y(i);  D.yaw = d.pf_yaw(i);
end
