function cfg = nav_cfg_default()
%NAV_CFG_DEFAULT  Default configuration of a navigation run.
cfg.seed = 1;            % rng seed of the sensor noise
cfg.noise = 'motion';    % 'motion' (in-motion accelerometer noise, LKF Q consistent) or 'standstill'
cfg.fb = 'lkf';          % feedback source: 'lkf' (estimated) or 'true' (ideal)
cfg.vmax = 2.5;          % max track speed [m/s]
cfg.alat = 2.5;          % lateral acceleration limit of the speed profile [m/s^2]
cfg.aacc = 1.5;          % forward acceleration limit [m/s^2]
cfg.adec = 2.0;          % deceleration limit [m/s^2]
cfg.runout = 8;          % run-out after the finish line [m]
cfg.t_stand = 5;         % standstill phase for sensor bias calibration [s]
cfg.offset0 = 0.1;       % initial lateral offset (left positive) [m]
cfg.tla = 0.12;          % curvature look-ahead time [s]
cfg.ffmode = 2;          % longitudinal feedforward: 1 = v_ref, 2 = plant-inverse of Gv
cfg.vlat_min = 0.4;      % lateral loop active above this estimated speed [m/s]
cfg.kr_scale = 1.0;      % plant mismatch: gain of Gr
cfg.kv_scale = 1.0;      % plant mismatch: gain of Gv
cfg.W = 80;              % projection search half window [points]
cfg.servo_rate = (pi/3)/0.15;  % steering-servo rate limit [rad/s]: servo spec 0.15 s/60 deg = 6.981 rad/s; 1e3 = none
cfg.dmode = 2;           % lateral D action: 1 = differenced e (pid_step D), 2 = on measurement: Kd * d(ey)/dt from the LKF velocity
cfg.tauc = 0.5;          % dmode 3: time constant of the position-difference correction of the velocity rate [s]
cfg.usev = 1;            % 1 = additionally fuse the wheel speed (projected on the heading) at the fix instants (03 optional design)
cfg.Kc = 0.1;            % complementary-filter gain of the heading fix
end
