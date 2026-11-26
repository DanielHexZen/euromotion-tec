function xs = nav_est_init(P, p0, psi0)
%NAV_EST_INIT  State of nav_est_step (LKF, heading filter, calibration).
xs.x = [p0(1); 0; p0(2); 0; 0; 0];
xs.Pm = P.lkf.P0;
xs.psi = psi0; xs.wzc = 0; xs.u = [0; 0];
xs.cal = [0; 0; 0]; xs.cb = [0; 0; 0];
end
