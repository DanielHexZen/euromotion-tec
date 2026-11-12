function es = sim_est_init(p, c)
%SIM_EST_INIT  State of app_est (the estimator part of S_CTRL under D-30).
%   p must already be cast to the working class (ekf_param_cast).
%#codegen
cl = class(p.ekf_kd);
es.ekf = ekf_init(p, c.start(1), c.start(2), c.start(3));
es.started  = 0;
es.prev_cnt = [0; 0];
es.prev_t   = 0;
es.prev_psi = cast(0, cl);
es.gnd_seq  = 0;
es.lat_seq  = 0;
es.lat      = lat_init(p);
es.n_skip   = zeros(1, 5);    % edge skipped: not LINE, no boundary, edge_skip zone, too old, y_off
es.imu_ok   = 0;
es.feat_imu = 1;
end
