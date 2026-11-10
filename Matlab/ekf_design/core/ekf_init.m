function e = ekf_init(p, x0, y0, th0)
%EKF_INIT  Filter state at the course start (spec §7.17.1, §7.17.5). C: ekf_init().
%
%   e = ekf_init(p, x0, y0, th0)
%
%   State order x = [x y theta b_g s]' (m, m, rad, rad/s, -). The working class
%   (single/double) is class(p.ekf_kd); see ekf_param_cast. Timestamps are
%   integer microseconds held in double (uint32 in C; differences are formed
%   in integer arithmetic there and converted with a 1e-6f factor).
%
%   Fields (all fixed size, codegen-compatible):
%     x, P              state and full 5x5 covariance (REQ-EKF-01)
%     t_us              timestamp of the last prediction
%     still, t_nochg    standstill flag and time without count change (§7.17.4)
%     z_sum, z_T        ZUPT accumulator (sum of dpsi, sum of dt)
%     slip, slip_cnt, slip_episodes, r_slip   slip detector (REQ-EKF-04)
%     hist_t, hist_p, hist_head, hist_n       pose history ring (§7.17.1)
%     corr              cumulative measurement correction of x,y,theta (see ekf_hist_push)
%     n_acc, n_rej, nis_sum   per update type: 1 ZUPT 2 EDGE 3 FACE 4 HEADING 5 POINT
%     rec, n_rec        update records of the current call of app_est (MATLAB logging only)
%     fault, n_fault    health failures (REQ-EKF-05 step 5 -> F_EKF)
%#codegen
c  = class(p.ekf_kd);
N  = p.ekf_hist_n;
e.x = cast([x0; y0; th0; 0; 1], c);
e.P = diag(cast([p.ekf_p0_xy^2; p.ekf_p0_xy^2; p.ekf_p0_th^2; p.ekf_p0_bg^2; p.ekf_p0_s^2], c));
if p.mx_freeze_s > 0
    e.P(5,5) = cast(0, c);
end
e.t_us   = 0;
e.still  = 0;
if p.mx_still_init > 0
    % The vehicle is stationary when the EKF is (re)initialised: AUTO_INIT
    % requires it (REQ-MSN-02). Without this the first ekf_still_s seconds are
    % integrated as motion with b_g = 0 (0.1 deg heading error at 0.5 deg/s bias).
    e.t_nochg = cast(p.ekf_still_s, c);
else
    e.t_nochg = cast(0, c);
end
e.z_sum  = cast(0, c);
e.z_T    = cast(0, c);
e.slip   = 0;
e.slip_cnt = 0;
e.slip_episodes = 0;
e.r_slip = cast(0, c);
e.w_meas = cast(0, c);          % bias-corrected yaw rate of the last step (for nav_turn)
e.hist_t = zeros(N, 1);
e.hist_p = zeros(3, N, c);
e.hist_head = 0;
e.hist_n = 0;
e.corr   = zeros(3, 1, c);
e.n_acc  = zeros(1, 5);
e.n_rej  = zeros(1, 5);
e.nis_sum = zeros(1, 5);
e.rec    = zeros(8, 7);
e.n_rec  = 0;
e.fault  = 0;
e.n_fault = 0;
end
