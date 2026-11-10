function y = ekf_get(e)
%EKF_GET  Output vector (ekf_out_t of spec §7.17.6, flattened, double).
%
%   y( 1: 5)  state x y theta b_g s
%   y( 6:10)  diagonal of P
%   y(11)     P_xy
%   y(12)     EKF_STILL         y(13) EKF_SLIP       y(14) slip episodes
%   y(15)     w_meas (bias-corrected yaw rate of the last step, rad/s)
%   y(16)     t_us of the last prediction
%   y(17:21)  accepted updates per type (ZUPT EDGE FACE HEADING POINT)
%   y(22:26)  rejected updates per type
%   y(27:31)  NIS sum per type (accepted)
%   y(32)     fault flag (health)  y(33) number of health failures
%   y(34:58)  P (column-major, full)
%#codegen
y = zeros(58, 1);
y(1:5)   = double(e.x);
y(6:10)  = double(diag(e.P));
y(11)    = double(e.P(1,2));
y(12)    = e.still;
y(13)    = e.slip;
y(14)    = e.slip_episodes;
y(15)    = double(e.w_meas);
y(16)    = e.t_us;
y(17:21) = e.n_acc(:);
y(22:26) = e.n_rej(:);
y(27:31) = e.nis_sum(:);
y(32)    = e.fault;
y(33)    = e.n_fault;
y(34:58) = double(e.P(:));
end
