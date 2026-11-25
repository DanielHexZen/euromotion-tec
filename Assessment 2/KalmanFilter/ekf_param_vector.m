function pv = ekf_param_vector(p)
%EKF_PARAM_VECTOR  Pack the parameter struct into the numeric vector used by
%   ekf_step (a plain vector is easiest to pass into Simulink).
%
%   pv = [Ts L tau_v tau_d k_delta k_g  q(1:6)  R_v R_g R_xy R_psi]
pv = [p.Ts; p.L; p.tau_v; p.tau_d; p.k_delta; p.k_g; ...
      p.q(:); p.R_v; p.R_g; p.R_xy; p.R_psi];
end
