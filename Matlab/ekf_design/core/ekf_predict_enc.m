function e = ekf_predict_enc(e, p, ds_l, ds_r, dt)
%EKF_PREDICT_ENC  Prediction step, encoder-heading mode (spec §7.17.7, REQ-EKF-10).
%
%   Heading from the wheel difference with b = odo_wheelbase, b_g frozen
%   (identity row/column, no ZUPT, no Q_bb), Q_tt += ekf_q_th_enc*|ds_r - ds_l|,
%   slip detector off.
%
%   Note: the spec does not say whether Q_bb (ekf_q_bg*dt) is still added in
%   this mode. "b_g is frozen" is read as: no random walk either.
%#codegen
c     = class(e.x);
still = e.still;
[xn, F, dse, ~, phi] = ekf_model_enc(e.x, ds_l, ds_r, dt, still, p);
s  = e.x(5);
cp = cos(phi);
sp = sin(phi);
Q = zeros(5, 5, c);
sig2_ds = p.ekf_kd*abs(dse);
g = [s*cp; s*sp; 0; 0; 0];
Q = Q + g*sig2_ds*g';
Q(3,3) = Q(3,3) + p.ekf_q_th_enc*abs(ds_r - ds_l);
if p.mx_freeze_s == 0
    Q(5,5) = Q(5,5) + p.ekf_q_s*dt;
end
if dt > 0
    e.w_meas = (ds_r - ds_l)/(p.odo_wheelbase*dt);
end
e.x = xn;
FP  = F*e.P;
e.P = FP*F' + Q;
e.P = cast(0.5, c)*(e.P + e.P');
end
