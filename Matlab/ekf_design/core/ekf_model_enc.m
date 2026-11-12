function [xn, F, dse, dth, phi] = ekf_model_enc(x, ds_l, ds_r, dt, still, p)
%EKF_MODEL_ENC  Process model and Jacobian, encoder-heading mode (spec §7.17.7).
%
%   dth = still ? 0 : (ds_r - ds_l)/b    with b = odo_wheelbase
%   b_g is frozen: its row and column of F are those of the identity.
%   dt is unused (kept for a uniform interface).
%#codegen
c   = class(x);
dse = (ds_l + ds_r)*cast(0.5, c);
if still > 0
    dth = cast(0, c);
    ds  = cast(0, c);
else
    dth = (ds_r - ds_l)/p.odo_wheelbase;
    ds  = x(5)*dse;
end
phi = x(3) + cast(0.5, c)*dth;
cp  = cos(phi);
sp  = sin(phi);
xn    = x;
xn(1) = x(1) + ds*cp;
xn(2) = x(2) + ds*sp;
xn(3) = ekf_wrap(x(3) + dth);
F = eye(5, c);
if still == 0
    F(1,3) = -ds*sp;
    F(1,5) =  dse*cp;
    F(2,3) =  ds*cp;
    F(2,5) =  dse*sp;
end
end
