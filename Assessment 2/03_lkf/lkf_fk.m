function par = lkf_fk(par, psi_mid)
%LKF_FK  Heading-dependent bias columns of F for the 6-state model:
%   F(1:4,5:6) = -G(1:4,:) * R(psi_mid),  R(psi) = [cos -sin; sin cos].
%   No-op for the 4-state model. psi_mid = mean heading over the sample interval.
if size(par.F,1) == 6
    c = cos(psi_mid); s = sin(psi_mid);
    par.F(1:4,5:6) = -par.G(1:4,:)*[c -s; s c];
end
end
