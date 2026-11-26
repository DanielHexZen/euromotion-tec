function dp = pid_dpoles(zeta, wn, kappa, T)
%PID_DPOLES  Desired closed-loop poles (z-plane) from the time-domain specs.
%   Dominant pair  s_{1,2} = -zeta*wn +/- j*wn*sqrt(1-zeta^2),   wn = 4/(zeta*ts)   (2 % criterion)
%   Extra poles    s_{3,4} = -kappa(1)*zeta*wn, -kappa(2)*zeta*wn   (real; kappa scalar -> double pole)
%   Mapping        z = exp(s*T)   (exact for the sampled system)
%   Returns a 4x1 column vector dp (complex conjugate pair first).
if isscalar(kappa), kappa = [kappa kappa]; end
s12 = -zeta*wn + 1i*wn*sqrt(1 - zeta^2) * [1; -1];
s34 = -kappa(:) * zeta * wn;
dp  = exp([s12; s34] * T);
end
