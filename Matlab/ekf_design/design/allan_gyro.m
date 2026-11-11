function A = allan_gyro(w, fs, varargin)
%ALLAN_GYRO  Overlapping Allan deviation of a stationary gyro record and the
%   two EKF parameters derived from it.
%
%   A = allan_gyro(w, fs)
%   w   yaw-rate samples, rad/s (after axis mapping, IMU at rest), any length
%   fs  sample rate, Hz (960 for the LSM6DSV setting of §7.11.1)
%
%   Model fitted (IEEE Std 952 terms that the EKF represents):
%     sigma^2(tau) = N^2/tau + K^2*tau/3
%     N  angle random walk, rad/sqrt(s)  ->  ekf_gyro_nd = N   (rad/s/sqrt(Hz))
%     K  rate random walk, rad/s/sqrt(s) ->  ekf_q_bg    = K^2 ((rad/s)^2/s)
%   Weighted linear least squares in (N^2, K^2) with weights 1/sigma^4 and
%   the number of independent clusters (relative error ~ 1/sqrt(2(n/m - 1))).
%   Bias instability (flicker, the flat part of the curve) is not a random
%   walk; A.B reports the minimum of the curve divided by 0.664 for
%   reference. If the flat part dominates, choose ekf_q_bg so that the random
%   walk matches the curve at the longest stop-to-stop interval of the course
%   (about 60 s here) rather than the fitted K.
%
%   A.tau, A.adev, A.N, A.K, A.B, A.ekf_gyro_nd, A.ekf_q_bg
%   Base MATLAB only (cumsum), also runs in Octave.
w  = w(:);
n  = numel(w);
t0 = 1/fs;
mmax = floor(n/9);
m  = unique(round(logspace(0, log10(mmax), 60)))';
th = [0; cumsum(w)*t0];                 % integrated angle
av = zeros(numel(m), 1);
for i = 1:numel(m)
    mi = m(i);
    d  = th(1+2*mi:end) - 2*th(1+mi:end-mi) + th(1:end-2*mi);
    av(i) = sum(d.^2)/(2*(mi*t0)^2*(n - 2*mi));
end
tau = m*t0;
ncl = n./m;
wgt = (ncl - 1)./av.^2;                 % 1/var(sigma^2) ~ (n/m)/sigma^4
X   = [1./tau, tau/3];
WX  = X.*[wgt wgt];
th2 = (WX'*X)\(WX'*av);
th2 = max(th2, 0);
A.tau  = tau;
A.adev = sqrt(av);
A.N    = sqrt(th2(1));
A.K    = sqrt(th2(2));
A.B    = min(sqrt(av))/0.664;
A.ekf_gyro_nd = A.N;
A.ekf_q_bg    = A.K^2;
end
