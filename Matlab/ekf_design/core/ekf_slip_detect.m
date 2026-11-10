function e = ekf_slip_detect(e, p, ds_l, ds_r, dpsi, dt)
%EKF_SLIP_DETECT  Slip detector, IMU mode (REQ-EKF-04).
%
%   r_slip = (ds_r - ds_l)/(b*dt) - (dpsi/dt - b_g)
%   |r_slip| > ekf_slip_thr for >= ekf_slip_n consecutive steps -> slip = 1
%   while it persists; each rising edge is one episode.
%
%   Causality note: "slip = true for those steps" cannot include the first
%   ekf_slip_n - 1 steps of an episode in a causal filter; slip is set from the
%   n-th step on. At 50 Hz (D-30) that is 60 ms after onset, at 200 Hz 15 ms.
%#codegen
if dt <= 0
    return;
end
r = (ds_r - ds_l)/(p.odo_wheelbase*dt) - (dpsi/dt - e.x(4));
e.r_slip = r;
if abs(r) > p.ekf_slip_thr
    e.slip_cnt = e.slip_cnt + 1;
else
    e.slip_cnt = 0;
end
was = e.slip;
e.slip = double(e.slip_cnt >= p.ekf_slip_n);
if e.slip > 0 && was == 0
    e.slip_episodes = e.slip_episodes + 1;
end
end
