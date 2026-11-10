function e = ekf_hist_push(e, p, t_us)
%EKF_HIST_PUSH  Append the pose after a prediction to the history ring (§7.17.1).
%
%   The ring holds ekf_hist_n entries {t_us, x, y, theta}, one per estimator
%   step: 5 ms apart at 200 Hz, 20 ms apart at 50 Hz (D-30).
%
%   DESIGN NOTE (finding F-6): the back-shift must be the estimated MOTION
%   between the measurement time and now. If raw poses are stored, every
%   correction applied after the measurement time (another edge event, a ZUPT
%   moving theta through P_theta,bg ...) leaks into the back-shift and is
%   undone for the delayed measurement. With p.mx_hist_corr = 1 the ring stores
%   pose - corr, where corr is the sum of all update corrections; the
%   back-shift (current pose - corr) - (stored entry) then contains motion only.
%   C cost: 3 floats and 3 additions per update.
%#codegen
N = p.ekf_hist_n;
e.hist_head = mod(e.hist_head, N) + 1;
e.hist_t(e.hist_head) = t_us;
if p.mx_hist_corr > 0
    e.hist_p(:, e.hist_head) = e.x(1:3) - e.corr;
else
    e.hist_p(:, e.hist_head) = e.x(1:3);
end
if e.hist_n < N
    e.hist_n = e.hist_n + 1;
end
end
