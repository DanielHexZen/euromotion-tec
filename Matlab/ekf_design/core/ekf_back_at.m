function [bk, st] = ekf_back_at(e, p, t_us)
%EKF_BACK_AT  Back-shift {dx, dy, dth} for a measurement taken at t_us (REQ-EKF-02).
%
%   [bk, st] = ekf_back_at(e, p, t_us)
%   bk = estimated motion between t_us and the newest history entry (now).
%   st = 0 OK, 2 ST_RANGE (older than ekf_meas_max_age or than the ring:
%   the measurement is discarded).
%
%   The pose at t_us is interpolated linearly between the two history
%   entries that bracket it (heading through the wrapped difference). At 50 Hz
%   (D-30) the entries are 20 ms apart; verify_rate_50_200.m quantifies the
%   interpolation error against the 5 ms spacing of the 200 Hz design.
%   A t_us newer than the newest entry gives bk = 0 (measurement belongs to now).
%#codegen
c  = class(e.x);
bk = zeros(3, 1, c);
st = 0;
N  = p.ekf_hist_n;
if e.hist_n == 0
    st = 2;
    return;
end
t_new = e.hist_t(e.hist_head);
if (t_new - t_us)*1e-6 > p.ekf_meas_max_age
    st = 2;
    return;
end
if p.mx_hist_corr > 0
    m_now = e.x(1:3) - e.corr;
else
    m_now = e.x(1:3);
end
if t_us >= t_new
    return;                               % bk = 0
end
% walk back from the newest entry to the first one at or before t_us
j = e.hist_head;                          % newer neighbour
found = 0;
i = j;
for k = 1:(e.hist_n - 1)
    i = j - 1;
    if i < 1
        i = N;
    end
    if e.hist_t(i) <= t_us
        found = 1;
        break;
    end
    j = i;
end
if found == 0
    st = 2;                               % older than the ring
    return;
end
ti = e.hist_t(i);
tj = e.hist_t(j);
a  = cast((t_us - ti)/(tj - ti), c);
mi = e.hist_p(:, i);
mj = e.hist_p(:, j);
m_past    = mi + a*(mj - mi);
m_past(3) = mi(3) + a*ekf_wrap(mj(3) - mi(3));
bk    = m_now - m_past;
bk(3) = ekf_wrap(bk(3));
end
