function [l, out, n_out] = lat_step(l, smp, th_est, seg, s_seg, c, p)
%LAT_STEP  Feed one lateral sample; emit LAT_HEADING / LAT_FACE_RANGE / LAT_POINT (§7.15).
%
%   smp    = [t_us id r ok s_enc] (id 1 = S1 left, 2 = S2 right)
%   th_est = current EKF heading, seg = current segment, s_seg = EKF along-segment s
%   out    = 4 x 8 rows [type t_us lm v1 v2 v3 v4 0]:
%            type 1 HEADING  v1 = theta_m, v2 = var
%            type 2 FACE     v1 = r_end, v2 = var, v3 = sensor id
%            type 3 POINT    v1 = qx, v2 = qy, v3 = survey var, v4 = sensor id
%   Face fit as specified: LS line r = a + b*s over the window; accept if
%   N >= lat_n_min, span >= lat_span_min, rms <= lat_rms_max, heading change
%   < 0.5 deg; delta = asin(-b) (left) / asin(b) (right); theta_m = psi_face +
%   delta; var = max(12 rms^2/(N span^2), lat_sig_face_min^2); r_end = a + b*s_end,
%   var rms^2. Post notch: background = median of the last 10 samples outside a
%   notch; start at r < r_bg - lat_notch_depth for 2 samples; centre of the
%   flanks; width check [0.5, 2]*(2 r_post + spot); q = r_s + (r_min + r_post) u_s.
%   Simplification: samples without a target count as r = lat_r_max for the
%   post background; the window test uses the EKF along-segment position.
%#codegen
out = zeros(4, 8);
n_out = 0;
sid  = smp(2);
side = 3 - 2*sid;
for m = 1:c.n_lm
    if c.lm_seg(m) ~= seg || c.lm_side(m) ~= side || l.done(m) > 0
        continue;
    end
    inwin = s_seg >= c.lm_from(m) && s_seg <= c.lm_to(m);
    if inwin && l.act(m) == 0
        l.act(m) = 1;  l.n(m) = 0;  l.th0(m) = th_est;
        l.nbg(m) = 0;  l.below(m) = 0;  l.in(m) = 0;  l.ns(m) = 0;
    end
    if l.act(m) == 0
        continue;
    end
    finish = ~inwin;
    if inwin
        if c.lm_type(m) == 1
            if smp(4) > 0 && l.n(m) < double(p.lat_n_max)
                l.n(m) = l.n(m) + 1;
                l.s(l.n(m), m) = smp(5);  l.r(l.n(m), m) = smp(3);  l.t(l.n(m), m) = smp(1);
            end
            if l.n(m) >= double(p.lat_n_max), finish = true; end
        else
            [l, o, has] = post_feed(l, m, smp, c, p);
            if has > 0 && n_out < 4
                n_out = n_out + 1;  out(n_out, :) = o;
            end
        end
    end
    if finish
        l.act(m) = 0;  l.done(m) = 1;
        if c.lm_type(m) == 1
            [l, o1, o2, ok] = face_fit(l, m, th_est, c, p, sid);
            if ok > 0 && n_out <= 2
                out(n_out+1, :) = o1;  out(n_out+2, :) = o2;  n_out = n_out + 2;
            end
        end
    end
end
end

function [l, o1, o2, ok] = face_fit(l, m, th_est, c, p, sid)
o1 = zeros(1, 8);  o2 = zeros(1, 8);  ok = 0;
N = l.n(m);
if N < double(p.lat_n_min)
    l.n_face_rej = l.n_face_rej + 1;  return;
end
s = l.s(1:N, m);  r = l.r(1:N, m);
sm = mean(s);  rmn = mean(r);
Sss = sum((s - sm).^2);
b = sum((s - sm).*(r - rmn))/Sss;
a = rmn - b*sm;
res = r - (a + b*s);
rms = sqrt(sum(res.^2)/N);
span = s(N) - s(1);
if span < double(p.lat_span_min) || rms > double(p.lat_rms_max) || abs(ekf_wrap(th_est - l.th0(m))) >= double(p.lat_dth_max)
    l.n_face_rej = l.n_face_rej + 1;  return;
end
if sid == 1, dl = asin(-b); else, dl = asin(b); end
psi_face = atan2(c.lm_ty(m), c.lm_tx(m));
var_d = max(12*rms^2/(N*span^2), double(p.lat_sig_face_min)^2);
o1 = [1, l.t(N, m), m, ekf_wrap(psi_face + dl), var_d, 0, 0, 0];
o2 = [2, l.t(N, m), m, a + b*s(N), rms^2, sid, 0, 0];
ok = 1;
l.n_face = l.n_face + 1;
end

function [l, o, has] = post_feed(l, m, smp, c, p)
o = zeros(1, 8);  has = 0;
r = smp(3);
if smp(4) == 0, r = double(p.lat_r_max); end
nb = min(l.nbg(m), 10);
if nb >= 3
    rbg = median(l.bg(1:nb, m));
else
    rbg = -1e9;     % no background yet: the sample counts as background, not as a notch
end
if l.in(m) == 0
    if r < rbg - double(p.lat_notch_depth)
        l.below(m) = l.below(m) + 1;
        l.ns(m) = min(l.ns(m) + 1, 200);
        l.ps(l.ns(m), m) = smp(5);  l.pt(l.ns(m), m) = smp(1);  l.pr(l.ns(m), m) = r;
        if l.below(m) >= 2, l.in(m) = 1; end
    else
        l.below(m) = 0;  l.ns(m) = 0;
        l.nbg(m) = l.nbg(m) + 1;
        l.bg(mod(l.nbg(m) - 1, 10) + 1, m) = r;
    end
else
    if r < rbg - double(p.lat_notch_depth)
        l.ns(m) = min(l.ns(m) + 1, 200);
        l.ps(l.ns(m), m) = smp(5);  l.pt(l.ns(m), m) = smp(1);  l.pr(l.ns(m), m) = r;
    else
        n  = l.ns(m);
        s_in = l.ps(1, m);  s_out = l.ps(n, m);
        sc = 0.5*(s_in + s_out);
        rmin = min(l.pr(1:n, m));
        spot = 2*rmin*tan(7.5*pi/180);
        w = s_out - s_in;
        wn = 2*c.lm_r(m) + spot;
        l.in(m) = 0;  l.below(m) = 0;  l.ns(m) = 0;
        if w >= 0.5*wn && w <= 2.0*wn
            [~, jc] = min(abs(l.ps(1:n, m) - sc));
            sgn = c.lm_side(m);
            qx = double(p.lat_x);
            qy = sgn*(double(p.lat_y) + rmin + c.lm_r(m));
            o = [3, l.pt(jc, m), m, qx, qy, c.lm_sig(m)^2, smp(2), 0];
            has = 1;  l.n_post = l.n_post + 1;  l.done(m) = 1;  l.act(m) = 0;
        else
            l.n_post_rej = l.n_post_rej + 1;
        end
    end
end
end
