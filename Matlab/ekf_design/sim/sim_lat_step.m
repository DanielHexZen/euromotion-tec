function [st, y] = sim_lat_step(st, yp, tr, c, p)
%SIM_LAT_STEP  1 kHz: lateral range samples against the TRUE faces and posts.
%
%   50 Hz ranging per sensor, t_meas = t_ready - tb/2 (EXTI, REQ-VL-03), ray
%   from the body mount (lat_x, +-lat_y) along +-y, range noise N(0, sig_lat_r).
%   True landmark = map position + tr.lm_err (survey error). A face is a
%   straight line segment covering its window +-0.3 m, a post a circle.
%   Samples without a target in [lat_r_min, lat_r_max] have ok = 0.
%   s_enc = mean encoder distance at the processing tick (what lat_on_meas gets).
%   y = [seq; ev(:)], ev = 16 x [t_us id r ok s_enc]
%#codegen
t  = yp(1);
k  = yp(13);
ix = mod(k, 64) + 1;
st.hx(ix) = yp(2);  st.hy(ix) = yp(3);  st.hth(ix) = yp(4);
if tr.lat_on > 0
    s_enc = 0.5*(floor(yp(5)/p.odo_m_per_cnt) + floor(yp(6)/p.odo_m_per_cnt))*p.odo_m_per_cnt;
    for i = 1:2
        if t >= st.t_end(i) - 1e-9
            te = st.t_end(i);
            st.t_end(i) = te + st.imp;
            tm = te - st.tb/2;
            j  = mod(max(0, min(k, round(tm*1000))), 64) + 1;
            sgn = 3 - 2*i;                       % +1 left (S1), -1 right (S2)
            ct = cos(st.hth(j));  sn = sin(st.hth(j));
            ox = st.hx(j) + ct*p.lat_x - sn*sgn*p.lat_y;
            oy = st.hy(j) + sn*p.lat_x + ct*sgn*p.lat_y;
            dx = -sn*sgn;  dy = ct*sgn;          % R(theta)*(0, sgn)
            r  = ray_cast(c, tr, ox, oy, dx, dy);
            [st.rng, nr] = sim_randn(st.rng);
            ok = 0;
            rm = p.lat_r_max + 0.1;
            if r < 1e3
                rm = r + nr*tr.sig_lat_r;
                if rm >= p.lat_r_min && rm <= p.lat_r_max, ok = 1; end
            end
            st.seq = st.seq + 1;
            q = mod(st.seq - 1, 16) + 1;
            st.ev(q, :) = [round(tm*1e6), i, rm, ok, s_enc];
        end
    end
end
y = [st.seq; st.ev(:)];
end

function r = ray_cast(c, tr, ox, oy, dx, dy)
r = 1e9;
for m = 1:c.n_lm
    px = c.lm_px(m) + tr.lm_err(m, 1);
    py = c.lm_py(m) + tr.lm_err(m, 2);
    if c.lm_type(m) == 1                 % face: o + r d = p + u t
        tx = c.lm_tx(m);  ty = c.lm_ty(m);
        den = dx*(-ty) - dy*(-tx);
        if abs(den) < 1e-9, continue; end
        bx = px - ox;  by = py - oy;
        rr = (bx*(-ty) - by*(-tx))/den;
        u  = (dx*by - dy*bx)/den;
        k  = c.lm_seg(m);
        tsx = c.x1(k) - c.x0(k);  tsy = c.y1(k) - c.y0(k);
        Ls = sqrt(tsx^2 + tsy^2);
        sp = ((px - c.x0(k))*tsx + (py - c.y0(k))*tsy)/Ls;
        if rr > 0 && u >= c.lm_from(m) - 0.3 - sp && u <= c.lm_to(m) + 0.3 - sp && rr < r
            r = rr;
        end
    else                                 % post: |o + r d - C| = r_post
        bx = ox - px;  by = oy - py;
        bb = bx*dx + by*dy;
        cc = bx*bx + by*by - c.lm_r(m)^2;
        disc = bb*bb - cc;
        if disc >= 0
            rr = -bb - sqrt(disc);
            if rr > 0 && rr < r, r = rr; end
        end
    end
end
end
