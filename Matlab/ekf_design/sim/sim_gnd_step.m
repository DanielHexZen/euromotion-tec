function [st, y] = sim_gnd_step(st, yp, tr, c, p)
%SIM_GND_STEP  1 kHz: ground-sensor physics, timing, debounce and edge events.
%
%   Physical mode (tr.gnd_mode = 1), per sensor:
%     - continuous ranging, window vl_tb_ms, period vl_imp_ms (50 Hz), sensors staggered
%     - data ready at the window end, seen at the next poll (vl_poll_ms, round
%       robin), result available 1 ms later (I2C read)
%     - footprint at the window middle; raw class GRASS if the footprint plus
%       spot noise N(0, sig_spot) lies beyond the TRUE boundary = mapped line
%       + raggedness (8 sinusoids, sigma sig_wobble) + bnd_offset
%     - timestamp REQ-VL-03: t_meas = t_seen - poll/2 - tb/2
%     - debounce gnd_n_deb: class published after n consecutive samples, event
%       timestamp = t_meas of the FIRST of them (§7.14 step 4); every published
%       PATH<->GRASS change is an event (REQ-GND-03)
%   Periodic mode (tr.gnd_mode = 2, t_ekf_line): every gnd_period_m of true
%   travel one event of the reference-side sensors (inner/outer alternating);
%   it carries its own line offset d = n'f_true + N(0, sig_edge_true), i.e. a
%   perfectly surveyed boundary feature under the footprint. Timestamp error
%   +-poll/2, delivery after tb/2 + poll + 1 ms + (n_deb-1)*imp.
%
%   y = [seq; ev(:)] with ev = 16 x [t_us id from to d_ovr has_ovr] (ring)
%#codegen
t = yp(1);
k = yp(13);
ix = mod(k, 64) + 1;
st.hx(ix) = yp(2);  st.hy(ix) = yp(3);  st.hth(ix) = yp(4);

if tr.gnd_mode == 1
    for i = 1:4
        if st.pend(i) > 0 && t >= st.t_deliv(i) - 1e-9
            st.pend(i) = 0;
            st = debounce(st, i, st.s_cls(i), st.s_tmeas(i), p.gnd_n_deb);
        end
        if t >= st.t_end(i) - 1e-9
            te   = st.t_end(i);
            tmid = te - st.tb/2;
            km   = round(tmid*1000);
            if km < 0, km = 0; end
            if km > k, km = k; end
            j  = mod(km, 64) + 1;
            ct = cos(st.hth(j));  sn = sin(st.hth(j));
            fx = st.hx(j) + ct*st.rs(1,i) - sn*st.rs(2,i);
            fy = st.hy(j) + sn*st.rs(1,i) + ct*st.rs(2,i);
            [st.rng, ns] = sim_randn(st.rng);
            cls = truth_class(st, c, tr, fx, fy, ns*tr.sig_spot);
            tseen = st.poll_off(i) + ceil((te - st.poll_off(i))/st.poll - 1e-9)*st.poll;
            st.pend(i)    = 1;
            st.t_deliv(i) = tseen + 0.001;
            st.s_cls(i)   = cls;
            st.s_tmeas(i) = tseen - st.poll/2 - st.tb/2;
            st.t_end(i)   = te + st.imp;
            st.n_samples  = st.n_samples + 1;
        end
    end
elseif tr.gnd_mode == 2
    dxy = sqrt((yp(2) - st.lastx)^2 + (yp(3) - st.lasty)^2);
    st.lastx = yp(2);  st.lasty = yp(3);
    st.s_trav = st.s_trav + dxy;
    if st.s_trav >= st.next_s
        st.next_s = st.next_s + tr.gnd_period_m;
        [kseg, side] = ref_segment(c, yp(2), yp(3));
        if kseg > 0
            if side > 0, id = 1 + st.alt; else, id = 4 - st.alt; end
            st.alt = 1 - st.alt;
            [ln, ok] = map_boundary(c, kseg, side);
            if ok > 0
                ct = cos(yp(4));  sn = sin(yp(4));
                fx = yp(2) + ct*st.rs(1,id) - sn*st.rs(2,id);
                fy = yp(3) + sn*st.rs(1,id) + ct*st.rs(2,id);
                [st.rng, ne] = sim_randn(st.rng);
                [st.rng, ut] = sim_rand(st.rng);
                [st.rng, ud] = sim_rand(st.rng);
                dov = ln(1)*fx + ln(2)*fy + ne*tr.sig_edge_true;
                tm  = t + (ut - 0.5)*st.poll;
                td  = t + st.tb/2 + ud*st.poll + 0.001 + (p.gnd_n_deb - 1)*st.imp;
                for q = 1:4
                    if st.pq(q, 1) == 0
                        st.pq(q, :) = [1, td, round(tm*1e6), id, dov, 0, 0];
                        break;
                    end
                end
            end
        end
    end
    for q = 1:4
        if st.pq(q, 1) > 0 && t >= st.pq(q, 2)
            st = push_event(st, st.pq(q, 3), st.pq(q, 4), 0, 1, st.pq(q, 5), 1);
            st.pq(q, 1) = 0;
        end
    end
end
y = [st.seq; st.ev(:)];
end

% ------------------------------------------------------------------------
function st = debounce(st, i, cls, tm, ndeb)
if st.pub(i) < 0
    st.pub(i) = cls;                    % first sample: publish without event
    return;
end
if cls == st.pub(i)
    st.cand_n(i) = 0;
    return;
end
if st.cand_n(i) > 0 && cls == st.cand(i)
    st.cand_n(i) = st.cand_n(i) + 1;
else
    st.cand(i)   = cls;
    st.cand_n(i) = 1;
    st.cand_t(i) = tm;
end
if st.cand_n(i) >= ndeb
    st = push_event(st, round(st.cand_t(i)*1e6), i, st.pub(i), cls, 0, 0);
    st.pub(i)    = cls;
    st.cand_n(i) = 0;
end
end

function st = push_event(st, t_us, id, from, to, dov, has)
st.seq = st.seq + 1;
j = mod(st.seq - 1, 16) + 1;
st.ev(j, :) = [t_us, id, from, to, dov, has];
end

function cls = truth_class(st, c, tr, fx, fy, nspot)
% 0 PATH, 1 GRASS. Grass band 1.5 m wide beyond the true boundary.
cls = 0;
for k = 1:c.n_seg
    if c.type(k) ~= 1, continue; end
    [s, L, ey] = map_line_frame(c, k, fx, fy, 0);
    if s < 0 || s > L, continue; end
    if c.br(k) > 0
        w = c.wr(k) + tr.bnd_offset + wobble(st, k, 2, s);
        if ey + nspot < -w && ey > -w - 1.5, cls = 1; end
    end
    if c.bl(k) > 0
        w = c.wl(k) + tr.bnd_offset + wobble(st, k, 1, s);
        if ey - nspot > w && ey < w + 1.5, cls = 1; end
    end
end
end

function w = wobble(st, k, sd, s)
w = 0;
for j = 1:8
    w = w + st.wob_a*sin(2*pi*s/st.wob_l(k, sd, j) + st.wob_ph(k, sd, j));
end
end

function [kseg, side] = ref_segment(c, x, y)
% LINE segment with a reference side that contains (x, y)
kseg = 0;  side = 0;
for k = 1:c.n_seg
    if c.type(k) ~= 1 || c.ref(k) == 0, continue; end
    [s, L, ey] = map_line_frame(c, k, x, y, 0);
    if s >= 0 && s <= L && abs(ey) < 1.0
        kseg = k;  side = c.ref(k);
        return;
    end
end
end
