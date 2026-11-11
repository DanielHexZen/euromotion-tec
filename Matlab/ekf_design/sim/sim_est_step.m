function [es, y, yu] = sim_est_step(es, y_enc, y_yaw, y_gnd, y_lat, y_gst, p, c)
%SIM_EST_STEP  app_est: one estimator step (D-30: once per 50 Hz control period).
%
%   Order (as the C code will do it):
%     1. ekf_odo_t from the encoder snapshot of this tick: ds = dcnt*m_per_cnt*scale,
%        dt from the encoder timestamps, dpsi = Psi(t_now) - Psi(t_prev)
%        interpolated at the ENCODER timestamps (imu_yaw_integral_at)
%     2. ekf_still_update (needs drv status of the previous control tick)
%     3. ekf_predict, pose history push
%     4. ZUPT if a window completed
%     5. queued edge events (gnd), each with its own back-shift
%     6. lateral samples -> lat -> heading / face range / point updates
%   Inputs: y_gst = [drv_idle; seg; y_off; state] from guidance, one tick old.
%   Outputs: y  = [ekf_get (58); s_enc; imu_ok; n_skip(5); lat counters(4); spare] (70)
%            yu = update records of this step (8 x 7, column-major)
%#codegen
cl = class(p.ekf_kd);
y  = zeros(70, 1);
r  = sim_yaw_unpack(y_yaw, p);
t  = y_enc(3);
seg = y_gst(2);
e  = es.ekf;
e.n_rec = 0;
e.rec(:) = 0;                      % records of THIS step only
if es.started == 0
    [psi, stp] = imu_yaw_integral_at(r, p, t);
    es.prev_cnt = y_enc(1:2);
    es.prev_t   = t;
    es.prev_psi = psi;
    es.imu_ok   = double(stp == 0);
    es.started  = 1;
    e.t_us = t;
    e = ekf_hist_push(e, p, t);
    es.ekf = e;
    y(1:58) = ekf_get(e);
    yu = e.rec(:);
    return;
end
% 1. odometry increment
dcl = y_enc(1) - es.prev_cnt(1);
dcr = y_enc(2) - es.prev_cnt(2);
ds_l = cast(dcl, cl)*p.odo_m_per_cnt*p.odo_scale_l;
ds_r = cast(dcr, cl)*p.odo_m_per_cnt*p.odo_scale_r;
dt   = cast((t - es.prev_t)*1e-6, cl);
dpsi = cast(NaN, cl);
if es.feat_imu > 0
    [psi, stp] = imu_yaw_integral_at(r, p, t);
    if stp == 0
        if es.imu_ok > 0
            dpsi = psi - es.prev_psi;
        end
        es.prev_psi = psi;
        es.imu_ok = 1;
    else
        es.imu_ok = 0;            % F_IMU_TIMEOUT path: encoder heading for this step
    end
end
% 2. standstill
changed = double(dcl ~= 0 || dcr ~= 0);
[e, zdue, rm, T] = ekf_still_update(e, p, changed, y_gst(1), dpsi, t - es.prev_t);
% 3. prediction
e = ekf_predict(e, p, ds_l, ds_r, dpsi, dt, t, 0);
% 4. ZUPT
if zdue > 0
    [e, ~] = ekf_update_zupt(e, p, rm, T);
end
% 5. edge events
ngs = y_gnd(1);
if ngs - es.gnd_seq > 16
    es.gnd_seq = ngs - 16;        % ring overrun: oldest lost
end
EV = reshape(y_gnd(2:97), 16, 6);
for n = (es.gnd_seq + 1):ngs
    j  = mod(n - 1, 16) + 1;
    id = EV(j, 2);
    side = 1;
    if id >= 3, side = -1; end
    ys = p.gnd_y_in;
    if id == 1 || id == 4, ys = p.gnd_y_out; end
    [ln, okb] = map_boundary(c, seg, side);
    if seg < 1 || seg > c.n_seg || c.type(seg) ~= 1
        es.n_skip(1) = es.n_skip(1) + 1;  continue;
    end
    if okb == 0
        es.n_skip(2) = es.n_skip(2) + 1;  continue;
    end
    [s_al, L] = map_line_frame(c, seg, e.x(1), e.x(2), e.x(3));
    if s_al < p.ekf_edge_skip || s_al > L - p.ekf_edge_skip
        es.n_skip(3) = es.n_skip(3) + 1;  continue;   % REQ-EKF-07
    end
    if EV(j, 6) > 0
        ln(3) = EV(j, 5);                % periodic-mode surveyed feature
    end
    [bk, stb] = ekf_back_at(e, p, EV(j, 1));
    if stb ~= 0
        es.n_skip(4) = es.n_skip(4) + 1;  continue;
    end
    [e, ~] = ekf_update_edge(e, p, cast(ln, cl), cast(p.gnd_x, cl), cast(side*ys, cl), bk);
end
es.gnd_seq = ngs;
% 6. lateral landmarks
nls = y_lat(1);
if nls - es.lat_seq > 16
    es.lat_seq = nls - 16;
end
LV = reshape(y_lat(2:81), 16, 5);
if abs(y_gst(3)) <= p.ekf_lm_y_off_max && seg >= 1 && seg <= c.n_seg && c.type(seg) == 1
    for n = (es.lat_seq + 1):nls
        j = mod(n - 1, 16) + 1;
        s_al = map_line_frame(c, seg, double(e.x(1)), double(e.x(2)), double(e.x(3)));
        [es.lat, out, nout] = lat_step(es.lat, LV(j, :), double(e.x(3)), seg, s_al, c, p);
        for q = 1:nout
            o = out(q, :);
            if o(1) == 1
                [e, ~] = ekf_update_heading(e, p, cast(o(4), cl), cast(o(5), cl));
            elseif o(1) == 2
                m  = o(3);
                ln = [-c.lm_ty(m); c.lm_tx(m); 0];
                ln(3) = ln(1)*c.lm_px(m) + ln(2)*c.lm_py(m);
                sg = 3 - 2*o(6);
                [bk, stb] = ekf_back_at(e, p, o(2));
                if stb == 0
                    [e, ~] = ekf_update_face_range(e, p, cast(ln, cl), cast(p.lat_x, cl), ...
                        cast(sg*p.lat_y, cl), cast(0, cl), cast(sg, cl), cast(o(4), cl), cast(o(5), cl), bk);
                end
            elseif o(1) == 3
                m = o(3);
                [bk, stb] = ekf_back_at(e, p, o(2));
                if stb == 0
                    [e, ~] = ekf_update_point(e, p, cast(c.lm_px(m), cl), cast(c.lm_py(m), cl), ...
                        cast(o(4), cl), cast(o(5), cl), cast(o(6), cl), bk);
                end
            end
        end
    end
end
es.lat_seq = nls;
es.prev_cnt = y_enc(1:2);
es.prev_t   = t;
es.ekf = e;
y(1:58) = ekf_get(e);
y(59) = 0.5*(y_enc(1) + y_enc(2))*double(p.odo_m_per_cnt);
y(60) = es.imu_ok;
y(61:65) = es.n_skip(:);
y(66) = es.lat.n_face;  y(67) = es.lat.n_face_rej;  y(68) = es.lat.n_post;  y(69) = es.lat.n_post_rej;
y(70) = e.t_nochg_us*1e-6;
yu = e.rec(:);
end
