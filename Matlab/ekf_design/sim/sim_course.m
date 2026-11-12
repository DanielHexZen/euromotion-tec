function c = sim_course(name)
%SIM_COURSE  Course data in fixed-size arrays (map.h of spec §7.18, simplified).
%
%   c = sim_course(name), name one of
%     'rect10'     10 x 10 m rectangle of §7.18.5: line to (10,0), waypoint 1,
%                  turn to 90 deg, three legs with the grass boundary on the
%                  right at 0.20 m (placement rule §7.18.3), waypoints 2 and 3,
%                  finish at the start. Holds 10.5 s. Leg 1 has no boundary, as
%                  in the YAML example.
%     'rect10_lm'  as rect10 plus the face and post landmarks of the YAML
%                  example on leg 2 (face 0.8 m left, post 0.6 m right).
%     'rect10_all' as rect10 with the right boundary also on leg 1.
%     'rect10_post' as rect10 plus one post 0.6 m to the right of every leg,
%                  1.5 m before its end (an along-track reference near each
%                  waypoint; survey sigma 0.03 m as in the YAML example, r = 0.10 m:
%                  sim_lat casts a ray without spot width, so the r = 0.05 m
%                  post of the example is seen 0.10 m wide and fails the
%                  width check of lat_step, 0.5*(2 r + spot)).
%     'line40'     t_ekf_line: 40 m straight with the right boundary, then a
%                  waypoint and finish.
%     'maneuver'   open-loop command schedule for the 50/200 Hz comparison:
%                  straight, arcs at nav_w_max = 1.0 rad/s, in-place turns at
%                  nav_turn_w = 0.8 rad/s and at 1.5 rad/s.
%
%   Segment types: 1 LINE, 3 TURN, 4 WAYPOINT, 6 FINISH (ARC and PAUSE are
%   not needed by these courses). Boundary codes: 0 none, 1 grass.
%   flags bit 1 = STOP_AT_END (set automatically before WAYPOINT/TURN/FINISH).
%   This builder is not codegen code; the struct it returns is.

NS = 16;  NL = 4;  NO = 16;
z  = zeros(1, NS);
c.mode = 0;
c.start = [0 0 0];
c.n_seg = 0;
c.type = z; c.x0 = z; c.y0 = z; c.x1 = z; c.y1 = z; c.th = z; c.hold = z; c.wp_id = z;
c.vmax = z; c.wl = z; c.wr = z; c.bl = z; c.br = z; c.ref = z; c.flags = z;
zl = zeros(1, NL);
c.n_lm = 0;
c.lm_type = zl; c.lm_seg = zl; c.lm_side = zl; c.lm_from = zl; c.lm_to = zl;
c.lm_px = zl; c.lm_py = zl; c.lm_tx = zl; c.lm_ty = zl; c.lm_r = zl; c.lm_sig = zl;
c.n_ol = 0;
c.ol_t = zeros(1, NO); c.ol_v = zeros(1, NO); c.ol_w = zeros(1, NO);

V = 0.30;  W = 0.60;  WB = 0.20;  H = 10.5;
switch name
    case {'rect10', 'rect10_lm', 'rect10_all', 'rect10_post'}
        b1 = 0;  w1 = W;
        if strcmp(name, 'rect10_all'), b1 = 1; w1 = WB; end
        c = add_line(c, [0 0],   [10 0],  V, W, w1, 0, b1, -1*(b1>0));
        c = add_seg(c, 4, 0, H, 1);
        c = add_seg(c, 3, pi/2, 0, 0);
        c = add_line(c, [10 0],  [10 10], V, W, WB, 0, 1, -1);
        c = add_seg(c, 4, 0, H, 2);
        c = add_seg(c, 3, pi, 0, 0);
        c = add_line(c, [10 10], [0 10],  V, W, WB, 0, 1, -1);
        c = add_seg(c, 4, 0, H, 3);
        c = add_seg(c, 3, -pi/2, 0, 0);
        c = add_line(c, [0 10],  [0 0],   V, W, WB, 0, 1, -1);
        c = add_seg(c, 6, 0, 0, 0);
        if strcmp(name, 'rect10_lm')
            % YAML example of §7.18.5, leg 4 = the second LINE (segment 4 here)
            c = add_lm(c, 1, 4, +1, 2.0, 5.0, 9.2, 3.0, 0, 1, 0, 0.02);
            c = add_lm(c, 2, 4, -1, 6.0, 8.0, 10.6, 7.1, 0, 1, 0.05, 0.03);
        end
        if strcmp(name, 'rect10_post')
            % LINE segments 1, 4, 7, 10; post at s = 8.5 m, window 7.5..9.5 m
            c = add_lm(c, 2,  1, -1, 7.5, 9.5,  8.5, -0.6, 1, 0, 0.10, 0.03);
            c = add_lm(c, 2,  4, -1, 7.5, 9.5, 10.6,  8.5, 0, 1, 0.10, 0.03);
            c = add_lm(c, 2,  7, -1, 7.5, 9.5,  1.5, 10.6, -1, 0, 0.10, 0.03);
            c = add_lm(c, 2, 10, -1, 7.5, 9.5, -0.6,  1.5, 0, -1, 0.10, 0.03);
        end
    case 'line40'
        c = add_line(c, [0 0], [40 0], V, W, WB, 0, 1, -1);
        c = add_seg(c, 4, 0, H, 1);
        c = add_seg(c, 6, 0, 0, 0);
    case 'maneuver'
        c.mode = 1;
        % [duration v w]; times are cumulative ends
        tab = [ 5.0   0.0  0.0     % standstill: ZUPT
               20.0   0.3  0.0     % straight 6 m
               4*pi   0.3  1.0     % two circles at nav_w_max (R = 0.3 m)
               10.0   0.3  0.0     % straight
                3.0   0.0  0.0
               4*pi/0.8 0.0 0.8    % two in-place turns at nav_turn_w
                3.0   0.0  0.0
               4*pi/1.5 0.0 1.5    % two in-place turns at msn_man_w_max
                3.0   0.0  0.0];
        n = size(tab, 1);
        c.n_ol = n;
        c.ol_t(1:n) = cumsum(tab(:,1))';
        c.ol_v(1:n) = tab(:,2)';
        c.ol_w(1:n) = tab(:,3)';
    otherwise
        error('sim_course: unknown course %s', name);
end
end

function c = add_line(c, p0, p1, v, wl, wr, bl, br, ref)
k = c.n_seg + 1;
c.n_seg = k;
c.type(k) = 1;
c.x0(k) = p0(1); c.y0(k) = p0(2); c.x1(k) = p1(1); c.y1(k) = p1(2);
c.vmax(k) = v; c.wl(k) = wl; c.wr(k) = wr; c.bl(k) = bl; c.br(k) = br; c.ref(k) = ref;
end

function c = add_seg(c, type, th, hold, id)
k = c.n_seg + 1;
c.n_seg = k;
c.type(k) = type; c.th(k) = th; c.hold(k) = hold; c.wp_id(k) = id;
if k > 1 && any(type == [3 4 6])
    c.flags(k-1) = bitor(c.flags(k-1), 1);     % STOP_AT_END
end
end

function c = add_lm(c, type, seg, side, s0, s1, px, py, tx, ty, r, sig)
k = c.n_lm + 1;
c.n_lm = k;
c.lm_type(k) = type; c.lm_seg(k) = seg; c.lm_side(k) = side;
c.lm_from(k) = s0; c.lm_to(k) = s1; c.lm_px(k) = px; c.lm_py(k) = py;
c.lm_tx(k) = tx; c.lm_ty(k) = ty; c.lm_r(k) = r; c.lm_sig(k) = sig;
end
