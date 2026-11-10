function [gs, y] = sim_gdn_step(gs, y_est, p, c)
%SIM_GDN_STEP  msn_step -> nav_* -> drv_vel, once per 50 Hz control period.
%
%   Closes the loop on the ESTIMATED pose, as on the vehicle, so estimation
%   errors feed back into the driven path.
%   - nav_follow (REQ-NAV-01): speed profile, k_psi = 2 zeta wn, k_y = wn^2/max(|v|, v_min),
%     omega = kappa v - k_psi e_psi - k_y e_y, |omega| <= nav_w_max
%   - final stop (REQ-NAV-02): MOVE_FROM(D = L - s, v0) frozen in the ENCODER frame
%     (simplified to a sqrt(2 a d) speed profile on the encoder distance, omega = 0)
%   - nav_turn (REQ-NAV-03): trapezoidal angle profile nav_turn_w / nav_turn_ta,
%     omega = theta_dot_prof + k*wrap(theta0 + theta_prof - theta_hat), no reversal
%     inside 0.5 deg, done at |err| < nav_turn_tol and |w_meas| < 0.02 for 0.2 s
%     or 2 s after the profile
%   - mission (§7.21.3, reduced): INIT (converged + 3 s still), DRIVE, FINAL,
%     HOLD (timer starts with EKF still), PRETURN (still + 1 s), TURN, FINISHED
%   - drv_vel limits (REQ-DRV-03) in the stated order
%   The wheel PI/position cascade is not modelled: the plant's first-order lag
%   stands for the closed speed loop.
%
%   y = [refL; refR; v; w; drv_idle; seg; y_off; state; wp_flag; t]   (10)
%#codegen
dt = gs.dt;
gs.t = gs.t + dt;
gs.t_st = gs.t_st + dt;
x  = y_est(1);  yy = y_est(2);  th = y_est(3);
still = y_est(12);
w_meas = y_est(15);
s_enc = y_est(59);
if still > 0, gs.t_still = gs.t_still + dt; else, gs.t_still = 0; end
gs.wp_flag = 0;
v = 0;  w = 0;  idle = 1;

for it = 1:3                                  % allow immediate segment dispatch
    st0 = gs.state;
    switch gs.state
        case 1   % INIT
            conv = sqrt(max(y_est(9), 0)) < p.ekf_bg_conv;
            if conv && gs.t_still >= p.msn_init_min_s
                gs = enter_seg(gs, c, 1, th);
            end
        case 2   % DRIVE (LINE)
            k = gs.seg;
            [s, L, ey, epsi] = map_line_frame(c, k, x, yy, th);
            v_end = 0;
            if bitand(c.flags(k), 1) == 0 && k < c.n_seg, v_end = c.vmax(k+1); end
            vr = min([c.vmax(k), sqrt(v_end^2 + 2*p.nav_a_dec*max(L - s, 0)), gs.v_prev + p.nav_a_acc*dt]);
            gs.v_prev = vr;
            kpsi = 2*p.nav_zeta*p.nav_wn;
            ky   = p.nav_wn^2/max(abs(vr), p.nav_v_min);
            w = -kpsi*epsi - ky*ey;
            w = max(-p.nav_w_max, min(p.nav_w_max, w));
            v = vr;  idle = 0;
            if bitand(c.flags(k), 1) > 0
                aligned = abs(ey) < 0.05 && abs(epsi) < 3*pi/180;
                if (L - s <= p.nav_final_dist && aligned) || L - s <= 0.10
                    gs.state = 3;  gs.t_st = 0;
                    gs.fin_D = L - s;  gs.fin_s0 = s_enc;  gs.fin_v0 = vr;
                end
            elseif s >= L
                gs = enter_seg(gs, c, k + 1, th);
            end
        case 3   % FINAL (encoder frame)
            drem = gs.fin_D - (s_enc - gs.fin_s0);
            if drem <= p.drv_pos_tol
                v = 0;  w = 0;  idle = 1;
                gs.v = 0;  gs.w = 0;  gs.v_prev = 0;
                gs = enter_seg(gs, c, gs.seg + 1, th);
            else
                v = min(gs.fin_v0, sqrt(2*p.nav_a_dec*drem));
                v = max(v, 0.02);  w = 0;  idle = 0;
            end
        case 4   % PRETURN
            if gs.t_still >= p.msn_preturn_s
                gs.state = 5;  gs.t_st = 0;
                D = ekf_wrap(c.th(gs.seg) - th);
                gs.tu_th0 = th;  gs.tu_D = abs(D);  gs.tu_sgn = sign(D);
                if gs.tu_sgn == 0, gs.tu_sgn = 1; end
                [~, ~, gs.tu_T] = turn_prof(gs.tu_D, 0, p);
                gs.tu_ok_t = 0;
            end
        case 5   % TURN
            [tp, wp] = turn_prof(gs.tu_D, gs.t_st, p);
            err = ekf_wrap(gs.tu_th0 + gs.tu_sgn*tp - th);
            w = gs.tu_sgn*wp + p.nav_turn_k*err;
            rem = ekf_wrap(c.th(gs.seg) - th);
            if abs(rem) < 0.5*pi/180 && sign(w) ~= sign(rem)
                w = 0;                              % no reversal inside 0.5 deg
            end
            idle = 0;
            if abs(rem) < p.nav_turn_tol && abs(w_meas) < 0.02
                gs.tu_ok_t = gs.tu_ok_t + dt;
            else
                gs.tu_ok_t = 0;
            end
            if (gs.tu_ok_t >= 0.2 && gs.t_st > gs.tu_T) || gs.t_st > gs.tu_T + 2
                w = 0;  idle = 1;  gs.w = 0;
                gs = enter_seg(gs, c, gs.seg + 1, th);
            end
        case 6   % HOLD
            if gs.hold_on == 0 && still > 0
                gs.hold_on = 1;  gs.t_hold = 0;
                gs.wp_flag = c.wp_id(gs.seg);
            end
            if gs.hold_on > 0
                gs.t_hold = gs.t_hold + dt;
                if gs.t_hold >= c.hold(gs.seg)
                    gs = enter_seg(gs, c, gs.seg + 1, th);
                end
            end
        case 7   % FINISHED
            if gs.hold_on == 0 && still > 0
                gs.hold_on = 1;  gs.wp_flag = 99;
            end
        case 8   % OPENLOOP
            k = 1;
            while k < c.n_ol && gs.t > c.ol_t(k)
                k = k + 1;
            end
            v = c.ol_v(k);  w = c.ol_w(k);
            if gs.t > c.ol_t(c.n_ol), v = 0; w = 0; end
            idle = double(v == 0 && w == 0 && abs(gs.v) < 1e-9 && abs(gs.w) < 1e-9);
    end
    if gs.state == st0 || gs.state == 7
        break;
    end
end

% drv_vel limits (REQ-DRV-03), in this order
v = max(-p.drv_v_max, min(p.drv_v_max, v));
w = max(-p.drv_w_max, min(p.drv_w_max, w));
if gs.state == 3 || gs.state == 6 || gs.state == 7 || (gs.state == 2 && idle > 0)
    gs.v = v;                                   % MOVE profile / HOLD: no VEL ramp
else
    if abs(v) > abs(gs.v), a = p.drv_a_max; else, a = p.drv_a_stop; end
    gs.v = gs.v + max(-a*dt, min(a*dt, v - gs.v));
end
gs.w = gs.w + max(-p.drv_alpha_max*dt, min(p.drv_alpha_max*dt, w - gs.w));
if idle > 0 && gs.state ~= 8, gs.v = 0; gs.w = 0; end
vw = abs(gs.v) + abs(gs.w)*gs.b/2;
if vw > p.drv_v_whl_max
    gs.v = sign(gs.v)*max(p.drv_v_whl_max - abs(gs.w)*gs.b/2, 0);
end
gs.idle = idle;
refL = gs.v - gs.w*gs.b/2;
refR = gs.v + gs.w*gs.b/2;
y = [refL; refR; gs.v; gs.w; idle; gs.seg; 0; gs.state; gs.wp_flag; gs.t];
end

function gs = enter_seg(gs, c, k, ~)
gs.seg = k;  gs.t_st = 0;  gs.hold_on = 0;  gs.v_prev = 0;
if k > c.n_seg
    gs.state = 7;  return;
end
switch c.type(k)
    case 1, gs.state = 2;
    case 3, gs.state = 4;
    case 4, gs.state = 6;
    otherwise, gs.state = 7;
end
end

function [thp, wp, T] = turn_prof(D, t, p)
% trapezoid (or triangle) on |angle| D with peak rate nav_turn_w, accel nav_turn_w/nav_turn_ta
wm = p.nav_turn_w;  a = wm/p.nav_turn_ta;
if D >= wm*wm/a
    ta = wm/a;  tc = (D - wm*wm/a)/wm;
else
    wm = sqrt(D*a);  ta = wm/a;  tc = 0;
end
T = 2*ta + tc;
if t <= 0
    thp = 0;  wp = 0;
elseif t < ta
    thp = 0.5*a*t*t;  wp = a*t;
elseif t < ta + tc
    thp = 0.5*a*ta*ta + wm*(t - ta);  wp = wm;
elseif t < T
    tr = T - t;
    thp = D - 0.5*a*tr*tr;  wp = a*tr;
else
    thp = D;  wp = 0;
end
end
