function gs = sim_gdn_init(p, c)
%SIM_GDN_INIT  State of mission + guidance + drv_vel limits (S_CTRL, 50 Hz).
%   States: 1 INIT 2 DRIVE 3 FINAL 4 PRETURN 5 TURN 6 HOLD 7 FINISHED 8 OPENLOOP
%#codegen
gs.state = 1;
if c.mode == 1, gs.state = 8; end
gs.seg    = 1;
gs.t      = 0;              % mission time
gs.t_st   = 0;              % time in state
gs.t_still = 0;             % time with EKF still
gs.v_prev = 0;              % nav speed profile memory
gs.v = 0;  gs.w = 0;        % drv_vel output after limits
gs.idle = 1;
gs.fin_D = 0;  gs.fin_s0 = 0;  gs.fin_v0 = 0;
gs.tu_th0 = 0;  gs.tu_D = 0;  gs.tu_sgn = 1;  gs.tu_T = 0;  gs.tu_ok_t = 0;
gs.hold_on = 0;  gs.t_hold = 0;
gs.wp_flag = 0;             % waypoint id at the instant the hold timer starts
gs.done_seg = 0;
gs.dt = 0.02;
gs.b = p.odo_wheelbase;
end
