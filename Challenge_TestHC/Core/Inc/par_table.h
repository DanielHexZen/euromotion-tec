/* par_table.h — the single parameter list (REQ-PAR-01/02, spec §10).
 * X(id, name, default, unit, min, max, flags)   flags: PAR_L live, PAR_S stationary only.
 * IDs are never reused (REQ-PAR-05). Groups join this list as their phase arrives. */
#ifndef PAR_TABLE_H
#define PAR_TABLE_H

#define PAR_L 1U
#define PAR_S 2U

#define PAR_TABLE(X) \
    /* 10.1 actuation and wheel control */ \
    X(100, mot_d_max,        1.0f,      "-",       0.5f,    1.0f,   PAR_L) \
    X(101, mot_d_min,        0.0f,      "-",       0.0f,    0.2f,   PAR_L) \
    X(102, mot_slew_ol,      2.0f,      "1/s",     0.5f,    10.0f,  PAR_L) \
    X(110, whl_kp_v,         0.45f,     "1/(m/s)", 0.0f,    5.0f,   PAR_L) \
    X(111, whl_ki_v,         8.98f,     "1/m",     0.0f,    100.0f, PAR_L) \
    X(112, whl_i_max,        0.30f,     "-",       0.0f,    1.0f,   PAR_L) \
    X(113, whl_kp_x,         2.0f,      "1/s",     0.0f,    10.0f,  PAR_L) \
    X(114, whl_v_corr_max,   0.10f,     "m/s",     0.0f,    0.3f,   PAR_L) \
    X(115, whl_tau_m,        0.04975f,   "s",       0.0f,    0.5f,   PAR_L) \
    X(116, whl_ks_l,         0.0524f,   "-",       0.0f,    0.5f,   PAR_L) \
    X(117, whl_kv_l,         1.3605f,   "1/(m/s)", 0.5f,    3.0f,   PAR_L) \
    X(118, whl_ks_r,         0.0612f,   "-",       0.0f,    0.5f,   PAR_L) \
    X(119, whl_kv_r,         1.3077f,   "1/(m/s)", 0.5f,    3.0f,   PAR_L) \
    X(120, whl_ksb_l,        0.1100f,   "-",       0.0f,    0.6f,   PAR_L) \
    X(121, whl_ksb_r,        0.1350f,   "-",       0.0f,    0.6f,   PAR_L) \
    X(122, whl_v_moving,     0.008f,    "m/s",     0.002f,  0.05f,  PAR_L) \
    X(123, whl_v_stopped,    0.003f,    "m/s",     0.001f,  0.02f,  PAR_L) \
    X(124, whl_v_zero,       0.002f,    "m/s",     0.0f,    0.01f,  PAR_L) \
    X(125, whl_speed_alpha,  0.5f,      "-",       0.05f,   1.0f,   PAR_L) \
    X(126, whl_fault_time,   0.6f,      "s",       0.2f,    3.0f,   PAR_L) \
    X(127, whl_track_max,    0.04f,     "m",       0.01f,   0.2f,   PAR_L) \
    X(128, whl_ff_norm_en,   0.0f,      "bool",    0.0f,    1.0f,   PAR_S) \
    X(129, whl_ff_u_ref,     12.0f,     "V",       9.0f,    13.0f,  PAR_S) \
    X(150, drv_pos_tol,      0.002f,    "m",       0.0005f, 0.02f,  PAR_L) \
    X(151, drv_v_settle,     0.020f,    "m/s",     0.005f,  0.1f,   PAR_L) \
    X(152, drv_settle_timeout, 1.0f,    "s",       0.2f,    5.0f,   PAR_L) \
    X(153, drv_k_sync,       3.0f,      "1/s",     0.0f,    20.0f,  PAR_L) \
    X(154, drv_v_max,        0.45f,     "m/s",     0.1f,    0.6f,   PAR_L) \
    X(157, drv_a_stop,       0.5f,      "m/s2",    0.1f,    2.0f,   PAR_L) \
    X(161, drv_v_nom,        0.30f,     "m/s",     0.02f,   0.6f,   PAR_L) \
    X(162, drv_t_acc,        1.0f,      "s",       0.05f,   5.0f,   PAR_L) \
    X(163, drv_t_dec,        1.0f,      "s",       0.05f,   5.0f,   PAR_L) \
    /* 10.2 odometry and battery */ \
    X(200, odo_m_per_cnt,    6.35444e-5f,"m",       5.0e-5f, 8.0e-5f,PAR_S) \
    X(201, odo_scale_l,      1.00084f,  "-",       0.95f,   1.05f,  PAR_S) \
    X(202, odo_scale_r,      0.99916f,  "-",       0.95f,   1.05f,  PAR_S) \
    X(203, odo_wheelbase,    0.21926f,  "m",       0.15f,   0.35f,  PAR_S) \
    X(300, bat_ratio,        5.7f,      "-",       4.0f,    8.0f,   PAR_S) \
    X(301, bat_tau,          0.2f,      "s",       0.05f,   5.0f,   PAR_L) \
    X(302, bat_warn_v,       10.5f,     "V",       9.5f,    12.0f,  PAR_L) \
    X(303, bat_cut_v,        10.0f,     "V",       9.5f,    11.5f,  PAR_L) \
    /* 10.2 imu */ \
    X(400, imu_tc1,          0.0f,      "rad/s/K", -1.0e-3f, 1.0e-3f, PAR_S) \
    X(401, imu_tc2,          0.0f,      "rad/s/K2",-1.0e-4f, 1.0e-4f, PAR_S) \
    X(402, imu_gz_scale,     1.0f,      "-",       0.95f,   1.05f,  PAR_S) \
    X(403, imu_timeout,      0.020f,    "s",       0.005f,  0.2f,   PAR_L) \
    X(404, imu_extrap_max,   0.002f,    "s",       0.0f,    0.005f, PAR_L) \
    /* 10.2 attitude filter */ \
    X(410, att_tau,          1.0f,      "s",       0.1f,    10.0f,  PAR_L) \
    X(411, att_acc_gate,     0.1f,      "g",       0.02f,   0.5f,   PAR_L) \
    /* 10.3 estimator */ \
    X(500, ekf_kd,           1.0e-4f,   "m",       1.0e-6f, 1.0e-2f,PAR_L) \
    X(501, ekf_slip_infl,    100.0f,    "-",       1.0f,    1.0e4f, PAR_L) \
    X(502, ekf_slip_thr,     0.15f,     "rad/s",   0.02f,   1.0f,   PAR_L) \
    X(503, ekf_gyro_nd,      1.0e-4f,   "rad/s/rtHz", 1.0e-6f, 1.0e-2f, PAR_L) \
    X(504, ekf_gyro_sf,      0.003f,    "-",       0.0f,    0.05f,  PAR_L) \
    X(505, ekf_q_bg,         1.0e-10f,  "(rad/s)2/s", 1.0e-14f, 1.0e-6f, PAR_L) \
    X(506, ekf_q_s,          1.0e-9f,   "1/s",     0.0f,    1.0e-5f,PAR_L) \
    X(507, ekf_q_th_enc,     1.0e-4f,   "rad2/m",  1.0e-6f, 1.0e-2f,PAR_L) \
    X(508, ekf_zupt_floor,   2.0e-4f,   "rad/s",   0.0f,    1.0e-3f,PAR_L) \
    X(509, ekf_zupt_T,       1.0f,      "s",       0.2f,    5.0f,   PAR_L) \
    X(510, ekf_still_s,      0.2f,      "s",       0.05f,   2.0f,   PAR_L) \
    X(511, ekf_gate1,        9.0f,      "-",       1.0f,    100.0f, PAR_L) \
    X(512, ekf_gate2,        9.21f,     "-",       1.0f,    100.0f, PAR_L) \
    X(513, ekf_sig_edge,     0.02f,     "m",       0.005f,  0.2f,   PAR_L) \
    X(514, ekf_sig_post,     0.03f,     "m",       0.005f,  0.3f,   PAR_L) \
    X(515, ekf_edge_skip,    0.25f,     "m",       0.0f,    1.0f,   PAR_L) \
    X(516, ekf_bg_conv,      3.5e-4f,   "rad/s",   1.0e-5f, 1.0e-2f,PAR_L) \
    X(517, ekf_p0_xy,        0.01f,     "m",       1.0e-3f, 1.0f,   PAR_S) \
    X(518, ekf_p0_th,        0.0087f,   "rad",     1.0e-3f, 0.5f,   PAR_S) \
    X(519, ekf_p0_bg,        0.087f,    "rad/s",   1.0e-4f, 0.1f,   PAR_S) \
    X(520, ekf_p0_s,         0.005f,    "-",       1.0e-4f, 0.1f,   PAR_S) \
    X(521, ekf_sf_corr,      6.2832f,   "rad",     0.1f,    100.0f, PAR_L) \
    X(522, ekf_slip_T,       0.04f,     "s",       0.02f,   0.5f,   PAR_L) \
    /* 10.4 sensors (D-70: G1-G4 not fitted, gnd runs on S2) */ \
    X(603, vl_timeout,       0.2f,      "s",       0.05f,   1.0f,   PAR_L) \
    X(614, gnd_d_drop,       0.03f,     "m",       0.01f,   0.1f,   PAR_L) \
    X(615, gnd_d_up,         0.02f,     "m",       0.01f,   0.1f,   PAR_L) \
    X(616, gnd_sd_grass,     0.008f,    "m",       0.001f,  0.05f,  PAR_L) \
    X(617, gnd_sig_grass,    0.0f,      "kcps",    0.0f,    100.0f, PAR_L) \
    X(618, gnd_sd_n,         5.0f,      "-",       3.0f,    15.0f,  PAR_S) \
    X(619, gnd_n_deb,        2.0f,      "-",       1.0f,    5.0f,   PAR_L) \
    X(632, fwd_t_lat,        0.10f,     "s",       0.02f,   0.5f,   PAR_L) \
    X(633, fwd_a_brake,      0.5f,      "m/s2",    0.1f,    3.0f,   PAR_L) \
    X(634, fwd_margin,       0.15f,     "m",       0.05f,   0.5f,   PAR_L) \
    X(635, fwd_slow_range,   1.0f,      "m",       0.3f,    2.0f,   PAR_L) \
    X(636, fwd_v_slow,       0.25f,     "m/s",     0.05f,   0.45f,  PAR_L) \
    X(670, lat_tilt,         0.332f,    "rad",     0.0f,    0.7f,   PAR_S) \
    X(671, lat_h,            0.07f,     "m",       0.03f,   0.4f,   PAR_S) \
    /* 10.5 communication */ \
    X(904, con_deadman_s,    2.0f,      "s",       0.5f,    10.0f,  PAR_L) \
    X(905, wdg_enable,       1.0f,      "bool",    0.0f,    1.0f,   PAR_S)

#endif
