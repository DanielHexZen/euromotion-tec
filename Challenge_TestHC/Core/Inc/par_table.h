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
    X(115, whl_tau_m,        0.050f,    "s",       0.0f,    0.5f,   PAR_L) \
    X(116, whl_ks_l,         0.098f,    "-",       0.0f,    0.5f,   PAR_L) \
    X(117, whl_kv_l,         1.275f,    "1/(m/s)", 0.5f,    3.0f,   PAR_L) \
    X(118, whl_ks_r,         0.041f,    "-",       0.0f,    0.5f,   PAR_L) \
    X(119, whl_kv_r,         1.361f,    "1/(m/s)", 0.5f,    3.0f,   PAR_L) \
    X(120, whl_ksb_l,        0.141f,    "-",       0.0f,    0.6f,   PAR_L) \
    X(121, whl_ksb_r,        0.169f,    "-",       0.0f,    0.6f,   PAR_L) \
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
    X(200, odo_m_per_cnt,    6.3638e-5f,"m",       5.0e-5f, 8.0e-5f,PAR_S) \
    X(201, odo_scale_l,      1.0f,      "-",       0.95f,   1.05f,  PAR_S) \
    X(202, odo_scale_r,      1.0f,      "-",       0.95f,   1.05f,  PAR_S) \
    X(203, odo_wheelbase,    0.22325f,  "m",       0.15f,   0.35f,  PAR_S) \
    X(300, bat_ratio,        5.7f,      "-",       4.0f,    8.0f,   PAR_S) \
    X(301, bat_tau,          0.2f,      "s",       0.05f,   5.0f,   PAR_L) \
    X(302, bat_warn_v,       10.5f,     "V",       9.5f,    12.0f,  PAR_L) \
    X(303, bat_cut_v,        10.0f,     "V",       9.5f,    11.5f,  PAR_L) \
    /* 10.5 communication */ \
    X(904, con_deadman_s,    2.0f,      "s",       0.5f,    10.0f,  PAR_L)

#endif
