/* console.h — text console (spec §7.23, §9). System layer.
 * con parses lines into SI commands and formats every reply in legacy units (mm, mm/s, %, deg);
 * app_cmd.c executes. No other module sees legacy units (spec §6.4). */
#ifndef CON_H
#define CON_H

#include "com_uart.h"
#include "par.h"

typedef enum {
    CMD_NONE = 0,
    CMD_HELP, CMD_PARAMS, CMD_STOP, CMD_RESET, CMD_PRINT, CMD_AUTOPRINT,
    CMD_MOVE,        /* f[0] = distance [m]                                          */
    CMD_SPEED,       /* f[0] = nominal speed [m/s]                                   */
    CMD_RAMP,        /* f[0], f[1] = accel, decel time [s]                           */
    CMD_FF,          /* which: 0 both, 1 L, 2 R; f[0] = Ks [-], f[1] = Kv [1/(m/s)]  */
    CMD_GAIN,        /* f[0] = Kp_v [1/(m/s)], f[1] = Ki_v [1/m]                     */
    CMD_SYNC,        /* f[0] = K_sync [1/s]                                          */
    CMD_DUTY,        /* f[0] = duty both wheels, fraction                            */
    CMD_PWM,         /* f[0], f[1] = duty L, R                                       */
    CMD_TURN,        /* f[0] = angle [rad], CCW positive                             */
    CMD_MODE,        /* which: 0 IDLE, 1 BENCH                                       */
    CMD_UNAVAILABLE, /* valid syntax, not in this build (MANUAL, AUTO, RUN, M)       */
    CMD_CLEAR, CMD_PSET, CMD_PGET, CMD_DUMP, CMD_STATUS, CMD_INFO, CMD_EVLOG, CMD_FAULTS,
    CMD_FAULTINJ,    /* f[0] = fault code (Debug builds only)                        */
    CMD_IMU, CMD_IMUS, CMD_EKF
} con_cmd_type_t;

typedef struct {
    con_cmd_type_t type;
    com_port_t     port;
    uint8_t        which;
    float          f[2];
    const par_desc_t *par;      /* CMD_PSET / CMD_PGET */
} con_cmd_t;

typedef struct {
    int32_t     cnt[2];
    float       x_m[2], v_mps[2];
    const char *mode;           /* "IDLE", "BENCH", "FAULT" */
    const char *phase;          /* drv state                */
} con_status_t;

typedef enum {
    CON_JOB_NONE = 0, CON_JOB_HELP, CON_JOB_PARAMS, CON_JOB_DUMP, CON_JOB_EVLOG, CON_JOB_FAULTS, CON_JOB_INFO,
    CON_JOB_IMU, CON_JOB_EKF
} con_job_t;

/* Snapshots for IMU and EKF, SI; console.c formats them in legacy units. */
#define CON_EKF_NU 5U                                   /* update types: ZUPT, edge, face, heading, point */
typedef struct {
    bool     present, alive;
    float    w_radps[3], temp_c, roll_rad, pitch_rad, bg_radps;
    uint32_t n_samples, n_drop, n_sat;
} con_imu_t;
typedef struct {
    float    x_m, y_m, th_rad, bg_radps, s;
    float    sig_x_m, sig_y_m, sig_th_rad, sig_bg_radps;
    float    nis_mean[CON_EKF_NU];
    uint32_t n_acc[CON_EKF_NU], n_rej[CON_EKF_NU], slip_episodes, step_us;
    bool     still, slip, degraded;
} con_ekf_t;

void     con_rx_byte(com_port_t p, uint8_t b);          /* main loop, from com_uart_poll */
/* Main loop, every pass: advances chunked output, executes at most one pending line. True when
 * a parsed command is in *cmd. Parse errors are answered here and return false. */
bool     con_poll(con_cmd_t *cmd);
uint32_t con_last_rx_ms(com_port_t p);                  /* REQ-CON-03 dead-man */
bool     con_autoprint_toggle(com_port_t p);            /* returns the new state */
bool     con_autoprint(com_port_t p);

void con_printf(com_port_t p, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
void con_job(com_port_t p, con_job_t j);
/* Line i of the `I` report, -1 past the end. Defined by the application (app_cmd.c), which owns the data. */
int  con_info_line(uint32_t i, char *buf, size_t n);
/* Defined by the application, called for every line of the IMU / EKF job. */
void con_imu_snapshot(con_imu_t *s);
void con_ekf_snapshot(con_ekf_t *s);
void con_err_state(com_port_t p, const char *mode);
void con_err_range_par(com_port_t p, const par_desc_t *d);
void con_print_status(com_port_t p, const con_status_t *s);
void con_print_move(com_port_t p, float dist_m, float v_mps, float t_s);
void con_print_done(com_port_t p, bool timeout, const float d_m[2], float sync_m, float t_s);
void con_print_param(com_port_t p, const par_desc_t *d, float v);
void con_print_turn(com_port_t p, float achieved_rad, float target_rad);   /* W with gyro check */
/* IMUS stream line (H7, D-74): "IMUS <t_us> <n> <wx> <wy> <wz> <T>", rates in mdps, T in 0.01 C. */
void con_print_imus(com_port_t p, uint32_t t_us, uint32_t n_total, const float w_radps[3], float temp_c);

#endif
