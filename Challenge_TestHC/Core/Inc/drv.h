/* drv.h — drive modes and wheel references (spec §7.9). Pure.
 * Modes: DISABLED, HOLD, OPEN, MOVE, TURN_ENC, VEL (phase 3, host track). */
#ifndef DRV_H
#define DRV_H

#include "prof.h"
#include "sys_fault.h"
#include "units.h"
#include "whl.h"

typedef enum { DRV_DISABLED = 0, DRV_HOLD, DRV_OPEN, DRV_MOVE, DRV_TURN_ENC, DRV_VEL } drv_mode_t;
typedef enum { DRV_PH_IDLE = 0, DRV_PH_MOVE, DRV_PH_SETTLE, DRV_PH_STOPPING } drv_phase_t;
typedef enum { DRV_EV_NONE = 0, DRV_EV_DONE, DRV_EV_TIMEOUT } drv_event_t;

typedef struct {
    int32_t  cnt[2];      /* accumulated encoder counts, forward positive (enc_read) */
    uint32_t t_us;        /* timestamp of that reading                               */
    float    u_bat_v;     /* filtered pack voltage                                   */
} drv_meas_t;

typedef struct {
    float       duty_d[2];  /* to mot_set()                                          */
    bool        enable;     /* EN lines                                              */
    drv_event_t event;      /* set for exactly one step                              */
    fault_t     fault;      /* first drivetrain fault of this step, F_NONE otherwise */
} drv_out_t;

typedef struct {
    drv_mode_t  mode;
    drv_phase_t phase;
    float       sync_max_m;      /* max |L/R error difference| of the current move   */
    float       t_s;             /* time since the move started                      */
    float       d_m[2];          /* wheel displacement since the move started        */
    float       v_mps[2];        /* filtered wheel speed                             */
} drv_status_t;

typedef struct {
    drv_mode_t  mode;
    drv_phase_t phase;
    whl_t       whl[2];
    bool        have_prev;
    uint32_t    t_prev_us;
    float       x_m[2], v_hat[2];
    prof_t      prof;
    float       t_s, settle_s, sync_max_m;
    float       x0[2], sig[2];   /* start position, wheel sign relative to the profile */
    float       x_ref[2], v_ref[2];
    bool        reached[2];
    ramp_t      stop[2];
    float       ol_target[2], ol_cur[2], ol_dwell_s[2];
    int8_t      ol_sign[2];
    ramp_t      vel_v, vel_w;    /* VEL: rate-limited v, omega                       */
    float       vel_v_cmd, vel_w_cmd;
    bool        vel_entry, vel_stop;
} drv_t;

void drv_init(drv_t *d);
/* Straight move; keeps the semantics of the old D<mm> command (REQ-DRV-01). ST_STATE while a
 * move is running, ST_RANGE from prof_plan. Main loop. */
st_t drv_move(drv_t *d, float dist_m, float v_mps, float t_acc_s, float t_dec_s);
/* Straight move starting at speed v0 >= 0 (REQ-NAV-02, §7.19): accelerates to v_mps, stops at dist_m
 * with a_dec, raised up to drv_a_stop if v0 is too high. Allowed from HOLD and VEL (integrators kept
 * from VEL). ST_STATE otherwise, ST_RANGE from prof_plan_v0. */
st_t drv_move_from(drv_t *d, float dist_m, float v0_mps, float v_mps, float a_acc, float a_dec);
/* Turn in place by encoder angle, CCW positive; wheels follow -/+ (b/2) theta. */
st_t drv_turn_enc(drv_t *d, float dtheta_rad, float w_radps, float t_acc_s);
/* Open-loop duty (fraction), slew-limited by mot_slew_ol (REQ-MOT-06). ST_STATE while moving. */
st_t drv_open(drv_t *d, float duty_l, float duty_r);
/* Velocity command (REQ-DRV-03/04/05): call every control tick while in VEL. Limits are applied
 * here, the command is not trusted. A call after drv_stop() cancels the stop. */
void drv_vel(drv_t *d, float v_mps, float w_radps);
void drv_hold(drv_t *d);       /* EN high, duty 0 (brake)                                  */
void drv_stop(drv_t *d);       /* controlled: ramp to rest at drv_a_stop, then HOLD        */
void drv_brake(drv_t *d);      /* immediate: duty 0, EN high, HOLD                         */
void drv_disable(drv_t *d);    /* EN low                                                   */
/* Counts were re-zeroed (enc_zero): forget the previous sample. */
void drv_reset_meas(drv_t *d);
void drv_step(drv_t *d, const drv_meas_t *m, drv_out_t *out);   /* S_CTRL */
drv_status_t drv_status(const drv_t *d);

#endif
