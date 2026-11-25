/* msn.h — modes and mission state machine (spec §7.21, REQ-MSN-01..06). Pure.
 * AUTO without evasion: AUTO_EVADE and the evd planning in AUTO_OBST_WAIT arrive with phase 6.
 * msn does not call drv: msn_step() fills msn_out_t and the caller applies it (app, vehicle track). */
#ifndef MSN_H
#define MSN_H

#include "drv.h"
#include "ekf.h"
#include "gnd.h"
#include "map.h"
#include "nav.h"
#include "sys_fault.h"

typedef enum { MSN_IDLE, MSN_BENCH, MSN_MANUAL, MSN_AUTO, MSN_FAULT } msn_mode_t;
typedef enum { MSN_REQ_NONE, MSN_REQ_IDLE, MSN_REQ_BENCH, MSN_REQ_MANUAL, MSN_REQ_AUTO, MSN_REQ_STOP,
               MSN_REQ_CLEAR } msn_req_t;
typedef enum {
    MSN_AUTO_NONE,                 /* not in AUTO */
    MSN_AUTO_INIT, MSN_AUTO_SEG, MSN_AUTO_DRIVE, MSN_AUTO_FINAL, MSN_AUTO_PRETURN, MSN_AUTO_TURN,
    MSN_AUTO_HOLD, MSN_AUTO_PAUSE, MSN_AUTO_OBST_STOP, MSN_AUTO_OBST_WAIT, MSN_AUTO_RECOVER,
    MSN_AUTO_STOPPED, MSN_AUTO_FINISHED, MSN_AUTO_N
} msn_state_t;

/* first unmet AUTO start condition (REQ-MSN-02) */
typedef enum { MSN_START_OK, MSN_REF_COURSE, MSN_REF_FAULT, MSN_REF_BATTERY, MSN_REF_IMU,
               MSN_REF_SENSORS, MSN_REF_MOVING } msn_refuse_t;

typedef struct {
    uint32_t    t_ms;
    ekf_out_t   est;               /* ekf_get(); est.flags & EKF_STILL = stationary             */
    float       sig_perp;          /* EKF cross-track sigma on the current segment (caller)     */
    float       s_enc;             /* mean wheel travel, m                                      */
    gnd_side_t  gnd_l, gnd_r;      /* gnd_side()                                                */
    bool        interlock;         /* gnd_interlock(): false today (D-70, OQ-12)                */
    bool        reflex, fwd_stale; /* fwd_reflex(), fwd_stale()                                 */
    float       v_limit;           /* fwd_v_limit(), +inf = none                                */
    drv_event_t drv_ev;            /* event of this tick, DRV_EV_NONE otherwise                 */
    float       u_bat_v;
    reaction_t  fault_reaction;    /* sys_fault_reaction()                                      */
    uint32_t    fault_mask;        /* sys_fault_mask(), 0 = no active fault                     */
    bool        feat_imu, imu_alive;       /* FEAT_IMU, IMU delivering                          */
    bool        feat_vl, sensors_fresh;    /* FEAT_VL, F1, F2 (and S2) fresh                    */
} msn_in_t;

typedef enum { MSN_EV_STATE = 1, MSN_EV_MODE, MSN_EV_WP_ARRIVED, MSN_EV_WP_DONE, MSN_EV_FINISHED,
               MSN_EV_MAN_TIMEOUT, MSN_EV_OBSTACLE, MSN_EV_OBST_WAIT, MSN_EV_START_REFUSED,
               MSN_EV_FINAL_DONE } msn_ev_t;
/* STATE/MODE: a old, b new. WP_ARRIVED: a id, x/y m, z sigma m. WP_DONE: a id. FINISHED: x total s.
 * START_REFUSED: a msn_refuse_t. FINAL_DONE: a drv_event_t, x EKF miss L - s (m). */
typedef struct { msn_ev_t code; int32_t a, b; float x, y, z; } msn_event_t;
#define MSN_EV_N 8U

/* One command per tick, applied by the caller. VEL is repeated every tick while in VEL (drv_vel);
 * every other command is issued once (drv_move_from, drv_move, drv_turn_enc, drv_stop, drv_brake,
 * drv_hold, drv_disable). */
typedef enum { MC_NONE, MC_VEL, MC_MOVE_FROM, MC_MOVE, MC_TURN_ENC, MC_STOP, MC_BRAKE, MC_HOLD,
               MC_DISABLE } msn_cmd_t;

typedef struct {
    msn_cmd_t cmd;
    float     v, w;            /* VEL; MOVE_FROM/MOVE: v cruise; TURN_ENC: w                  */
    float     D, v0;           /* MOVE_FROM/MOVE: m (signed for MOVE); TURN_ENC: D is dtheta   */
    float     a_acc, a_dec;    /* MOVE_FROM                                                   */
    float     t_acc, t_dec;    /* MOVE, TURN_ENC (t_acc)                                      */
    float     y_off;           /* lateral path offset in use, m                               */
    uint16_t  seg;             /* current segment index                                       */
    bool      reset_pose;      /* AUTO start: ekf_reset_pose(pose[0..2])                      */
    float     pose[3];
    fault_t   fault;           /* to sys_fault_raise(), F_NONE if none                        */
    msn_refuse_t refused;      /* set in the tick a start was refused                         */
    msn_event_t ev[MSN_EV_N];
    uint8_t   n_ev;
} msn_out_t;

typedef struct {
    msn_mode_t  mode;
    msn_state_t st, resume;
    msn_req_t   req;
    const course_t *course;    /* map_course(); tests may point it elsewhere before msn_step    */
    bool        course_ok;
    uint16_t    seg;
    uint8_t     rec_cnt, rec_ph;
    float       rec_dir;
    nav_t       nav;
    nav_cmd_t   fin;
    uint32_t    t0, t_tmr, t_run0, t_prev_ms, still_t, last_ev, acc_prev, man_t;
    bool        have_t, still_on, tmr_on, turn_enc, degraded, man_valid, man_to, disable_pending, il_prev;
    float       dt, bias, bias_end, d_noref, s_prev, man_v, man_w;
} msn_t;

void msn_init(msn_t *m);
void msn_request(msn_t *m, msn_req_t req);       /* from con/proto/B1; the last request of a tick wins */
void msn_manual(msn_t *m, float v, float w, uint32_t t_rx_ms);   /* DRIVE frame */
void msn_step(msn_t *m, const msn_in_t *in, msn_out_t *out);     /* S_CTRL      */
msn_mode_t  msn_mode(const msn_t *m);
msn_state_t msn_state(const msn_t *m);

#endif
