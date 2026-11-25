/* tlm.h — telemetry, events, radio window (spec §7.25, §8.3, D-34). Pure part only (tlm_pure.c):
 * payload assembly from a plain input struct, per-port event queues, the radio window planner.
 * Sending, the capture buffer and the integration into app.c are phase-4 vehicle work. */
#ifndef TLM_H
#define TLM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "proto.h"

/* ---- TELEM_FAST / TELEM_SLOW assembly (§8.4) ------------------------------------- */
/* Flag bits of TELEM_FAST.flags. */
enum {
    TLM_F_REFLEX = 1U << 0, TLM_F_INTERLOCK = 1U << 1, TLM_F_STILL = 1U << 2, TLM_F_SLIP = 1U << 3,
    TLM_F_DEGRADED = 1U << 4, TLM_F_EVADE = 1U << 5, TLM_F_BAT_WARN = 1U << 6, TLM_F_FAULT = 1U << 7,
    TLM_F_IMU_OK = 1U << 8, TLM_F_VL_OK = 1U << 9, TLM_F_MAN_TIMEOUT = 1U << 10, TLM_F_CAPTURE = 1U << 11
};

typedef struct {                       /* SI units, as the modules publish them            */
    uint32_t t_ms;
    uint8_t  mode, state;
    uint16_t flags;                    /* TLM_F_*                                          */
    float    x_m, y_m, th_rad;         /* EKF pose                                         */
    float    v_mps, w_radps;           /* measured speed, yaw rate                         */
    float    duty_l, duty_r;           /* [-1, 1]                                          */
    uint8_t  seg, wp;
    float    e_y_m;
    uint8_t  gnd_l, gnd_r;             /* gnd_side_t, 3 bits each                          */
    float    fwd_m;                    /* minimum forward range; negative, NaN or >= 2.55 m = none */
} tlm_fast_in_t;

typedef struct {
    uint32_t t_ms;
    float    bat_v;
    float    sig_x_m, sig_y_m, sig_th_rad;
    float    bg_radps, scale, imu_t_degc;   /* scale = s - 1 (dimensionless) */
    uint16_t fault_first;
    uint8_t  n_faults, cpu;
    uint16_t ovr_ctrl, rx_err, ekf_acc, ekf_rej, stack_free;
    uint8_t  vl_alive, sens;
    uint8_t  ev_drop;                  /* tlm_ev_drop()                                    */
} tlm_slow_in_t;

/* Scale with rounding and saturation (NaN gives 0), then pack. out holds the payload;
 * the return value is its length (32 / 35). */
void   tlm_fast_build(const tlm_fast_in_t *in, proto_telem_fast_t *out);
void   tlm_slow_build(const tlm_slow_in_t *in, proto_telem_slow_t *out);
size_t tlm_fast_payload(const tlm_fast_in_t *in, uint8_t *out);
size_t tlm_slow_payload(const tlm_slow_in_t *in, uint8_t *out);

/* ---- event queues (REQ-TLM-03, D-34) --------------------------------------------- */
#define TLM_EV_DEPTH 16U
#define TLM_NPORT    2U
enum { TLM_PORT_USB = 0, TLM_PORT_RADIO = 1 };   /* same values as com_port_t */

typedef struct { proto_event_t q[TLM_EV_DEPTH]; uint8_t head, n; } tlm_evq_t;
typedef struct { tlm_evq_t port[TLM_NPORT]; uint8_t drop; } tlm_ev_t;

void    tlm_ev_init(tlm_ev_t *e);
/* Appends to every port queue. A full queue drops the NEW event (oldest kept); each drop
 * adds 1 to ev_drop, saturating at 255. */
void    tlm_ev_push(tlm_ev_t *e, const proto_event_t *ev);
bool    tlm_ev_peek(const tlm_ev_t *e, unsigned port, proto_event_t *ev);   /* oldest */
void    tlm_ev_pop(tlm_ev_t *e, unsigned port);
uint8_t tlm_ev_count(const tlm_ev_t *e, unsigned port);
uint8_t tlm_ev_drop(const tlm_ev_t *e);

/* ---- radio window planner (§8.3, REQ-PRO-04, D-34) ------------------------------- */
typedef struct {
    uint32_t baud;
    uint32_t period_ms;      /* tlm_period_ms (902)                                        */
    uint32_t air_max_ms;     /* tlm_air_max_ms (903)                                       */
    uint32_t guard_ms;       /* HC-12 turnaround before the PC may send, 5 ms [VERIFY]     */
    uint32_t uplink_ms;      /* longest uplink frame, 30 ms                                */
} tlm_cfg_t;

typedef enum { TLM_REPLY_NONE, TLM_REPLY_ACK, TLM_REPLY_PARAM_VAL, TLM_REPLY_ACK_HELLO, TLM_REPLY_HELLO } tlm_reply_t;
typedef enum { TLM_T_NONE, TLM_T_FAST, TLM_T_SLOW } tlm_telem_t;

typedef struct {
    tlm_telem_t telem;       /* NONE only when a HELLO replaces the telemetry              */
    uint8_t     n_events;    /* radio events that still fit, oldest first                  */
    uint32_t    air_us;      /* reply + telemetry + events, on the wire                    */
    uint32_t    pc_end_us;   /* air_us + guard + uplink: end of the PC window              */
    bool        over;        /* air_us > air_max (reply + telemetry are sent anyway)       */
    bool        pc_late;     /* pc_end_us >= period                                        */
} tlm_plan_t;

#define TLM_SLOW_EVERY 10U   /* TELEM_SLOW in every 10th period */

/* us on the wire for a frame of `bytes` (10 bits per byte), rounded down. */
uint32_t tlm_air_us(uint32_t baud, size_t bytes);
/* Longest uplink frame in bytes that fits uplink_ms (28 at 9600 Bd, 57 at 19200 Bd). */
uint32_t tlm_uplink_max_bytes(const tlm_cfg_t *c);
/* Order on the air: reply frames, telemetry, then `n_events` events. period_no counts
 * periods since boot; TELEM_SLOW when period_no % 10 == 9. */
void tlm_plan(const tlm_cfg_t *c, tlm_reply_t reply, uint32_t period_no, uint8_t events_pending,
              tlm_plan_t *p);
/* CAP_DATA is never sent on the radio (D-33). */
bool tlm_cap_data_allowed(unsigned port);

#endif
