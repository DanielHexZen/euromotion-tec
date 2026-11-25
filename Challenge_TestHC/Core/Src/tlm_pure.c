/* tlm_pure.c — spec §7.25 / §8.3 without hardware: payload assembly, event queues, window planner. */
#include "tlm.h"

#include <string.h>

#include "units.h"

/* Round and saturate; NaN -> 0 (never UB on the cast). */
static int32_t sat_i(float v, float lo, float hi)
{
    if (!(v == v)) { return 0; }
    return (int32_t)roundf(clampf(v, lo, hi));
}
static int16_t i16(float v) { return (int16_t)sat_i(v, -32768.0f, 32767.0f); }
static uint16_t u16(float v) { return (uint16_t)sat_i(v, 0.0f, 65535.0f); }

void tlm_fast_build(const tlm_fast_in_t *in, proto_telem_fast_t *o)
{
    o->t_ms = in->t_ms;
    o->mode = in->mode;
    o->state = in->state;
    o->flags = in->flags;
    o->x = sat_i(in->x_m * 1000.0f, -2.0e9f, 2.0e9f);
    o->y = sat_i(in->y_m * 1000.0f, -2.0e9f, 2.0e9f);
    o->th = i16(clampf(wrap_pi(in->th_rad) * RAD2DEG_F * 100.0f, -18000.0f, 18000.0f));
    o->v = i16(in->v_mps * 1000.0f);
    o->w = i16(in->w_radps * 1000.0f);
    o->duty_l = i16(in->duty_l * 1000.0f);
    o->duty_r = i16(in->duty_r * 1000.0f);
    o->seg = in->seg;
    o->wp = in->wp;
    o->e_y = i16(in->e_y_m * 1000.0f);
    o->gnd = (uint8_t)((in->gnd_l & 7U) | ((in->gnd_r & 7U) << 3));
    const float cm = in->fwd_m * 100.0f;
    o->fwd = (in->fwd_m >= 0.0f && cm < 254.5f) ? (uint8_t)sat_i(cm, 0.0f, 254.0f) : 255U;
}

void tlm_slow_build(const tlm_slow_in_t *in, proto_telem_slow_t *o)
{
    o->t_ms = in->t_ms;
    o->bat = u16(in->bat_v * 1000.0f);
    o->sig_x = u16(in->sig_x_m * 1000.0f);
    o->sig_y = u16(in->sig_y_m * 1000.0f);
    o->sig_th = u16(in->sig_th_rad * RAD2DEG_F * 100.0f);
    o->bg = i16(in->bg_radps * RAD2DEG_F * 1000.0f);
    o->scale = i16(in->scale * 1.0e6f);
    o->imu_t = i16(in->imu_t_degc * 100.0f);
    o->fault_first = in->fault_first;
    o->n_faults = in->n_faults;
    o->cpu = in->cpu;
    o->ovr_ctrl = in->ovr_ctrl;
    o->rx_err = in->rx_err;
    o->ekf_acc = in->ekf_acc;
    o->ekf_rej = in->ekf_rej;
    o->stack_free = in->stack_free;
    o->vl_alive = in->vl_alive;
    o->sens = in->sens;
    o->ev_drop = in->ev_drop;
}

size_t tlm_fast_payload(const tlm_fast_in_t *in, uint8_t *out)
{
    proto_telem_fast_t m;
    tlm_fast_build(in, &m);
    return proto_pack_telem_fast(&m, out);
}

size_t tlm_slow_payload(const tlm_slow_in_t *in, uint8_t *out)
{
    proto_telem_slow_t m;
    tlm_slow_build(in, &m);
    return proto_pack_telem_slow(&m, out);
}

/* ---- events ----------------------------------------------------------------------- */

void tlm_ev_init(tlm_ev_t *e) { memset(e, 0, sizeof *e); }

void tlm_ev_push(tlm_ev_t *e, const proto_event_t *ev)
{
    for (unsigned p = 0U; p < TLM_NPORT; p++) {
        tlm_evq_t *q = &e->port[p];
        if (q->n >= TLM_EV_DEPTH) {
            if (e->drop < 255U) { e->drop++; }
        } else {
            q->q[(q->head + q->n) % TLM_EV_DEPTH] = *ev;
            q->n++;
        }
    }
}

bool tlm_ev_peek(const tlm_ev_t *e, unsigned port, proto_event_t *ev)
{
    if (port >= TLM_NPORT || e->port[port].n == 0U) { return false; }
    *ev = e->port[port].q[e->port[port].head];
    return true;
}

void tlm_ev_pop(tlm_ev_t *e, unsigned port)
{
    if (port >= TLM_NPORT || e->port[port].n == 0U) { return; }
    e->port[port].head = (uint8_t)((e->port[port].head + 1U) % TLM_EV_DEPTH);
    e->port[port].n--;
}

uint8_t tlm_ev_count(const tlm_ev_t *e, unsigned port) { return (port < TLM_NPORT) ? e->port[port].n : 0U; }
uint8_t tlm_ev_drop(const tlm_ev_t *e) { return e->drop; }

/* ---- planner ---------------------------------------------------------------------- */

uint32_t tlm_air_us(uint32_t baud, size_t bytes)
{
    /* bytes <= 104 and 10e6 * 104 < 2^32, so 32 bits are enough */
    return (baud == 0U) ? 0U : (uint32_t)bytes * 10000000U / baud;
}

uint32_t tlm_uplink_max_bytes(const tlm_cfg_t *c)
{
    return (uint32_t)((uint64_t)c->uplink_ms * c->baud / 10000U);
}

#define WIRE(len) PROTO_WIRE((size_t)(len))

void tlm_plan(const tlm_cfg_t *c, tlm_reply_t reply, uint32_t period_no, uint8_t events_pending,
              tlm_plan_t *p)
{
    uint32_t us = 0U;
    switch (reply) {
    case TLM_REPLY_ACK:       us += tlm_air_us(c->baud, WIRE(PROTO_LEN_ACK)); break;
    case TLM_REPLY_PARAM_VAL: us += tlm_air_us(c->baud, WIRE(PROTO_LEN_PARAM_VAL)); break;
    case TLM_REPLY_ACK_HELLO: us += tlm_air_us(c->baud, WIRE(PROTO_LEN_ACK)); /* fall through */
    case TLM_REPLY_HELLO:     us += tlm_air_us(c->baud, WIRE(PROTO_LEN_HELLO)); break;
    default: break;
    }
    /* A HELLO replaces the telemetry of that period (ACK + HELLO + TELEM_FAST would push the PC window over). */
    if (reply == TLM_REPLY_ACK_HELLO || reply == TLM_REPLY_HELLO) {
        p->telem = TLM_T_NONE;
    } else if (period_no % TLM_SLOW_EVERY == TLM_SLOW_EVERY - 1U) {
        p->telem = TLM_T_SLOW;
        us += tlm_air_us(c->baud, WIRE(PROTO_LEN_TELEM_SLOW));
    } else {
        p->telem = TLM_T_FAST;
        us += tlm_air_us(c->baud, WIRE(PROTO_LEN_TELEM_FAST));
    }
    const uint32_t max_us = c->air_max_ms * 1000U;
    const uint32_t ev_us = tlm_air_us(c->baud, WIRE(PROTO_LEN_EVENT));
    p->over = us > max_us;               /* replies and telemetry are sent regardless */
    p->n_events = 0U;
    while (p->n_events < events_pending && us + ev_us <= max_us) {
        us += ev_us;
        p->n_events++;
    }
    p->air_us = us;
    p->pc_end_us = us + (c->guard_ms + c->uplink_ms) * 1000U;
    p->pc_late = p->pc_end_us >= c->period_ms * 1000U;
}

bool tlm_cap_data_allowed(unsigned port) { return port == (unsigned)TLM_PORT_USB; }
