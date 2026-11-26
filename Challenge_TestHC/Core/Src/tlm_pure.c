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

void tlm_ev_push(tlm_ev_t *e, const proto_event_t *ev, unsigned port_mask)
{
    for (unsigned p = 0U; p < TLM_NPORT; p++) {
        if (((port_mask >> p) & 1U) == 0U) { continue; }
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

/* ---- duplicates ------------------------------------------------------------------- */

#define DUP_WINDOW_MS 1000U

bool tlm_dup_check(tlm_dup_t *d, uint8_t type, uint8_t seq, uint32_t now_ms)
{
    if (d->valid && d->type == type && d->seq == seq && now_ms - d->t_ms < DUP_WINDOW_MS) { return true; }
    *d = (tlm_dup_t){ .type = type, .seq = seq, .t_ms = now_ms, .valid = true };
    return false;
}

/* ---- capture ---------------------------------------------------------------------- */

void tlm_cap_init(tlm_cap_t *c, uint8_t *buf, uint32_t bytes)
{
    *c = (tlm_cap_t){ .buf = buf, .bytes = bytes };
}

st_t tlm_cap_start(tlm_cap_t *c, uint8_t set, uint8_t decim, uint16_t duration_ds, uint32_t now_ms)
{
    const size_t size = proto_cap_rec_size(set);
    if (size == 0U || decim == 0U) { return ST_RANGE; }
    c->set = set;
    c->decim = decim;
    c->size = (uint16_t)size;
    c->n = 0U;
    c->n_max = c->bytes / (uint32_t)size;
    c->skip = 0U;
    c->timed = duration_ds > 0U;
    c->t_end_ms = now_ms + (uint32_t)duration_ds * 100U;
    c->run = c->n_max > 0U;
    return ST_OK;
}

bool tlm_cap_want(tlm_cap_t *c, uint8_t set, uint32_t now_ms)
{
    if (!c->run || set != c->set) { return false; }
    if (c->timed && (int32_t)(now_ms - c->t_end_ms) >= 0) { c->run = false; return false; }
    if (++c->skip < c->decim) { return false; }
    c->skip = 0U;
    return true;
}

void tlm_cap_put(tlm_cap_t *c, const uint8_t *rec)
{
    if (!c->run) { return; }
    memcpy(&c->buf[c->n * c->size], rec, c->size);
    if (++c->n >= c->n_max) { c->run = false; }
}

void tlm_cap_stop(tlm_cap_t *c) { c->run = false; }

const uint8_t *tlm_cap_rec(const tlm_cap_t *c, uint32_t i) { return &c->buf[i * c->size]; }

void tlm_cap_range(uint32_t n, uint16_t from, uint16_t count, uint32_t *first, uint32_t *cnt)
{
    *first = (from < n) ? from : n;
    const uint32_t left = n - *first;
    *cnt = (count < left) ? count : left;
}

static void le16(uint8_t *o, int16_t v) { const uint16_t u = (uint16_t)v; o[0] = (uint8_t)u; o[1] = (uint8_t)(u >> 8); }
static void le32(uint8_t *o, uint32_t u)
{
    o[0] = (uint8_t)u; o[1] = (uint8_t)(u >> 8); o[2] = (uint8_t)(u >> 16); o[3] = (uint8_t)(u >> 24);
}

size_t tlm_rec_wheel_pack(const tlm_rec_wheel_t *r, uint8_t *o)
{
    le32(&o[0], r->t_us);
    le16(&o[4], i16(r->v_ref[0] * 1000.0f));
    le16(&o[6], i16(r->v[0] * 1000.0f));
    le16(&o[8], i16(r->v_ref[1] * 1000.0f));
    le16(&o[10], i16(r->v[1] * 1000.0f));
    le16(&o[12], i16(r->e_x[0] * 1.0e4f));
    le16(&o[14], i16(r->e_x[1] * 1.0e4f));
    le16(&o[16], i16(r->duty[0] * 1.0e4f));
    le16(&o[18], i16(r->duty[1] * 1.0e4f));
    le16(&o[20], i16(r->integ[0] * 1.0e4f));
    le16(&o[22], i16(r->integ[1] * 1.0e4f));
    return 24U;
}

/* bg in 1e-5 rad/s, not the 1e-6 of §7.25: the H7 bias of -2.4 deg/s (-0.042 rad/s) would
 * saturate i16 at 1e-6 (+-0.033 rad/s). D-80. */
size_t tlm_rec_est_pack(const tlm_rec_est_t *r, uint8_t *o)
{
    le32(&o[0], r->t_us);
    le32(&o[4], (uint32_t)sat_i(r->x * 1.0e4f, -2.0e9f, 2.0e9f));
    le32(&o[8], (uint32_t)sat_i(r->y * 1.0e4f, -2.0e9f, 2.0e9f));
    le16(&o[12], i16(wrap_pi(r->th) * 1.0e4f));
    le16(&o[14], i16(r->bg * 1.0e5f));
    le16(&o[16], i16(r->ds[0] * 1.0e6f));
    le16(&o[18], i16(r->ds[1] * 1.0e6f));
    le16(&o[20], i16(r->dpsi * 1.0e5f));
    o[22] = r->flags;
    o[23] = 0U;
    return 24U;
}
