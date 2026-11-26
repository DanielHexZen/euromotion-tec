/* tlm.c — spec §7.25, §8.1, §8.3 vehicle part (REQ-PRO-02, REQ-PRO-04, REQ-PRO-05, REQ-TLM-01..03,
 * REQ-COM-06, D-33, D-34, D-80): binary receivers per port, replies, the radio window, event
 * queues, the capture buffer and the CAP_DATA stream. Payloads, queues and the window arithmetic
 * are the pure part (tlm_pure.c); command execution and the telemetry inputs come from app. */
#include "tlm.h"

#include "app_config.h"
#include "com_uart.h"
#include "par.h"
#include "sys_fault.h"
#include "sys_time.h"

#define GUARD_MS  5U        /* HC-12 turnaround before the PC may send [VERIFY], D-76 item 7 */
#define UPLINK_MS 30U       /* longest uplink frame, §8.3                                     */

typedef struct {
    tlm_reply_t       kind;
    proto_ack_t       ack;
    proto_param_val_t pv;
} reply_t;

typedef struct {
    proto_rx_t rx;
    uint8_t    seq;         /* vehicle frame counter of this port, +1 per frame actually queued */
    tlm_dup_t  dup;
    reply_t    last;        /* reply to the last executed command, repeated for a duplicate  */
    reply_t    pend;        /* radio: sent at the start of the next period                   */
    bool       to_text;     /* radio: MODE_TEXT acknowledged, switch after the reply is out  */
} tport_t;

static tport_t       T[TLM_NPORT];
static tlm_ev_t      s_ev;
static uint32_t      s_ev_next;                 /* number of the next sys event to queue */
static proto_hello_t s_hello;
static uint32_t      s_baud, s_t_period, s_period_no;
static tlm_cap_t     s_cap;
#if FEAT_CAPTURE
static uint8_t       s_cap_buf[TLM_CAP_BYTES];
#endif
static struct { bool on; uint8_t set; uint32_t next, end; } s_stream;

static bool binary(unsigned p) { return com_mode((com_port_t)p) == COM_BINARY; }

static bool send(unsigned p, uint8_t type, const uint8_t *pl, size_t n)
{
    uint8_t f[PROTO_FRAME_MAX];
    const size_t len = proto_encode(type, T[p].seq, pl, n, f, sizeof f);
    if (len == 0U || com_write((com_port_t)p, f, len) != len) { return false; }   /* tx_drops counts it */
    T[p].seq++;
    return true;
}

static void send_hello(unsigned p)
{
    uint8_t b[PROTO_LEN_HELLO];
    (void)send(p, PROTO_T_HELLO, b, proto_pack_hello(&s_hello, b));
}

static void send_reply(unsigned p, const reply_t *r)
{
    uint8_t b[PROTO_LEN_PARAM_VAL];
    switch (r->kind) {
    case TLM_REPLY_ACK:
    case TLM_REPLY_ACK_HELLO:
        (void)send(p, PROTO_T_ACK, b, proto_pack_ack(&r->ack, b));
        if (r->kind == TLM_REPLY_ACK_HELLO) { send_hello(p); }
        break;
    case TLM_REPLY_PARAM_VAL: (void)send(p, PROTO_T_PARAM_VAL, b, proto_pack_param_val(&r->pv, b)); break;
    case TLM_REPLY_HELLO:     send_hello(p); break;
    case TLM_REPLY_NONE:
    default:                  break;
    }
}

/* USB has no timing rules (§8.3); on the radio one reply slot: a second command in the same
 * period replaces the reply of the first (the PC sends at most one per period). */
static void post(unsigned p, const reply_t *r)
{
    T[p].last = *r;
    if (p == (unsigned)TLM_PORT_RADIO) { T[p].pend = *r; }
    else { send_reply(p, r); }
}

static reply_t ack_of(const proto_frame_t *f, st_t result, tlm_reply_t kind)
{
    return (reply_t){ .kind = kind, .ack = { .type = f->type, .seq = f->seq, .result = (uint8_t)result } };
}

void tlm_ack(unsigned p, const proto_frame_t *f, st_t result)
{
    const reply_t r = ack_of(f, result, TLM_REPLY_ACK);
    post(p, &r);
}

void tlm_ack_hello(unsigned p, const proto_frame_t *f)
{
    const reply_t r = ack_of(f, ST_OK, TLM_REPLY_ACK_HELLO);
    post(p, &r);
}

void tlm_param_val(unsigned p, uint16_t id, float value, st_t result)
{
    const reply_t r = { .kind = TLM_REPLY_PARAM_VAL, .pv = { .id = id, .value = value, .result = (uint8_t)result } };
    post(p, &r);
}

static void to_text(unsigned p)
{
    com_set_mode((com_port_t)p, COM_TEXT);
    T[p].to_text = false;
    T[p].pend.kind = TLM_REPLY_NONE;
    if (p == (unsigned)TLM_PORT_USB) { s_stream.on = false; }
}

void tlm_mode_text(unsigned p, const proto_frame_t *f)
{
    tlm_ack(p, f, ST_OK);
    if (p == (unsigned)TLM_PORT_RADIO) { T[p].to_text = true; }    /* after the ACK in the next window */
    else { to_text(p); }                                             /* ACK is already in the TX buffer  */
}

void tlm_bin(unsigned p)
{
    proto_rx_init(&T[p].rx, (uint8_t)par()->com_vid);   /* drop what a text session left behind */
    T[p].dup.valid = false;
    T[p].to_text = false;
    com_set_mode((com_port_t)p, COM_BINARY);
    static const uint8_t delim = 0U;                 /* ends the text before it as one bad frame */
    (void)com_write((com_port_t)p, &delim, 1U);
    const reply_t r = { .kind = TLM_REPLY_HELLO };
    post(p, &r);
}

void tlm_vid_changed(void)
{
    for (unsigned p = 0U; p < TLM_NPORT; p++) { proto_rx_set_vid(&T[p].rx, (uint8_t)par()->com_vid); }
}

void tlm_set_hello(const proto_hello_t *h) { s_hello = *h; }
tlm_cap_t *tlm_capture(void) { return &s_cap; }

st_t tlm_capture_start(uint8_t set, uint8_t decim, uint16_t duration_ds)
{
    s_stream.on = false;                           /* §7.25: a new CAPTURE ends the stream */
    return tlm_cap_start(&s_cap, set, decim, duration_ds, sys_ms());
}

void tlm_cap_get(unsigned p, const proto_frame_t *f, uint16_t from, uint16_t count)
{
    tlm_ack(p, f, ST_OK);
    uint32_t first, cnt;
    tlm_cap_range(s_cap.n, from, count, &first, &cnt);
    s_stream.on = true;                            /* replaces a running stream, no end frame */
    s_stream.set = s_cap.set;
    s_stream.next = first;
    s_stream.end = first + cnt;
}

void tlm_init(uint32_t radio_baud)
{
    s_baud = radio_baud;
    for (unsigned p = 0U; p < TLM_NPORT; p++) { proto_rx_init(&T[p].rx, (uint8_t)par()->com_vid); }
    tlm_ev_init(&s_ev);
#if FEAT_CAPTURE
    tlm_cap_init(&s_cap, s_cap_buf, sizeof s_cap_buf);
#else
    tlm_cap_init(&s_cap, NULL, 0U);
#endif
    if (par()->com1_mode > 0.5f) {                 /* §8.1: radio default BINARY from phase 4 */
        com_set_mode(COM_RADIO, COM_BINARY);
        T[TLM_PORT_RADIO].pend.kind = TLM_REPLY_HELLO;   /* boot HELLO in the first window */
    }
    s_t_period = sys_ms();
}

void tlm_rx_byte(unsigned p, uint8_t b) { proto_rx_byte(&T[p].rx, b); }

/* D-80: frames lost to line errors on the radio; other vehicle IDs are not errors. */
static uint16_t rx_err(void)
{
    const proto_stats_t *s = &T[TLM_PORT_RADIO].rx.st;
    const uint32_t e = s->oversize + s->cobs + s->crc + s->runt + s->overrun + com_stats(COM_RADIO).errors;
    return (e > 65535U) ? 65535U : (uint16_t)e;
}

static void ev_pull(void)
{
    unsigned mask = 0U;
    for (unsigned p = 0U; p < TLM_NPORT; p++) { if (binary(p)) { mask |= 1U << p; } }
    const uint32_t total = sys_event_total();
    for (; s_ev_next != total; s_ev_next++) {
        sys_event_rec_t e;
        if (!sys_event_nth(s_ev_next, &e)) { continue; }            /* overwritten: still in no queue */
        const proto_event_t ev = { .t_ms = e.t_ms, .code = e.code, .arg = e.arg };
        tlm_ev_push(&s_ev, &ev, mask);
    }
}

static bool send_event(unsigned p)
{
    proto_event_t ev;
    uint8_t b[PROTO_LEN_EVENT];
    if (!tlm_ev_peek(&s_ev, p, &ev) || !send(p, PROTO_T_EVENT, b, proto_pack_event(&ev, b))) { return false; }
    tlm_ev_pop(&s_ev, p);
    return true;
}

static void send_telem(unsigned p, tlm_telem_t k, const tlm_fast_in_t *fi, const tlm_slow_in_t *si)
{
    uint8_t b[PROTO_LEN_TELEM_SLOW];
    if (k == TLM_T_FAST) { (void)send(p, PROTO_T_TELEM_FAST, b, tlm_fast_payload(fi, b)); }
    else if (k == TLM_T_SLOW) { (void)send(p, PROTO_T_TELEM_SLOW, b, tlm_slow_payload(si, b)); }
}

static void period(void)
{
    if (!binary(TLM_PORT_USB) && !binary(TLM_PORT_RADIO)) { s_period_no++; return; }
    const tlm_telem_t k = (s_period_no % TLM_SLOW_EVERY == TLM_SLOW_EVERY - 1U) ? TLM_T_SLOW : TLM_T_FAST;
    tlm_fast_in_t fi = { 0 };
    tlm_slow_in_t si = { 0 };
    if (k == TLM_T_FAST) {
        tlm_fill_fast(&fi);
        if (s_cap.run) { fi.flags |= TLM_F_CAPTURE; }
    } else {
        tlm_fill_slow(&si);
        si.rx_err = rx_err();
        si.ev_drop = tlm_ev_drop(&s_ev);
    }

    if (binary(TLM_PORT_USB)) { send_telem(TLM_PORT_USB, k, &fi, &si); }

    tport_t *r = &T[TLM_PORT_RADIO];
    if (binary(TLM_PORT_RADIO)) {
        const par_t *q = par();
        const tlm_cfg_t c = { .baud = s_baud, .period_ms = (uint32_t)q->tlm_period_ms,
                              .air_max_ms = (uint32_t)q->tlm_air_max_ms, .guard_ms = GUARD_MS, .uplink_ms = UPLINK_MS };
        tlm_plan_t pl;
        tlm_plan(&c, r->pend.kind, s_period_no, tlm_ev_count(&s_ev, TLM_PORT_RADIO), &pl);
        send_reply(TLM_PORT_RADIO, &r->pend);          /* (1) reply, (2) telemetry, (3) events: §8.3 */
        r->pend.kind = TLM_REPLY_NONE;
        if (r->to_text) {
            to_text(TLM_PORT_RADIO);
        } else {
            if (pl.telem == k) { send_telem(TLM_PORT_RADIO, k, &fi, &si); }   /* NONE when a HELLO replaced it */
            for (uint8_t i = 0U; i < pl.n_events && send_event(TLM_PORT_RADIO); i++) { }
        }
    }
    s_period_no++;
}

/* REQ-TLM-02: one CAP_DATA frame per pass while the USB TX buffer keeps room for a telemetry frame. */
static void stream(void)
{
    if (!s_stream.on) { return; }
    if (!binary(TLM_PORT_USB)) { s_stream.on = false; return; }
    const uint32_t left = s_stream.end - s_stream.next;
    const uint32_t nmax = (uint32_t)proto_cap_n_max(s_stream.set);
    const uint32_t n = (left < nmax) ? left : nmax;
    const size_t len = PROTO_LEN_CAP_HDR + n * proto_cap_rec_size(s_stream.set);
    if (com_tx_free(COM_USB) < PROTO_WIRE(len) + PROTO_WIRE(PROTO_LEN_TELEM_SLOW)) { return; }
    uint8_t b[PROTO_PAYLOAD_MAX];
    const proto_cap_data_t m = { .index = (uint16_t)s_stream.next, .set = s_stream.set, .n = (uint8_t)n,
                                 .rec = (n > 0U) ? tlm_cap_rec(&s_cap, s_stream.next) : NULL };
    if (!send(TLM_PORT_USB, PROTO_T_CAP_DATA, b, proto_pack_cap_data(&m, b, sizeof b))) { return; }
    s_stream.next += n;
    if (n == 0U) { s_stream.on = false; }          /* that was the end frame */
}

void tlm_poll(void)
{
    for (unsigned p = 0U; p < TLM_NPORT; p++) {
        if (!binary(p)) { continue; }
        if (proto_rx_text_request(&T[p].rx)) { to_text(p); continue; }      /* REQ-PRO-02 */
        proto_frame_t f;
        if (!proto_rx_frame(&T[p].rx, &f)) { continue; }
        if (tlm_dup_check(&T[p].dup, f.type, f.seq, sys_ms())) {           /* REQ-PRO-05 */
            if (T[p].last.kind != TLM_REPLY_NONE) { post(p, &T[p].last); }
        } else {
            T[p].last.kind = TLM_REPLY_NONE;
            tlm_exec(p, &f);
        }
    }
    ev_pull();
    if (binary(TLM_PORT_USB)) { while (send_event(TLM_PORT_USB)) { } }     /* USB: at once */
    stream();

    const uint32_t per = (uint32_t)par()->tlm_period_ms;
    const uint32_t now = sys_ms();
    if (now - s_t_period >= per) {
        s_t_period = (now - s_t_period >= 2U * per) ? now : s_t_period + per;   /* fixed phase, resync after a stall */
        period();
    }
}
