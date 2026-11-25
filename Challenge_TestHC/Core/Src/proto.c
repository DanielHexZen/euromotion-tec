/* proto.c — spec §7.24: CRC, COBS, frames, one pack/unpack per §8.4 message. Pure. */
#include "proto.h"

#include <string.h>

#include "par.h"

uint16_t crc16_ccitt(const uint8_t *d, size_t n)
{
    uint32_t crc = 0xFFFFU;
    for (size_t i = 0U; i < n; i++) {
        crc ^= (uint32_t)d[i] << 8;
        for (uint8_t b = 0U; b < 8U; b++) {
            crc = ((crc & 0x8000U) != 0U) ? ((crc << 1) ^ 0x1021U) : (crc << 1);
        }
    }
    return (uint16_t)(crc & 0xFFFFU);
}

/* ---- COBS ------------------------------------------------------------------------ */

size_t cobs_encode(const uint8_t *in, size_t n, uint8_t *out, size_t out_max)
{
    size_t  o = 1U, ci = 0U;
    uint8_t code = 1U;
    if (out_max < 1U) { return 0U; }
    for (size_t i = 0U; i < n; i++) {
        if (in[i] != 0U) {
            if (o >= out_max) { return 0U; }
            out[o++] = in[i];
            code++;
        }
        if (in[i] == 0U || (code == 0xFFU && i + 1U < n)) {
            out[ci] = code;
            ci = o;
            if (o >= out_max) { return 0U; }
            o++;
            code = 1U;
        }
    }
    out[ci] = code;
    return o;
}

bool cobs_decode(const uint8_t *in, size_t n, uint8_t *out, size_t out_max, size_t *out_n)
{
    size_t i = 0U, o = 0U;
    if (n == 0U) { return false; }
    while (i < n) {
        const uint8_t code = in[i++];
        const size_t  run = (size_t)code - 1U;
        if (code == 0U || i + run > n || o + run > out_max) { return false; }
        for (size_t k = 0U; k < run; k++) {
            if (in[i + k] == 0U) { return false; }
            out[o++] = in[i + k];
        }
        i += run;
        if (code != 0xFFU && i < n) {
            if (o >= out_max) { return false; }
            out[o++] = 0U;
        }
    }
    *out_n = o;
    return true;
}

/* ---- little-endian helpers (REQ-PRO-01) ------------------------------------------- */

static void w16(uint8_t *o, uint16_t v) { o[0] = (uint8_t)(v & 0xFFU); o[1] = (uint8_t)(v >> 8); }
static void w32(uint8_t *o, uint32_t v)
{
    for (unsigned k = 0U; k < 4U; k++) { o[k] = (uint8_t)((v >> (8U * k)) & 0xFFU); }
}
static void wf(uint8_t *o, float v) { uint32_t u; memcpy(&u, &v, sizeof u); w32(o, u); }
static uint16_t r16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8)); }
static uint32_t r32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static float rf(const uint8_t *p) { const uint32_t u = r32(p); float v; memcpy(&v, &u, sizeof v); return v; }
static int16_t r16s(const uint8_t *p) { return (int16_t)r16(p); }
static int32_t r32s(const uint8_t *p) { return (int32_t)r32(p); }
static void w16s(uint8_t *o, int16_t v) { w16(o, (uint16_t)v); }
static void w32s(uint8_t *o, int32_t v) { w32(o, (uint32_t)v); }

/* ---- frames ----------------------------------------------------------------------- */

size_t proto_encode_vid(uint8_t vid, uint8_t type, uint8_t seq, const uint8_t *payload, size_t n,
                        uint8_t *out, size_t out_max)
{
    uint8_t raw[PROTO_RAW_MAX];
    if (n > PROTO_PAYLOAD_MAX) { return 0U; }
    raw[0] = vid;
    raw[1] = type;
    raw[2] = seq;
    if (n > 0U) { memcpy(&raw[3], payload, n); }
    const uint16_t crc = crc16_ccitt(raw, n + 3U);
    raw[n + 3U] = (uint8_t)(crc & 0xFFU);
    raw[n + 4U] = (uint8_t)(crc >> 8);
    const size_t e = cobs_encode(raw, n + 5U, out, out_max);
    if (e == 0U || e >= out_max) { return 0U; }
    out[e] = 0U;
    return e + 1U;
}

size_t proto_encode(uint8_t type, uint8_t seq, const uint8_t *payload, size_t n,
                    uint8_t *out, size_t out_max)
{
    return proto_encode_vid((uint8_t)par()->com_vid, type, seq, payload, n, out, out_max);
}

void proto_rx_init(proto_rx_t *rx, uint8_t vid)
{
    memset(rx, 0, sizeof *rx);
    rx->vid = vid;
}

void proto_rx_set_vid(proto_rx_t *rx, uint8_t vid) { rx->vid = vid; }

static void rx_finish(proto_rx_t *rx)
{
    uint8_t raw[PROTO_ENC_MAX];
    size_t  len = 0U;
    if (rx->n == 0U && !rx->overflow) { return; }          /* empty frame: idle delimiters */
    if (rx->overflow) { rx->st.oversize++; return; }
    if (!cobs_decode(rx->buf, rx->n, raw, sizeof raw, &len)) { rx->st.cobs++; return; }
    if (len > PROTO_RAW_MAX) { rx->st.oversize++; return; }
    if (len < 5U) { rx->st.runt++; return; }
    const uint16_t crc = (uint16_t)((uint16_t)raw[len - 2U] | (uint16_t)((uint16_t)raw[len - 1U] << 8));
    if (crc16_ccitt(raw, len - 2U) != crc) { rx->st.crc++; return; }
    if (!(raw[0] == rx->vid || (raw[0] == PROTO_VID_BCAST && raw[1] == (uint8_t)PROTO_T_STOP))) {
        rx->st.vid++;
        return;
    }
    if (rx->ready) { rx->st.overrun++; }
    rx->frame.vid = raw[0];
    rx->frame.type = raw[1];
    rx->frame.seq = raw[2];
    rx->frame.len = (uint8_t)(len - 5U);
    memcpy(rx->frame.payload, &raw[3], len - 5U);
    rx->ready = true;
    rx->st.ok++;
}

/* REQ-PRO-02: "#TEXT" followed by CR or LF (the console's line terminators) in the raw stream. */
static void rx_scan_text(proto_rx_t *rx, uint8_t b)
{
    static const char key[] = "#TEXT";
    if (rx->scan >= 5U) {
        if (b == (uint8_t)'\r' || b == (uint8_t)'\n') { rx->text_req = true; }
        rx->scan = 0U;
    }
    rx->scan = (b == (uint8_t)key[rx->scan]) ? (uint8_t)(rx->scan + 1U) : ((b == (uint8_t)'#') ? 1U : 0U);
}

void proto_rx_byte(proto_rx_t *rx, uint8_t b)
{
    rx_scan_text(rx, b);
    if (b == 0U) {
        rx_finish(rx);
        rx->n = 0U;
        rx->overflow = false;
        return;
    }
    if (rx->n < PROTO_ENC_MAX) { rx->buf[rx->n++] = b; } else { rx->overflow = true; }
}

bool proto_rx_frame(proto_rx_t *rx, proto_frame_t *f)
{
    if (!rx->ready) { return false; }
    *f = rx->frame;
    rx->ready = false;
    return true;
}

bool proto_rx_text_request(proto_rx_t *rx)
{
    const bool r = rx->text_req;
    rx->text_req = false;
    return r;
}

/* ---- messages --------------------------------------------------------------------- */

size_t proto_pack_hello(const proto_hello_t *m, uint8_t *o)
{
    o[0] = m->proto_ver; o[1] = m->fw_major; o[2] = m->fw_minor; o[3] = m->reset_cause;
    w32(&o[4], m->git_hash);
    o[8] = m->dirty; o[9] = m->features;
    for (size_t k = 0U; k < 8U; k++) { o[10U + k] = (uint8_t)m->course[k]; }
    w32(&o[18], m->last_fault_code);
    w32(&o[22], m->last_fault_pc);
    return PROTO_LEN_HELLO;
}

bool proto_unpack_hello(const uint8_t *p, size_t n, proto_hello_t *m)
{
    if (n != PROTO_LEN_HELLO) { return false; }
    m->proto_ver = p[0]; m->fw_major = p[1]; m->fw_minor = p[2]; m->reset_cause = p[3];
    m->git_hash = r32(&p[4]);
    m->dirty = p[8]; m->features = p[9];
    for (size_t k = 0U; k < 8U; k++) { m->course[k] = (char)p[10U + k]; }
    m->last_fault_code = r32(&p[18]);
    m->last_fault_pc = r32(&p[22]);
    return true;
}

size_t proto_pack_telem_fast(const proto_telem_fast_t *m, uint8_t *o)
{
    w32(&o[0], m->t_ms);
    o[4] = m->mode; o[5] = m->state;
    w16(&o[6], m->flags);
    w32s(&o[8], m->x); w32s(&o[12], m->y);
    w16s(&o[16], m->th); w16s(&o[18], m->v); w16s(&o[20], m->w);
    w16s(&o[22], m->duty_l); w16s(&o[24], m->duty_r);
    o[26] = m->seg; o[27] = m->wp;
    w16s(&o[28], m->e_y);
    o[30] = m->gnd; o[31] = m->fwd;
    return PROTO_LEN_TELEM_FAST;
}

bool proto_unpack_telem_fast(const uint8_t *p, size_t n, proto_telem_fast_t *m)
{
    if (n != PROTO_LEN_TELEM_FAST) { return false; }
    m->t_ms = r32(&p[0]);
    m->mode = p[4]; m->state = p[5];
    m->flags = r16(&p[6]);
    m->x = r32s(&p[8]); m->y = r32s(&p[12]);
    m->th = r16s(&p[16]); m->v = r16s(&p[18]); m->w = r16s(&p[20]);
    m->duty_l = r16s(&p[22]); m->duty_r = r16s(&p[24]);
    m->seg = p[26]; m->wp = p[27];
    m->e_y = r16s(&p[28]);
    m->gnd = p[30]; m->fwd = p[31];
    return true;
}

size_t proto_pack_telem_slow(const proto_telem_slow_t *m, uint8_t *o)
{
    w32(&o[0], m->t_ms);
    w16(&o[4], m->bat); w16(&o[6], m->sig_x); w16(&o[8], m->sig_y); w16(&o[10], m->sig_th);
    w16s(&o[12], m->bg); w16s(&o[14], m->scale); w16s(&o[16], m->imu_t);
    w16(&o[18], m->fault_first);
    o[20] = m->n_faults; o[21] = m->cpu;
    w16(&o[22], m->ovr_ctrl); w16(&o[24], m->rx_err); w16(&o[26], m->ekf_acc); w16(&o[28], m->ekf_rej);
    w16(&o[30], m->stack_free);
    o[32] = m->vl_alive; o[33] = m->sens; o[34] = m->ev_drop;
    return PROTO_LEN_TELEM_SLOW;
}

bool proto_unpack_telem_slow(const uint8_t *p, size_t n, proto_telem_slow_t *m)
{
    if (n != PROTO_LEN_TELEM_SLOW) { return false; }
    m->t_ms = r32(&p[0]);
    m->bat = r16(&p[4]); m->sig_x = r16(&p[6]); m->sig_y = r16(&p[8]); m->sig_th = r16(&p[10]);
    m->bg = r16s(&p[12]); m->scale = r16s(&p[14]); m->imu_t = r16s(&p[16]);
    m->fault_first = r16(&p[18]);
    m->n_faults = p[20]; m->cpu = p[21];
    m->ovr_ctrl = r16(&p[22]); m->rx_err = r16(&p[24]); m->ekf_acc = r16(&p[26]); m->ekf_rej = r16(&p[28]);
    m->stack_free = r16(&p[30]);
    m->vl_alive = p[32]; m->sens = p[33]; m->ev_drop = p[34];
    return true;
}

size_t proto_pack_event(const proto_event_t *m, uint8_t *o)
{
    w32(&o[0], m->t_ms); w16(&o[4], m->code); w32s(&o[6], m->arg);
    return PROTO_LEN_EVENT;
}

bool proto_unpack_event(const uint8_t *p, size_t n, proto_event_t *m)
{
    if (n != PROTO_LEN_EVENT) { return false; }
    m->t_ms = r32(&p[0]); m->code = r16(&p[4]); m->arg = r32s(&p[6]);
    return true;
}

size_t proto_pack_ack(const proto_ack_t *m, uint8_t *o)
{
    o[0] = m->type; o[1] = m->seq; o[2] = m->result;
    return PROTO_LEN_ACK;
}

bool proto_unpack_ack(const uint8_t *p, size_t n, proto_ack_t *m)
{
    if (n != PROTO_LEN_ACK) { return false; }
    m->type = p[0]; m->seq = p[1]; m->result = p[2];
    return true;
}

size_t proto_pack_param_val(const proto_param_val_t *m, uint8_t *o)
{
    w16(&o[0], m->id); wf(&o[2], m->value); o[6] = m->result;
    return PROTO_LEN_PARAM_VAL;
}

bool proto_unpack_param_val(const uint8_t *p, size_t n, proto_param_val_t *m)
{
    if (n != PROTO_LEN_PARAM_VAL) { return false; }
    m->id = r16(&p[0]); m->value = rf(&p[2]); m->result = p[6];
    return true;
}

size_t proto_pack_mode(uint8_t mode, uint8_t *o) { o[0] = mode; return PROTO_LEN_MODE; }

bool proto_unpack_mode(const uint8_t *p, size_t n, uint8_t *mode)
{
    if (n != PROTO_LEN_MODE) { return false; }
    *mode = p[0];
    return true;
}

size_t proto_pack_drive(const proto_drive_t *m, uint8_t *o)
{
    w16s(&o[0], m->v); w16s(&o[2], m->w); o[4] = m->flags;
    return PROTO_LEN_DRIVE;
}

bool proto_unpack_drive(const uint8_t *p, size_t n, proto_drive_t *m)
{
    if (n != PROTO_LEN_DRIVE) { return false; }
    m->v = r16s(&p[0]); m->w = r16s(&p[2]); m->flags = p[4];
    return true;
}

size_t proto_pack_param_get(uint16_t id, uint8_t *o) { w16(o, id); return PROTO_LEN_PARAM_GET; }

bool proto_unpack_param_get(const uint8_t *p, size_t n, uint16_t *id)
{
    if (n != PROTO_LEN_PARAM_GET) { return false; }
    *id = r16(p);
    return true;
}

size_t proto_pack_param_set(const proto_param_set_t *m, uint8_t *o)
{
    w16(&o[0], m->id); wf(&o[2], m->value);
    return PROTO_LEN_PARAM_SET;
}

bool proto_unpack_param_set(const uint8_t *p, size_t n, proto_param_set_t *m)
{
    if (n != PROTO_LEN_PARAM_SET) { return false; }
    m->id = r16(&p[0]); m->value = rf(&p[2]);
    return true;
}

size_t proto_pack_capture(const proto_capture_t *m, uint8_t *o)
{
    o[0] = m->set; o[1] = m->decim; w16(&o[2], m->duration);
    return PROTO_LEN_CAPTURE;
}

bool proto_unpack_capture(const uint8_t *p, size_t n, proto_capture_t *m)
{
    if (n != PROTO_LEN_CAPTURE) { return false; }
    m->set = p[0]; m->decim = p[1]; m->duration = r16(&p[2]);
    return true;
}

size_t proto_pack_cap_get(const proto_cap_get_t *m, uint8_t *o)
{
    w16(&o[0], m->from); w16(&o[2], m->count);
    return PROTO_LEN_CAP_GET;
}

bool proto_unpack_cap_get(const uint8_t *p, size_t n, proto_cap_get_t *m)
{
    if (n != PROTO_LEN_CAP_GET) { return false; }
    m->from = r16(&p[0]); m->count = r16(&p[2]);
    return true;
}

size_t proto_cap_rec_size(uint8_t set)
{
    return (set <= 2U) ? 24U : ((set == 3U) ? 16U : 0U);
}

size_t proto_cap_n_max(uint8_t set)
{
    const size_t s = proto_cap_rec_size(set);
    return (s == 0U) ? 0U : (PROTO_PAYLOAD_MAX - PROTO_LEN_CAP_HDR) / s;
}

size_t proto_pack_cap_data(const proto_cap_data_t *m, uint8_t *o, size_t out_max)
{
    const size_t s = proto_cap_rec_size(m->set);
    if (s == 0U || m->n > proto_cap_n_max(m->set)) { return 0U; }
    const size_t len = PROTO_LEN_CAP_HDR + (size_t)m->n * s;
    if (len > out_max) { return 0U; }
    w16(&o[0], m->index); o[2] = m->set; o[3] = m->n;
    if (m->n > 0U) { memcpy(&o[4], m->rec, (size_t)m->n * s); }
    return len;
}

bool proto_unpack_cap_data(const uint8_t *p, size_t n, proto_cap_data_t *m)
{
    if (n < PROTO_LEN_CAP_HDR) { return false; }
    const size_t s = proto_cap_rec_size(p[2]);
    if (s == 0U || p[3] > proto_cap_n_max(p[2]) || n != PROTO_LEN_CAP_HDR + (size_t)p[3] * s) { return false; }
    m->index = r16(&p[0]); m->set = p[2]; m->n = p[3]; m->rec = &p[4];
    return true;
}

size_t proto_pack_text(const char *s, size_t n, uint8_t *o)
{
    if (n == 0U || n > PROTO_PAYLOAD_MAX) { return 0U; }
    for (size_t k = 0U; k < n; k++) { o[k] = (uint8_t)s[k]; }
    return n;
}
