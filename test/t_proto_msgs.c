#include <string.h>

#include "par.h"
#include "proto.h"
#include "test.h"

static unsigned hexv(const char *s, uint8_t *out)
{
    unsigned n = 0U;
    for (; s[0] != '\0'; s += 2) {
        unsigned v = 0U;
        for (unsigned k = 0U; k < 2U; k++) {
            const char c = s[k];
            v = (v << 4) | (unsigned)((c >= 'a') ? (c - 'a' + 10) : (c - '0'));
        }
        out[n++] = (uint8_t)v;
    }
    return n;
}

static uint32_t at32(const uint8_t *p, unsigned o)
{
    return (uint32_t)p[o] | ((uint32_t)p[o + 1U] << 8) | ((uint32_t)p[o + 2U] << 16) | ((uint32_t)p[o + 3U] << 24);
}
static uint16_t at16(const uint8_t *p, unsigned o) { return (uint16_t)(p[o] | (p[o + 1U] << 8)); }

/* One frame must equal the byte vector shared with tools/gs_log.py --selftest. */
static void frame_vec(const char *hex, uint8_t vid, uint8_t type, uint8_t seq, const uint8_t *pl, size_t n)
{
    uint8_t want[PROTO_FRAME_MAX], got[PROTO_FRAME_MAX];
    const unsigned wn = hexv(hex, want);
    const size_t gn = proto_encode_vid(vid, type, seq, pl, n, got, sizeof got);
    CHECK(gn == wn && memcmp(got, want, wn) == 0);
    proto_rx_t rx;
    proto_frame_t f;
    proto_rx_init(&rx, 0x42U);
    for (unsigned i = 0U; i < wn; i++) { proto_rx_byte(&rx, want[i]); }
    CHECK(proto_rx_frame(&rx, &f) && f.vid == vid && f.type == type && f.seq == seq && f.len == n &&
          (n == 0U || memcmp(f.payload, pl, n) == 0));
}

void t_proto_msgs(void)
{
    uint8_t p[PROTO_PAYLOAD_MAX], q[PROTO_PAYLOAD_MAX];

    REQ("REQ-PRO-01: CRC-16/CCITT-FALSE check value, little-endian on the wire");
    CHECK(crc16_ccitt((const uint8_t *)"123456789", 9U) == 0x29B1U);
    {
        uint8_t fr[PROTO_FRAME_MAX], d[PROTO_FRAME_MAX];
        size_t dn = 0U;
        const size_t l = proto_encode_vid(0x42U, 0x20U, 1U, NULL, 0U, fr, sizeof fr);
        CHECK(cobs_decode(fr, l - 1U, d, sizeof d, &dn) && dn == 5U);
        const uint16_t crc = crc16_ccitt(d, 3U);
        CHECK(d[3] == (crc & 0xFFU) && d[4] == (crc >> 8));        /* crc_lo, crc_hi */
    }

    REQ("REQ-PRO-01: known frame vectors (same bytes as tools/gs_log.py --selftest)");
    {
        uint8_t pl[PROTO_PAYLOAD_MAX];
        proto_ack_t a = { 0x22U, 7U, 0U };
        proto_param_val_t pv = { 113U, 2.0f, 0U };
        proto_event_t e = { 0x12345U, 0x0100U, -2 };
        proto_telem_fast_t t = { 123456U, 3U, 4U, 0x0A05U, 1234567, -7654321, -18000, 450, -1234, 500, -250,
                                 7U, 9U, -33, 0x1BU, 87U };
        frame_vec("0642200196a900", 0x42U, 0x20U, 1U, NULL, 0U);
        frame_vec("06ff21034b0600", 0xFFU, 0x21U, 3U, NULL, 0U);
        size_t n = proto_pack_ack(&a, pl);
        frame_vec("06420505220703630100", 0x42U, 0x05U, 5U, pl, n);
        n = proto_pack_param_val(&pv, pl);
        frame_vec("0542060971010101024003e47f00", 0x42U, 0x06U, 9U, pl, n);
        n = proto_pack_event(&e, pl);
        frame_vec("03420404452301010801feffffffe5cd00", 0x42U, 0x04U, 0U, pl, n);
        n = proto_pack_telem_fast(&t, pl);
        frame_vec("074202ff40e201080304050a87d612174f348bffb0b9c2012efbf40106ff0709dfff1b57e46b00",
                  0x42U, 0x02U, 255U, pl, n);
    }

    REQ("REQ-PRO-01: TELEM_FAST layout (32 B, §8.4 offsets, little-endian, signed fields)");
    {
        const proto_telem_fast_t m = { 0x01020304U, 5U, 6U, 0x0A0BU, -2, INT32_MIN, -18000, 32767, -32768, -1, 1,
                                       0x77U, 0x88U, -300, 0x2DU, 255U };
        proto_telem_fast_t r;
        CHECK(proto_pack_telem_fast(&m, p) == 32U && PROTO_LEN_TELEM_FAST == 32);
        CHECK(at32(p, 0) == 0x01020304U && p[0] == 0x04U && p[4] == 5U && p[5] == 6U && at16(p, 6) == 0x0A0BU);
        CHECK(at32(p, 8) == 0xFFFFFFFEU && at32(p, 12) == 0x80000000U);
        CHECK(at16(p, 16) == (uint16_t)-18000 && at16(p, 18) == 32767U && at16(p, 20) == 0x8000U);
        CHECK(at16(p, 22) == 0xFFFFU && at16(p, 24) == 1U && p[26] == 0x77U && p[27] == 0x88U);
        CHECK(at16(p, 28) == (uint16_t)-300 && p[30] == 0x2DU && p[31] == 255U);
        CHECK(proto_unpack_telem_fast(p, 32U, &r) && r.t_ms == m.t_ms && r.mode == 5U && r.state == 6U &&
              r.flags == m.flags && r.x == -2 && r.y == INT32_MIN && r.th == -18000 && r.v == 32767 &&
              r.w == -32768 && r.duty_l == -1 && r.duty_r == 1 && r.seg == 0x77U && r.wp == 0x88U &&
              r.e_y == -300 && r.gnd == 0x2DU && r.fwd == 255U);
        CHECK(!proto_unpack_telem_fast(p, 31U, &r) && !proto_unpack_telem_fast(p, 33U, &r));
    }

    REQ("REQ-PRO-01, D-34: TELEM_SLOW is 35 bytes with ev_drop at offset 34");
    {
        const proto_telem_slow_t m = { 0xA1B2C3D4U, 12100U, 11U, 12U, 13U, -14, -15, -16, 0x1234U, 2U, 37U,
                                       0x2222U, 0x3333U, 0x4444U, 0x5555U, 0x6666U, 0x0FU, 0x07U, 200U };
        proto_telem_slow_t r;
        CHECK(proto_pack_telem_slow(&m, p) == 35U && PROTO_LEN_TELEM_SLOW == 35);
        CHECK(at32(p, 0) == 0xA1B2C3D4U && at16(p, 4) == 12100U && at16(p, 6) == 11U && at16(p, 8) == 12U &&
              at16(p, 10) == 13U && at16(p, 12) == (uint16_t)-14 && at16(p, 14) == (uint16_t)-15 &&
              at16(p, 16) == (uint16_t)-16 && at16(p, 18) == 0x1234U && p[20] == 2U && p[21] == 37U &&
              at16(p, 22) == 0x2222U && at16(p, 24) == 0x3333U && at16(p, 26) == 0x4444U &&
              at16(p, 28) == 0x5555U && at16(p, 30) == 0x6666U && p[32] == 0x0FU && p[33] == 0x07U &&
              p[34] == 200U);
        CHECK(proto_unpack_telem_slow(p, 35U, &r) && r.t_ms == m.t_ms && r.bat == 12100U && r.bg == -14 &&
              r.scale == -15 && r.imu_t == -16 && r.fault_first == 0x1234U && r.n_faults == 2U && r.cpu == 37U &&
              r.ovr_ctrl == 0x2222U && r.rx_err == 0x3333U && r.ekf_acc == 0x4444U && r.ekf_rej == 0x5555U &&
              r.stack_free == 0x6666U && r.vl_alive == 0x0FU && r.sens == 7U && r.ev_drop == 200U &&
              r.sig_x == 11U && r.sig_y == 12U && r.sig_th == 13U);
        CHECK(!proto_unpack_telem_slow(p, 34U, &r));
    }

    REQ("REQ-PRO-01: HELLO (26 B), EVENT (10 B), ACK (3 B), PARAM_VAL (7 B)");
    {
        proto_hello_t h = { 1U, 2U, 3U, 4U, 0xDEADBEEFU, 1U, 0x0FU, { 'j', 'a', 'r', 'd', 'i', 'n', 0, 0 }, 0x11223344U,
                            0x08001234U }, hr;
        CHECK(proto_pack_hello(&h, p) == 26U);
        CHECK(p[0] == 1U && p[1] == 2U && p[2] == 3U && p[3] == 4U && at32(p, 4) == 0xDEADBEEFU && p[8] == 1U &&
              p[9] == 0x0FU && memcmp(&p[10], "jardin\0\0", 8U) == 0 && at32(p, 18) == 0x11223344U &&
              at32(p, 22) == 0x08001234U);
        CHECK(proto_unpack_hello(p, 26U, &hr) && hr.git_hash == 0xDEADBEEFU && hr.dirty == 1U &&
              memcmp(hr.course, "jardin\0\0", 8U) == 0 && hr.last_fault_pc == 0x08001234U &&
              hr.last_fault_code == 0x11223344U && hr.features == 0x0FU && hr.proto_ver == 1U);
        CHECK(!proto_unpack_hello(p, 25U, &hr));

        proto_event_t e = { 0xCAFEBABEU, 0xBEEFU, -123456 }, er;
        CHECK(proto_pack_event(&e, p) == 10U && at32(p, 0) == 0xCAFEBABEU && at16(p, 4) == 0xBEEFU &&
              (int32_t)at32(p, 6) == -123456);
        CHECK(proto_unpack_event(p, 10U, &er) && er.t_ms == e.t_ms && er.code == e.code && er.arg == e.arg);
        CHECK(!proto_unpack_event(p, 9U, &er));

        proto_ack_t a = { 0x25U, 200U, 5U }, ar;
        CHECK(proto_pack_ack(&a, p) == 3U && p[0] == 0x25U && p[1] == 200U && p[2] == 5U);
        CHECK(proto_unpack_ack(p, 3U, &ar) && ar.type == 0x25U && ar.seq == 200U && ar.result == 5U);
        CHECK(!proto_unpack_ack(p, 4U, &ar));

        proto_param_val_t v = { 0x0385U, -1.5f, 4U }, vr;
        CHECK(proto_pack_param_val(&v, p) == 7U && at16(p, 0) == 0x0385U && at32(p, 2) == 0xBFC00000U && p[6] == 4U);
        CHECK(proto_unpack_param_val(p, 7U, &vr) && vr.id == 0x0385U && vr.value == -1.5f && vr.result == 4U);
        CHECK(!proto_unpack_param_val(p, 6U, &vr));
    }

    REQ("REQ-PRO-01: uplink messages MODE, DRIVE, PARAM_GET, PARAM_SET, CAPTURE, CAP_GET");
    {
        uint8_t mode = 0U;
        CHECK(proto_pack_mode(3U, p) == 1U && p[0] == 3U && proto_unpack_mode(p, 1U, &mode) && mode == 3U);
        CHECK(!proto_unpack_mode(p, 2U, &mode));

        const proto_drive_t d = { -450, 1200, 0x80U };
        proto_drive_t dr;
        CHECK(proto_pack_drive(&d, p) == 5U && at16(p, 0) == (uint16_t)-450 && at16(p, 2) == 1200U && p[4] == 0x80U);
        CHECK(proto_unpack_drive(p, 5U, &dr) && dr.v == -450 && dr.w == 1200 && dr.flags == 0x80U);
        CHECK(!proto_unpack_drive(p, 4U, &dr));

        uint16_t id = 0U;
        CHECK(proto_pack_param_get(903U, p) == 2U && at16(p, 0) == 903U && proto_unpack_param_get(p, 2U, &id) && id == 903U);
        CHECK(!proto_unpack_param_get(p, 3U, &id));

        const proto_param_set_t s = { 111U, 8.98f };
        proto_param_set_t sr;
        CHECK(proto_pack_param_set(&s, p) == 6U && at16(p, 0) == 111U);
        CHECK(proto_unpack_param_set(p, 6U, &sr) && sr.id == 111U && sr.value == 8.98f);
        CHECK(!proto_unpack_param_set(p, 7U, &sr));

        const proto_capture_t c = { 1U, 2U, 600U };
        proto_capture_t cr;
        CHECK(proto_pack_capture(&c, p) == 4U && p[0] == 1U && p[1] == 2U && at16(p, 2) == 600U);
        CHECK(proto_unpack_capture(p, 4U, &cr) && cr.set == 1U && cr.decim == 2U && cr.duration == 600U);
        CHECK(!proto_unpack_capture(p, 3U, &cr));

        const proto_cap_get_t g = { 0x0102U, 0x0304U };
        proto_cap_get_t gr;
        CHECK(proto_pack_cap_get(&g, p) == 4U && at16(p, 0) == 0x0102U && at16(p, 2) == 0x0304U);
        CHECK(proto_unpack_cap_get(p, 4U, &gr) && gr.from == 0x0102U && gr.count == 0x0304U);
        CHECK(!proto_unpack_cap_get(p, 5U, &gr));
    }

    REQ("REQ-TLM-02, D-33: CAP_DATA packed at n_max per record size, payload <= 96 bytes");
    {
        uint8_t rec[5U * 24U];
        for (unsigned k = 0U; k < sizeof rec; k++) { rec[k] = (uint8_t)(k + 1U); }
        CHECK(proto_cap_rec_size(0U) == 24U && proto_cap_rec_size(1U) == 24U && proto_cap_rec_size(2U) == 24U &&
              proto_cap_rec_size(3U) == 16U && proto_cap_rec_size(4U) == 0U);
        CHECK(proto_cap_n_max(0U) == 3U && proto_cap_n_max(2U) == 3U && proto_cap_n_max(3U) == 5U &&
              proto_cap_n_max(4U) == 0U);
        for (uint8_t set = 0U; set <= 3U; set++) {
            const size_t sz = proto_cap_rec_size(set), nm = proto_cap_n_max(set);
            proto_cap_data_t m = { 0x0102U, set, (uint8_t)nm, rec }, r;
            const size_t len = proto_pack_cap_data(&m, p, sizeof p);
            CHECK(len == 4U + nm * sz && len <= PROTO_PAYLOAD_MAX);
            CHECK(len == ((set == 3U) ? 84U : 76U));
            CHECK(at16(p, 0) == 0x0102U && p[2] == set && p[3] == nm && memcmp(&p[4], rec, nm * sz) == 0);
            CHECK(proto_unpack_cap_data(p, len, &r) && r.index == 0x0102U && r.set == set && r.n == nm &&
                  memcmp(r.rec, rec, nm * sz) == 0);
            CHECK(!proto_unpack_cap_data(p, len - 1U, &r) && !proto_unpack_cap_data(p, len + 1U, &r));
            {   /* n_max + 1 fails on n_max, not on the buffer size */
                uint8_t big[200], rec2[6U * 24U] = { 0 };
                const proto_cap_data_t m2 = { 0U, set, (uint8_t)(nm + 1U), rec2 };
                CHECK(proto_pack_cap_data(&m2, big, sizeof big) == 0U);
                memset(big, 0, sizeof big);                                 /* 100-byte buffer, n = n_max + 1 */
                big[2] = set; big[3] = (uint8_t)(nm + 1U);
                CHECK(!proto_unpack_cap_data(big, 4U + (nm + 1U) * sz, &r));
            }
            m.n = 1U;
            CHECK(proto_pack_cap_data(&m, p, 4U + sz - 1U) == 0U);       /* out too small */
            m.n = 0U;                                                    /* end frame */
            CHECK(proto_pack_cap_data(&m, p, sizeof p) == 4U && p[3] == 0U &&
                  proto_unpack_cap_data(p, 4U, &r) && r.n == 0U);
        }
        const proto_cap_data_t bad = { 0U, 4U, 1U, rec };
        CHECK(proto_pack_cap_data(&bad, p, sizeof p) == 0U);
        CHECK(!proto_unpack_cap_data(p, 3U, &(proto_cap_data_t){ 0 }));
    }

    REQ("REQ-PRO-01: TEXT payload, frame wire sizes of §8.3 (payload + 7)");
    {
        CHECK(proto_pack_text("hi\n", 3U, p) == 3U && memcmp(p, "hi\n", 3U) == 0);
        CHECK(proto_pack_text("x", PROTO_PAYLOAD_MAX + 1U, p) == 0U);
        static const struct { uint8_t type; size_t payload, wire; } W[] = {
            { 0x02U, 32U, 39U }, { 0x03U, 35U, 42U }, { 0x05U, 3U, 10U }, { 0x06U, 7U, 14U },
            { 0x04U, 10U, 17U }, { 0x01U, 26U, 33U }, { 0x23U, 5U, 12U }, { 0x25U, 6U, 13U },
        };
        uint8_t fr[PROTO_FRAME_MAX];
        memset(q, 0x5A, sizeof q);                     /* no zero byte: COBS adds exactly one */
        for (unsigned i = 0U; i < sizeof W / sizeof W[0]; i++) {
            CHECK(PROTO_WIRE(W[i].payload) == W[i].wire);
            CHECK(proto_encode_vid(0x42U, W[i].type, 0U, q, W[i].payload, fr, sizeof fr) == W[i].wire);
        }
        (void)par_init();
        CHECK(proto_encode((uint8_t)PROTO_T_ACK, 0U, q, 3U, fr, sizeof fr) == 10U && (uint8_t)par()->com_vid == 0x42U);
    }
}
