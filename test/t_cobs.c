#include <string.h>

#include "par.h"
#include "proto.h"
#include "test.h"

static uint32_t rng_s = 0x12345678U;
static uint32_t rnd(void)
{
    rng_s ^= rng_s << 13;
    rng_s ^= rng_s >> 17;
    rng_s ^= rng_s << 5;
    return rng_s;
}

/* encode must give exp, decode must give the input back */
static void vec(const uint8_t *in, size_t n, const uint8_t *exp, size_t m)
{
    uint8_t e[300], d[300];
    size_t  dn = 0U;
    const size_t en = cobs_encode(in, n, e, sizeof e);
    CHECK(en == m && memcmp(e, exp, m) == 0);
    CHECK(cobs_decode(e, en, d, sizeof d, &dn) && dn == n && memcmp(d, in, n) == 0);
}

static void feed(proto_rx_t *rx, const uint8_t *b, size_t n)
{
    for (size_t i = 0U; i < n; i++) { proto_rx_byte(rx, b[i]); }
}

void t_cobs(void)
{
    uint8_t in[600], e[700], d[700];
    size_t  dn = 0U;

    REQ("REQ-PRO-01: COBS known vectors (Cheshire/Baker, Wikipedia table)");
    { const uint8_t i[] = { 0x00 }, x[] = { 0x01, 0x01 }; vec(i, 1U, x, 2U); }
    { const uint8_t i[] = { 0x00, 0x00 }, x[] = { 0x01, 0x01, 0x01 }; vec(i, 2U, x, 3U); }
    { const uint8_t i[] = { 0x00, 0x11, 0x00 }, x[] = { 0x01, 0x02, 0x11, 0x01 }; vec(i, 3U, x, 4U); }
    { const uint8_t i[] = { 0x11, 0x22, 0x00, 0x33 }, x[] = { 0x03, 0x11, 0x22, 0x02, 0x33 }; vec(i, 4U, x, 5U); }
    { const uint8_t i[] = { 0x11, 0x22, 0x33, 0x44 }, x[] = { 0x05, 0x11, 0x22, 0x33, 0x44 }; vec(i, 4U, x, 5U); }
    { const uint8_t i[] = { 0x11, 0x00, 0x00, 0x00 }, x[] = { 0x02, 0x11, 0x01, 0x01, 0x01 }; vec(i, 4U, x, 5U); }
    { const uint8_t x[] = { 0x01 }; vec(in, 0U, x, 1U); }                       /* 0 bytes */

    REQ("REQ-PRO-01: COBS 254/255-byte runs");
    for (unsigned k = 0U; k < 255U; k++) { in[k] = (uint8_t)(k + 1U); }          /* 01..FF */
    e[0] = 0xFFU; memcpy(&e[1], in, 254U);
    vec(in, 254U, e, 255U);                                                       /* FF 01..FE */
    e[255] = 0x02U; e[256] = 0xFFU;
    vec(in, 255U, e, 257U);                                                       /* FF 01..FE 02 FF */
    in[254] = 0x00U;
    e[255] = 0x01U; e[256] = 0x01U;
    vec(in, 255U, e, 257U);                                                       /* FF 01..FE 01 01 */
    /* decoder also takes the form with a trailing 0x01 after a full block */
    e[0] = 0xFFU; memcpy(&e[1], in, 254U); e[255] = 0x01U;
    CHECK(cobs_decode(e, 256U, d, sizeof d, &dn) && dn == 254U && memcmp(d, in, 254U) == 0);

    REQ("REQ-PRO-01: COBS all-zero and no-zero buffers, 0..101 bytes");
    for (size_t n = 0U; n <= PROTO_RAW_MAX; n++) {
        memset(in, 0, n);
        size_t en = cobs_encode(in, n, e, sizeof e);
        CHECK(en == n + 1U && cobs_decode(e, en, d, sizeof d, &dn) && dn == n && memcmp(d, in, n) == 0);
        for (size_t k = 0U; k < en; k++) { CHECK(e[k] == 0x01U); }
        for (size_t k = 0U; k < n; k++) { in[k] = (uint8_t)(1U + k % 255U); }
        en = cobs_encode(in, n, e, sizeof e);
        CHECK(en == n + 1U && cobs_decode(e, en, d, sizeof d, &dn) && dn == n && memcmp(d, in, n) == 0);
    }

    REQ("t_cobs: round trip of 10^4 random buffers 0..101 B, plus 2000 of up to 600 B");
    unsigned bad = 0U, zero_in_enc = 0U;
    for (unsigned it = 0U; it < 12000U; it++) {
        const size_t n = (it < 10000U) ? rnd() % (PROTO_RAW_MAX + 1U) : rnd() % 601U;
        const uint32_t dens = rnd() % 4U;                       /* zero density: 0, 1/4, 1/2, all but few */
        for (size_t k = 0U; k < n; k++) {
            const uint8_t b = (uint8_t)rnd();
            in[k] = (dens == 0U) ? (uint8_t)(b | 1U) : ((dens == 1U) ? ((b & 3U) ? b : 0U)
                                                     : ((dens == 2U) ? ((b & 1U) ? b : 0U) : ((b & 15U) ? 0U : b)));
        }
        const size_t en = cobs_encode(in, n, e, sizeof e);
        for (size_t k = 0U; k < en; k++) { if (e[k] == 0U) { zero_in_enc++; } }
        if (!(en >= 1U && en <= n + 1U + n / 254U && cobs_decode(e, en, d, sizeof d, &dn) && dn == n &&
              (n == 0U || memcmp(d, in, n) == 0))) { bad++; }
    }
    CHECK(bad == 0U);
    CHECK(zero_in_enc == 0U);

    REQ("REQ-PRO-03: decoder rejects malformed COBS, encoder rejects a short output buffer");
    { const uint8_t x[] = { 0x00 }; CHECK(!cobs_decode(x, 1U, d, sizeof d, &dn)); }
    { const uint8_t x[] = { 0x03, 0x11 }; CHECK(!cobs_decode(x, 2U, d, sizeof d, &dn)); }             /* block past end */
    { const uint8_t x[] = { 0x03, 0x11, 0x00 }; CHECK(!cobs_decode(x, 3U, d, sizeof d, &dn)); }       /* zero in block  */
    { const uint8_t x[] = { 0x05, 0x11, 0x22, 0x33, 0x44 }; CHECK(!cobs_decode(x, 5U, d, 3U, &dn)); } /* out too small  */
    CHECK(!cobs_decode(in, 0U, d, sizeof d, &dn));                                                   /* empty input    */
    in[0] = 1U; in[1] = 2U; in[2] = 3U;
    CHECK(cobs_encode(in, 3U, e, 3U) == 0U && cobs_encode(in, 3U, e, 4U) == 4U && cobs_encode(in, 0U, e, 0U) == 0U);

    REQ("REQ-PRO-01/03: frame encode, decode, back-to-back frames, idle delimiters");
    (void)par_init();
    proto_rx_t rx;
    proto_frame_t f;
    proto_rx_init(&rx, 0x42U);
    uint8_t pl[PROTO_PAYLOAD_MAX], fr[PROTO_FRAME_MAX + 2U];
    unsigned mism = 0U;
    for (unsigned it = 0U; it < 10000U; it++) {
        const size_t n = (it < PROTO_PAYLOAD_MAX + 1U) ? it : rnd() % (PROTO_PAYLOAD_MAX + 1U);
        for (size_t k = 0U; k < n; k++) {
            pl[k] = (it % 3U == 0U) ? 0U : ((it % 3U == 1U) ? (uint8_t)(1U + rnd() % 255U) : (uint8_t)rnd());
        }
        const uint8_t seq = (uint8_t)rnd();
        const size_t l = proto_encode_vid(0x42U, 0x02U, seq, pl, n, fr, sizeof fr);
        proto_rx_byte(&rx, 0U);                                  /* idle delimiter before */
        feed(&rx, fr, l);
        if (!(l >= 7U + n && l <= PROTO_FRAME_MAX && proto_rx_frame(&rx, &f) && f.vid == 0x42U && f.type == 0x02U &&
              f.seq == seq && f.len == n && (n == 0U || memcmp(f.payload, pl, n) == 0) &&
              !proto_rx_frame(&rx, &f))) { mism++; }
    }
    CHECK(mism == 0U);
    CHECK(rx.st.ok == 10000U && rx.st.crc == 0U && rx.st.cobs == 0U && rx.st.oversize == 0U && rx.st.overrun == 0U);
    CHECK(proto_encode_vid(0x42U, 2U, 0U, pl, PROTO_PAYLOAD_MAX + 1U, fr, sizeof fr) == 0U);
    CHECK(proto_encode_vid(0x42U, 2U, 0U, pl, 10U, fr, 10U) == 0U);          /* output too small */
    {   /* longest frame: 96 B payload without zeros: raw 101, COBS 102, + delimiter */
        memset(pl, 0xA5, sizeof pl);
        CHECK(proto_encode_vid(0x42U, 2U, 0U, pl, 96U, fr, sizeof fr) <= PROTO_FRAME_MAX);
    }

    REQ("REQ-PRO-03: corrupted frames are dropped and counted (CRC, COBS, oversize, runt, wrong vid)");
    proto_rx_init(&rx, 0x42U);
    unsigned accepted = 0U;
    for (unsigned it = 0U; it < 5000U; it++) {
        const size_t n = rnd() % (PROTO_PAYLOAD_MAX + 1U);
        for (size_t k = 0U; k < n; k++) { pl[k] = (uint8_t)rnd(); }
        const size_t l = proto_encode_vid(0x42U, 0x02U, (uint8_t)it, pl, n, fr, sizeof fr);
        fr[rnd() % (l - 1U)] ^= (uint8_t)(1U << (rnd() % 8U));   /* one flipped bit, delimiter untouched */
        feed(&rx, fr, l);
        proto_rx_byte(&rx, 0U);
        while (proto_rx_frame(&rx, &f)) { accepted++; }
    }
    CHECK(accepted == 0U);
    CHECK(rx.st.crc + rx.st.cobs + rx.st.runt + rx.st.oversize + rx.st.vid > 4000U);
    {   /* truncated frame, then a good one: the good one still arrives */
        proto_rx_init(&rx, 0x42U);
        const size_t l = proto_encode_vid(0x42U, 0x20U, 1U, NULL, 0U, fr, sizeof fr);
        feed(&rx, fr, l - 3U);
        proto_rx_byte(&rx, 0U);
        feed(&rx, fr, l);
        CHECK(proto_rx_frame(&rx, &f) && f.type == 0x20U && !proto_rx_frame(&rx, &f));
        CHECK(rx.st.crc + rx.st.cobs + rx.st.runt == 1U);
    }
    {   /* oversize: 102 raw bytes with a valid CRC, and 104 non-zero bytes before a delimiter */
        proto_rx_init(&rx, 0x42U);
        uint8_t raw[110];
        memset(raw, 0x55, sizeof raw);
        raw[0] = 0x42U; raw[1] = 0x02U; raw[2] = 0U;
        const uint16_t crc = crc16_ccitt(raw, 100U);
        raw[100] = (uint8_t)(crc & 0xFFU); raw[101] = (uint8_t)(crc >> 8);
        const size_t en = cobs_encode(raw, 102U, e, sizeof e);
        feed(&rx, e, en);
        proto_rx_byte(&rx, 0U);
        CHECK(rx.st.oversize == 1U && !proto_rx_frame(&rx, &f));
        memset(e, 0x11, 104U);
        feed(&rx, e, 104U);
        proto_rx_byte(&rx, 0U);
        CHECK(rx.st.oversize == 2U && !proto_rx_frame(&rx, &f));
        const size_t l = proto_encode_vid(0x42U, 0x20U, 1U, NULL, 0U, fr, sizeof fr);   /* recovers */
        feed(&rx, fr, l);
        CHECK(proto_rx_frame(&rx, &f));
    }
    {   /* runt: valid CRC but fewer than 5 raw bytes */
        proto_rx_init(&rx, 0x42U);
        const uint8_t raw[] = { 0x42, 0x20, 0x00, 0x00 };
        const size_t en = cobs_encode(raw, 4U, e, sizeof e);
        feed(&rx, e, en);
        proto_rx_byte(&rx, 0U);
        CHECK(rx.st.runt == 1U && !proto_rx_frame(&rx, &f));
    }
    {   /* vid filter; STOP broadcast 0xFF is honoured, other types with 0xFF are not */
        proto_rx_init(&rx, 0x42U);
        size_t l = proto_encode_vid(0x43U, 0x20U, 1U, NULL, 0U, fr, sizeof fr);
        feed(&rx, fr, l);
        CHECK(!proto_rx_frame(&rx, &f) && rx.st.vid == 1U);
        l = proto_encode_vid(0xFFU, (uint8_t)PROTO_T_STOP, 2U, NULL, 0U, fr, sizeof fr);
        feed(&rx, fr, l);
        CHECK(proto_rx_frame(&rx, &f) && f.type == PROTO_T_STOP && f.vid == 0xFFU);
        l = proto_encode_vid(0xFFU, (uint8_t)PROTO_T_MODE, 3U, pl, 1U, fr, sizeof fr);
        feed(&rx, fr, l);
        CHECK(!proto_rx_frame(&rx, &f) && rx.st.vid == 2U);
        l = proto_encode_vid(0x42U, (uint8_t)PROTO_T_STOP, 4U, NULL, 0U, fr, sizeof fr);
        feed(&rx, fr, l);
        CHECK(proto_rx_frame(&rx, &f) && f.vid == 0x42U);
        /* proto_encode takes com_vid from par() */
        l = proto_encode((uint8_t)PROTO_T_PING, 5U, NULL, 0U, fr, sizeof fr);
        feed(&rx, fr, l);
        CHECK(proto_rx_frame(&rx, &f) && f.vid == (uint8_t)par()->com_vid);
    }
    {   /* overrun: second frame before the first was read */
        proto_rx_init(&rx, 0x42U);
        size_t l = proto_encode_vid(0x42U, 0x20U, 1U, NULL, 0U, fr, sizeof fr);
        feed(&rx, fr, l);
        l = proto_encode_vid(0x42U, 0x20U, 2U, NULL, 0U, fr, sizeof fr);
        feed(&rx, fr, l);
        CHECK(rx.st.overrun == 1U && proto_rx_frame(&rx, &f) && f.seq == 2U);
    }

    REQ("REQ-PRO-02: #TEXT + CR/LF in the raw stream");
    {
        static const char *const yes[] = { "#TEXT\r\n", "#TEXT\n", "#TEXT\r", "xx##TEXT\r\n", "\x01\x02#TEXT\n" };
        for (unsigned i = 0U; i < sizeof yes / sizeof yes[0]; i++) {
            proto_rx_init(&rx, 0x42U);
            feed(&rx, (const uint8_t *)yes[i], strlen(yes[i]));
            CHECK(proto_rx_text_request(&rx));
            CHECK(!proto_rx_text_request(&rx));                  /* cleared by the read */
        }
        static const char *const no[] = { "#TEXT", "#TEX\r\n", "#TEXTX\r\n", "TEXT\r\n", "#TEST\r\n", "# TEXT\r\n" };
        for (unsigned i = 0U; i < sizeof no / sizeof no[0]; i++) {
            proto_rx_init(&rx, 0x42U);
            feed(&rx, (const uint8_t *)no[i], strlen(no[i]));
            CHECK(!proto_rx_text_request(&rx));
        }
        /* split across a frame: bytes of a binary frame never trigger it by accident */
        proto_rx_init(&rx, 0x42U);
        for (unsigned it = 0U; it < 2000U; it++) {
            const size_t l = proto_encode_vid(0x42U, 0x02U, (uint8_t)it, pl, rnd() % 97U, fr, sizeof fr);
            feed(&rx, fr, l);
        }
        CHECK(!proto_rx_text_request(&rx));
    }
}
