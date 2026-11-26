/* tlm.c (vehicle part) on the host against stubbed com_uart / sys_*: reply timing in the radio
 * window, duplicates, mode switches, USB events and the CAP_DATA stream. Own binary (`t_link`):
 * t_con.c stubs the same com_* symbols for the console test. */
#include <stdio.h>
#include <string.h>

#include "com_uart.h"
#include "par.h"
#include "sys_fault.h"
#include "sys_time.h"
#include "test.h"
#include "tlm.h"

int g_checks, g_fails;

/* ---- stubs ------------------------------------------------------------------------ */
static uint8_t    g_tx[2][8192];
static size_t     g_tx_n[2], g_free[2] = { 4096, 4096 };
static com_mode_t g_mode[2];
static uint32_t   g_now;
static sys_event_rec_t g_ev[8];
static uint32_t   g_ev_n;

size_t com_write(com_port_t p, const uint8_t *d, size_t n)
{
    if (n > g_free[p] || g_tx_n[p] + n > sizeof g_tx[p]) { return 0U; }
    memcpy(&g_tx[p][g_tx_n[p]], d, n);
    g_tx_n[p] += n;
    return n;
}
size_t      com_tx_free(com_port_t p) { return g_free[p]; }
com_mode_t  com_mode(com_port_t p) { return g_mode[p]; }
void        com_set_mode(com_port_t p, com_mode_t m) { g_mode[p] = m; }
com_stats_t com_stats(com_port_t p) { (void)p; return (com_stats_t){ 0 }; }
uint32_t    sys_ms(void) { return g_now; }
uint32_t    sys_event_total(void) { return g_ev_n; }
bool        sys_event_nth(uint32_t k, sys_event_rec_t *o) { if (k >= g_ev_n) { return false; } *o = g_ev[k % 8U]; return true; }

/* application side: records what tlm hands over */
static int g_exec;
void tlm_exec(unsigned port, const proto_frame_t *f)
{
    g_exec++;
    switch (f->type) {
    case PROTO_T_PING:      tlm_ack_hello(port, f); break;
    case PROTO_T_PARAM_GET: tlm_param_val(port, 902U, 100.0f, ST_OK); break;
    case PROTO_T_MODE_TEXT: tlm_mode_text(port, f); break;
    case PROTO_T_CAP_GET:   { proto_cap_get_t g; (void)proto_unpack_cap_get(f->payload, f->len, &g); tlm_cap_get(port, f, g.from, g.count); } break;
    case PROTO_T_DRIVE:     break;                                   /* no reply */
    default:                tlm_ack(port, f, ST_OK); break;
    }
}
void tlm_fill_fast(tlm_fast_in_t *in) { *in = (tlm_fast_in_t){ .t_ms = g_now, .x_m = 1.0f, .fwd_m = -1.0f }; }
void tlm_fill_slow(tlm_slow_in_t *in) { *in = (tlm_slow_in_t){ .t_ms = g_now, .bat_v = 12.0f }; }

/* ---- helpers ---------------------------------------------------------------------- */
typedef struct { uint8_t type[32], seq[32]; proto_frame_t f[32]; int n; uint32_t bad; } got_t;

/* Decodes and clears what port p sent. */
static got_t take(com_port_t p)
{
    got_t g = { 0 };
    proto_rx_t rx;
    proto_rx_init(&rx, 0x42U);
    for (size_t i = 0; i < g_tx_n[p]; i++) {
        proto_rx_byte(&rx, g_tx[p][i]);
        if (proto_rx_frame(&rx, &g.f[g.n]) && g.n < 32) { g.type[g.n] = g.f[g.n].type; g.seq[g.n] = g.f[g.n].seq; g.n++; }
    }
    g.bad = rx.st.cobs + rx.st.crc + rx.st.oversize + rx.st.runt;
    g_tx_n[p] = 0U;
    return g;
}

static void uplink(com_port_t p, uint8_t vid, uint8_t type, uint8_t seq, const uint8_t *pl, size_t n)
{
    uint8_t f[PROTO_FRAME_MAX];
    const size_t len = proto_encode_vid(vid, type, seq, pl, n, f, sizeof f);
    for (size_t i = 0; i < len; i++) { tlm_rx_byte((unsigned)p, f[i]); }
}

/* Runs tlm_poll for ms milliseconds, 1 kHz. */
static void run(uint32_t ms) { for (uint32_t i = 0; i < ms; i++) { g_now++; tlm_poll(); } }

static void event(uint16_t code) { g_ev[g_ev_n % 8U] = (sys_event_rec_t){ .t_ms = g_now, .code = code, .arg = 1 }; g_ev_n++; }

int main(void)
{
    (void)par_init();
    got_t g;

    REQ("§8.1, REQ-COM-06: com1_mode 1 puts the radio in BINARY at init; boot HELLO in the first window, no telemetry");
    g_now = 1000U;
    tlm_init(19200U);
    proto_hello_t h = { .proto_ver = 1U, .fw_minor = 4U };
    tlm_set_hello(&h);
    CHECK(g_mode[COM_RADIO] == COM_BINARY && g_mode[COM_USB] == COM_TEXT);
    run(99);
    CHECK(g_tx_n[COM_RADIO] == 0U);
    run(1);
    g = take(COM_RADIO);
    CHECK(g.n == 1 && g.type[0] == PROTO_T_HELLO && g.seq[0] == 0U);
    CHECK(g_tx_n[COM_USB] == 0U);                                   /* TEXT port: nothing binary */

    REQ("REQ-PRO-04, §8.3: TELEM_FAST every 100 ms, TELEM_SLOW in every 10th period; one seq counter");
    run(100);
    g = take(COM_RADIO);
    CHECK(g.n == 1 && g.type[0] == PROTO_T_TELEM_FAST && g.seq[0] == 1U);
    int slow = 0, fast = 0;
    for (int k = 0; k < 10; k++) {
        run(100);
        g = take(COM_RADIO);
        for (int i = 0; i < g.n; i++) { slow += g.type[i] == PROTO_T_TELEM_SLOW; fast += g.type[i] == PROTO_T_TELEM_FAST; }
    }
    CHECK(slow == 1 && fast == 9);

    REQ("REQ-PRO-04, D-34: the radio reply waits for the next window and goes first, then the telemetry");
    uplink(COM_RADIO, 0x42U, PROTO_T_STOP, 7U, NULL, 0U);
    run(1);
    CHECK(g_exec == 1 && g_tx_n[COM_RADIO] == 0U);
    run(99);
    g = take(COM_RADIO);
    CHECK(g.n == 2 && g.type[0] == PROTO_T_ACK && g.f[0].payload[1] == 7U && g.type[1] == PROTO_T_TELEM_FAST);

    REQ("REQ-PRO-05: a repeated command (same type, seq) within 1 s is acknowledged again, not executed");
    uplink(COM_RADIO, 0x42U, PROTO_T_STOP, 7U, NULL, 0U);
    run(100);
    g = take(COM_RADIO);
    CHECK(g_exec == 1 && g.n == 2 && g.type[0] == PROTO_T_ACK);
    uplink(COM_RADIO, 0x42U, PROTO_T_DRIVE, 8U, (const uint8_t[]){ 0, 0, 0, 0, 0 }, 5U);
    run(100);
    uplink(COM_RADIO, 0x42U, PROTO_T_DRIVE, 8U, (const uint8_t[]){ 0, 0, 0, 0, 0 }, 5U);   /* duplicate DRIVE: no reply, not the old ACK */
    run(100);
    g = take(COM_RADIO);
    CHECK(g_exec == 2 && g.n == 2 && g.type[0] != PROTO_T_ACK && g.type[1] != PROTO_T_ACK);

    REQ("REQ-PRO-06: STOP with vid 0xFF is executed; other commands with another vid are not");
    uplink(COM_RADIO, 0xFFU, PROTO_T_STOP, 9U, NULL, 0U);
    run(1);
    CHECK(g_exec == 3);
    uplink(COM_RADIO, 0x43U, PROTO_T_PING, 10U, NULL, 0U);
    run(1);
    CHECK(g_exec == 3);
    run(98);
    (void)take(COM_RADIO);

    REQ("§8.3: PING -> ACK + HELLO, the HELLO replaces that period's telemetry");
    uplink(COM_RADIO, 0x42U, PROTO_T_PING, 11U, NULL, 0U);
    run(100);
    g = take(COM_RADIO);
    CHECK(g.n == 2 && g.type[0] == PROTO_T_ACK && g.type[1] == PROTO_T_HELLO);

    REQ("REQ-TLM-03, §8.3: radio events only inside the window, behind the telemetry, as many as fit");
    while (((g_now - 1000U) / 100U) % 10U != 0U) { run(100); }       /* next window carries TELEM_FAST */
    (void)take(COM_RADIO);
    for (uint16_t k = 0; k < 5; k++) { event((uint16_t)(0x100U + k)); }
    run(1);
    CHECK(g_tx_n[COM_RADIO] == 0U);
    run(99);
    g = take(COM_RADIO);
    CHECK(g.n == 4 && g.type[0] == PROTO_T_TELEM_FAST && g.type[1] == PROTO_T_EVENT && g.type[3] == PROTO_T_EVENT);   /* 3 fit at 19200 Bd */
    run(100);
    g = take(COM_RADIO);
    CHECK(g.n == 3 && g.type[1] == PROTO_T_EVENT && g.type[2] == PROTO_T_EVENT);    /* the other 2, oldest first */
    CHECK(g.f[1].payload[4] == 0x03U && g.f[2].payload[4] == 0x04U);

    REQ("§8.1, REQ-PRO-02: MODE_TEXT on the radio: ACK in the next window, then TEXT; #TEXT switches at once");
    uplink(COM_RADIO, 0x42U, PROTO_T_MODE_TEXT, 12U, NULL, 0U);
    run(1);
    CHECK(g_mode[COM_RADIO] == COM_BINARY);
    run(99);
    g = take(COM_RADIO);
    CHECK(g.n == 1 && g.type[0] == PROTO_T_ACK && g_mode[COM_RADIO] == COM_TEXT);
    run(200);
    CHECK(g_tx_n[COM_RADIO] == 0U);
    tlm_bin(TLM_PORT_RADIO);
    const char *esc = "xx#TEXT\r";
    for (const char *c = esc; *c; c++) { tlm_rx_byte(TLM_PORT_RADIO, (uint8_t)*c); }
    run(1);
    CHECK(g_mode[COM_RADIO] == COM_TEXT);
    g_tx_n[COM_RADIO] = 0U;

    REQ("§8.1, §8.3: BIN on USB: delimiter + HELLO at once; replies and events at once; telemetry every period");
    g_tx[COM_USB][0] = 'T';
    g_tx_n[COM_USB] = 1U;                                             /* text before the switch */
    tlm_bin(TLM_PORT_USB);
    g = take(COM_USB);
    CHECK(g.n == 1 && g.type[0] == PROTO_T_HELLO && g.bad == 1U);      /* the text ends as one bad frame */
    uplink(COM_USB, 0x42U, PROTO_T_PARAM_GET, 1U, (const uint8_t[]){ 0x86, 0x03 }, 2U);
    event(0x200U);
    run(1);
    g = take(COM_USB);
    CHECK(g.n == 2 && g.type[0] == PROTO_T_PARAM_VAL && g.type[1] == PROTO_T_EVENT);
    run(100);
    g = take(COM_USB);
    CHECK(g.n == 1 && (g.type[0] == PROTO_T_TELEM_FAST || g.type[0] == PROTO_T_TELEM_SLOW));

    REQ("REQ-TLM-02, D-33: CAP_GET streams n_max records per frame, then the end frame; clipped to the records held");
    CHECK(tlm_capture_start(0U, 1U, 0U) == ST_OK);
    tlm_cap_t *cap = tlm_capture();
    uint8_t rec[24];
    for (uint8_t k = 0; k < 7; k++) { memset(rec, k, sizeof rec); if (tlm_cap_want(cap, 0U, g_now)) { tlm_cap_put(cap, rec); } }
    uplink(COM_USB, 0x42U, PROTO_T_CAP_GET, 2U, (const uint8_t[]){ 1, 0, 0xFF, 0xFF }, 4U);   /* from 1, count 65535 */
    run(5);
    g = take(COM_USB);
    CHECK(g.n == 4 && g.type[0] == PROTO_T_ACK && g.type[1] == PROTO_T_CAP_DATA);
    proto_cap_data_t m;
    CHECK(proto_unpack_cap_data(g.f[1].payload, g.f[1].len, &m) && m.index == 1U && m.n == 3U && m.rec[0] == 1U);
    CHECK(proto_unpack_cap_data(g.f[3].payload, g.f[3].len, &m) && m.index == 7U && m.n == 0U);   /* 3 + 3, then end */

    REQ("REQ-TLM-02: the stream waits while the USB buffer cannot take a frame and a telemetry frame");
    uplink(COM_USB, 0x42U, PROTO_T_CAP_GET, 3U, (const uint8_t[]){ 0, 0, 7, 0 }, 4U);
    g_free[COM_USB] = 100U;                                           /* < 83 + 42 */
    run(3);
    g = take(COM_USB);
    CHECK(g.n == 1 && g.type[0] == PROTO_T_ACK);
    g_free[COM_USB] = 4096U;
    run(4);                                                           /* one frame per pass: 3 + 3 + 1, end */
    g = take(COM_USB);
    CHECK(g.n >= 4 && g.type[0] == PROTO_T_CAP_DATA && g.type[3] == PROTO_T_CAP_DATA);

    printf("t_link: %d checks, %d failed\n", g_checks, g_fails);
    return g_fails != 0;
}
