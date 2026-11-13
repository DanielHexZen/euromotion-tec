/* calib.c — open-loop calibration firmware (spec §7.27, Appendix 19.2).
 * Console on USART2 (115200) mirrored on USART1 (HC-12). Lines starting with '#' are comments or
 * CSV headers. G and B also print the vehicle console's status line and `> <duty>` echoes, so
 * tools/ffit.py steady|breakaway reads the log unchanged (capture it like a phase-1 log).
 * Units on the wire: counts, mm, mm/s, duty in percent; every header names its unit. */
#include "app_config.h"

#if APP_CALIB

#include "calib.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "enc.h"
#include "fmt.h"
#include "mot.h"
#include "par.h"
#include "sys_fault.h"

#define HZ            (TICK_HZ / CTRL_DIV)          /* 50, TIM6 ARR is set to 19999 in main.c */
#define T_S(s)        ((uint16_t)((s) * (float)HZ))
#define DUTY_MAX      0.6f                          /* hard clamp on every command */
#define STALL_DUTY    0.35f                         /* at or above this a wheel must move ... */
#define STALL_TICKS   T_S(0.5f)                     /* ... within this time, else FAULT */
#define REST_TICKS    T_S(1.5f)
#define MOVE_CNT      10                            /* ffit.py breakaway: --move-counts */
#define C_DUTY        0.30f                         /* [DECIDE] procedure values, not parameters */
#define C_MIN_CNT     100
#define L_DUTY        0.25f
#define B_START_PCT   8
#define B_MAX_PCT     34                            /* below STALL_DUTY: B ends, it does not fault */
#define K_DEFAULT_PCT 40
#define K_N           100U                          /* 2 s at 50 Hz */
#define TXN           1024U
#define RXN           64U

static const uint8_t G_PCT[] = { 15, 20, 30, 40, 50, 60 };
#define G_N ((uint16_t)(sizeof G_PCT / sizeof G_PCT[0]))

typedef struct { float u[2]; uint16_t hold, rest; bool slew; } step_t;
typedef struct { UART_HandleTypeDef *h; uint8_t rx; uint32_t tail; uint16_t len; volatile bool busy; } uart_t;

static uart_t U[2];                                 /* 0 = USB, 1 = radio */
static uint8_t s_txbuf[TXN];
static uint32_t s_head;
static uint8_t s_rxring[RXN];
static volatile uint32_t s_rx_h, s_rx_t;            /* ISR writes h, main writes t */
static volatile bool s_abort;
static volatile uint32_t s_tick;
static uint32_t s_seen;
static char s_line[24];
static uint8_t s_ll;

static bool s_mot_ok, s_enc_ok;
static char s_cmd;                                  /* running test, 0 = none */
static enum { PH_IDLE, PH_DRIVE, PH_REST } s_ph;
static step_t s_step;
static uint16_t s_idx, s_t, s_pt, s_first[2], s_combo, s_pct, s_kpct;
static uint8_t s_stall[2];
static bool s_ok;
static int32_t s_prev[2], s_d[2], s_sum[2], s_late[2], s_fwd_cnt, s_fwd_lat;
static float s_u[2], s_vh[2];
static struct { int16_t d[2]; int16_t u_milli; } s_rec[K_N];
static uint16_t s_dump, s_kn;
static bool s_dump_on;

static const char WHEEL[2] = { 'L', 'R' };

/* ---- output: one ring, one read index per UART (the 9600 Bd radio is the slow one) ----------- */
static uint32_t tx_free(void)
{
    const uint32_t a = s_head - U[0].tail, b = s_head - U[1].tail;
    return TXN - ((a > b) ? a : b);
}

__attribute__((format(printf, 1, 2)))
static void out(const char *f, ...)
{
    char b[112];
    va_list ap;
    va_start(ap, f);
    int n = vsnprintf(b, sizeof b, f, ap);
    va_end(ap);
    if (n <= 0) { return; }
    if ((size_t)n >= sizeof b) { n = (int)sizeof b - 1; }
    if (tx_free() < (uint32_t)n) { return; }        /* whole line or nothing */
    for (int i = 0; i < n; i++) { s_txbuf[s_head++ % TXN] = (uint8_t)b[i]; }
}

static void tx_kick(void)
{
    for (uint32_t i = 0U; i < 2U; i++) {
        uart_t *u = &U[i];
        if (u->busy) { continue; }
        u->tail += u->len;
        u->len = 0U;
        const uint32_t off = u->tail % TXN;
        uint32_t n = s_head - u->tail;
        if (n > TXN - off) { n = TXN - off; }
        if (n > 64U) { n = 64U; }
        if (n == 0U) { continue; }
        u->len = (uint16_t)n;
        u->busy = true;
        if (HAL_UART_Transmit_IT(u->h, &s_txbuf[off], (uint16_t)n) != HAL_OK) {
            u->busy = false;                        /* retried on the next pass */
            u->len = 0U;
        }
    }
}

/* ---- UART callbacks (app_isr.c has none in this build) ---------------------------------------- */
static uart_t *by_handle(UART_HandleTypeDef *h) { return (U[0].h == h) ? &U[0] : ((U[1].h == h) ? &U[1] : NULL); }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uart_t *u = by_handle(huart);
    if (u == NULL) { return; }
    if (u->rx == (uint8_t)'s') {                    /* abort: bridge off before anything else */
        sys_fault_hard_stop();
        s_abort = true;
    } else if (s_rx_h - s_rx_t < RXN) {
        s_rxring[s_rx_h % RXN] = u->rx;
        s_rx_h++;
    }
    (void)HAL_UART_Receive_IT(huart, &u->rx, 1U);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uart_t *u = by_handle(huart);
    if (u != NULL) { u->busy = false; }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uart_t *u = by_handle(huart);
    if (u == NULL) { return; }
    if (huart->gState == HAL_UART_STATE_READY) { u->busy = false; }
    (void)HAL_UART_Receive_IT(huart, &u->rx, 1U);   /* HAL_BUSY if still armed: harmless */
}

/* ---- motor side ------------------------------------------------------------------------------- */
static float m_per_cnt(uint32_t i) { return par()->odo_m_per_cnt * ((i == 0U) ? par()->odo_scale_l : par()->odo_scale_r); }

static void apply(void)
{
    for (uint8_t i = 0U; i < 2U; i++) { mot_set(i, clampf(s_u[i], -DUTY_MAX, DUTY_MAX)); }
    mot_apply();
}

static void stop_outputs(void)
{
    s_u[0] = 0.0f;
    s_u[1] = 0.0f;
    apply();
    mot_enable(false);
}

static void stop_all(void)
{
    stop_outputs();
    s_ph = PH_IDLE;
    s_cmd = 0;
    s_dump_on = false;
}

static void fault(uint32_t w)
{
    sys_fault_hard_stop();
    stop_all();
    out("FAULT STALL_OR_SIGN wheel %c\r\n", WHEEL[w]);
}

/* ---- test sequencer: a test is a list of steps (duty, hold, rest), generated one at a time ---- */
static bool next_step(void)
{
    const uint16_t k = s_idx;
    const float sg = ((k & 1U) != 0U) ? -1.0f : 1.0f;
    s_step = (step_t){ .rest = REST_TICKS, .slew = true };
    switch (s_cmd) {
    case 'C':
        if (k >= 4U) { return false; }
        s_step.u[k >> 1] = sg * C_DUTY;
        s_step.hold = T_S(0.8f);
        s_step.rest = T_S(0.5f);
        break;
    case 'G':
        if (k >= 2U * G_N) { return false; }
        s_step.u[0] = s_step.u[1] = sg * (float)G_PCT[k >> 1] / 100.0f;
        s_step.hold = T_S(6.0f);
        break;
    case 'B':
        if (s_combo >= 4U) { return false; }
        s_step.u[s_combo >> 1] = (((s_combo & 1U) != 0U) ? -1.0f : 1.0f) * (float)s_pct / 100.0f;
        s_step.hold = T_S(1.8f);
        s_step.rest = T_S(0.9f);
        break;
    case 'K':
        if (k >= 3U) { return false; }
        s_step.u[0] = s_step.u[1] = (k == 1U) ? (float)s_kpct / 100.0f : 0.0f;
        s_step.hold = (k == 0U) ? 10U : ((k == 1U) ? 60U : 30U);
        s_step.rest = 0U;
        s_step.slew = false;                        /* a real step, for tau_m */
        break;
    default:                                        /* 'L' */
        if (k >= 4U) { return false; }
        s_step.u[k >> 1] = sg * L_DUTY;
        s_step.hold = T_S(1.0f);
        s_step.rest = T_S(0.5f);
        break;
    }
    return true;
}

static void begin_step(void)
{
    s_ph = PH_DRIVE;
    s_t = 0U;
    s_first[0] = s_first[1] = 0U;
    s_sum[0] = s_sum[1] = s_late[0] = s_late[1] = 0;
    if (s_cmd == 'K') { return; }
    const long l = lroundf(s_step.u[0] * 100.0f), r = lroundf(s_step.u[1] * 100.0f);
    if (l == r) { out("> %ld\r\n", l); } else { out("> PWM %ld %ld\r\n", l, r); }
}

static void finish(void)
{
    const char c = s_cmd;
    stop_all();
    if (c == 'K') { s_dump = 0U; s_dump_on = true; return; }
    out("DONE %c%s\r\n", c, (c != 'C') ? "" : (s_ok ? " PASS" : " FAIL"));
}

static void advance(void)
{
    if (next_step()) { s_idx++; begin_step(); } else { finish(); }
}

static void step_done(void)
{
    const uint16_t k = s_idx - 1U;
    const uint32_t w = k >> 1;
    const int32_t sg = ((k & 1U) != 0U) ? -1 : 1;
    switch (s_cmd) {
    case 'C': {
        const bool ok = (s_sum[w] * sg >= C_MIN_CNT) && (labs(s_sum[1U - w]) < C_MIN_CNT);
        s_ok = s_ok && ok;
        out("C,%c,%s,%ld,%ld,%ld,%s\r\n", WHEEL[w], (sg > 0) ? "fwd" : "rev", lroundf(C_DUTY * 100.0f),
            (long)s_sum[0], (long)s_sum[1], ok ? "PASS" : "FAIL");
        break;
    }
    case 'G': {
        const float s = 1000.0f / ((float)(s_step.hold - s_step.hold / 2U) * CTRL_DT_S);
        out("G,%ld,%ld,%ld\r\n", lroundf(sg * (float)G_PCT[w]),
            lroundf((float)s_late[0] * m_per_cnt(0U) * s), lroundf((float)s_late[1] * m_per_cnt(1U) * s));
        break;
    }
    case 'B': {
        const uint32_t cw = s_combo >> 1;
        const int32_t sb = ((s_combo & 1U) != 0U) ? -1 : 1;
        const bool moved = s_sum[cw] * sb >= MOVE_CNT;
        if (moved || (s_pct >= B_MAX_PCT)) {
            if (moved) { out("B,%c,%s,%u\r\n", WHEEL[cw], (sb > 0) ? "fwd" : "rev", (unsigned)s_pct); }
            else       { out("B,%c,%s,NONE\r\n", WHEEL[cw], (sb > 0) ? "fwd" : "rev"); }
            s_combo++;
            s_pct = B_START_PCT;
        } else {
            s_pct++;
        }
        break;
    }
    case 'L':
        if ((k & 1U) == 0U) {
            s_fwd_cnt = s_sum[w];
            s_fwd_lat = (int32_t)s_first[w];
        } else {
            out("L,%c,%ld,%ld,%ld,%ld\r\n", WHEEL[w], (long)s_fwd_cnt, (long)s_sum[w],
                (long)(s_fwd_lat * 1000 / (int32_t)HZ), (long)((int32_t)s_first[w] * 1000 / (int32_t)HZ));
        }
        break;
    default:
        break;
    }
}

static void print_status(const char *phase)
{
    char x[2][16];
    for (uint32_t i = 0U; i < 2U; i++) { (void)fmt_fixed(x[i], sizeof x[i], (float)s_prev[i] * m_per_cnt(i) * 1000.0f, 1); }
    out("L=%ld (%s mm, %ld mm/s)  R=%ld (%s mm, %ld mm/s)  CALIB %s\r\n", (long)s_prev[0], x[0],
        lroundf(s_vh[0] * 1000.0f), (long)s_prev[1], x[1], lroundf(s_vh[1] * 1000.0f), phase);
}

/* One 50 Hz tick. The counts seen here are the motion of the interval that just ended, under the
 * duty applied at the end of the previous tick. */
static void tick(void)
{
    enc_update();
    const enc_reading_t e = enc_read();
    for (uint32_t i = 0U; i < 2U; i++) {
        s_d[i] = e.cnt[i] - s_prev[i];
        s_prev[i] = e.cnt[i];
        s_vh[i] += 0.5f * ((float)s_d[i] * m_per_cnt(i) / CTRL_DT_S - s_vh[i]);
    }
    if (s_ph == PH_IDLE) { return; }

    if (s_ph == PH_DRIVE) {
        for (uint32_t i = 0U; i < 2U; i++) {
            const int32_t dir = (s_step.u[i] > 0.0f) ? 1 : ((s_step.u[i] < 0.0f) ? -1 : 0);
            s_sum[i] += s_d[i];
            if (s_t >= s_step.hold / 2U) { s_late[i] += s_d[i]; }
            if (dir != 0 && s_first[i] == 0U && s_d[i] * dir > 0) { s_first[i] = s_t + 1U; }
            if (fabsf(s_u[i]) >= STALL_DUTY && s_d[i] * ((s_u[i] > 0.0f) ? 1 : -1) <= 0) {
                if (++s_stall[i] >= STALL_TICKS) { fault(i); return; }
            } else {
                s_stall[i] = 0U;
            }
        }
        if (s_cmd == 'K' && s_kn < K_N) {
            s_rec[s_kn].d[0] = (int16_t)s_d[0];
            s_rec[s_kn].d[1] = (int16_t)s_d[1];
            s_rec[s_kn].u_milli = (int16_t)lroundf(s_u[0] * 1000.0f);
            s_kn++;
        }
        if (++s_t >= s_step.hold) {
            step_done();
            if (s_step.rest == 0U) { advance(); } else {
                out("> s\r\n");
                s_ph = PH_REST;
                s_t = 0U;
            }
        }
    } else if (++s_t >= s_step.rest) {
        advance();
    }
    if (s_ph == PH_IDLE) { return; }

    const bool drive = (s_ph == PH_DRIVE);
    const float slew = par()->mot_slew_ol * CTRL_DT_S;
    for (uint32_t i = 0U; i < 2U; i++) {
        const float tgt = drive ? s_step.u[i] : 0.0f;
        s_u[i] = (drive && s_step.slew) ? s_u[i] + clampf(tgt - s_u[i], -slew, slew) : tgt;
    }
    apply();
    if (s_cmd != 'K' && ++s_pt >= 15U) {            /* 0.3 s, the period ffit.py assumes */
        s_pt = 0U;
        print_status(drive ? "OPEN" : "HOLD");
    }
}

static void start(char c)
{
    if (!s_mot_ok || !s_enc_ok) { out("ERR driver not ready\r\n"); return; }
    s_cmd = c;
    s_idx = 0U;
    s_pt = 0U;
    s_ok = true;
    s_combo = 0U;
    s_kn = 0U;
    s_pct = B_START_PCT;
    s_stall[0] = s_stall[1] = 0U;
    switch (c) {
    case 'C': out("# C wheel,dir,duty_pct,dcnt_l,dcnt_r,result (counts in the pulse, wheel-forward positive; check by eye that the wheel turned forward)\r\n"); break;
    case 'G': out("# G duty_pct,v_l_mm_s,v_r_mm_s (mean of the second half of each step; ffit.py steady reads the status lines)\r\n"); break;
    case 'B': out("# B wheel,dir,breakaway_duty_pct (first 1 pct step with at least 10 counts; ffit.py breakaway reads the status lines)\r\n"); break;
    case 'K': out("# K t_ms,duty_milli,dcnt_l,dcnt_r (counts per 20 ms ending at t_ms, duty applied in that interval; step at 200 ms, off at 1400 ms)\r\n"); break;
    default:  out("# L wheel,fwd_cnt,rev_cnt,fwd_latency_ms,rev_latency_ms (motor-side counts and time to the first count in the commanded direction; see calib.c)\r\n"); break;
    }
    mot_enable(true);
    advance();
}

static void help(void)
{
    out("# C sign check | G steady steps | B breakaway | K [pct] step response (default 40)\r\n");
    out("# L reversal latency | odo raw counts | r reset counts | s stop | U,A not available\r\n");
}

static void exec(void)
{
    const char c = s_line[0];
    if (c == '?') { help(); return; }
    if (strncmp(s_line, "odo", 3) == 0) {
        out("# odo cnt_l,cnt_r (counts since r, wheel-forward positive)\r\nodo,%ld,%ld\r\n", (long)s_prev[0], (long)s_prev[1]);
        return;
    }
    if (c == 'U' || c == 'A') { out("ERR not available\r\n"); return; }
    if (s_cmd != 0 || s_dump_on) { out("ERR busy, send s to stop\r\n"); return; }
    switch (c) {
    case 'r':
        enc_zero();
        s_prev[0] = s_prev[1] = 0;
        out("Counters reset\r\n");
        break;
    case 'K': {
        long p = strtol(&s_line[1], NULL, 10);
        s_kpct = (uint16_t)((p <= 0) ? K_DEFAULT_PCT : ((p > lroundf(DUTY_MAX * 100.0f)) ? lroundf(DUTY_MAX * 100.0f) : p));
        start(c);
        break;
    }
    case 'C': case 'G': case 'B': case 'L': start(c); break;
    default: out("ERR unknown, ? for help\r\n"); break;
    }
}

/* ---- public ------------------------------------------------------------------------------------ */
void Calib_Init(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim8,
                TIM_HandleTypeDef *htim6, UART_HandleTypeDef *huart2, UART_HandleTypeDef *huart1)
{
    U[0].h = huart2;
    U[1].h = huart1;
    (void)HAL_UART_Receive_IT(huart2, &U[0].rx, 1U);
    (void)HAL_UART_Receive_IT(huart1, &U[1].rx, 1U);
    const bool par_ok = (par_init() == ST_OK);
    s_mot_ok = (mot_init(htim1) == ST_OK);
    s_enc_ok = (enc_init(htim3, htim8) == ST_OK);
    out("# MR2006B calibration firmware, ? for help\r\n");
    if (!par_ok || !s_mot_ok || !s_enc_ok) { out("ERR init par=%d mot=%d enc=%d (drive commands refused)\r\n", par_ok, s_mot_ok, s_enc_ok); }
    s_seen = s_tick;
    (void)HAL_TIM_Base_Start_IT(htim6);
}

void Calib_Tick(void)
{
    enc_snapshot_isr(0U);                           /* no timestamp: TIM2 is not started here */
    s_tick++;
}

void Calib_Loop(void)
{
    if (s_abort) {
        s_abort = false;
        stop_all();
        out("> s\r\nSTOP\r\n");
    }
    while (s_rx_t != s_rx_h) {
        const char ch = (char)s_rxring[s_rx_t % RXN];
        s_rx_t++;
        if (ch == '\r' || ch == '\n') {
            if (s_ll != 0U) { s_line[s_ll] = '\0'; exec(); s_ll = 0U; }
        } else if (s_ll < sizeof s_line - 1U) {
            s_line[s_ll++] = ch;
        }
    }
    const uint32_t t = s_tick;
    if (t != s_seen) {
        if (t - s_seen > 1U && s_ph != PH_IDLE) { out("# WARN tick overrun\r\n"); }
        s_seen = t;
        tick();
    }
    if (s_dump_on && tx_free() >= 48U) {            /* paced: the radio drains 96 bytes per 100 ms */
        if (s_dump < K_N) {
            const uint16_t k = (uint16_t)s_dump++;
            out("K,%ld,%d,%d,%d\r\n", (long)(((int32_t)k + 1) * 1000 / (int32_t)HZ), s_rec[k].u_milli, s_rec[k].d[0], s_rec[k].d[1]);
        } else {
            s_dump_on = false;
            s_cmd = 0;
            out("DONE K\r\n");
        }
    }
    tx_kick();
}

#endif /* APP_CALIB */
