/* app.c — spec §5.4, §5.8, §7.26 (REQ-EXE-14, REQ-FLT-03, REQ-APP-01). Phase 0: no msn yet,
 * so the three modes IDLE / BENCH / FAULT live here and move to msn in phase 3. */
#include "app.h"

#include <stdio.h>

#include "app_config.h"
#include "app_priv.h"
#include "bat.h"
#include "enc.h"
#include "mot.h"
#include "par.h"
#include "sys_fault.h"
#include "sys_sched.h"
#include "sys_time.h"

#if __has_include("build_info.h")
#include "build_info.h"
#endif
#ifndef BUILD_GIT
#define BUILD_GIT "nogit"
#endif

#define APP_VERSION        "0.1-phase0"
#define CLOCK_CHECK_MS     100U
#define CLOCK_CHECK_TOL_US 100U      /* §5.8 step 6 */
#define FAULT_STOP_MAX_MS  1500U     /* give the drive this long to reach rest before EN goes low */
#define REST_V_MPS         0.010f
#define STATIONARY_V_MPS   0.010f
#define TICK_HZ_TOL_X10    10U       /* 0.1 % of 1000.0 Hz, REQ-SCH-02 */
#define LED_FAULT_DIV      1U
#define LED_IDLE_DIV       5U

app_state_t g_app;

static uint32_t s_slow_t_us, s_slow_ticks, s_slow_overrun;
static bool     s_slow_primed;

const char *app_mode_name(void)
{
    switch (g_app.mode) {
    case APP_IDLE:  return "IDLE";
    case APP_BENCH: return "BENCH";
    case APP_FAULT: return "FAULT";
    default:        return "?";
    }
}

static void bcast_printf_fault(const char *what, uint32_t first)
{
    for (uint32_t i = 0U; i < (uint32_t)COM_N; i++) {
        con_printf((com_port_t)i, "%s 0x%02lX\r\n", what, (unsigned long)first);
    }
}

void app_enter_fault(reaction_t r)
{
    g_app.mode = APP_FAULT;
    g_app.stopping = false;
    g_app.fault_t0_ms = sys_ms();
    if (r >= R4) { drv_disable(&g_app.drv); }
    else if (r == R3) { drv_brake(&g_app.drv); }
    else { drv_stop(&g_app.drv); }
    sys_event(EV_MODE, (int32_t)APP_FAULT);
    bcast_printf_fault("FAULT, motors stopped. first fault", sys_fault_first());
}

bool app_clear_faults(void)
{
    if (bat_low()) { return false; }                     /* cause still present */
    sys_fault_clear();
    if (sys_fault_reaction() >= R2) { return false; }    /* F_CLOCK / F_PARAM stay */
    if (g_app.mode == APP_FAULT) {
        g_app.mode = APP_IDLE;
        drv_disable(&g_app.drv);
    }
    return true;
}

/* ---- init ----------------------------------------------------------------------------- */
/* Platform-configuration mismatches (TIM1 ARR, TIM6, clock) all raise F_CLOCK: the catalogue has
 * no better code and the reaction (R4, boot stops in FAULT) is the one wanted. */
static void step(uint32_t n, const char *name, st_t s, fault_t on_fail)
{
    if (s == ST_OK) {
        con_printf(COM_USB, "%2lu %-8s OK\r\n", (unsigned long)n, name);
    } else if (s == ST_NODEV) {
        con_printf(COM_USB, "%2lu %-8s absent\r\n", (unsigned long)n, name);
    } else {
        con_printf(COM_USB, "%2lu %-8s FAIL %d\r\n", (unsigned long)n, name, (int)s);
        g_app.boot_status |= (1UL << n);
        if (on_fail != F_NONE) { sys_fault_raise(on_fail, (int32_t)s); }
    }
    com_flush(100U);
}

static void hello(void)
{
    const uint8_t rc = sys_reset_cause();
    for (uint32_t i = 0U; i < (uint32_t)COM_N; i++) {
        con_printf((com_port_t)i, "\r\nHELLO MR2006B fw %s %s %s reset=0x%02X calib=%d\r\n", APP_VERSION, BUILD_GIT,
#ifdef DEBUG
                   "Debug",
#else
                   "Release",
#endif
                   (unsigned)rc, APP_CALIB);
    }
    if ((rc & SYS_RST_IWDG) != 0U) { sys_fault_raise(F_WDG_RESET, 0); con_printf(COM_USB, "reset by watchdog\r\n"); }
    uint32_t code, msp, w[SYS_HARD_WORDS];
    if (sys_fault_take_hard_record(&code, &msp, w)) {
        con_printf(COM_USB, "last hard fault: code 0x%02lX msp 0x%08lX\r\n", (unsigned long)code, (unsigned long)msp);
        for (uint32_t i = 0U; i < SYS_HARD_WORDS; i += 4U) {
            con_printf(COM_USB, "  %08lX %08lX %08lX %08lX\r\n", (unsigned long)w[i], (unsigned long)w[i + 1U],
                       (unsigned long)w[i + 2U], (unsigned long)w[i + 3U]);
        }
    }
}

/* TIM2 microseconds across 100 SysTick milliseconds (§5.8 step 6). Init only. */
static st_t clock_check(void)
{
    uint32_t m = HAL_GetTick();
    while (HAL_GetTick() == m) { }                       /* align to a SysTick edge */
    m = HAL_GetTick();
    const uint32_t t0 = sys_time_us();
    while (HAL_GetTick() - m < CLOCK_CHECK_MS) { }
    const uint32_t dt = sys_time_us() - t0;
    const uint32_t want = CLOCK_CHECK_MS * 1000U;
    return (dt + CLOCK_CHECK_TOL_US >= want && dt <= want + CLOCK_CHECK_TOL_US) ? ST_OK : ST_RANGE;
}

void app_init(const app_hw_t *hw)
{
    drv_init(&g_app.drv);
    g_app.mode = APP_IDLE;

    /* Steps 1-4 run before the console exists (step 5); their results are printed afterwards. */
    const st_t r1 = sys_time_init(hw->tim2);
    const st_t r2 = mot_init(hw->tim1);
    const st_t r3 = enc_init(hw->tim3, hw->tim8);
    const st_t r4 = par_init();
    com_uart_init(hw->uart2, hw->uart1);
    hello();
    step(1, "sys_time", r1, F_CLOCK);
    step(2, "mot", r2, F_CLOCK);
    step(3, "enc", r3, F_ASSERT);
    step(4, "par", r4, F_PARAM);
    step(6, "clock", clock_check(), F_CLOCK);
    step(7, "bat", bat_init(hw->adc1), F_NONE);
    sys_event(EV_BOOT, (int32_t)g_app.boot_status);

    g_app.sched_ok = (sys_sched_start(hw->tim6) == ST_OK);
    step(12, "sched", g_app.sched_ok ? ST_OK : ST_RANGE, F_CLOCK);

    if (sys_fault_reaction() >= R2) { app_enter_fault(sys_fault_reaction()); }
    else { con_printf(COM_USB, "ready, ? for help\r\n"); }
    com_flush(200U);
}

/* ---- slots ---------------------------------------------------------------------------- */
static void app_fast(void)
{
    enc_update();
    com_uart_poll();
    bat_service();
}

static void report_event(drv_event_t ev)
{
    const drv_status_t s = drv_status(&g_app.drv);
    sys_event(ev == DRV_EV_DONE ? EV_DRV_DONE : EV_DRV_TIMEOUT, (int32_t)(s.sync_max_m * 1.0e6f));
    for (uint32_t i = 0U; i < (uint32_t)COM_N; i++) {
        con_print_done((com_port_t)i, ev == DRV_EV_TIMEOUT, s.d_m, s.sync_max_m, s.t_s);
    }
}

static void app_ctrl(void)
{
    const enc_reading_t r = enc_read();
    const drv_meas_t m = { .cnt = { r.cnt[0], r.cnt[1] }, .t_us = r.t_us, .u_bat_v = bat_v() };

    const reaction_t fr = sys_fault_reaction();                    /* REQ-FLT-03 */
    if (fr >= R2 && g_app.mode != APP_FAULT) { app_enter_fault(fr); }

    const drv_status_t before = drv_status(&g_app.drv);
    const bool at_rest = fabsf(before.v_mps[0]) < REST_V_MPS && fabsf(before.v_mps[1]) < REST_V_MPS;
    if (g_app.mode == APP_BENCH && g_app.stopping && before.mode == DRV_HOLD) {
        drv_disable(&g_app.drv);                                   /* BENCH -> IDLE, EN low */
        g_app.mode = APP_IDLE;
        g_app.stopping = false;
    } else if (g_app.mode == APP_FAULT && before.mode != DRV_DISABLED
               && ((before.mode == DRV_HOLD && at_rest) || sys_ms() - g_app.fault_t0_ms > FAULT_STOP_MAX_MS)) {
        drv_disable(&g_app.drv);
    }
    par_set_stationary((before.mode == DRV_HOLD || before.mode == DRV_DISABLED) && at_rest);

    if (before.mode == DRV_OPEN
        && sys_ms() - con_last_rx_ms(g_app.ol_port) > (uint32_t)(par()->con_deadman_s * 1000.0f)) {
        drv_brake(&g_app.drv);                                     /* REQ-CON-03 */
        g_app.stopping = true;
        sys_event(EV_DEADMAN, 0);
        con_printf(g_app.ol_port, "DEADMAN: no input for %ld s, motors stopped\r\n", lroundf(par()->con_deadman_s));
    }

    drv_out_t out;
    drv_step(&g_app.drv, &m, &out);
    if (out.fault != F_NONE) { sys_fault_raise(out.fault, 0); }    /* R4 stops the bridge inside */
    if (out.event != DRV_EV_NONE) {
        report_event(out.event);
        if (g_app.mode == APP_BENCH) { g_app.stopping = true; }
    }

    mot_set(WHL_L, out.duty_d[WHL_L]);
    mot_set(WHL_R, out.duty_d[WHL_R]);
    if (out.enable) { mot_apply(); mot_enable(true); }             /* duty first, then EN */
    else { mot_enable(false); mot_apply(); }
}

static void app_tlm(void)
{
    static uint32_t n, t_print[COM_N];
    const uint32_t div = (g_app.mode == APP_FAULT) ? LED_FAULT_DIV : LED_IDLE_DIV;
    if (++n % div == 0U) { HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); }
    for (uint32_t i = 0U; i < (uint32_t)COM_N; i++) {
        if (con_autoprint((com_port_t)i) && sys_ms() - t_print[i] >= 250U) {
            t_print[i] = sys_ms();
            app_print_status((com_port_t)i);
        }
    }
}

static void app_slow(void)
{
    sys_sched_stats_t st;
    sys_sched_stats(&st);
    uint32_t sum = 0U;
    for (uint32_t i = 0U; i < (uint32_t)SLOT_N; i++) { sum += st.sum_cycles[i]; }
    g_app.cpu_permille = sum / (SystemCoreClock / 1000U);

    const uint32_t now = sys_time_us();
    if (s_slow_primed) {
        const uint32_t dus = now - s_slow_t_us;
        g_app.tick_hz_x10 = (uint32_t)((uint64_t)(st.ticks - s_slow_ticks) * 10000000ULL / dus);
        if (g_app.tick_hz_x10 + TICK_HZ_TOL_X10 < TICK_HZ * 10U || g_app.tick_hz_x10 > TICK_HZ * 10U + TICK_HZ_TOL_X10) {
            con_printf(COM_USB, "WARN tick rate %lu.%lu Hz, TIM6 misconfigured?\r\n",
                       (unsigned long)(g_app.tick_hz_x10 / 10U), (unsigned long)(g_app.tick_hz_x10 % 10U));
        }
    }
    s_slow_primed = true;
    s_slow_t_us = now;
    s_slow_ticks = st.ticks;

    const uint32_t over = st.overrun[SLOT_CTRL] - s_slow_overrun;      /* REQ-EXE-12 */
    s_slow_overrun = st.overrun[SLOT_CTRL];
    if (over > 10U) { sys_fault_raise_as(F_OVERRUN, (int32_t)over, R2); }
    else if (over > 0U) { sys_fault_raise(F_OVERRUN, (int32_t)over); }
}

void app_loop(void)
{
    uint32_t p = sys_sched_take();
    if (!g_app.sched_ok) { p |= S_FAST; }                  /* keep the console alive after a boot failure */
    if ((p & S_FAST) != 0U) { sys_slot_begin(SLOT_FAST); app_fast(); sys_slot_end(SLOT_FAST); }
    if ((p & S_CTRL) != 0U) { sys_slot_begin(SLOT_CTRL); app_ctrl(); sys_slot_end(SLOT_CTRL); }
    if ((p & S_TLM) != 0U)  { sys_slot_begin(SLOT_TLM);  app_tlm();  sys_slot_end(SLOT_TLM); }
    if ((p & S_SLOW) != 0U) { sys_slot_begin(SLOT_SLOW); app_slow(); sys_slot_end(SLOT_SLOW); }

    con_cmd_t c;
    if (con_poll(&c)) { app_cmd_exec(&c); }
    __WFI();
}
