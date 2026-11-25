/* app.c — spec §5.4, §5.8, §7.26 (REQ-EXE-14, REQ-FLT-03, REQ-APP-01). Phase 0: no msn yet,
 * so the three modes IDLE / BENCH / FAULT live here and move to msn in phase 3. */
#include "app.h"

#include <stdio.h>

#include "app_config.h"
#include "app_priv.h"
#include "bat.h"
#include "enc.h"
#if FEAT_IMU
#include "imu.h"
#endif
#if FEAT_VL
#include "vl.h"
#endif
#include "mot.h"
#include "par.h"
#include "sys_fault.h"
#include "sys_irq.h"
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
#define TURN_TOL_RAD       (1.0f * DEG2RAD_F)   /* W: residual not corrected any more; >= drive resolution:
                                                  * drv_pos_tol 2 mm on half the wheelbase = 1.05 deg (D-68) */
#define TURN_FIX_MAX       2U                   /* W: correction turns after the encoder turn  [DECIDE] */
#define TURN_FIX_T_ACC_S   0.3f                 /* W: ramp of a correction turn; the main turn uses drv_t_acc (D-68) */

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
    const bool ekf_lost = sys_fault_active(F_EKF);
    sys_fault_clear();
    if (sys_fault_reaction() >= R2) { return false; }    /* F_CLOCK / F_PARAM stay */
    if (g_app.mode == APP_FAULT) {
        g_app.mode = APP_IDLE;
        drv_disable(&g_app.drv);
    }
    if (ekf_lost) { ekf_init(&g_app.ekf); g_app.ekf_primed = false; }   /* §11 F_EKF: re-initialised in IDLE */
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
    att_init(&g_app.att);
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
#if FEAT_IMU
    step(8, "imu", imu_init(hw->spi1, hw->tim2), F_NONE);
#endif
    ekf_init(&g_app.ekf);
#if FEAT_VL
    gnd_init(&g_app.gnd);
    (void)vl_init(hw->i2c2);                       /* step 9: one line per sensor = wiring check */
    for (uint8_t i = 0U; i < VL_N; i++) { step(9, vl_name(i), vl_state(i), F_NONE); }
#endif
    step(11, "ekf", ST_OK, F_ASSERT);
    sys_event(EV_BOOT, (int32_t)g_app.boot_status);

    g_app.sched_ok = (sys_sched_start(hw->tim6) == ST_OK);
    step(12, "sched", g_app.sched_ok ? ST_OK : ST_RANGE, F_CLOCK);

    if (sys_fault_reaction() >= R2) { app_enter_fault(sys_fault_reaction()); }
    else { con_printf(COM_USB, "ready, ? for help\r\n"); }
    com_flush(200U);
    if (g_app.sched_ok) { sys_wdg_start(hw->iwdg); }       /* step 14, last: no blocking call after this (D-49) */
}

/* ---- slots ---------------------------------------------------------------------------- */
#if FEAT_VL
/* VLS stream and LATCAL on every new S2 result (D-79, REQ-VL-10). */
static void vl_s2_tools(const vl_meas_t *m)
{
    static const char *const CLS[] = { "UNKNOWN", "PATH", "GRASS", "EDGE_UP", "DROP" };
    const par_t *q = par();
    const float roll = att_roll(&g_app.att);
    if (g_app.vls) {
        con_print_vls(COM_USB, m->t_us, m->seq, m->range_m, m->status, m->signal_kcps, m->ambient_kcps,
                      m->range_m * sinf(q->lat_tilt + roll), CLS[gnd_class(&g_app.gnd, VL_S2)]);
    }
    if (!g_app.latcal.on) { return; }
    if (m->status == VL_ST_VALID && g_app.latcal.n < 64U) { g_app.latcal.r[g_app.latcal.n++] = m->range_m; }
    if ((uint32_t)(sys_time_us() - g_app.latcal.t0_us) >= 1000000U) {   /* not m->t_us: it lies before the start */
        const uint8_t n = g_app.latcal.n;
        const float r = gnd_median(g_app.latcal.r, n);
        const float x = q->lat_h / r;
        con_print_latcal(g_app.latcal.port, r, n, (x > 0.0f && x <= 1.0f) ? asinf(x) : NAN, q->lat_tilt, roll);
        g_app.latcal.on = false;
    }
}
#endif

static void app_fast(void)
{
    enc_update();
    com_uart_poll();
#if FEAT_IMU
    imu_drain(&g_app.att);
#endif
#if FEAT_VL
    vl_service();
    const vl_meas_t *m = vl_get(VL_S2);
    if (m->seq != g_app.vl_seq[VL_S2]) {                 /* REQ-LAT-07 via gnd (D-70) */
        g_app.vl_seq[VL_S2] = m->seq;
        gnd_on_meas(&g_app.gnd, VL_S2, m, att_roll(&g_app.att), att_pitch(&g_app.att));
        vl_s2_tools(m);
    }
#endif
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

/* EKF step, first work of S_CTRL (D-30, REQ-EXE-03): encoder count differences over the period and
 * the yaw integral between the two encoder timestamps, both read after this tick's imu_drain.
 * NAN yaw (no IMU, dead IMU, or a timestamp outside the ring) runs this step in encoder mode. */
static void ekf_step(const enc_reading_t *r)
{
    static int32_t c0[2];
    static uint32_t t0;
    const par_t *q = par();
    if (g_app.ekf_primed && r->t_us != t0) {
        const drv_mode_t dm = drv_status(&g_app.drv).mode;
        ekf_odo_t u = { .ds_l = (float)(r->cnt[0] - c0[0]) * q->odo_m_per_cnt * q->odo_scale_l,
                        .ds_r = (float)(r->cnt[1] - c0[1]) * q->odo_m_per_cnt * q->odo_scale_r,
                        .dpsi = NAN, .dt = (float)(uint32_t)(r->t_us - t0) * 1.0e-6f, .t_us = r->t_us,
                        .still = (dm == DRV_HOLD || dm == DRV_DISABLED) };
#if FEAT_IMU
        float p0, p1;
        if (imu_alive() && imu_yaw_integral_at(t0, &p0) == ST_OK && imu_yaw_integral_at(r->t_us, &p1) == ST_OK) {
            u.dpsi = p1 - p0;
            g_app.turn_acc_rad += u.dpsi - g_app.ekf.x[3] * u.dt;   /* minus the EKF gyro bias (ZUPT) */
        }
#endif
        const uint32_t c = sys_cycles();
        ekf_predict(&g_app.ekf, &u);
        const uint32_t used = sys_cycles() - c;
        if (used > g_app.ekf_cyc_max) { g_app.ekf_cyc_max = used; }
        if (g_app.ekf.degraded && !sys_fault_active(F_EKF)) { sys_fault_raise(F_EKF, 0); }
    }
    c0[0] = r->cnt[0];
    c0[1] = r->cnt[1];
    t0 = r->t_us;
    g_app.ekf_primed = true;
}

/* W with an IMU: the encoder turn ends first, then the yaw integral decides whether the residual
 * is turned again. True if a correction turn was started (the DONE is then not reported). */
static bool turn_correct(drv_event_t ev, drv_mode_t mode)
{
    if (!g_app.turn_gyro || mode != DRV_TURN_ENC || ev != DRV_EV_DONE) { return false; }
    const float res = g_app.turn_target_rad - g_app.turn_acc_rad;
    if (fabsf(res) >= TURN_TOL_RAD && g_app.turn_fix < TURN_FIX_MAX) {
        const par_t *q = par();
        if (drv_turn_enc(&g_app.drv, res, app_turn_w(q), TURN_FIX_T_ACC_S) == ST_OK) { g_app.turn_fix++; return true; }
    }
    g_app.turn_gyro = false;
    for (uint32_t i = 0U; i < (uint32_t)COM_N; i++) {
        con_print_turn((com_port_t)i, g_app.turn_acc_rad, g_app.turn_target_rad);
    }
    return false;
}

static void app_ctrl(void)
{
    const enc_reading_t r = enc_read();
    ekf_step(&r);
#if FEAT_VL
    gnd_step(&g_app.gnd, r.t_us);
#endif
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
    if (out.event != DRV_EV_NONE && !turn_correct(out.event, before.mode)) {
        report_event(out.event);
        if (g_app.mode == APP_BENCH) { g_app.stopping = true; }
    }

    mot_set(WHL_L, out.duty_d[WHL_L]);
    mot_set(WHL_R, out.duty_d[WHL_R]);
    if (out.enable) { mot_apply(); mot_enable(true); }             /* duty first, then EN */
    else { mot_enable(false); mot_apply(); }
#if FEAT_IMU
    if (g_app.imus) {
        float w[3];
        uint32_t t_us, n;
        if (imu_take_mean(w, &t_us, &n) > 0U) { con_print_imus(COM_USB, t_us, n, w, imu_temp_c()); }
    }
#endif
    sys_wdg_ctrl_done();
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
    sys_wdg_service();

    /* Race-free sleep (REQ-NFR-05). A tick that fired after sys_sched_take() has set a pending
     * bit: do not sleep on it. With interrupts masked the check and the WFI cannot be separated
     * by the tick; WFI still wakes on a pending interrupt, which is taken at the restore.
     * UART bytes need no check: they are drained in S_FAST, at most 1 ms later. Without a
     * scheduler (boot failure) nothing is ever pending and SysTick wakes the loop every 1 ms. */
    const uint32_t pm = sys_irq_save();
    if (!sys_sched_pending()) { __WFI(); }
    sys_irq_restore(pm);
}
