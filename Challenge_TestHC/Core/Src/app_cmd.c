/* app_cmd.c — executes the console commands of §9 (phase-0 subset). Part of the app module. */
#include <stdio.h>

#include "app_config.h"
#include "app_priv.h"
#include "bat.h"
#include "enc.h"
#if FEAT_IMU
#include "imu.h"
#endif
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

#define ID_V_NOM   161U
#define ID_T_ACC   162U
#define ID_T_DEC   163U
#define ID_KP_V    110U
#define ID_KI_V    111U
#define ID_KS_L    116U
#define ID_KV_L    117U
#define ID_KS_R    118U
#define ID_KV_R    119U
#define ID_K_SYNC  153U

static const char *drv_state_name(const drv_status_t *s)
{
    switch (s->mode) {
    case DRV_DISABLED: return "OFF";
    case DRV_HOLD:     return "HOLD";
    case DRV_OPEN:     return "OPEN";
    case DRV_MOVE:
    case DRV_TURN_ENC:
        switch (s->phase) {
        case DRV_PH_MOVE:     return "MOVE";
        case DRV_PH_SETTLE:   return "SETTLE";
        case DRV_PH_STOPPING: return "STOPPING";
        case DRV_PH_IDLE:
        default:              return "MOVE";
        }
    default:           return "?";
    }
}

void app_print_status(com_port_t p)
{
    const enc_reading_t r = enc_read();
    const drv_status_t s = drv_status(&g_app.drv);
    const par_t *q = par();
    con_status_t cs = { .cnt = { r.cnt[0], r.cnt[1] }, .v_mps = { s.v_mps[0], s.v_mps[1] },
                        .mode = app_mode_name(), .phase = drv_state_name(&s) };
    cs.x_m[0] = (float)r.cnt[0] * q->odo_m_per_cnt * q->odo_scale_l;
    cs.x_m[1] = (float)r.cnt[1] * q->odo_m_per_cnt * q->odo_scale_r;
    con_print_status(p, &cs);
}

/* Sets n parameters or none: validates every value first. */
static bool par_set_all(com_port_t p, uint8_t n, const uint16_t id[], const float v[])
{
    for (uint8_t i = 0U; i < n; i++) {
        const par_desc_t *d = par_desc(id[i]);
        if (d == NULL || !(v[i] >= d->min && v[i] <= d->max)) { con_err_range_par(p, d); return false; }
    }
    for (uint8_t i = 0U; i < n; i++) {
        (void)par_set(id[i], v[i]);
        sys_event(EV_PARAM, (int32_t)id[i]);
    }
    return true;
}

/* BENCH motion needs IDLE or BENCH. Prints the refusal. */
static bool motion_allowed(com_port_t p)
{
    if (g_app.mode == APP_FAULT) { con_err_state(p, "FAULT"); return false; }
    return true;
}

static void enter_bench(com_port_t p)
{
    g_app.mode = APP_BENCH;
    g_app.stopping = false;
    g_app.ol_port = p;
    sys_event(EV_MODE, (int32_t)APP_BENCH);
}

static void reply_move_result(com_port_t p, st_t s)
{
    if (s == ST_STATE) { con_printf(p, "Busy, send s to stop\r\n"); }
    else if (s != ST_OK) { con_printf(p, "ERR range\r\n"); }
}

int con_info_line(uint32_t i, char *buf, size_t n)
{
    sys_sched_stats_t st;
    const uint32_t us = SystemCoreClock / 1000000U;
    switch (i) {
    case 0:
        return snprintf(buf, n, "MR2006B fw 0.1-phase0 %s calib=%d", BUILD_GIT, APP_CALIB);
    case 1:
        return snprintf(buf, n, "reset cause 0x%02X, tick %lu.%lu Hz, cpu %lu.%lu %%", (unsigned)sys_reset_cause(),
                        (unsigned long)(g_app.tick_hz_x10 / 10U), (unsigned long)(g_app.tick_hz_x10 % 10U),
                        (unsigned long)(g_app.cpu_permille / 10U), (unsigned long)(g_app.cpu_permille % 10U));
    case 2:
        sys_sched_stats_peek(&st);       /* not sys_sched_stats(): that would steal the CPU-load sums */
        return snprintf(buf, n, "S_CTRL max %lu us, overruns %lu; S_FAST max %lu us",
                        (unsigned long)(st.max_cycles[SLOT_CTRL] / us), (unsigned long)st.overrun[SLOT_CTRL],
                        (unsigned long)(st.max_cycles[SLOT_FAST] / us));
    case 3:                              /* H2 self-measurement, REQ-EXE-11 */
        sys_sched_stats_peek(&st);
        return sys_sched_format_timing(&st, SystemCoreClock, buf, n);
    case 4:
        return snprintf(buf, n, "boot failures 0x%04lX, par generation %lu, m/cnt %ld nm", (unsigned long)g_app.boot_status,
                        (unsigned long)par_generation(), lroundf(par()->odo_m_per_cnt * 1.0e9f));
    default:
        return -1;
    }
}

void con_imu_snapshot(con_imu_t *s)
{
    *s = (con_imu_t){ .roll_rad = att_roll(&g_app.att), .pitch_rad = att_pitch(&g_app.att),
                      .bg_radps = g_app.ekf.x[3] };
#if FEAT_IMU
    const imu_info_t i = imu_info();
    s->present = i.present;
    s->alive = i.alive;
    for (uint32_t k = 0U; k < 3U; k++) { s->w_radps[k] = i.w_radps[k]; }
    s->temp_c = i.temp_c;
    s->n_samples = i.n_samples;
    s->n_drop = i.n_drop;
    s->n_sat = i.n_sat;
    s->n_glitch = i.n_glitch;
#endif
}

void con_ekf_snapshot(con_ekf_t *s)
{
    ekf_out_t o;
    ekf_get(&g_app.ekf, &o);
    *s = (con_ekf_t){ .x_m = o.x[0], .y_m = o.x[1], .th_rad = o.x[2], .bg_radps = o.x[3], .s = o.x[4],
                      .sig_x_m = sqrtf(o.P_diag[0]), .sig_y_m = sqrtf(o.P_diag[1]),
                      .sig_th_rad = sqrtf(o.P_diag[2]), .sig_bg_radps = sqrtf(o.P_diag[3]),
                      .slip_episodes = o.slip_episodes, .step_us = g_app.ekf_cyc_max / (SystemCoreClock / 1000000U),
                      .still = (o.flags & EKF_STILL) != 0U, .slip = (o.flags & EKF_SLIP) != 0U,
                      .degraded = (o.flags & EKF_DEGRADED) != 0U };
    for (uint32_t k = 0U; k < CON_EKF_NU; k++) {
        s->nis_mean[k] = o.nis_mean[k];
        s->n_acc[k] = o.n_acc[k];
        s->n_rej[k] = o.n_rej[k];
    }
}

bool con_vl_snapshot(uint32_t i, con_vl_t *s)
{
    static const char *const CLS[] = { "UNKNOWN", "PATH", "GRASS", "EDGE_UP", "DROP" };
    if (i >= VL_N) { return false; }
    const uint8_t id = (uint8_t)i;
    const vl_meas_t *m = vl_get(id);
    *s = (con_vl_t){ .name = vl_name(id) + 3, .cls = "", .state = (int)vl_state(id), .range_m = m->range_m,
                     .signal_kcps = m->signal_kcps, .ambient_kcps = m->ambient_kcps, .status = m->status,
                     .seen = m->seq != 0U, .age_s = (float)(uint32_t)(sys_time_us() - m->t_us) * 1.0e-6f };
    if (id == VL_S2) { s->cls = CLS[gnd_class(&g_app.gnd, VL_S2)]; }
    return true;
}

void app_cmd_exec(const con_cmd_t *c)
{
    const com_port_t p = c->port;
    drv_t *d = &g_app.drv;
    const par_t *q = par();
    const drv_status_t ds = drv_status(d);
    const bool moving = (ds.phase != DRV_PH_IDLE) || (ds.mode == DRV_OPEN);
    float v[4];
    uint16_t id[4];

    switch (c->type) {
    case CMD_HELP:      con_job(p, CON_JOB_HELP); break;
    case CMD_PARAMS:    con_job(p, CON_JOB_PARAMS); break;
    case CMD_DUMP:      con_job(p, CON_JOB_DUMP); break;
    case CMD_EVLOG:     con_job(p, CON_JOB_EVLOG); break;
    case CMD_FAULTS:    con_job(p, CON_JOB_FAULTS); break;
    case CMD_IMU:       con_job(p, CON_JOB_IMU); break;
    case CMD_EKF:       con_job(p, CON_JOB_EKF); break;
    case CMD_VL:        con_job(p, CON_JOB_VL); break;
    case CMD_IMUS:
#if FEAT_IMU
        if (p != COM_USB) { con_err_state(p, "USB only"); break; }   /* 2 kB/s: too much for the radio */
        if (!g_app.imus) { float w[3]; uint32_t t, n; (void)imu_take_mean(w, &t, &n); }   /* empty the window */
        g_app.imus = !g_app.imus;
        con_printf(p, g_app.imus ? "IMUS ON\r\n" : "IMUS OFF\r\n");
#else
        con_err_state(p, "no IMU");
#endif
        break;
    case CMD_PRINT:     app_print_status(p); break;
    case CMD_INFO:      con_job(p, CON_JOB_INFO); break;
    case CMD_STATUS:
        app_print_status(p);
        con_printf(p, "mode %s, faults 0x%08lX, bat %ld mV%s\r\n", app_mode_name(), (unsigned long)sys_fault_mask(),
                   lroundf(bat_v() * 1000.0f), bat_present() ? "" : " (absent)");
        break;
    case CMD_AUTOPRINT: con_printf(p, con_autoprint_toggle(p) ? "Auto print ON\r\n" : "Auto print OFF\r\n"); break;

    case CMD_STOP:
        drv_stop(d);
        g_app.stopping = (g_app.mode == APP_BENCH);
        con_printf(p, "STOP\r\n");
        break;

    case CMD_RESET:
        if (g_app.mode == APP_FAULT) {
            con_printf(p, app_clear_faults() ? "Faults cleared\r\n" : "ERR state FAULT, cause still present\r\n");
        } else if (moving) {
            con_printf(p, "Busy, send s first\r\n");
        } else {
            enc_zero();
            g_app.ekf_primed = false;                        /* counts restart at 0: skip one EKF step */
            drv_reset_meas(d);
            sys_sched_stats_reset();                         /* `I` maxima and period restart too */
            con_printf(p, "Counters reset\r\n");
        }
        break;

    case CMD_CLEAR:
        con_printf(p, app_clear_faults() ? "Faults cleared\r\n" : "ERR state FAULT, cause still present\r\n");
        break;

    case CMD_MOVE:
        if (!motion_allowed(p)) { break; }
        {
            const st_t s = drv_move(d, c->f[0], q->drv_v_nom, q->drv_t_acc, q->drv_t_dec);
            if (s == ST_OK) {
                enter_bench(p);
                con_print_move(p, c->f[0], d->prof.V, d->prof.T);
            } else {
                reply_move_result(p, s);
            }
        }
        break;

    case CMD_TURN:
        if (!motion_allowed(p)) { break; }
        {
            const st_t s = drv_turn_enc(d, c->f[0], app_turn_w(q), q->drv_t_acc);
            if (s == ST_OK) {
                bool gyro = false;
#if FEAT_IMU
                gyro = imu_alive();                          /* app_ctrl checks the end angle against the yaw integral */
#endif
                g_app.turn_gyro = gyro;
                g_app.turn_target_rad = c->f[0];
                g_app.turn_acc_rad = 0.0f;
                g_app.turn_fix = 0U;
                enter_bench(p);
                con_printf(p, "Turn %ld deg%s\r\n", lroundf(c->f[0] * RAD2DEG_F), gyro ? " (gyro)" : "");
            } else { reply_move_result(p, s); }
        }
        break;

    case CMD_DUTY:
    case CMD_PWM:
        if (!motion_allowed(p)) { break; }
        {
            const float l = c->f[0];
            const float r = (c->type == CMD_PWM) ? c->f[1] : c->f[0];
            if (l == 0.0f && r == 0.0f) {                      /* old firmware: 0 = brake */
                drv_stop(d);
                g_app.stopping = (g_app.mode == APP_BENCH);
                con_printf(p, "Open loop duty 0 %%\r\n");
            } else {
                const st_t s = drv_open(d, l, r);
                if (s == ST_OK) {
                    enter_bench(p);
                    con_printf(p, "Open loop duty %ld %% %ld %%\r\n", lroundf(l * 100.0f), lroundf(r * 100.0f));
                } else {
                    reply_move_result(p, s);
                }
            }
        }
        break;

    case CMD_SPEED:
        id[0] = ID_V_NOM; v[0] = c->f[0];
        if (par_set_all(p, 1U, id, v)) { con_printf(p, "Vnom %ld mm/s\r\n", lroundf(c->f[0] * 1000.0f)); }
        break;

    case CMD_RAMP:
        id[0] = ID_T_ACC; v[0] = c->f[0]; id[1] = ID_T_DEC; v[1] = c->f[1];
        if (par_set_all(p, 2U, id, v)) {
            con_printf(p, "Ramps: accel %ld ms, decel %ld ms\r\n", lroundf(c->f[0] * 1000.0f), lroundf(c->f[1] * 1000.0f));
        }
        break;

    case CMD_FF: {
        uint16_t idf[4];
        float vf[4];
        uint8_t n = 0U;
        if (c->which != 2U) { idf[n] = ID_KS_L; vf[n++] = c->f[0]; idf[n] = ID_KV_L; vf[n++] = c->f[1]; }
        if (c->which != 1U) { idf[n] = ID_KS_R; vf[n++] = c->f[0]; idf[n] = ID_KV_R; vf[n++] = c->f[1]; }
        if (par_set_all(p, n, idf, vf)) { con_printf(p, "OK\r\n"); }
        break;
    }

    case CMD_GAIN:
        id[0] = ID_KP_V; v[0] = c->f[0]; id[1] = ID_KI_V; v[1] = c->f[1];
        if (par_set_all(p, 2U, id, v)) { con_printf(p, "OK\r\n"); }
        break;

    case CMD_SYNC:
        id[0] = ID_K_SYNC; v[0] = c->f[0];
        if (par_set_all(p, 1U, id, v)) { con_printf(p, "OK\r\n"); }
        break;

    case CMD_MODE:
        if (g_app.mode == APP_FAULT) { con_err_state(p, "FAULT"); break; }
        if (c->which == 1U) {
            if (g_app.mode == APP_IDLE) { drv_hold(d); enter_bench(p); }
            con_printf(p, "OK BENCH\r\n");
        } else {
            drv_stop(d);
            g_app.stopping = true;
            con_printf(p, "OK IDLE\r\n");
        }
        break;

    case CMD_PSET: {
        const st_t s = par_set(c->par->id, c->f[0]);
        if (s == ST_OK) {
            sys_event(EV_PARAM, (int32_t)c->par->id);
            float now;
            (void)par_get(c->par->id, &now);
            con_print_param(p, c->par, now);
        } else if (s == ST_STATE) {
            con_err_state(p, app_mode_name());
        } else {
            con_err_range_par(p, c->par);
        }
        break;
    }

    case CMD_PGET: {
        float now;
        (void)par_get(c->par->id, &now);
        con_print_param(p, c->par, now);
        break;
    }

    case CMD_FAULTINJ:
#ifdef DEBUG
        sys_fault_raise((fault_t)(int)c->f[0], 0);              /* H5 */
        con_printf(p, "injected 0x%02X\r\n", (unsigned)c->f[0]);
#else
        con_err_state(p, "Release build");
#endif
        break;

    case CMD_UNAVAILABLE:
        con_printf(p, "ERR state %s, not in this build (MANUAL/AUTO need phases 3 and 6)\r\n", app_mode_name());
        break;

    case CMD_NONE:
    default:
        break;
    }
}
