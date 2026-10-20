/* drv.c — spec §7.9 (REQ-DRV-01, -02, -05..08 for the phase-0 modes). */
#include "drv.h"

#include <string.h>

#include "par.h"

#define DRV_DONE_V_MPS  0.010f   /* "slow" at the end of a move, REQ-DRV-01 */
#define DRV_OL_DWELL_S  0.050f   /* zero time on an open-loop sign change, REQ-MOT-06 */

void drv_init(drv_t *d)
{
    memset(d, 0, sizeof *d);
    whl_init(&d->whl[WHL_L], WHL_L);
    whl_init(&d->whl[WHL_R], WHL_R);
}

void drv_reset_meas(drv_t *d) { d->have_prev = false; }

void drv_hold(drv_t *d)
{
    d->mode  = DRV_HOLD;
    d->phase = DRV_PH_IDLE;
}

void drv_brake(drv_t *d) { drv_hold(d); }   /* same state in phase 0; differs once VEL ramps exist */

void drv_disable(drv_t *d)
{
    d->mode  = DRV_DISABLED;
    d->phase = DRV_PH_IDLE;
}

static void start_move(drv_t *d, const prof_t *pr, float sig_l, float sig_r, drv_mode_t mode)
{
    d->prof       = *pr;
    d->mode       = mode;
    d->phase      = DRV_PH_MOVE;
    d->t_s        = 0.0f;
    d->settle_s   = 0.0f;
    d->sync_max_m = 0.0f;
    d->sig[WHL_L] = sig_l;
    d->sig[WHL_R] = sig_r;
    for (uint8_t i = 0U; i < 2U; i++) {
        d->x0[i]      = d->x_m[i];
        d->x_ref[i]   = d->x_m[i];
        d->v_ref[i]   = 0.0f;
        d->reached[i] = false;
        whl_reset(&d->whl[i]);
    }
}

st_t drv_move(drv_t *d, float dist_m, float v_mps, float t_acc_s, float t_dec_s)
{
    if (d->phase != DRV_PH_IDLE || !d->have_prev) { return ST_STATE; }
    prof_t pr;
    const st_t s = prof_plan(&pr, dist_m, v_mps, t_acc_s, t_dec_s);
    if (s != ST_OK) { return s; }
    start_move(d, &pr, 1.0f, 1.0f, DRV_MOVE);
    return ST_OK;
}

st_t drv_turn_enc(drv_t *d, float dtheta_rad, float w_radps, float t_acc_s)
{
    if (d->phase != DRV_PH_IDLE || !d->have_prev) { return ST_STATE; }
    const float half_b = 0.5f * par()->odo_wheelbase;      /* REQ-DRV-08 */
    prof_t pr;
    const st_t s = prof_plan(&pr, half_b * dtheta_rad, half_b * w_radps, t_acc_s, t_acc_s);
    if (s != ST_OK) { return s; }
    start_move(d, &pr, -1.0f, 1.0f, DRV_TURN_ENC);
    return ST_OK;
}

st_t drv_open(drv_t *d, float duty_l, float duty_r)
{
    if (d->phase != DRV_PH_IDLE) { return ST_STATE; }
    if (d->mode != DRV_OPEN) {
        d->mode = DRV_OPEN;                                 /* REQ-DRV-05: reset on entry */
        for (uint8_t i = 0U; i < 2U; i++) {
            d->ol_cur[i] = 0.0f;
            d->ol_dwell_s[i] = 0.0f;
            d->ol_sign[i] = 0;
        }
    }
    d->ol_target[WHL_L] = duty_l;
    d->ol_target[WHL_R] = duty_r;
    return ST_OK;
}

void drv_stop(drv_t *d)
{
    switch (d->mode) {
    case DRV_MOVE:
    case DRV_TURN_ENC:
        if (d->phase == DRV_PH_MOVE || d->phase == DRV_PH_SETTLE) {
            d->phase = DRV_PH_STOPPING;
            d->stop[WHL_L].y = d->v_ref[WHL_L];
            d->stop[WHL_R].y = d->v_ref[WHL_R];
        }
        break;
    case DRV_OPEN:
        drv_hold(d);
        break;
    case DRV_HOLD:
    case DRV_DISABLED:
    default:
        break;
    }
}

drv_status_t drv_status(const drv_t *d)
{
    drv_status_t s;
    s.mode       = d->mode;
    s.phase      = d->phase;
    s.sync_max_m = d->sync_max_m;
    s.t_s        = d->t_s;
    for (uint8_t i = 0U; i < 2U; i++) {
        s.d_m[i]   = d->x_m[i] - d->x0[i];
        s.v_mps[i] = d->v_hat[i];
    }
    return s;
}

/* ---- open loop: slew limit, 50 ms of zero on a sign change ---------------------------- */
static void open_step(drv_t *d, const par_t *p, float dt, drv_out_t *out)
{
    const float step = p->mot_slew_ol * dt;
    for (uint8_t i = 0U; i < 2U; i++) {
        const float t = clampf(d->ol_target[i], -1.0f, 1.0f);
        float c = d->ol_cur[i];
        if (d->ol_dwell_s[i] > 0.0f) {
            d->ol_dwell_s[i] -= dt;
            c = 0.0f;
        } else {
            const float want = (c * t < 0.0f) ? 0.0f : t;      /* reverse through zero */
            c += clampf(want - c, -step, step);
            if (c != 0.0f) {
                d->ol_sign[i] = (c > 0.0f) ? 1 : -1;
            } else if (t != 0.0f && d->ol_sign[i] != 0 && ((t > 0.0f) != (d->ol_sign[i] > 0))) {
                d->ol_dwell_s[i] = DRV_OL_DWELL_S;
                d->ol_sign[i] = 0;
            }
        }
        d->ol_cur[i] = c;
        out->duty_d[i] = c;
        whl_reset(&d->whl[i]);
    }
}

/* ---- closed-loop moves --------------------------------------------------------------- */
static void finish(drv_t *d, drv_out_t *out, drv_event_t ev)
{
    out->event = ev;
    drv_hold(d);
}

static fault_t first_fault(const whl_out_t wo[2])
{
    for (uint8_t i = 0U; i < 2U; i++) {
        if ((wo[i].flags & WHL_RUNAWAY) != 0U) { return (i == WHL_L) ? F_RUNAWAY_L : F_RUNAWAY_R; }
    }
    for (uint8_t i = 0U; i < 2U; i++) {
        if ((wo[i].flags & WHL_STALL) != 0U) { return (i == WHL_L) ? F_STALL_L : F_STALL_R; }
    }
    for (uint8_t i = 0U; i < 2U; i++) {
        if ((wo[i].flags & WHL_TRACK) != 0U) { return (i == WHL_L) ? F_TRACK_L : F_TRACK_R; }
    }
    return F_NONE;
}

static void move_step(drv_t *d, const par_t *p, const drv_meas_t *m, float dt, float dt_raw,
                      drv_out_t *out)
{
    d->t_s += dt;
    if (d->phase == DRV_PH_MOVE && d->t_s >= d->prof.T) {
        d->phase = DRV_PH_SETTLE;
        d->settle_s = 0.0f;
    }

    whl_in_t in[2];
    bool brake[2] = { false, false };
    float x_ref[2], v_ref[2], a_ref[2];

    if (d->phase == DRV_PH_SETTLE) {
        d->settle_s += dt;
        for (uint8_t i = 0U; i < 2U; i++) {
            const float fin  = d->x0[i] + d->sig[i] * d->prof.D;
            const float wdir = d->sig[i] * (float)d->prof.dir;
            const float e    = fin - d->x_m[i];
            if (!d->reached[i] && wdir * e < p->drv_pos_tol) { d->reached[i] = true; }
            x_ref[i] = fin;
            a_ref[i] = 0.0f;
            v_ref[i] = clampf(p->whl_kp_x * e, -p->drv_v_settle, p->drv_v_settle);
            if (wdir * v_ref[i] < 0.0f) { v_ref[i] = 0.0f; }   /* never backwards */
            brake[i] = d->reached[i];
            in[i].v_sync_mps = 0.0f;
            in[i].pos_loop   = false;
        }
        if (d->reached[WHL_L] && d->reached[WHL_R]
            && fabsf(d->v_hat[WHL_L]) < DRV_DONE_V_MPS && fabsf(d->v_hat[WHL_R]) < DRV_DONE_V_MPS) {
            finish(d, out, DRV_EV_DONE);
            return;
        }
        if (d->settle_s >= p->drv_settle_timeout) {
            finish(d, out, DRV_EV_TIMEOUT);
            return;
        }
    } else {
        if (d->phase == DRV_PH_MOVE) {
            float x, v, a;
            prof_eval(&d->prof, d->t_s, &x, &v, &a);
            for (uint8_t i = 0U; i < 2U; i++) {
                x_ref[i] = d->x0[i] + d->sig[i] * x;
                v_ref[i] = d->sig[i] * v;
                a_ref[i] = d->sig[i] * a;
            }
        } else {                                                /* DRV_PH_STOPPING */
            for (uint8_t i = 0U; i < 2U; i++) {
                const float v_old = d->stop[i].y;
                const float v_new = ramp_step(&d->stop[i], 0.0f, p->drv_a_stop, p->drv_a_stop, dt);
                x_ref[i] = d->x_ref[i] + 0.5f * (v_old + v_new) * dt;
                v_ref[i] = v_new;
                a_ref[i] = (v_new - v_old) / dt;
            }
        }
        /* REQ-DRV-02: sig maps wheel errors onto the common "along the path" direction */
        const float delta = d->sig[WHL_L] * (x_ref[WHL_L] - d->x_m[WHL_L])
                          - d->sig[WHL_R] * (x_ref[WHL_R] - d->x_m[WHL_R]);
        if (fabsf(delta) > d->sync_max_m) { d->sync_max_m = fabsf(delta); }
        in[WHL_L].v_sync_mps =  d->sig[WHL_L] * 0.5f * p->drv_k_sync * delta;
        in[WHL_R].v_sync_mps = -d->sig[WHL_R] * 0.5f * p->drv_k_sync * delta;
        in[WHL_L].pos_loop = true;
        in[WHL_R].pos_loop = true;
    }

    whl_out_t wo[2];
    for (uint8_t i = 0U; i < 2U; i++) {
        d->x_ref[i] = x_ref[i];
        d->v_ref[i] = v_ref[i];
        if (brake[i]) {                                         /* reached: brake, forget I */
            whl_reset(&d->whl[i]);
            memset(&wo[i], 0, sizeof wo[i]);
            continue;
        }
        in[i].x_ref_m = x_ref[i];
        in[i].v_ref_mps = v_ref[i];
        in[i].a_ref_mps2 = a_ref[i];
        in[i].x_m = d->x_m[i];
        in[i].v_mps = d->v_hat[i];
        in[i].u_bat_v = m->u_bat_v;
        in[i].dt_s = dt_raw;
        whl_step(&d->whl[i], &in[i], &wo[i]);
        out->duty_d[i] = wo[i].duty_d;
    }

    const fault_t f = first_fault(wo);                          /* REQ-DRV-06 */
    if (f != F_NONE) {
        out->fault = f;
        out->duty_d[WHL_L] = 0.0f;
        out->duty_d[WHL_R] = 0.0f;
        if (f == F_RUNAWAY_L || f == F_RUNAWAY_R) { drv_disable(d); } else { drv_hold(d); }
        return;
    }
    if (d->phase == DRV_PH_STOPPING && d->stop[WHL_L].y == 0.0f && d->stop[WHL_R].y == 0.0f) {
        drv_hold(d);
        out->duty_d[WHL_L] = 0.0f;
        out->duty_d[WHL_R] = 0.0f;
    }
}

void drv_step(drv_t *d, const drv_meas_t *m, drv_out_t *out)
{
    const par_t *p = par();
    memset(out, 0, sizeof *out);

    float dt_raw = CTRL_DT_S;
    if (d->have_prev) { dt_raw = (float)(uint32_t)(m->t_us - d->t_prev_us) * 1.0e-6f; }
    const float dt = whl_dt_guard(dt_raw, NULL);

    const float mpc[2] = { p->odo_m_per_cnt * p->odo_scale_l, p->odo_m_per_cnt * p->odo_scale_r };
    for (uint8_t i = 0U; i < 2U; i++) {
        const float x = (float)m->cnt[i] * mpc[i];
        if (d->have_prev) {
            d->v_hat[i] += p->whl_speed_alpha * ((x - d->x_m[i]) / dt - d->v_hat[i]);
        } else {
            d->v_hat[i] = 0.0f;
        }
        d->x_m[i] = x;
    }
    d->have_prev = true;
    d->t_prev_us = m->t_us;

    switch (d->mode) {
    case DRV_HOLD:
        out->enable = true;
        whl_reset(&d->whl[WHL_L]);
        whl_reset(&d->whl[WHL_R]);
        break;
    case DRV_OPEN:
        out->enable = true;
        open_step(d, p, dt, out);
        break;
    case DRV_MOVE:
    case DRV_TURN_ENC:
        out->enable = true;
        move_step(d, p, m, dt, dt_raw, out);
        out->enable = (d->mode != DRV_DISABLED);       /* a runaway fault disables */
        break;
    case DRV_DISABLED:
    default:
        break;
    }
}
