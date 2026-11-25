/* msn.c — spec §7.21 (REQ-MSN-01..06, REQ-GND-02/04 reactions, REQ-FWD-01 staleness). */
#include "msn.h"

#include <string.h>

#include "par.h"

/* Literals of the spec that have no parameter ID. */
#define MSN_BIAS_LEN_M     2.0f    /* §7.21.3 RECOVER: bias holds for the next 2 m */
#define MSN_FINISH_S       2.0f    /* AUTO_FINISHED -> IDLE */
#define MSN_OBST_EV_S      10.0f   /* AUTO_OBST_WAIT: event period */
#define MSN_FWD_STALE_V    0.15f   /* §11 F_FWD_STALE in MANUAL: forward limit, m/s */
#define MSN_RECOVER_MAX    3U      /* recoveries per segment */

typedef struct {
    void        (*entry)(msn_t *, const msn_in_t *, msn_out_t *);
    msn_state_t (*cycle)(msn_t *, const msn_in_t *, msn_out_t *);
} state_ops_t;

/* ---- helpers --------------------------------------------------------------------------- */
static void emit(msn_out_t *o, msn_ev_t code, int32_t a, int32_t b, float x, float y, float z)
{
    if (o->n_ev < MSN_EV_N) { o->ev[o->n_ev++] = (msn_event_t){ code, a, b, x, y, z }; }
}

static float el_s(uint32_t now, uint32_t t0) { return (float)(int32_t)(now - t0) * 1.0e-3f; }

static bool still(const msn_in_t *in) { return (in->est.flags & EKF_STILL) != 0U; }

static bool still_for(const msn_t *m, const msn_in_t *in, float s)
{
    return still(in) && el_s(in->t_ms, m->still_t) >= s;
}

static bool drv_done(const msn_in_t *in) { return in->drv_ev == DRV_EV_DONE || in->drv_ev == DRV_EV_TIMEOUT; }

/* +1: turning left moves away from a tripped right side, -1: the opposite, 0: unknown/both */
static float away_dir(const msn_in_t *in)
{
    const bool l = (in->gnd_l == GND_TOO_CLOSE), r = (in->gnd_r == GND_TOO_CLOSE);
    return (r && !l) ? 1.0f : ((l && !r) ? -1.0f : 0.0f);
}

static void set_mode(msn_t *m, msn_mode_t nm, msn_out_t *o)
{
    emit(o, MSN_EV_MODE, (int32_t)m->mode, (int32_t)nm, 0.0f, 0.0f, 0.0f);
    m->mode = nm;
    m->st = MSN_AUTO_NONE;
    if (nm == MSN_MANUAL) { m->man_valid = false; }
}

/* R2: controlled stop, R3/R4: brake; EN goes low once the vehicle stands (§7.21.2) */
static void enter_fault(msn_t *m, reaction_t r, fault_t code, msn_out_t *o)
{
    if (code != F_NONE) { o->fault = code; }
    if (m->st != MSN_AUTO_NONE) { emit(o, MSN_EV_STATE, (int32_t)m->st, (int32_t)MSN_AUTO_NONE, 0.0f, 0.0f, 0.0f); }
    o->cmd = (r >= R3) ? MC_BRAKE : MC_STOP;
    set_mode(m, MSN_FAULT, o);
    m->disable_pending = true;
}

static void next_seg(msn_t *m)
{
    if ((uint16_t)(m->seg + 1U) < m->course->n_seg) { m->seg++; }
    m->rec_cnt = 0U;
}

static const seg_t *cur(const msn_t *m) { return &m->course->seg[m->seg]; }

static uint32_t acc_sum(const ekf_out_t *e)
{
    return e->n_acc[EKF_U_EDGE] + e->n_acc[EKF_U_FACE] + e->n_acc[EKF_U_HEADING] + e->n_acc[EKF_U_POINT];
}

/* REQ-MSN-05 after the fault reaction: interlock, then reflex. Returns the state to go to, or the
 * current one. Only the moving states call it. */
static msn_state_t safety(const msn_t *m, const msn_in_t *in, bool reflex_applies)
{
    if (in->interlock) { return MSN_AUTO_RECOVER; }
    if (reflex_applies && in->reflex) { return MSN_AUTO_OBST_STOP; }
    return m->st;
}

/* ---- AUTO states ----------------------------------------------------------------------- */
static void en_init(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    o->cmd = MC_HOLD;
    m->t0 = in->t_ms;
    m->acc_prev = acc_sum(&in->est);
    m->d_noref = 0.0f;
    m->s_prev = in->s_enc;
    m->degraded = false;
}

static msn_state_t cy_init(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    const par_t *p = par();
    if (still_for(m, in, p->msn_init_min_s) && sqrtf(in->est.P_diag[3]) < p->ekf_bg_conv) {
        m->t_run0 = in->t_ms;
        return MSN_AUTO_SEG;
    }
    if (el_s(in->t_ms, m->t0) > p->msn_init_max_s) { enter_fault(m, R2, F_INIT, o); }
    return m->st;
}

static void en_seg(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    (void)o;
    nav_begin_segment(&m->nav, cur(m), &in->est, in->s_enc);
}

static msn_state_t cy_seg(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    (void)in;
    (void)o;
    switch (cur(m)->type) {
    case SEG_LINE:
    case SEG_ARC:      return MSN_AUTO_DRIVE;
    case SEG_TURN:     return MSN_AUTO_PRETURN;
    case SEG_WAYPOINT: return MSN_AUTO_HOLD;
    case SEG_PAUSE:    return MSN_AUTO_PAUSE;
    case SEG_FINISH:
    default:           return MSN_AUTO_FINISHED;
    }
}

static msn_state_t cy_drive(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    const par_t *p = par();
    const msn_state_t sf = safety(m, in, true);
    if (sf != m->st) { return sf; }
    if (m->bias != 0.0f && in->s_enc >= m->bias_end) { m->bias = 0.0f; }
    float vl = in->v_limit;
    if (m->degraded) { vl = fminf(vl, p->msn_v_degraded); }
    const nav_cmd_t c = nav_follow(&m->nav, &in->est, vl, m->bias, m->dt);
    o->y_off = m->bias;
    switch (c.type) {
    case NAV_VEL:
        o->cmd = MC_VEL;
        o->v = c.v;
        o->w = c.w;
        if (c.done) { next_seg(m); return MSN_AUTO_SEG; }
        break;
    case NAV_MOVE_FROM:
        m->fin = c;
        return MSN_AUTO_FINAL;
    case NAV_STOP:                                  /* pose not finite: the EKF raises F_EKF */
        o->cmd = MC_STOP;
        break;
    case NAV_NONE:
    default:
        break;
    }
    return m->st;
}

static void en_final(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    const par_t *p = par();
    o->cmd = MC_MOVE_FROM;
    o->D = m->fin.D;
    o->v0 = m->fin.v0;
    o->v = m->fin.v;
    o->a_acc = p->nav_a_acc;
    o->a_dec = p->nav_a_dec;
    (void)nav_final(&m->nav, &in->est, in->s_enc);   /* latches the encoder start of the final stop */
}

static msn_state_t cy_final(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    const msn_state_t sf = safety(m, in, true);
    if (sf != m->st) { return sf; }
    (void)nav_final(&m->nav, &in->est, in->s_enc);
    if (drv_done(in)) {
        emit(o, MSN_EV_FINAL_DONE, (int32_t)in->drv_ev, 0, nav_debug(&m->nav)->miss_m, 0.0f, 0.0f);
        next_seg(m);
        return MSN_AUTO_SEG;
    }
    return m->st;
}

static void en_hold(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    (void)m;
    o->cmd = MC_HOLD;
    (void)in;
}

static msn_state_t cy_preturn(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    (void)o;
    return still_for(m, in, par()->msn_preturn_s) ? MSN_AUTO_TURN : m->st;     /* ZUPT before every turn */
}

static void en_turn(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    const par_t *p = par();
    m->turn_enc = !(in->feat_imu && in->imu_alive);
    if (m->turn_enc) {                               /* no gyro heading: encoder turn (§7.19) */
        o->cmd = MC_TURN_ENC;
        o->D = wrap_pi(cur(m)->g.turn.theta - in->est.x[2]);
        o->w = p->nav_turn_w;
        o->t_acc = p->nav_turn_ta;
    }
}

static msn_state_t cy_turn(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    if (in->interlock) { return MSN_AUTO_RECOVER; }
    bool done;
    if (m->turn_enc) {
        done = drv_done(in);
    } else {
        const nav_cmd_t c = nav_turn(&m->nav, &in->est, in->est.w_meas_radps, m->dt);
        o->cmd = MC_VEL;
        o->v = 0.0f;
        o->w = c.w;
        done = c.done;
    }
    if (done) { next_seg(m); return MSN_AUTO_SEG; }
    return m->st;
}

static void en_wp_hold(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    o->cmd = MC_HOLD;
    m->tmr_on = false;
    emit(o, MSN_EV_WP_ARRIVED, (int32_t)cur(m)->g.wp.id, 0, in->est.x[0], in->est.x[1],
         sqrtf(in->est.P_diag[0] + in->est.P_diag[1]));
}

static msn_state_t cy_wp_hold(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    if (!m->tmr_on && still(in)) {                   /* the hold counts from the first still tick */
        m->tmr_on = true;
        m->t_tmr = in->t_ms;
    }
    const float hold = (cur(m)->g.wp.hold_s > 0.0f) ? cur(m)->g.wp.hold_s : par()->msn_wp_hold;
    if (m->tmr_on && el_s(in->t_ms, m->t_tmr) >= hold) {
        emit(o, MSN_EV_WP_DONE, (int32_t)cur(m)->g.wp.id, 0, 0.0f, 0.0f, 0.0f);
        next_seg(m);
        return MSN_AUTO_SEG;
    }
    return m->st;
}

static void en_pause(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    o->cmd = MC_HOLD;
    m->t0 = in->t_ms;
}

static msn_state_t cy_pause(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    (void)o;
    if (el_s(in->t_ms, m->t0) >= cur(m)->g.pause.t_s) { next_seg(m); return MSN_AUTO_SEG; }
    return m->st;
}

static void en_obst_stop(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    (void)m;
    (void)in;
    o->cmd = MC_BRAKE;
    emit(o, MSN_EV_OBSTACLE, 0, 0, 0.0f, 0.0f, 0.0f);
}

static msn_state_t cy_obst_stop(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    (void)o;
    return still(in) ? MSN_AUTO_OBST_WAIT : m->st;
}

static void en_obst_wait(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    (void)o;
    m->t0 = in->t_ms;
    m->last_ev = in->t_ms;
}

/* The 0.5 s clear time of the spec table is the one inside fwd (REQ-FWD-01), not added twice. Evasion
 * (phase 6): after msn_obst_wait_s with a free plan this goes to AUTO_EVADE; today it only waits. */
static msn_state_t cy_obst_wait(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    if (!in->reflex) { return MSN_AUTO_SEG; }        /* re-plan the segment (DRIVE, or FINAL with the new distance) */
    if (el_s(in->t_ms, m->last_ev) >= MSN_OBST_EV_S) {
        m->last_ev = in->t_ms;
        emit(o, MSN_EV_OBST_WAIT, (int32_t)el_s(in->t_ms, m->t0), 0, 0.0f, 0.0f, 0.0f);
    }
    return m->st;
}

static void en_recover(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    o->cmd = MC_BRAKE;
    m->rec_cnt++;
    m->rec_ph = 0U;
    m->rec_dir = away_dir(in);
}

static msn_state_t cy_recover(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    const par_t *p = par();
    if (m->rec_cnt > MSN_RECOVER_MAX) { enter_fault(m, R3, F_GND_INTERLOCK, o); return m->st; }
    if (m->rec_ph == 0U && still(in)) {
        o->cmd = MC_MOVE;
        o->D = -p->msn_rec_back_m;
        o->v = p->drv_v_nom;
        o->t_acc = p->drv_t_acc;
        o->t_dec = p->drv_t_dec;
        m->rec_ph = 1U;
    } else if (m->rec_ph == 1U && drv_done(in)) {
        if (in->interlock) { enter_fault(m, R3, F_GND_INTERLOCK, o); return m->st; }
        m->bias = m->rec_dir * p->msn_rec_bias_m;
        m->bias_end = in->s_enc + MSN_BIAS_LEN_M;
        return MSN_AUTO_SEG;                         /* back to DRIVE with a fresh nav speed */
    }
    return m->st;
}

static void en_stopped(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    (void)m;
    (void)in;
    o->cmd = MC_STOP;
}

static void en_finished(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    o->cmd = MC_HOLD;
    m->t0 = in->t_ms;
    emit(o, MSN_EV_FINISHED, 0, 0, el_s(in->t_ms, m->t_run0), 0.0f, 0.0f);
}

static msn_state_t cy_finished(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    (void)o;
    return (el_s(in->t_ms, m->t0) >= MSN_FINISH_S) ? MSN_AUTO_NONE : m->st;
}

static msn_state_t cy_stay(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    (void)in;
    (void)o;
    return m->st;
}

static const state_ops_t OPS[MSN_AUTO_N] = {
    [MSN_AUTO_INIT]      = { en_init,      cy_init },
    [MSN_AUTO_SEG]       = { en_seg,       cy_seg },
    [MSN_AUTO_DRIVE]     = { NULL,         cy_drive },
    [MSN_AUTO_FINAL]     = { en_final,     cy_final },
    [MSN_AUTO_PRETURN]   = { en_hold,      cy_preturn },
    [MSN_AUTO_TURN]      = { en_turn,      cy_turn },
    [MSN_AUTO_HOLD]      = { en_wp_hold,   cy_wp_hold },
    [MSN_AUTO_PAUSE]     = { en_pause,     cy_pause },
    [MSN_AUTO_OBST_STOP] = { en_obst_stop, cy_obst_stop },
    [MSN_AUTO_OBST_WAIT] = { en_obst_wait, cy_obst_wait },
    [MSN_AUTO_RECOVER]   = { en_recover,   cy_recover },
    [MSN_AUTO_STOPPED]   = { en_stopped,   cy_stay },
    [MSN_AUTO_FINISHED]  = { en_finished,  cy_finished },
};

static void goto_state(msn_t *m, msn_state_t nx, const msn_in_t *in, msn_out_t *o)
{
    emit(o, MSN_EV_STATE, (int32_t)m->st, (int32_t)nx, 0.0f, 0.0f, 0.0f);     /* REQ-MSN-06 */
    if (nx == MSN_AUTO_NONE) {                      /* left AUTO by itself: IDLE, EN low when still */
        set_mode(m, MSN_IDLE, o);
        m->disable_pending = true;
        return;
    }
    m->st = nx;
    if (OPS[nx].entry != NULL) { OPS[nx].entry(m, in, o); }
}

static void auto_step(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    const par_t *p = par();
    /* REQ-MSN-04 */
    const uint32_t acc = acc_sum(&in->est);
    if (acc != m->acc_prev) {
        m->acc_prev = acc;
        m->d_noref = 0.0f;
        m->degraded = false;
    }
    m->d_noref += fabsf(in->s_enc - m->s_prev);
    m->s_prev = in->s_enc;
    if (m->d_noref > p->msn_noref_dist && in->sig_perp > p->msn_sig_lat_max) { m->degraded = true; }

    if (in->fwd_stale) { enter_fault(m, R2, F_FWD_STALE, o); return; }

    for (uint8_t k = 0U; k < 4U; k++) {              /* SEG hands over in the same tick */
        const msn_state_t s = m->st;
        const msn_state_t nx = OPS[s].cycle(m, in, o);
        if (m->mode != MSN_AUTO || nx == s) { return; }
        goto_state(m, nx, in, o);
        if (m->mode != MSN_AUTO) { return; }
    }
}

/* ---- modes ----------------------------------------------------------------------------- */
static msn_refuse_t start_check(const msn_t *m, const msn_in_t *in)
{
    const par_t *p = par();
    if (!m->course_ok) { return MSN_REF_COURSE; }
    if (in->fault_mask != 0U) { return MSN_REF_FAULT; }
    if (!((in->u_bat_v >= BAT_ABSENT_V && in->u_bat_v > p->bat_warn_v) || p->msn_allow_bench_supply > 0.5f)) {
        return MSN_REF_BATTERY;
    }
    if (in->feat_imu && !in->imu_alive) { return MSN_REF_IMU; }
    if (in->feat_vl && !in->sensors_fresh && p->msn_allow_nosensors < 0.5f) { return MSN_REF_SENSORS; }
    if (!still(in)) { return MSN_REF_MOVING; }
    return MSN_START_OK;
}

static void handle_req(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    const msn_req_t r = m->req;
    m->req = MSN_REQ_NONE;
    switch (r) {
    case MSN_REQ_STOP:
        if (m->mode == MSN_FAULT || m->mode == MSN_IDLE) { break; }
        o->cmd = MC_STOP;
        m->man_valid = false;
        if (m->mode == MSN_AUTO && m->st != MSN_AUTO_STOPPED && m->st != MSN_AUTO_FINISHED) {
            m->resume = (m->st == MSN_AUTO_INIT || m->st == MSN_AUTO_HOLD || m->st == MSN_AUTO_PAUSE)
                        ? m->st : MSN_AUTO_SEG;
            goto_state(m, MSN_AUTO_STOPPED, in, o);
        }
        break;
    case MSN_REQ_IDLE:
        if (m->mode == MSN_BENCH || m->mode == MSN_MANUAL || m->mode == MSN_AUTO) {
            if (m->mode == MSN_AUTO) { goto_state(m, MSN_AUTO_NONE, in, o); }
            else { set_mode(m, MSN_IDLE, o); m->disable_pending = true; }
            o->cmd = MC_STOP;
        }
        break;
    case MSN_REQ_BENCH:
    case MSN_REQ_MANUAL:
        if (m->mode == MSN_IDLE) {
            o->cmd = MC_HOLD;
            set_mode(m, (r == MSN_REQ_BENCH) ? MSN_BENCH : MSN_MANUAL, o);
            m->disable_pending = false;
        }
        break;
    case MSN_REQ_AUTO:
        if (m->mode == MSN_AUTO && m->st == MSN_AUTO_STOPPED) {
            goto_state(m, m->resume, in, o);
        } else if (m->mode == MSN_IDLE) {
            const msn_refuse_t why = start_check(m, in);
            if (why != MSN_START_OK) {
                o->refused = why;
                emit(o, MSN_EV_START_REFUSED, (int32_t)why, 0, 0.0f, 0.0f, 0.0f);
                break;
            }
            o->reset_pose = true;
            o->pose[0] = m->course->start_x;
            o->pose[1] = m->course->start_y;
            o->pose[2] = m->course->start_theta;
            m->seg = 0U;
            m->rec_cnt = 0U;
            m->bias = 0.0f;
            m->disable_pending = false;
            set_mode(m, MSN_AUTO, o);
            goto_state(m, MSN_AUTO_INIT, in, o);
        }
        break;
    case MSN_REQ_CLEAR:
        if (m->mode == MSN_FAULT && in->fault_reaction < R2) {
            set_mode(m, MSN_IDLE, o);
            m->disable_pending = false;
        }
        break;
    case MSN_REQ_NONE:
    default:
        break;
    }
}

/* REQ-MSN-03 */
static void manual_step(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    const par_t *p = par();
    if (!m->man_valid || m->man_to) { return; }
    const int32_t age_ms = (int32_t)(in->t_ms - m->man_t);
    if ((float)age_ms * 1.0e-3f > p->msn_man_timeout) {
        m->man_to = true;
        o->cmd = MC_STOP;
        emit(o, MSN_EV_MAN_TIMEOUT, 0, 0, 0.0f, 0.0f, 0.0f);
        return;
    }
    if (in->interlock && !m->il_prev) { o->cmd = MC_BRAKE; return; }
    float v = clampf(m->man_v, -p->msn_man_v_max, p->msn_man_v_max);
    float w = clampf(m->man_w, -p->msn_man_w_max, p->msn_man_w_max);
    if (in->reflex && v > 0.0f) { v = 0.0f; }        /* forward only; turning and reversing stay possible */
    if (in->fwd_stale) { v = fminf(v, MSN_FWD_STALE_V); }
    if (in->interlock) {                             /* only motion away from the tripped side */
        const float a = away_dir(in);
        v = fminf(v, 0.0f);
        w = (a > 0.0f) ? fmaxf(w, 0.0f) : ((a < 0.0f) ? fminf(w, 0.0f) : 0.0f);
    }
    o->cmd = MC_VEL;
    o->v = v;
    o->w = w;
}

void msn_init(msn_t *m)
{
    memset(m, 0, sizeof *m);
    nav_init(&m->nav);
    m->course = map_course();
    m->course_ok = (map_validate(m->course) == ST_OK);
}

void msn_request(msn_t *m, msn_req_t req) { m->req = req; }

void msn_manual(msn_t *m, float v, float w, uint32_t t_rx_ms)
{
    m->man_v = isfinite(v) ? v : 0.0f;
    m->man_w = isfinite(w) ? w : 0.0f;
    m->man_t = t_rx_ms;
    m->man_valid = true;
    m->man_to = false;
}

msn_mode_t msn_mode(const msn_t *m) { return m->mode; }
msn_state_t msn_state(const msn_t *m) { return m->st; }

void msn_step(msn_t *m, const msn_in_t *in, msn_out_t *o)
{
    memset(o, 0, sizeof *o);
    const float dt_raw = m->have_t ? (float)(uint32_t)(in->t_ms - m->t_prev_ms) * 1.0e-3f : CTRL_DT_S;
    m->dt = whl_dt_guard(dt_raw, NULL);
    m->have_t = true;
    m->t_prev_ms = in->t_ms;
    if (still(in)) {
        if (!m->still_on) { m->still_on = true; m->still_t = in->t_ms; }
    } else {
        m->still_on = false;
    }

    if (m->mode != MSN_FAULT && in->fault_reaction >= R2) {            /* REQ-MSN-05: faults first */
        m->req = MSN_REQ_NONE;
        enter_fault(m, in->fault_reaction, F_NONE, o);
    } else {
        handle_req(m, in, o);
    }

    switch (m->mode) {
    case MSN_BENCH:
        if (in->interlock && !m->il_prev && par()->gnd_interlock_bench > 0.5f) { o->cmd = MC_BRAKE; }
        break;
    case MSN_MANUAL:
        manual_step(m, in, o);
        break;
    case MSN_AUTO:
        auto_step(m, in, o);
        break;
    case MSN_IDLE:
    case MSN_FAULT:
    default:
        if (m->disable_pending && still(in) && o->cmd == MC_NONE) {
            o->cmd = MC_DISABLE;
            m->disable_pending = false;
        }
        break;
    }
    m->il_prev = in->interlock;
    o->seg = m->seg;
}
