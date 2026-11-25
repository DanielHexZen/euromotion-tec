/* nav.c — spec §7.19 (REQ-NAV-01..03). */
#include "nav.h"

#include <string.h>

#include "par.h"

#define NAV_ALIGN_EY_M        0.05f                /* REQ-NAV-02 window */
#define NAV_ALIGN_EPSI_RAD    (3.0f * DEG2RAD_F)
#define NAV_FINAL_FORCE_M     0.10f                /* final stop at this distance regardless of alignment */
#define NAV_TURN_NOREV_RAD    (0.5f * DEG2RAD_F)   /* REQ-NAV-03: no reversal inside */
#define NAV_TURN_W_STILL      0.02f                /* rad/s, REQ-NAV-03 */
#define NAV_TURN_T_STILL_S    0.2f
#define NAV_TURN_T_GIVEUP_S   2.0f

static bool is_geo(const seg_t *s) { return s->type == SEG_LINE || s->type == SEG_ARC; }

void nav_init(nav_t *n) { memset(n, 0, sizeof *n); }

void nav_begin_segment(nav_t *n, const seg_t *sg, const ekf_out_t *est, float s_enc)
{
    (void)est;
    (void)s_enc;
    const bool keep = (n->sg != NULL && is_geo(n->sg) && (n->sg->flags & SEG_F_STOP_AT_END) == 0U && n->sg != sg);
    if (!keep) { n->v_prev = 0.0f; }
    n->sg           = sg;
    n->L            = map_seg_len(sg);
    n->final_issued = false;
    n->fin_started  = false;
    n->turn_init    = false;
    /* shortcut: the next segment is sg + 1 (segments are an array); safe because a LINE/ARC is never
     * the last element (validation: FINISH is). Upgrade if segments become a linked list. */
    n->v_end = (!is_geo(sg) || (sg->flags & SEG_F_STOP_AT_END) != 0U || !is_geo(sg + 1)) ? 0.0f : sg[1].v_max;
}

nav_cmd_t nav_follow(nav_t *n, const ekf_out_t *est, float v_limit, float y_off, float dt)
{
    const par_t *p = par();
    nav_cmd_t c = { .type = NAV_VEL };
    map_frame_t f;
    map_seg_frame(n->sg, est->x[0], est->x[1], est->x[2], &f);
    if (!isfinite(f.s + f.e_y + f.e_psi)) {
        c.type = NAV_STOP;
        return c;
    }
    const float rem = f.L - f.s;
    const bool stop_end = (n->sg->flags & SEG_F_STOP_AT_END) != 0U;

    float v = fminf(n->sg->v_max, v_limit);
    v = fminf(v, sqrtf(n->v_end * n->v_end + 2.0f * p->nav_a_dec * fmaxf(rem, 0.0f)));
    v = fmaxf(fminf(v, n->v_prev + p->nav_a_acc * dt), 0.0f);
    n->v_prev = v;

    const float k_psi = 2.0f * p->nav_zeta * p->nav_wn;
    const float k_y   = p->nav_wn * p->nav_wn / fmaxf(fabsf(v), p->nav_v_min);
    const float w = clampf(f.kappa * v - k_psi * f.e_psi - k_y * (f.e_y - y_off), -p->nav_w_max, p->nav_w_max);

    c.v = v;
    c.w = w;
    c.aligned = fabsf(f.e_y) < NAV_ALIGN_EY_M && fabsf(f.e_psi) < NAV_ALIGN_EPSI_RAD;
    c.s_rem = rem;
    c.done = !stop_end && rem <= 0.0f;
    n->dbg = (nav_dbg_t){ f.s, f.L, f.e_y, f.e_psi, v, w, n->dbg.miss_m };

    if (stop_end && !n->final_issued && ((rem <= p->nav_final_dist && c.aligned) || rem <= NAV_FINAL_FORCE_M)) {
        n->final_issued = true;
        n->fin_D = fmaxf(rem, p->drv_pos_tol);
        c.type = NAV_MOVE_FROM;
        c.D  = n->fin_D;
        c.v0 = v;
        c.v  = fmaxf(v, p->nav_v_min);
    }
    return c;
}

nav_cmd_t nav_final(nav_t *n, const ekf_out_t *est, float s_enc)
{
    map_frame_t f;
    map_seg_frame(n->sg, est->x[0], est->x[1], est->x[2], &f);
    if (!n->fin_started) {
        n->fin_started = true;
        n->fin_s0 = s_enc;
    }
    n->v_prev = 0.0f;
    nav_cmd_t c = { .type = NAV_NONE };
    c.s_rem = n->fin_D - (s_enc - n->fin_s0);
    c.done = c.s_rem <= par()->drv_pos_tol;
    n->dbg = (nav_dbg_t){ f.s, f.L, f.e_y, f.e_psi, 0.0f, 0.0f, f.L - f.s };
    return c;
}

nav_cmd_t nav_turn(nav_t *n, const ekf_out_t *est, float w_meas, float dt)
{
    const par_t *p = par();
    const float th = est->x[2];
    if (!n->turn_init) {
        n->turn_init = true;
        n->th_t  = n->sg->g.turn.theta;
        n->th0   = th;
        const float d = wrap_pi(n->th_t - th);
        n->dir   = signf(d);
        n->prof_ok = (prof_plan(&n->prof, d, p->nav_turn_w, p->nav_turn_ta, p->nav_turn_ta) == ST_OK);
        n->t_s = 0.0f;
        n->t_ok_s = 0.0f;
        n->v_prev = 0.0f;
    }
    n->t_s += dt;
    float x = 0.0f, v = 0.0f, a;
    if (n->prof_ok) { prof_eval(&n->prof, n->t_s, &x, &v, &a); }
    const float res = wrap_pi(n->th_t - th);
    float w = clampf(v + p->nav_turn_k * wrap_pi(n->th0 + x - th), -p->nav_w_max, p->nav_w_max);
    if (fabsf(res) < NAV_TURN_NOREV_RAD && w * n->dir < 0.0f) { w = 0.0f; }   /* backlash is not worth a reversal */

    const bool in_tol = fabsf(res) < p->nav_turn_tol;
    n->t_ok_s = (in_tol && fabsf(w_meas) < NAV_TURN_W_STILL) ? n->t_ok_s + dt : 0.0f;
    nav_cmd_t c = { .type = NAV_VEL };
    c.done = n->t_ok_s >= NAV_TURN_T_STILL_S || n->t_s > (n->prof_ok ? n->prof.T : 0.0f) + NAV_TURN_T_GIVEUP_S;
    c.w = c.done ? 0.0f : w;
    c.aligned = in_tol;
    c.s_rem = res;
    n->dbg = (nav_dbg_t){ 0.0f, 0.0f, 0.0f, res, 0.0f, c.w, n->dbg.miss_m };
    return c;
}

const nav_dbg_t *nav_debug(const nav_t *n) { return &n->dbg; }
