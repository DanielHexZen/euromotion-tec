/* whl.c — spec §7.8 (REQ-WHL-01..05). */
#include "whl.h"

#include <string.h>

#include "par.h"

#define WHL_FF_U_MIN_V 9.5f     /* 3S pack, empty..full: normalisation clamp */
#define WHL_FF_U_MAX_V 12.8f

void whl_init(whl_t *w, uint8_t side)
{
    memset(w, 0, sizeof *w);
    w->side = side;
}

void whl_reset(whl_t *w)
{
    w->integ_d   = 0.0f;
    w->rolling   = false;
    w->n_stall   = 0U;
    w->n_runaway = 0U;
    w->n_track   = 0U;
}

float whl_dt_guard(float dt_s, bool *glitch)
{
    const bool bad = !(dt_s >= 0.5f * CTRL_DT_S && dt_s <= 2.0f * CTRL_DT_S);
    if (glitch != NULL) { *glitch = bad; }
    return bad ? CTRL_DT_S : dt_s;
}

/* Counts consecutive ticks of a condition; true once it has held for n ticks. */
static bool persist(uint16_t *cnt, bool cond, uint16_t n)
{
    if (!cond) { *cnt = 0U; return false; }
    if (*cnt < UINT16_MAX) { (*cnt)++; }
    return *cnt >= n;
}

void whl_step(whl_t *w, const whl_in_t *in, whl_out_t *out)
{
    const par_t *p = par();
    const bool   left = (w->side == WHL_L);
    const float  ks  = left ? p->whl_ks_l  : p->whl_ks_r;
    const float  kv  = left ? p->whl_kv_l  : p->whl_kv_r;
    const float  ksb = left ? p->whl_ksb_l : p->whl_ksb_r;

    bool glitch;
    const float dt = whl_dt_guard(in->dt_s, &glitch);
    if (glitch) { w->dt_glitch++; }

    const float e_x   = in->pos_loop ? (in->x_ref_m - in->x_m) : 0.0f;
    const float v_cmd = in->v_ref_mps + clampf(p->whl_kp_x * e_x, -p->whl_v_corr_max, p->whl_v_corr_max)
                      + in->v_sync_mps;
    const float e_v   = v_cmd - in->v_mps;

    const float av = fabsf(in->v_mps);
    if (av > p->whl_v_moving) { w->rolling = true; }
    else if (av < p->whl_v_stopped) { w->rolling = false; }

    const bool cmd_zero = fabsf(v_cmd) <= p->whl_v_zero;
    float ff = 0.0f;
    if (!cmd_zero) {
        const float ks_eff = w->rolling ? ks : ksb;
        ff = signf(v_cmd) * (ks_eff + kv * fabsf(v_cmd)) + kv * p->whl_tau_m * in->a_ref_mps2;
        if (p->whl_ff_norm_en > 0.5f && in->u_bat_v >= BAT_ABSENT_V) {
            ff *= p->whl_ff_u_ref / clampf(in->u_bat_v, WHL_FF_U_MIN_V, WHL_FF_U_MAX_V);
        }
    }
    /* Forget the integrator before using it, as the old Wheel_Control did (the spec lists the
     * reset last; the difference is one control tick of stale integrator at standstill). */
    if (cmd_zero && fabsf(in->v_ref_mps) <= p->whl_v_zero) { w->integ_d = 0.0f; }

    const float u = ff + p->whl_kp_v * e_v + w->integ_d;
    if (fabsf(u) < 1.0f) {
        w->integ_d = clampf(w->integ_d + p->whl_ki_v * e_v * dt, -p->whl_i_max, p->whl_i_max);
    }
    const float duty = clampf(u, -1.0f, 1.0f);

    const uint16_t n = (uint16_t)(p->whl_fault_time / CTRL_DT_S + 0.5f);
    const bool stall = persist(&w->n_stall, fabsf(duty) >= WHL_STALL_DUTY && av < WHL_STALL_V_MPS, n);
    const bool runaway = persist(&w->n_runaway,
        (duty > WHL_RUN_DUTY && in->v_mps < -WHL_RUN_V_MPS) || (duty < -WHL_RUN_DUTY && in->v_mps > WHL_RUN_V_MPS), n);
    const bool track = persist(&w->n_track, in->pos_loop && fabsf(e_x) > p->whl_track_max, n);

    out->duty_d    = duty;
    out->v_cmd_mps = v_cmd;
    out->e_x_m     = e_x;
    out->e_v_mps   = e_v;
    out->integ_d   = w->integ_d;
    out->ff_d      = ff;
    out->flags     = (uint8_t)((w->rolling ? WHL_ROLLING : 0U) | (fabsf(u) >= 1.0f ? WHL_SAT : 0U)
                   | (stall ? WHL_STALL : 0U) | (runaway ? WHL_RUNAWAY : 0U) | (track ? WHL_TRACK : 0U));
}
