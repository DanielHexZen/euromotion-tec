/* prof.c — spec §7.7 (REQ-PRF-01/02). */
#include "prof.h"

#define PROF_T_RAMP_MIN_S 0.05f
#define PROF_D_MAX        20.0f

st_t prof_plan(prof_t *p, float dist, float v_nom, float t_acc_s, float t_dec_s)
{
    const float D = fabsf(dist);
    if (!(v_nom > 0.0f) || !(t_acc_s >= PROF_T_RAMP_MIN_S) || !(t_dec_s >= PROF_T_RAMP_MIN_S)
        || !(D > 0.0f) || D > PROF_D_MAX) {
        return ST_RANGE;
    }
    p->D   = dist;
    p->dir = (dist > 0.0f) ? 1 : -1;
    p->ta  = t_acc_s;
    p->td  = t_dec_s;
    const float ramps = 0.5f * (t_acc_s + t_dec_s);
    if (D >= v_nom * ramps) {
        p->V = v_nom;
        p->T = D / v_nom + ramps;
    } else {
        p->V = D / ramps;               /* = 2 D / (ta + td) */
        p->T = t_acc_s + t_dec_s;
    }
    p->t1 = p->ta;
    p->t2 = p->T - p->td;
    return ST_OK;
}

void prof_eval(const prof_t *p, float t_s, float *x, float *v, float *a)
{
    const float D = fabsf(p->D);
    float xm, vm, am;
    if (t_s < p->t1) {
        const float t = (t_s > 0.0f) ? t_s : 0.0f;
        am = p->V / p->ta;
        vm = am * t;
        xm = 0.5f * am * t * t;
    } else if (t_s < p->t2) {
        am = 0.0f;
        vm = p->V;
        xm = 0.5f * p->V * p->ta + p->V * (t_s - p->t1);
    } else if (t_s < p->T) {
        const float tau = p->T - t_s;
        am = -p->V / p->td;
        vm = p->V * tau / p->td;
        xm = D - 0.5f * p->V * tau * tau / p->td;
    } else {
        am = 0.0f;
        vm = 0.0f;
        xm = D;
    }
    const float s = (float)p->dir;
    *x = (t_s >= p->T) ? p->D : s * xm;
    *v = s * vm;
    *a = s * am;
}

float ramp_step(ramp_t *r, float target, float rate_up, float rate_down, float dt)
{
    const float d    = target - r->y;
    const float rate = (r->y * d >= 0.0f) ? rate_up : rate_down;
    r->y += clampf(d, -rate * dt, rate * dt);
    return r->y;
}
