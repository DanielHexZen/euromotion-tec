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
    p->v0  = 0.0f;
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

st_t prof_plan_v0(prof_t *p, float dist, float v0, float v_nom, float a_acc, float a_dec)
{
    if (!(dist > 0.0f) || dist > PROF_D_MAX || !(v0 >= 0.0f) || !(v_nom > 0.0f) || !(a_acc > 0.0f)
        || !(a_dec > 0.0f)) {
        return ST_RANGE;
    }
    p->dir = 1;
    p->v0  = v0;
    float V = fmaxf(v_nom, v0);                         /* never slows down to cruise */
    if (0.5f * v0 * v0 / a_dec >= dist) {               /* brake as early as possible */
        p->D  = 0.5f * v0 * v0 / a_dec;
        p->V  = v0;
        p->ta = 0.0f;
        p->td = v0 / a_dec;
        p->T  = p->td;
    } else {
        if ((V * V - v0 * v0) / (2.0f * a_acc) + V * V / (2.0f * a_dec) > dist) {   /* triangle */
            V = sqrtf((2.0f * dist + v0 * v0 / a_acc) / (1.0f / a_acc + 1.0f / a_dec));
        }
        const float da = (V * V - v0 * v0) / (2.0f * a_acc);
        const float dd = V * V / (2.0f * a_dec);
        p->D  = dist;
        p->V  = V;
        p->ta = fmaxf(V - v0, 0.0f) / a_acc;
        p->td = V / a_dec;
        p->T  = p->ta + fmaxf(dist - da - dd, 0.0f) / V + p->td;   /* max: float noise in the triangle */
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
        am = (p->V - p->v0) / p->ta;
        vm = p->v0 + am * t;
        xm = p->v0 * t + 0.5f * am * t * t;
    } else if (t_s < p->t2) {
        am = 0.0f;
        vm = p->V;
        xm = 0.5f * (p->V + p->v0) * p->ta + p->V * (t_s - p->t1);
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
