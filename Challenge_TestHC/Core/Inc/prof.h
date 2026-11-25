/* prof.h — trapezoidal profiles and rate limiter (spec §7.7). Pure. Units are generic (m or rad). */
#ifndef PROF_H
#define PROF_H

#include "units.h"

typedef struct {
    float  D;          /* signed distance                                  */
    float  V;          /* cruise speed actually used (> 0)                 */
    float  v0;         /* start speed, 0 for prof_plan                     */
    float  ta, td;     /* ramp times                                       */
    float  T;          /* total duration                                   */
    float  t1, t2;     /* end of acceleration, start of deceleration       */
    int8_t dir;        /* sign of D                                        */
} prof_t;

/* ST_RANGE for V <= 0, ta/td < 0.05 s, D == 0 or |D| > 20 (REQ-PRF-02). Falls back to a
 * triangle when |D| < V (ta + td) / 2. Any context, pure. */
st_t  prof_plan(prof_t *p, float dist, float v_nom, float t_acc_s, float t_dec_s);
/* D > 0, start at v0 >= 0, accelerate at a_acc to v_nom (or the triangle peak), optional cruise,
 * stop at D with a_dec (REQ-NAV-02). If v0^2/(2 a_dec) > D it cannot stop in D: it brakes with a_dec and
 * p->D becomes the real stop distance. ST_RANGE for D <= 0 or > 20, v0 < 0, v_nom/a <= 0. */
st_t  prof_plan_v0(prof_t *p, float dist, float v0, float v_nom, float a_acc, float a_dec);
/* Signed x, v, a at time t. t >= T returns (D, 0, 0) (REQ-PRF-01). */
void  prof_eval(const prof_t *p, float t_s, float *x, float *v, float *a);

typedef struct { float y; } ramp_t;
/* Moves r->y towards target. rate_up applies while |y| grows or y is 0, rate_down while it
 * shrinks (units/s). Returns the new y. */
float ramp_step(ramp_t *r, float target, float rate_up, float rate_down, float dt);

#endif
