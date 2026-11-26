/* fwd.c — spec §7.16 REQ-FWD-01: median of 3, plausibility gate, stopping distance, reflex with
 * hysteresis, speed governor, staleness. */
#include "fwd.h"

#include <string.h>

#include "par.h"

#define JUMP_M      0.3f      /* plausibility margin over |v|·Δt (crosstalk) */
#define CLEAR_M     0.1f      /* reflex clears above d_stop + 0.1 m ...       */
#define CLEAR_US    500000U   /* ... for 0.5 s                                */

void fwd_init(fwd_t *f)
{
    memset(f, 0, sizeof *f);
    f->v_limit = INFINITY;
}

void fwd_on_meas(fwd_t *f, uint8_t id, const vl_meas_t *m, uint8_t zone)
{
    (void)zone;
    if (id != VL_F1 && id != VL_F2) { return; }
    fwd_sensor_t *s = &f->s[id - VL_F1];
    float r;
    if (m->status == VL_ST_VALID)            { r = m->range_m; }
    else if (m->status == VL_ST_SIGNAL_FAIL) { r = INFINITY; }      /* nothing in range: free */
    else                                     { return; }             /* unknown */
    /* Compared with the previous raw sample, not the last accepted one: a real obstacle that
     * appears suddenly is accepted from its second sample on. */
    const bool jump = s->seen && isfinite(r) && isfinite(s->last) &&
        fabsf(r - s->last) > fabsf(f->v_mps) * (float)(uint32_t)(m->t_us - s->t_last_us) * 1.0e-6f + JUMP_M;
    s->last = r;
    s->t_last_us = m->t_us;
    s->seen = true;
    if (jump) { return; }                                            /* implausible (crosstalk) */
    s->r[s->i] = r;
    s->i = (uint8_t)((s->i + 1U) % 3U);
    if (s->n < 3U) { s->n++; }
}

static float med(const fwd_sensor_t *s)
{
    if (s->n == 0U) { return NAN; }
    if (s->n == 1U) { return s->r[0]; }
    if (s->n == 2U) { return fminf(s->r[0], s->r[1]); }       /* the nearer one: safe side */
    const float a = s->r[0], b = s->r[1], c = s->r[2];
    return fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
}

float fwd_range(const fwd_t *f, uint8_t k) { return med(&f->s[k]); }

void fwd_step(fwd_t *f, float v_mps, uint32_t now_us)
{
    const par_t *q = par();
    f->v_mps = v_mps;
    const float v = fabsf(v_mps);
    f->d_stop = v * q->fwd_t_lat + v * v / (2.0f * q->fwd_a_brake) + q->fwd_margin;

    const uint32_t to_us = (uint32_t)(q->vl_timeout * 1.0e6f);
    float rmin = INFINITY;
    bool all_clear = true;
    uint8_t n_bad = 0U;
    for (uint8_t k = 0U; k < 2U; k++) {
        const fwd_sensor_t *s = &f->s[k];
        const bool bad = !s->seen || (uint32_t)(now_us - s->t_last_us) > to_us;
        const float r = med(s);
        if (bad) { n_bad++; all_clear = false; continue; }
        rmin = fminf(rmin, r);
        if (!(r > f->d_stop + CLEAR_M)) { all_clear = false; }
    }
    f->stale = (n_bad == 2U);

    if (v_mps > 0.0f && rmin < f->d_stop) { f->reflex = true; f->clearing = false; }
    if (f->reflex) {
        if (!all_clear) { f->clearing = false; }
        else if (!f->clearing) { f->clearing = true; f->clear_since_us = now_us; }
        else if ((uint32_t)(now_us - f->clear_since_us) >= CLEAR_US) { f->reflex = false; f->clearing = false; }
    }
    f->v_limit = (rmin < q->fwd_slow_range) ? q->fwd_v_slow : INFINITY;
}

bool  fwd_reflex(const fwd_t *f)  { return f->reflex; }
float fwd_v_limit(const fwd_t *f) { return f->v_limit; }
bool  fwd_stale(const fwd_t *f)   { return f->stale; }
