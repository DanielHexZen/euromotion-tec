/* fwd.h — forward reflex and speed governor (spec §7.16, REQ-FWD-01). Pure. The free-space
 * profile (REQ-FWD-02, phase 6) is not here yet. */
#ifndef FWD_H
#define FWD_H

#include "vl.h"

typedef struct {
    float    r[3];            /* last accepted ranges (m), INFINITY = free (signal fail) */
    uint8_t  n, i;
    float    last;            /* previous raw range, for the plausibility check        */
    uint32_t t_last_us;       /* time of the previous raw sample (also staleness)      */
    bool     seen;
} fwd_sensor_t;

typedef struct {
    fwd_sensor_t s[2];        /* F1, F2 */
    float    v_mps;           /* speed of the last fwd_step, for the plausibility check */
    bool     reflex;
    uint32_t clear_since_us;  /* both ranges above d_stop + 0.1 m since              */
    bool     clearing;
    float    d_stop, v_limit;
    bool     stale;
} fwd_t;

void  fwd_init(fwd_t *f);
void  fwd_on_meas(fwd_t *f, uint8_t id, const vl_meas_t *m, uint8_t zone);   /* zone unused until phase 6 */
void  fwd_step(fwd_t *f, float v_mps, uint32_t now_us);
bool  fwd_reflex(const fwd_t *f);
float fwd_v_limit(const fwd_t *f);            /* +inf if no limit                          */
float fwd_range(const fwd_t *f, uint8_t k);   /* filtered range of F1 (0) / F2 (1), m; NAN unknown */
bool  fwd_stale(const fwd_t *f);              /* both sensors unknown or stale -> F_FWD_STALE in AUTO */

#endif
