/* t_ekf.h — shared helpers of the t_ekf_* tests: a 50 Hz stepper and a seeded Gaussian. */
#ifndef T_EKF_H
#define T_EKF_H

#include <stdint.h>

#include "ekf.h"
#include "par.h"
#include "test.h"

typedef struct { ekf_t e; uint32_t t_us; uint32_t rng; } ekfsim_t;

static inline float gauss(uint32_t *s)           /* sum of 12 uniforms: mean 0, sigma 1 */
{
    float a = -6.0f;
    for (int i = 0; i < 12; i++) {
        *s = *s * 1664525U + 1013904223U;
        a += (float)(*s >> 8) / 16777216.0f;
    }
    return a;
}

static inline void sim_init(ekfsim_t *s)
{
    (void)par_init();
    ekf_init(&s->e);
    s->t_us = 0U;
    s->rng = 12345U;
}

/* One ekf_predict. dt in whole microseconds, idle = drive layer reports no motion commanded. */
static inline void sim_step(ekfsim_t *s, uint32_t dt_us, float dl, float dr, float dpsi, bool idle)
{
    s->t_us += dt_us;
    const ekf_odo_t u = { dl, dr, dpsi, (float)dt_us * 1.0e-6f, s->t_us, idle };
    ekf_predict(&s->e, &u);
}

#endif
