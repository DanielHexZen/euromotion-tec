/* att.c — spec §7.12 (REQ-ATT-01). */
#include "att.h"

#include <string.h>

#include "par.h"

#define ATT_INIT_S 0.5f

void att_init(att_t *a) { memset(a, 0, sizeof *a); }

float att_roll(const att_t *a)  { return a->phi; }
float att_pitch(const att_t *a) { return a->theta; }

void att_update(att_t *a, const float w[3], const float acc[3], float dt_s)
{
    const par_t *p = par();
    const float phi_a = atan2f(acc[1], acc[2]);
    const float th_a  = atan2f(-acc[0], sqrtf(acc[1] * acc[1] + acc[2] * acc[2]));

    if (a->t_init_s < ATT_INIT_S) {
        a->sum_phi += phi_a;
        a->sum_theta += th_a;
        a->n_init++;
        a->t_init_s += dt_s;
        a->phi   = a->sum_phi / (float)a->n_init;
        a->theta = a->sum_theta / (float)a->n_init;
        return;
    }

    const float sp = sinf(a->phi), cp = cosf(a->phi), tt = tanf(a->theta);
    const float phi_dot   = w[0] + (w[1] * sp + w[2] * cp) * tt;
    const float theta_dot = w[1] * cp - w[2] * sp;
    a->phi   += phi_dot * dt_s;
    a->theta += theta_dot * dt_s;

    const float an = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (fabsf(an - G_MPS2) < p->att_acc_gate * G_MPS2) {
        const float k = dt_s / p->att_tau;
        a->phi   += k * (phi_a - a->phi);
        a->theta += k * (th_a - a->theta);
    }
}
