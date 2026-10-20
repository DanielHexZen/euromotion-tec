#include "sim_plant.h"

#include <math.h>

#include "par.h"

void sim_init(sim_plant_t *s, double kv_mis_l, double kv_mis_r)
{
    const par_t *p = par();
    s->w[0] = (sim_wheel_t){ .ks = (double)p->whl_ks_l, .ksb = (double)p->whl_ksb_l,
                             .kv = (double)p->whl_kv_l * kv_mis_l, .tau = 0.05, .sign = 1.0 };
    s->w[1] = (sim_wheel_t){ .ks = (double)p->whl_ks_r, .ksb = (double)p->whl_ksb_r,
                             .kv = (double)p->whl_kv_r * kv_mis_r, .tau = 0.05, .sign = 1.0 };
    s->mpc = (double)p->odo_m_per_cnt;
}

void sim_advance(sim_plant_t *s, const float duty[2], bool enable, double dt_s)
{
    const double h = 0.001;
    for (double t = 0.0; t < dt_s - 1e-9; t += h) {
        for (int i = 0; i < 2; i++) {
            sim_wheel_t *w = &s->w[i];
            const double u = enable ? (double)duty[i] * w->sign : 0.0;
            double v_ss = 0.0;
            if (fabs(u) > w->ks) { v_ss = copysign((fabs(u) - w->ks) / w->kv, u); }
            if (fabs(w->v) < 1e-4 && fabs(u) < w->ksb) { w->v = 0.0; v_ss = 0.0; }   /* breakaway */
            w->v += (v_ss - w->v) * h / w->tau;
            if (fabs(w->v) < 1e-4 && v_ss == 0.0) { w->v = 0.0; }
            if (w->blocked) { w->v = 0.0; }
            w->x += w->v * h;
        }
    }
}

int32_t sim_counts(const sim_plant_t *s, int side)
{
    return (int32_t)floor(s->w[side].x / s->mpc);
}
