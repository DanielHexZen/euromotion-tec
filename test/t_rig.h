/* t_rig.h — drv + sim_plant + unicycle pose, shared by the VEL and nav tests (spec §14.1).
 * The pose integrates the true wheel travel of the plant with b = odo_wheelbase. */
#ifndef T_RIG_H
#define T_RIG_H

#include <string.h>

#include "drv.h"
#include "ekf.h"
#include "par.h"
#include "sim_plant.h"

#define RIG_TICK_US 20000U

typedef struct {
    drv_t d;
    sim_plant_t plant;
    uint32_t k;
    drv_out_t out;
    double x, y, th, w_true;           /* true pose and yaw rate */
    double pl[2];
    bool reversed;
    int32_t cnt_prev[2];
    int n_dir[2];                      /* expected travel direction (0 = unchecked) */
    fault_t fault;
} rig_t;

static inline void rig_tick(rig_t *r)
{
    const drv_meas_t m = { .cnt = { sim_counts(&r->plant, 0), sim_counts(&r->plant, 1) },
                           .t_us = r->k * RIG_TICK_US, .u_bat_v = 0.0f };
    for (int i = 0; i < 2; i++) {
        if (r->n_dir[i] != 0 && (m.cnt[i] - r->cnt_prev[i]) * r->n_dir[i] < 0) { r->reversed = true; }
        r->cnt_prev[i] = m.cnt[i];
    }
    drv_step(&r->d, &m, &r->out);
    if (r->out.fault != F_NONE && r->fault == F_NONE) { r->fault = r->out.fault; }
    sim_advance(&r->plant, r->out.duty_d, r->out.enable, (double)CTRL_DT_S);
    const double dl = r->plant.w[0].x - r->pl[0], dr = r->plant.w[1].x - r->pl[1];
    const double dth = (dr - dl) / (double)par()->odo_wheelbase;
    r->x += 0.5 * (dl + dr) * cos(r->th + 0.5 * dth);
    r->y += 0.5 * (dl + dr) * sin(r->th + 0.5 * dth);
    r->th += dth;
    r->w_true = dth / (double)CTRL_DT_S;
    r->pl[0] = r->plant.w[0].x;
    r->pl[1] = r->plant.w[1].x;
    r->k++;
}

/* Needs par_init(). Pose starts at (x, y, th); the first tick gives drv its first sample. */
static inline void rig_init(rig_t *r, double mis_l, double mis_r, double x, double y, double th)
{
    memset(r, 0, sizeof *r);
    drv_init(&r->d);
    sim_init(&r->plant, mis_l, mis_r);
    r->x = x; r->y = y; r->th = th;
    rig_tick(r);
}

/* Ideal estimate: the true pose. */
static inline void rig_est(const rig_t *r, ekf_out_t *e)
{
    memset(e, 0, sizeof *e);
    e->x[0] = (float)r->x;
    e->x[1] = (float)r->y;
    e->x[2] = (float)wrap_pi((float)r->th);
    e->w_meas_radps = (float)r->w_true;
}

/* Mean wheel travel in the encoder frame, as app will feed msn. */
static inline float rig_s_enc(const rig_t *r) { return 0.5f * (r->d.x_m[0] + r->d.x_m[1]); }

#endif
