#include <string.h>

#include "drv.h"
#include "par.h"
#include "sim_plant.h"
#include "test.h"

#define TICK_US 20000U

typedef struct {
    drv_t d;
    sim_plant_t plant;
    uint32_t k;
    drv_out_t out;
    int32_t cnt_prev[2];
    bool reversed;
    int n_dir[2];                  /* expected travel direction of each wheel */
    fault_t fault;
} rig_t;

static void rig_init(rig_t *r, double mis_l, double mis_r)
{
    memset(r, 0, sizeof *r);
    (void)par_init();
    drv_init(&r->d);
    sim_init(&r->plant, mis_l, mis_r);
    r->out.enable = false;
}

static void rig_tick(rig_t *r)
{
    const drv_meas_t m = { .cnt = { sim_counts(&r->plant, 0), sim_counts(&r->plant, 1) },
                           .t_us = r->k * TICK_US, .u_bat_v = 0.0f };
    for (int i = 0; i < 2; i++) {
        if (r->n_dir[i] != 0 && (m.cnt[i] - r->cnt_prev[i]) * r->n_dir[i] < 0) { r->reversed = true; }
        r->cnt_prev[i] = m.cnt[i];
    }
    drv_step(&r->d, &m, &r->out);
    if (r->out.fault != F_NONE && r->fault == F_NONE) { r->fault = r->out.fault; }
    sim_advance(&r->plant, r->out.duty_d, r->out.enable, (double)CTRL_DT_S);
    r->k++;
}

/* Runs until an event or 'limit_s'. Returns the event. */
static drv_event_t rig_run(rig_t *r, double limit_s)
{
    for (uint32_t n = 0; n < (uint32_t)(limit_s / (double)CTRL_DT_S); n++) {
        rig_tick(r);
        if (r->out.event != DRV_EV_NONE || r->out.fault != F_NONE) { return r->out.event; }
    }
    return DRV_EV_NONE;
}

static void straight(float dist_m)
{
    rig_t r;
    rig_init(&r, 1.10, 0.90);                                   /* +-10 % L/R mismatch */
    rig_tick(&r);                                               /* first sample */
    CHECK(drv_move(&r.d, dist_m, par()->drv_v_nom, par()->drv_t_acc, par()->drv_t_dec) == ST_OK);
    r.n_dir[0] = r.n_dir[1] = (dist_m > 0.0f) ? 1 : -1;
    const drv_event_t ev = rig_run(&r, 90.0);
    CHECK(ev == DRV_EV_DONE);
    const drv_status_t s = drv_status(&r.d);
    const double mpc = (double)par()->odo_m_per_cnt;
    for (int i = 0; i < 2; i++) {
        const double enc = (double)sim_counts(&r.plant, i) * mpc;     /* encoder frame */
        CHECK_NEAR(enc, (double)dist_m, 0.002);
        CHECK_NEAR(s.d_m[i], (double)dist_m, 0.002);
    }
    CHECK(!r.reversed);
    CHECK(s.sync_max_m < 0.003f);
    CHECK(s.mode == DRV_HOLD);
    CHECK(r.fault == F_NONE);
    printf("    D%+.0f: L=%.2f R=%.2f mm sync_max=%.2f mm t=%.2f s\n", (double)dist_m * 1000.0,
           (double)s.d_m[0] * 1000.0, (double)s.d_m[1] * 1000.0, (double)s.sync_max_m * 1000.0,
           (double)s.t_s);
}

static void turn_in_place(void)
{
    rig_t r;
    rig_init(&r, 1.10, 0.90);
    rig_tick(&r);
    const float th = PI_F * 0.5f;
    CHECK(drv_turn_enc(&r.d, th, 0.8f, 0.4f) == ST_OK);
    r.n_dir[0] = -1; r.n_dir[1] = 1;
    CHECK(rig_run(&r, 30.0) == DRV_EV_DONE);
    const double want = 0.5 * (double)par()->odo_wheelbase * (double)th;
    CHECK_NEAR((double)sim_counts(&r.plant, 0) * r.plant.mpc, -want, 0.002);
    CHECK_NEAR((double)sim_counts(&r.plant, 1) * r.plant.mpc, want, 0.002);
    CHECK(!r.reversed);
    CHECK(drv_status(&r.d).sync_max_m < 0.003f);
}

static void stop_midway(void)
{
    rig_t r;
    rig_init(&r, 1.10, 0.90);
    rig_tick(&r);
    CHECK(drv_move(&r.d, 3.0f, 0.3f, 1.0f, 1.0f) == ST_OK);
    r.n_dir[0] = r.n_dir[1] = 1;
    for (int i = 0; i < 100; i++) { rig_tick(&r); }             /* 2 s: cruising */
    const double v0 = r.plant.w[0].v;
    CHECK(v0 > 0.2);
    drv_stop(&r.d);
    int n = 0;
    while (drv_status(&r.d).mode == DRV_MOVE && n < 200) { rig_tick(&r); n++; }
    CHECK(drv_status(&r.d).mode == DRV_HOLD);
    CHECK(n * 0.02 < v0 / 0.5 + 0.5);                           /* ~ v / a_stop */
    CHECK(!r.reversed);
    for (int i = 0; i < 50; i++) { rig_tick(&r); }
    CHECK(fabs(r.plant.w[0].v) < 1e-3 && fabs(r.plant.w[1].v) < 1e-3);
    CHECK(drv_move(&r.d, 0.1f, 0.3f, 1.0f, 1.0f) == ST_OK);     /* usable again */
    CHECK(drv_move(&r.d, 0.1f, 0.3f, 1.0f, 1.0f) == ST_STATE);  /* but busy */
}

static void open_loop(void)
{
    rig_t r;
    rig_init(&r, 1.0, 1.0);
    rig_tick(&r);
    CHECK(drv_open(&r.d, 1.0f, -1.0f) == ST_OK);
    rig_tick(&r);
    CHECK_NEAR(r.out.duty_d[0], 2.0 * 0.02, 1e-6);              /* mot_slew_ol = 2 /s */
    CHECK_NEAR(r.out.duty_d[1], -2.0 * 0.02, 1e-6);
    for (int i = 0; i < 60; i++) { rig_tick(&r); }
    CHECK(r.out.duty_d[0] == 1.0f && r.out.duty_d[1] == -1.0f);
    CHECK(drv_open(&r.d, -1.0f, 1.0f) == ST_OK);                /* reverse: through zero, 50 ms */
    int zero_ticks = 0;
    float prev = r.out.duty_d[0];
    bool crossed = false;
    for (int i = 0; i < 120 && !crossed; i++) {
        rig_tick(&r);
        CHECK(r.out.duty_d[0] * prev >= 0.0f || prev == 0.0f);   /* never jumps across zero */
        if (r.out.duty_d[0] == 0.0f) { zero_ticks++; }
        if (r.out.duty_d[0] < 0.0f) { crossed = true; }
        prev = r.out.duty_d[0];
    }
    CHECK(crossed);
    CHECK(zero_ticks >= 3);                                     /* 0.05 s / 0.02 s, rounded up */
    drv_stop(&r.d);
    rig_tick(&r);
    CHECK(r.out.duty_d[0] == 0.0f && r.out.enable && drv_status(&r.d).mode == DRV_HOLD);
    drv_disable(&r.d);
    rig_tick(&r);
    CHECK(!r.out.enable);
}

static void faults(void)
{
    rig_t r;
    rig_init(&r, 1.0, 1.0);                                     /* wrong sign on the left motor */
    r.plant.w[0].sign = -1.0;
    rig_tick(&r);
    CHECK(drv_move(&r.d, 1.0f, 0.3f, 1.0f, 1.0f) == ST_OK);
    (void)rig_run(&r, 10.0);
    CHECK(r.fault == F_RUNAWAY_L);
    CHECK(drv_status(&r.d).mode == DRV_DISABLED);

    rig_init(&r, 1.0, 1.0);                                     /* right wheel blocked */
    r.plant.w[1].blocked = true;
    rig_tick(&r);
    CHECK(drv_move(&r.d, 1.0f, 0.3f, 1.0f, 1.0f) == ST_OK);
    (void)rig_run(&r, 10.0);
    CHECK(r.fault == F_STALL_R || r.fault == F_TRACK_R);
    CHECK(drv_status(&r.d).mode == DRV_HOLD);
}

void t_drv_move(void)
{
    REQ("REQ-DRV-01/02: D300, D1000, D5000, D-500 end within 2 mm, never reverse, sync < 3 mm");
    straight(0.3f);
    straight(1.0f);
    straight(5.0f);
    straight(-0.5f);
    REQ("REQ-DRV-01/02: turn in place by encoder angle");
    turn_in_place();
    REQ("REQ-DRV-07: drv_stop ramps to rest at drv_a_stop, then HOLD");
    stop_midway();
    REQ("REQ-MOT-06 (in drv): open-loop slew limit and 50 ms zero on a sign change");
    open_loop();
    REQ("REQ-DRV-06: runaway disables, stall/track holds, fault code reported");
    faults();
}
