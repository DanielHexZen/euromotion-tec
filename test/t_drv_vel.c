#include "drv.h"
#include "par.h"
#include "t_rig.h"
#include "test.h"

static void run(rig_t *r, int ticks, float v, float w)
{
    for (int i = 0; i < ticks; i++) {
        drv_vel(&r->d, v, w);
        rig_tick(r);
    }
}

static float wheel_w(const rig_t *r) { return (r->d.v_ref[1] - r->d.v_ref[0]) / par()->odo_wheelbase; }

static void limits(void)
{
    rig_t r;
    rig_init(&r, 1.0, 1.0, 0.0, 0.0, 0.0);
    float v_prev = 0.0f, dv_max = 0.0f;
    for (int i = 0; i < 400; i++) {                             /* 8 s at a command far above every limit */
        drv_vel(&r.d, 2.0f, 0.0f);
        rig_tick(&r);
        const float v = 0.5f * (r.d.v_ref[0] + r.d.v_ref[1]);
        dv_max = fmaxf(dv_max, fabsf(v - v_prev) / CTRL_DT_S);
        v_prev = v;
    }
    CHECK_NEAR(v_prev, par()->drv_v_max, 1e-4);                 /* |v| <= drv_v_max */
    CHECK(dv_max <= par()->drv_a_max + 1e-3f);                  /* rate limit */

    rig_init(&r, 1.0, 1.0, 0.0, 0.0, 0.0);
    float w_prev = 0.0f, dw_max = 0.0f;
    for (int i = 0; i < 100; i++) {
        drv_vel(&r.d, 0.0f, 10.0f);
        rig_tick(&r);
        dw_max = fmaxf(dw_max, fabsf(wheel_w(&r) - w_prev) / CTRL_DT_S);
        w_prev = wheel_w(&r);
    }
    CHECK_NEAR(w_prev, par()->drv_w_max, 1e-3);                 /* |w| <= drv_w_max */
    CHECK(dw_max <= par()->drv_alpha_max + 1e-2f);

    rig_init(&r, 1.0, 1.0, 0.0, 0.0, 0.0);                      /* reverse is limited the same way */
    run(&r, 400, -2.0f, 0.0f);
    CHECK_NEAR(0.5f * (r.d.v_ref[0] + r.d.v_ref[1]), -par()->drv_v_max, 1e-4);
    CHECK(r.fault == F_NONE);
}

static void wheel_saturation(void)
{
    rig_t r;
    rig_init(&r, 1.0, 1.0, 0.0, 0.0, 0.0);
    run(&r, 300, 0.45f, 1.0f);                                  /* 0.45 + 1.0 * b/2 > 0.45 */
    const float hb = 0.5f * par()->odo_wheelbase;
    CHECK(fabsf(r.d.v_ref[0]) <= par()->drv_v_whl_max + 1e-5f);
    CHECK(fabsf(r.d.v_ref[1]) <= par()->drv_v_whl_max + 1e-5f);
    CHECK_NEAR(wheel_w(&r), 1.0, 1e-4);                         /* omega kept */
    CHECK_NEAR(0.5f * (r.d.v_ref[0] + r.d.v_ref[1]), par()->drv_v_whl_max - hb, 1e-4);   /* v cut */
    CHECK_NEAR(r.w_true, 1.0, 0.05);                            /* the plant follows: curvature kept */
}

static void antiwindup(void)
{
    (void)par_init();
    drv_t d;
    drv_init(&d);
    drv_out_t out;
    const drv_meas_t m0 = { .cnt = { 0, 0 }, .t_us = 0U, .u_bat_v = 0.0f };
    drv_step(&d, &m0, &out);
    float worst = 0.0f;
    for (uint32_t k = 1U; k <= 42U; k++) {                      /* wheels blocked: counts never change */
        const drv_meas_t m = { .cnt = { 0, 0 }, .t_us = k * RIG_TICK_US, .u_bat_v = 0.0f };
        drv_vel(&d, 0.3f, 0.0f);
        drv_step(&d, &m, &out);
        worst = fmaxf(worst, fmaxf(d.x_ref[0] - d.x_m[0], d.x_ref[1] - d.x_m[1]));
        CHECK(out.fault == F_NONE);
    }
    CHECK_NEAR(worst, par()->drv_e_ref_max, 1e-6);              /* reached and held at the clamp */
}

static void sync_and_stop(void)
{
    rig_t r;
    rig_init(&r, 1.10, 0.90, 0.0, 0.0, 0.0);                    /* +-10 % L/R mismatch */
    r.n_dir[0] = r.n_dir[1] = 1;
    run(&r, 500, 0.3f, 0.0f);
    CHECK(drv_status(&r.d).sync_max_m < 0.003f);                /* REQ-DRV-02 in VEL */
    CHECK(fabsf(r.d.x_m[0] - r.d.x_m[1]) < 0.003f);              /* encoder frame, as t_drv_move */

    drv_stop(&r.d);                                             /* REQ-DRV-07 */
    int n = 0;
    while (drv_status(&r.d).mode == DRV_VEL && n < 200) { rig_tick(&r); n++; }
    CHECK(drv_status(&r.d).mode == DRV_HOLD);
    CHECK((double)n * 0.02 < 0.3 / (double)par()->drv_a_stop + 0.5);
    CHECK(!r.reversed);
    CHECK(r.fault == F_NONE);
    run(&r, 10, 0.0f, 0.0f);                                    /* a new command resumes */
    CHECK(drv_status(&r.d).mode == DRV_VEL);
}

static void bumpless(void)
{
    rig_t r;
    rig_init(&r, 1.10, 0.90, 0.0, 0.0, 0.0);
    CHECK(drv_move(&r.d, 5.0f, 0.3f, 1.0f, 1.0f) == ST_OK);
    for (int i = 0; i < 150; i++) { rig_tick(&r); }             /* cruising, integrators loaded */
    const float i_l = r.d.whl[0].integ_d;
    CHECK(fabsf(i_l) > 1e-4f);
    drv_vel(&r.d, 0.3f, 0.0f);                                  /* MOVE -> VEL: references from the measurement */
    CHECK(r.d.whl[0].integ_d == i_l);                           /* REQ-DRV-05: integrators kept */
    CHECK_NEAR(r.d.vel_v.y, 0.3, 0.01);
    rig_tick(&r);
    CHECK_NEAR(r.d.x_ref[0] - r.d.x_m[0], 0.3 * (double)CTRL_DT_S, 0.002);   /* starts at x_m, one step of motion */
    CHECK(fabsf(r.d.whl[0].integ_d - i_l) < 0.02f);
    CHECK(r.out.duty_d[0] > 0.1f && r.out.duty_d[0] < 0.6f);    /* no kick */

    drv_stop(&r.d);                                             /* VEL -> MOVE keeps them too */
    rig_tick(&r);
    drv_vel(&r.d, 0.3f, 0.0f);
    run(&r, 50, 0.3f, 0.0f);
    const float i_v = r.d.whl[1].integ_d;
    CHECK(drv_move(&r.d, 1.0f, 0.3f, 1.0f, 1.0f) == ST_OK);
    CHECK(r.d.whl[1].integ_d == i_v);

    drv_stop(&r.d);
    while (drv_status(&r.d).mode == DRV_MOVE) { rig_tick(&r); }
    drv_vel(&r.d, 0.3f, 0.0f);
    run(&r, 50, 0.3f, 0.0f);
    CHECK(drv_open(&r.d, 0.2f, 0.2f) == ST_OK);                 /* into OPEN: reset */
    rig_tick(&r);
    CHECK(r.d.whl[0].integ_d == 0.0f && r.d.whl[1].integ_d == 0.0f);

    rig_init(&r, 1.0, 1.0, 0.0, 0.0, 0.0);                      /* HOLD -> VEL starts from rest, no jump */
    drv_vel(&r.d, 0.3f, 0.0f);
    rig_tick(&r);
    CHECK_NEAR(r.d.x_ref[0] - r.d.x_m[0], 0.0, 0.001);
    CHECK(fabsf(r.d.v_ref[0]) <= par()->drv_a_max * CTRL_DT_S + 1e-6f);
}

static void move_from(float dist, float v0, double expect, double tol, bool exact)
{
    rig_t r;
    rig_init(&r, 1.10, 0.90, 0.0, 0.0, 0.0);
    r.n_dir[0] = r.n_dir[1] = 1;
    run(&r, 400, v0, 0.0f);
    const float x0 = rig_s_enc(&r);
    CHECK(drv_move_from(&r.d, dist, v0, v0, 0.3f, 0.3f) == ST_OK);
    drv_event_t ev = DRV_EV_NONE;
    for (int i = 0; i < 1000 && ev == DRV_EV_NONE && r.fault == F_NONE; i++) {
        rig_tick(&r);
        ev = r.out.event;
    }
    CHECK(ev == DRV_EV_DONE);
    CHECK(r.fault == F_NONE && !r.reversed);
    const double got = (double)(rig_s_enc(&r) - x0);
    if (exact) { CHECK_NEAR(got, expect, tol); } else { CHECK(got >= (double)dist - 0.002 && got < expect + tol); }
    CHECK(drv_status(&r.d).mode == DRV_HOLD);
}

void t_drv_vel(void)
{
    (void)par_init();
    REQ("REQ-DRV-03: |v|, |w|, rate limits (a_max, alpha_max), reverse symmetric");
    limits();
    REQ("REQ-DRV-03: wheel saturation cuts v and keeps omega (curvature preserved)");
    wheel_saturation();
    REQ("REQ-DRV-04: x_ref - x clamped to drv_e_ref_max on a blocked wheel");
    antiwindup();
    REQ("REQ-DRV-02/07: L/R sync in VEL, drv_stop ramps to HOLD without reversing");
    sync_and_stop();
    REQ("REQ-DRV-05: bumpless MOVE<->VEL, integrators kept; into OPEN reset");
    bumpless();
    REQ("REQ-DRV-01/05 (MOVE_FROM): stop at D from 0.3 m/s in the encoder frame, never reverse");
    move_from(0.30f, 0.3f, 0.30, 0.002, true);
    move_from(0.50f, 0.2f, 0.50, 0.002, true);
    REQ("REQ-NAV-02: D too short to stop: brakes at drv_a_stop, overshoots, never reverses");
    move_from(0.05f, 0.3f, 0.09, 0.01, false);
}
