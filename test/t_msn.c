#include <string.h>

#include "msn.h"
#include "par.h"
#include "test.h"

#define DT_MS 20U

/* ---- harness: scripted msn_in_t, a world that applies the commands, and checks that run every tick ---- */
typedef struct {
    msn_t m;
    msn_in_t in;
    msn_out_t out;
    uint32_t evm, cmdm, stm;       /* events, commands and AUTO states seen since clr() */
    uint32_t n_ev[16];
    int t_wait;
} h_t;

static void h_init(h_t *h)
{
    memset(h, 0, sizeof *h);
    (void)par_init();
    par_set_stationary(true);
    msn_init(&h->m);
    h->in.u_bat_v = 12.0f;
    h->in.est.flags = EKF_STILL;
    h->in.est.P_diag[3] = 1.0e-8f;
    h->in.v_limit = INFINITY;
    h->in.feat_imu = h->in.imu_alive = h->in.feat_vl = h->in.sensors_fresh = true;
    h->in.t_ms = 1000U;
}

static void clr(h_t *h) { h->evm = h->cmdm = h->stm = 0U; memset(h->n_ev, 0, sizeof h->n_ev); }
static bool ev(const h_t *h, msn_ev_t e) { return ((h->evm >> (unsigned)e) & 1U) != 0U; }
static bool cmd(const h_t *h, msn_cmd_t c) { return ((h->cmdm >> (unsigned)c) & 1U) != 0U; }
static bool seen(const h_t *h, msn_state_t s) { return ((h->stm >> (unsigned)s) & 1U) != 0U; }

static void step(h_t *h)
{
    const msn_state_t s0 = msn_state(&h->m);
    h->in.t_ms += DT_MS;
    msn_step(&h->m, &h->in, &h->out);
    h->cmdm |= 1U << (unsigned)h->out.cmd;
    int first_a = -1, last_b = -1;
    for (uint8_t i = 0U; i < h->out.n_ev; i++) {
        h->evm |= 1U << (unsigned)h->out.ev[i].code;
        h->n_ev[h->out.ev[i].code]++;
        if (h->out.ev[i].code == MSN_EV_STATE) {
            if (first_a < 0) { first_a = h->out.ev[i].a; }
            last_b = h->out.ev[i].b;
        }
    }
    const msn_state_t s1 = msn_state(&h->m);
    for (uint8_t i = 0U; i < h->out.n_ev; i++) {
        if (h->out.ev[i].code == MSN_EV_STATE) { h->stm |= 1U << (unsigned)h->out.ev[i].b; }
    }
    if (s0 != s1) {                                              /* REQ-MSN-06 */
        CHECK(first_a == (int)s0);
        CHECK(last_b == (int)s1);
    }
}

static void steps(h_t *h, int n) { for (int i = 0; i < n; i++) { step(h); } }

/* ---- world: the vehicle does what the last command said ---- */
static void place_end(h_t *h, const seg_t *sg)
{
    h->in.est.x[0] = sg->g.line.x1;
    h->in.est.x[1] = sg->g.line.y1;
    h->in.est.x[2] = atan2f(sg->g.line.y1 - sg->g.line.y0, sg->g.line.x1 - sg->g.line.x0);
}

static void world(h_t *h)
{
    msn_in_t *in = &h->in;
    const float dt = 0.02f;
    const seg_t *sg = &h->m.course->seg[h->m.seg];
    in->drv_ev = DRV_EV_NONE;
    in->est.w_meas_radps = 0.0f;
    in->est.flags = EKF_STILL;
    switch (msn_state(&h->m)) {
    case MSN_AUTO_DRIVE:
        if (h->out.cmd == MC_VEL) {
            in->est.x[0] += h->out.v * cosf(in->est.x[2]) * dt;
            in->est.x[1] += h->out.v * sinf(in->est.x[2]) * dt;
            in->est.x[2] = wrap_pi(in->est.x[2] + h->out.w * dt);
            in->s_enc += h->out.v * dt;
            in->est.flags = 0U;
        }
        break;
    case MSN_AUTO_FINAL:                                         /* drv finishes 0.5 s after the command */
        in->est.flags = 0U;
        if (++h->t_wait >= 25) {
            h->t_wait = 0;
            place_end(h, sg);
            in->drv_ev = DRV_EV_DONE;
        }
        break;
    case MSN_AUTO_TURN:
        h->t_wait++;
        if (h->m.turn_enc) {
            in->est.flags = 0U;
            if (h->t_wait >= 25) {
                in->est.x[2] = sg->g.turn.theta;
                in->drv_ev = DRV_EV_DONE;
                h->t_wait = 0;
            }
        } else if (h->out.cmd == MC_VEL) {
            in->est.x[2] = wrap_pi(in->est.x[2] + h->out.w * dt);
            in->est.w_meas_radps = h->out.w;
            in->est.flags = 0U;
        }
        break;
    default:
        h->t_wait = 0;
        break;
    }
}

static bool run_until(h_t *h, msn_state_t s, int max_ticks)
{
    for (int i = 0; i < max_ticks; i++) {
        world(h);
        step(h);
        if (msn_state(&h->m) == s) { return true; }
    }
    return false;
}

static void steps_w(h_t *h, int n) { for (int i = 0; i < n; i++) { world(h); step(h); } }

static void to_auto(h_t *h)
{
    msn_request(&h->m, MSN_REQ_AUTO);
    step(h);
}

static void to_drive(h_t *h)
{
    to_auto(h);
    CHECK(run_until(h, MSN_AUTO_DRIVE, 400));
}

/* ---- tests ---- */
static void modes_and_stop(void)
{
    h_t h;
    h_init(&h);
    CHECK(msn_mode(&h.m) == MSN_IDLE && msn_state(&h.m) == MSN_AUTO_NONE);
    msn_request(&h.m, MSN_REQ_BENCH);                            /* IDLE -> BENCH: drv_hold */
    step(&h);
    CHECK(msn_mode(&h.m) == MSN_BENCH && h.out.cmd == MC_HOLD);
    msn_request(&h.m, MSN_REQ_STOP);                             /* any: STOP, same mode */
    step(&h);
    CHECK(msn_mode(&h.m) == MSN_BENCH && h.out.cmd == MC_STOP);
    msn_request(&h.m, MSN_REQ_AUTO);                             /* only from IDLE */
    step(&h);
    CHECK(msn_mode(&h.m) == MSN_BENCH);
    clr(&h);
    msn_request(&h.m, MSN_REQ_IDLE);                             /* -> IDLE: stop, then EN low once standing */
    step(&h);
    CHECK(msn_mode(&h.m) == MSN_IDLE && h.out.cmd == MC_STOP);
    steps(&h, 3);
    CHECK(h.n_ev[MSN_EV_MODE] == 1U && cmd(&h, MC_DISABLE));
    clr(&h);
    steps(&h, 5);
    CHECK(!cmd(&h, MC_DISABLE));                                 /* once only */

    msn_request(&h.m, MSN_REQ_MANUAL);                           /* IDLE -> MANUAL: drv_hold */
    step(&h);
    CHECK(msn_mode(&h.m) == MSN_MANUAL && h.out.cmd == MC_HOLD);
    msn_request(&h.m, MSN_REQ_STOP);
    step(&h);
    CHECK(msn_mode(&h.m) == MSN_MANUAL && h.out.cmd == MC_STOP);
    msn_request(&h.m, MSN_REQ_IDLE);
    step(&h);
    CHECK(msn_mode(&h.m) == MSN_IDLE);

    /* fault reaction >= R2: any mode -> FAULT; R2 stops, R3 brakes, EN low once standing */
    h.in.fault_reaction = R2;
    h.in.fault_mask = 1U << 0x0A;
    step(&h);
    CHECK(msn_mode(&h.m) == MSN_FAULT && h.out.cmd == MC_STOP);
    clr(&h);
    step(&h);
    CHECK(cmd(&h, MC_DISABLE));
    msn_request(&h.m, MSN_REQ_CLEAR);                            /* fault still active: refused */
    step(&h);
    CHECK(msn_mode(&h.m) == MSN_FAULT);
    msn_request(&h.m, MSN_REQ_BENCH);
    step(&h);
    CHECK(msn_mode(&h.m) == MSN_FAULT);
    h.in.fault_reaction = R0;
    h.in.fault_mask = 0U;
    msn_request(&h.m, MSN_REQ_CLEAR);
    step(&h);
    CHECK(msn_mode(&h.m) == MSN_IDLE);

    h_init(&h);
    msn_request(&h.m, MSN_REQ_BENCH);
    step(&h);
    h.in.fault_reaction = R3;
    step(&h);
    CHECK(msn_mode(&h.m) == MSN_FAULT && h.out.cmd == MC_BRAKE);
}

static void start_conditions(void)
{
    h_t h;
    h_init(&h);
    h.m.course_ok = false;                                       /* first unmet condition is reported */
    h.in.fault_mask = 1U;
    h.in.u_bat_v = 2.0f;
    to_auto(&h);
    CHECK(msn_mode(&h.m) == MSN_IDLE && h.out.refused == MSN_REF_COURSE && ev(&h, MSN_EV_START_REFUSED));
    CHECK(!h.out.reset_pose);
    h.m.course_ok = true;
    to_auto(&h);
    CHECK(h.out.refused == MSN_REF_FAULT);
    h.in.fault_mask = 0U;
    to_auto(&h);
    CHECK(h.out.refused == MSN_REF_BATTERY);                     /* absent pack */
    h.in.u_bat_v = 10.0f;                                        /* below bat_warn_v */
    to_auto(&h);
    CHECK(h.out.refused == MSN_REF_BATTERY);
    CHECK(par_set(813, 1.0f) == ST_OK);                          /* msn_allow_bench_supply */
    h.in.u_bat_v = 2.0f;
    h.in.imu_alive = false;
    to_auto(&h);
    CHECK(h.out.refused == MSN_REF_IMU);
    h.in.imu_alive = true;
    h.in.sensors_fresh = false;
    to_auto(&h);
    CHECK(h.out.refused == MSN_REF_SENSORS);
    CHECK(par_set(812, 1.0f) == ST_OK);                          /* msn_allow_nosensors */
    h.in.est.flags = 0U;
    to_auto(&h);
    CHECK(h.out.refused == MSN_REF_MOVING);
    CHECK(msn_mode(&h.m) == MSN_IDLE);
    h.in.feat_imu = h.in.feat_vl = false;                        /* features compiled out */
    h.in.imu_alive = h.in.sensors_fresh = false;
    h.in.est.flags = EKF_STILL;
    to_auto(&h);
    CHECK(msn_mode(&h.m) == MSN_AUTO && msn_state(&h.m) == MSN_AUTO_INIT);
    CHECK(h.out.refused == MSN_START_OK && h.out.reset_pose && h.out.cmd == MC_HOLD);
    CHECK(h.out.pose[0] == 0.0f && h.out.pose[1] == 0.0f && h.out.pose[2] == 0.0f);
    h_init(&h);                                                  /* defaults: 12 V pack, all fresh */
    to_auto(&h);
    CHECK(msn_mode(&h.m) == MSN_AUTO);
}

static void init_state(void)
{
    h_t h;
    h_init(&h);
    h.in.est.P_diag[3] = 1.0e-6f;                                /* sigma_bg 1e-3 > ekf_bg_conv: not converged */
    to_auto(&h);
    steps(&h, 400);                                              /* 8 s */
    CHECK(msn_state(&h.m) == MSN_AUTO_INIT);
    h.in.est.P_diag[3] = 1.0e-8f;
    h.in.est.flags = 0U;                                         /* converged but moving: no standstill */
    steps(&h, 200);
    CHECK(msn_state(&h.m) == MSN_AUTO_INIT);
    h.in.est.flags = EKF_STILL;
    steps(&h, 140);                                              /* 2.8 s still */
    CHECK(msn_state(&h.m) == MSN_AUTO_INIT);
    steps(&h, 20);                                               /* msn_init_min_s = 3 s */
    CHECK(msn_state(&h.m) == MSN_AUTO_DRIVE);

    h_init(&h);                                                  /* never converges: F_INIT after 30 s */
    h.in.est.P_diag[3] = 1.0e-6f;
    to_auto(&h);
    clr(&h);
    steps(&h, 1450);
    CHECK(msn_mode(&h.m) == MSN_AUTO);
    while (msn_mode(&h.m) == MSN_AUTO) { step(&h); }
    CHECK(msn_mode(&h.m) == MSN_FAULT && h.out.cmd == MC_STOP);
    CHECK(msn_state(&h.m) == MSN_AUTO_NONE);
    h_init(&h);
    h.in.est.P_diag[3] = 1.0e-6f;
    to_auto(&h);
    for (int i = 0; i < 1600 && h.out.fault == F_NONE; i++) { step(&h); }
    CHECK(h.out.fault == F_INIT);
}

static void full_mission(void)
{
    h_t h;
    h_init(&h);
    to_auto(&h);
    CHECK(run_until(&h, MSN_AUTO_DRIVE, 400));
    CHECK(h.m.seg == 0U);
    clr(&h);
    CHECK(run_until(&h, MSN_AUTO_FINAL, 3000));                  /* LINE: follow, then the final stop */
    CHECK(cmd(&h, MC_VEL) && h.out.cmd == MC_MOVE_FROM);
    CHECK(h.out.D > 0.05f && h.out.D <= 0.31f && h.out.v0 > 0.2f && h.out.v >= h.out.v0);
    CHECK_NEAR(h.out.a_acc, 0.3, 1e-6);
    CHECK(run_until(&h, MSN_AUTO_HOLD, 100));                    /* DRV_EV_DONE -> next segment: WAYPOINT */
    CHECK(h.m.seg == 1U && h.out.cmd == MC_HOLD);
    CHECK(ev(&h, MSN_EV_FINAL_DONE) && ev(&h, MSN_EV_WP_ARRIVED));
    CHECK(h.out.ev[h.out.n_ev - 1U].code == MSN_EV_WP_ARRIVED || ev(&h, MSN_EV_WP_ARRIVED));
    clr(&h);
    steps_w(&h, 500);                                              /* 10.0 s */
    CHECK(msn_state(&h.m) == MSN_AUTO_HOLD && !ev(&h, MSN_EV_WP_DONE));
    CHECK(run_until(&h, MSN_AUTO_TURN, 40));                     /* 10.5 s hold; the standstill already lasts > 1 s */
    CHECK(ev(&h, MSN_EV_WP_DONE) && h.m.seg == 2U && seen(&h, MSN_AUTO_PRETURN));
    CHECK(run_until(&h, MSN_AUTO_DRIVE, 600));                   /* nav_turn done -> next LINE */
    CHECK(h.m.seg == 3U);
    CHECK_NEAR(h.in.est.x[2], 1.5708, 0.01);
    clr(&h);
    CHECK(run_until(&h, MSN_AUTO_FINISHED, 40000));              /* the rest of rect10 */
    CHECK(h.m.seg == 10U);
    CHECK(h.n_ev[MSN_EV_WP_ARRIVED] == 2U && h.n_ev[MSN_EV_WP_DONE] == 2U && h.n_ev[MSN_EV_FINISHED] == 1U);
    CHECK(seen(&h, MSN_AUTO_PRETURN) && seen(&h, MSN_AUTO_TURN) && seen(&h, MSN_AUTO_FINAL));
    float total = 0.0f;
    for (uint8_t i = 0U; i < h.out.n_ev; i++) { if (h.out.ev[i].code == MSN_EV_FINISHED) { total = h.out.ev[i].x; } }
    CHECK(total > 100.0f && total < 400.0f);
    printf("    rect10 in the ideal world: finished after %.0f s\n", (double)total);
    CHECK(h.out.cmd == MC_HOLD);
    steps_w(&h, 99);
    CHECK(msn_mode(&h.m) == MSN_AUTO);
    clr(&h);
    steps_w(&h, 3);                                                /* 2 s after FINISHED -> IDLE, EN low */
    CHECK(msn_mode(&h.m) == MSN_IDLE && cmd(&h, MC_DISABLE));
}

static void interlock_recover(void)
{
    h_t h;
    h_init(&h);
    to_drive(&h);
    steps(&h, 150);
    CHECK(msn_state(&h.m) == MSN_AUTO_DRIVE);
    h.in.interlock = true;
    h.in.gnd_r = GND_TOO_CLOSE;
    clr(&h);
    world(&h);
    step(&h);                                                    /* REQ-MSN-05: before the segment logic */
    CHECK(msn_state(&h.m) == MSN_AUTO_RECOVER && h.out.cmd == MC_BRAKE);
    steps(&h, 3);
    CHECK(!cmd(&h, MC_MOVE));                                    /* waits for standstill */
    h.in.est.flags = EKF_STILL;
    step(&h);
    CHECK(h.out.cmd == MC_MOVE && h.out.D < -0.099f && h.out.D > -0.101f);
    steps(&h, 5);
    h.in.interlock = false;
    h.in.drv_ev = DRV_EV_DONE;
    step(&h);
    h.in.drv_ev = DRV_EV_NONE;
    CHECK(msn_state(&h.m) == MSN_AUTO_DRIVE);
    steps(&h, 2);
    CHECK_NEAR(h.out.y_off, 0.05, 1e-6);                         /* away from the tripped right side: left */
    for (int i = 0; i < 400; i++) { world(&h); step(&h); }       /* 2 m later the bias is gone */
    CHECK(h.in.s_enc > 2.0f);
    CHECK(h.out.y_off == 0.0f);

    /* interlock persists after backing -> F_GND_INTERLOCK, brake */
    h.in.interlock = true;
    h.in.gnd_l = GND_TOO_CLOSE; h.in.gnd_r = GND_FAR;
    step(&h);
    CHECK(msn_state(&h.m) == MSN_AUTO_RECOVER);
    h.in.est.flags = EKF_STILL;
    steps(&h, 2);
    h.in.drv_ev = DRV_EV_DONE;
    step(&h);
    CHECK(msn_mode(&h.m) == MSN_FAULT && h.out.fault == F_GND_INTERLOCK && h.out.cmd == MC_BRAKE);

    /* the 4th recovery on one segment is a fault; the left side tripped biases to the right */
    h_init(&h);
    to_drive(&h);
    for (int k = 1; k <= 4; k++) {
        steps(&h, 5);
        h.in.interlock = true;
        h.in.gnd_l = GND_TOO_CLOSE;
        world(&h);
        step(&h);
        if (k == 4) { break; }
        CHECK(msn_state(&h.m) == MSN_AUTO_RECOVER);
        h.in.est.flags = EKF_STILL;
        step(&h);
        h.in.interlock = false;
        h.in.drv_ev = DRV_EV_DONE;
        step(&h);
        h.in.drv_ev = DRV_EV_NONE;
        CHECK(msn_state(&h.m) == MSN_AUTO_DRIVE);
        step(&h);
        CHECK_NEAR(h.out.y_off, -0.05, 1e-6);
    }
    CHECK(msn_mode(&h.m) == MSN_FAULT && h.out.fault == F_GND_INTERLOCK);

    /* FINAL and TURN react to the interlock too */
    h_init(&h);
    to_auto(&h);
    CHECK(run_until(&h, MSN_AUTO_FINAL, 3000));
    h.in.interlock = true;
    step(&h);
    CHECK(msn_state(&h.m) == MSN_AUTO_RECOVER);
    h_init(&h);
    to_auto(&h);
    CHECK(run_until(&h, MSN_AUTO_TURN, 8000));
    h.in.interlock = true;
    step(&h);
    CHECK(msn_state(&h.m) == MSN_AUTO_RECOVER);
}

static void reflex(void)
{
    h_t h;
    h_init(&h);
    to_drive(&h);
    steps(&h, 100);
    h.in.reflex = true;
    h.in.v_limit = 0.25f;
    clr(&h);
    world(&h);
    step(&h);
    CHECK(msn_state(&h.m) == MSN_AUTO_OBST_STOP && h.out.cmd == MC_BRAKE && ev(&h, MSN_EV_OBSTACLE));
    h.in.est.flags = 0U;
    steps(&h, 5);
    CHECK(msn_state(&h.m) == MSN_AUTO_OBST_STOP);                /* waits for standstill */
    h.in.est.flags = EKF_STILL;
    step(&h);
    CHECK(msn_state(&h.m) == MSN_AUTO_OBST_WAIT);
    clr(&h);
    steps(&h, 500);                                              /* 10 s: an event, no state change */
    CHECK(msn_state(&h.m) == MSN_AUTO_OBST_WAIT && ev(&h, MSN_EV_OBST_WAIT));
    h.in.reflex = false;
    step(&h);
    CHECK(msn_state(&h.m) == MSN_AUTO_DRIVE);
    world(&h);
    steps(&h, 100);
    CHECK(h.out.cmd == MC_VEL && h.out.v <= 0.25f + 1e-6f);      /* fwd_v_limit respected */

    /* in FINAL: stop, wait, resume re-plans the final stop with the new remaining distance */
    h_init(&h);
    to_auto(&h);
    CHECK(run_until(&h, MSN_AUTO_FINAL, 3000));
    h.in.reflex = true;
    step(&h);
    CHECK(msn_state(&h.m) == MSN_AUTO_OBST_STOP && h.out.cmd == MC_BRAKE);
    h.in.est.x[0] = 9.8f;                                        /* braked 0.1 m further than planned */
    steps_w(&h, 3);
    CHECK(msn_state(&h.m) == MSN_AUTO_OBST_WAIT);
    h.in.reflex = false;
    clr(&h);
    step(&h);
    CHECK(msn_state(&h.m) == MSN_AUTO_FINAL && cmd(&h, MC_MOVE_FROM));
    CHECK_NEAR(h.out.D, 0.2, 0.01);
    CHECK(h.out.v0 < 0.05f);                                     /* restarts from rest */
}

static void stop_resume(void)
{
    h_t h;
    h_init(&h);
    to_drive(&h);
    steps(&h, 50);
    msn_request(&h.m, MSN_REQ_STOP);
    step(&h);
    CHECK(msn_state(&h.m) == MSN_AUTO_STOPPED && h.out.cmd == MC_STOP);
    steps(&h, 20);
    CHECK(msn_state(&h.m) == MSN_AUTO_STOPPED);
    msn_request(&h.m, MSN_REQ_AUTO);                             /* resume the interrupted state */
    step(&h);
    CHECK(msn_state(&h.m) == MSN_AUTO_DRIVE && h.out.cmd == MC_VEL);
    CHECK(h.out.v < 0.05f);                                      /* from rest */
    msn_request(&h.m, MSN_REQ_STOP);
    step(&h);
    msn_request(&h.m, MSN_REQ_IDLE);
    step(&h);
    CHECK(msn_mode(&h.m) == MSN_IDLE && msn_state(&h.m) == MSN_AUTO_NONE && h.out.cmd == MC_STOP);

    h_init(&h);                                                  /* during INIT, and in a waypoint hold */
    to_auto(&h);
    steps(&h, 10);
    msn_request(&h.m, MSN_REQ_STOP);
    step(&h);
    CHECK(msn_state(&h.m) == MSN_AUTO_STOPPED);
    msn_request(&h.m, MSN_REQ_AUTO);
    step(&h);
    CHECK(msn_state(&h.m) == MSN_AUTO_INIT);
    h_init(&h);
    to_auto(&h);
    CHECK(run_until(&h, MSN_AUTO_HOLD, 4000));
    steps(&h, 100);
    msn_request(&h.m, MSN_REQ_STOP);
    step(&h);
    CHECK(msn_state(&h.m) == MSN_AUTO_STOPPED);
    msn_request(&h.m, MSN_REQ_AUTO);
    step(&h);
    CHECK(msn_state(&h.m) == MSN_AUTO_HOLD);
}

static void pause_and_nonstop(void)
{
    static const seg_t C2[4] = {
        { SEG_LINE, { .line = { 0.0f, 0.0f, 2.0f, 0.0f } }, 0.3f, 0.6f, 0.6f, BND_NONE, BND_NONE, 0, 0, 0, 0 },
        { SEG_LINE, { .line = { 2.0f, 0.0f, 4.0f, 0.0f } }, 0.3f, 0.6f, 0.6f, BND_NONE, BND_NONE, 0, SEG_F_STOP_AT_END, 0, 0 },
        { SEG_PAUSE, { .pause = { 1.0f } }, 0.0f, 0.0f, 0.0f, BND_NONE, BND_NONE, 0, 0, 0, 0 },
        { SEG_FINISH, { .turn = { 0.0f } }, 0.0f, 0.0f, 0.0f, BND_NONE, BND_NONE, 0, 0, 0, 0 },
    };
    static const course_t c2 = { "t_msn", "", 0.0f, 0.0f, 0.0f, C2, 4U, NULL, 0U };
    h_t h;
    h_init(&h);
    h.m.course = &c2;
    h.m.course_ok = true;
    to_drive(&h);
    clr(&h);
    float v_at_join = 0.0f;
    for (int i = 0; i < 2000 && h.m.seg == 0U; i++) { world(&h); step(&h); v_at_join = h.out.v; }
    CHECK(h.m.seg == 1U && msn_state(&h.m) == MSN_AUTO_DRIVE);
    CHECK(!seen(&h, MSN_AUTO_FINAL) && !seen(&h, MSN_AUTO_HOLD));  /* no stop in between */
    CHECK(v_at_join > 0.25f && h.out.v > 0.25f);                   /* speed carried over */
    CHECK(run_until(&h, MSN_AUTO_PAUSE, 2000));
    CHECK(h.m.seg == 2U && h.out.cmd == MC_HOLD);
    steps(&h, 49);
    CHECK(msn_state(&h.m) == MSN_AUTO_PAUSE);
    CHECK(run_until(&h, MSN_AUTO_FINISHED, 3));
    CHECK(h.m.seg == 3U && ev(&h, MSN_EV_FINISHED));
    clr(&h);
    steps(&h, 105);
    CHECK(msn_mode(&h.m) == MSN_IDLE && cmd(&h, MC_DISABLE));
}

static void preturn_wait(void)
{
    static const seg_t C3[4] = {
        { SEG_LINE, { .line = { 0.0f, 0.0f, 2.0f, 0.0f } }, 0.3f, 0.6f, 0.6f, BND_NONE, BND_NONE, 0, SEG_F_STOP_AT_END, 0, 0 },
        { SEG_TURN, { .turn = { 1.57079633f } }, 0.0f, 0.0f, 0.0f, BND_NONE, BND_NONE, 0, 0, 0, 0 },
        { SEG_LINE, { .line = { 2.0f, 0.0f, 2.0f, 2.0f } }, 0.3f, 0.6f, 0.6f, BND_NONE, BND_NONE, 0, SEG_F_STOP_AT_END, 0, 0 },
        { SEG_FINISH, { .turn = { 0.0f } }, 0.0f, 0.0f, 0.0f, BND_NONE, BND_NONE, 0, 0, 0, 0 },
    };
    static const course_t c3 = { "t_msn3", "", 0.0f, 0.0f, 0.0f, C3, 4U, NULL, 0U };
    h_t h;
    h_init(&h);
    h.m.course = &c3;
    h.m.course_ok = true;
    to_auto(&h);
    CHECK(run_until(&h, MSN_AUTO_PRETURN, 3000));                /* straight from the final stop */
    CHECK(h.out.cmd == MC_HOLD);
    int n = 0;
    while (msn_state(&h.m) == MSN_AUTO_PRETURN && n < 200) { world(&h); step(&h); n++; }
    CHECK(msn_state(&h.m) == MSN_AUTO_TURN);
    CHECK(n >= 50 && n <= 53);                                   /* msn_preturn_s = 1 s of standstill */
    h.in.est.flags = 0U;                                         /* moving: no turn */
    h.m.st = MSN_AUTO_PRETURN;
    steps(&h, 100);
    CHECK(msn_state(&h.m) == MSN_AUTO_PRETURN);
}

static void turn_without_imu(void)
{
    h_t h;
    h_init(&h);
    h.in.feat_imu = false;
    to_auto(&h);
    clr(&h);
    CHECK(run_until(&h, MSN_AUTO_TURN, 8000));
    CHECK(h.out.cmd == MC_TURN_ENC);
    CHECK_NEAR(h.out.D, 1.5708, 0.01);
    CHECK_NEAR(h.out.w, par()->nav_turn_w, 1e-6);
    CHECK_NEAR(h.out.t_acc, par()->nav_turn_ta, 1e-6);
    CHECK(run_until(&h, MSN_AUTO_DRIVE, 200));                   /* DRV_EV_DONE ends the turn */
    CHECK(h.m.seg == 3U);
}

static void degraded(void)
{
    h_t h;
    h_init(&h);
    to_drive(&h);
    steps(&h, 100);
    CHECK(h.out.v > 0.25f);
    h.in.s_enc += 31.0f;                                         /* 30 m without a reference ... */
    h.in.sig_perp = 0.1f;
    step(&h);
    CHECK(h.out.v > 0.25f);                                      /* ... but sigma still small: no change */
    h.in.sig_perp = 0.4f;                                        /* ... and sigma large: REQ-MSN-04 */
    step(&h);
    CHECK_NEAR(h.out.v, par()->msn_v_degraded, 1e-6);
    steps(&h, 50);
    CHECK(h.out.v <= par()->msn_v_degraded + 1e-6f);
    CHECK(msn_state(&h.m) == MSN_AUTO_DRIVE);                    /* no state change */
    h.in.est.n_acc[EKF_U_EDGE]++;                                /* the next accepted reference clears it */
    world(&h);
    steps(&h, 60);
    CHECK(h.out.v > 0.25f);
}

static void faults_in_auto(void)
{
    h_t h;
    h_init(&h);
    to_drive(&h);
    h.in.fwd_stale = true;                                       /* F_FWD_STALE: controlled stop */
    step(&h);
    CHECK(msn_mode(&h.m) == MSN_FAULT && h.out.fault == F_FWD_STALE && h.out.cmd == MC_STOP);
    h_init(&h);
    to_drive(&h);
    h.in.fault_reaction = R3;                                    /* R3: brake */
    step(&h);
    CHECK(msn_mode(&h.m) == MSN_FAULT && h.out.cmd == MC_BRAKE && h.out.fault == F_NONE);
    h_init(&h);
    to_drive(&h);
    h.in.fault_reaction = R2;
    h.in.reflex = true;                                          /* the fault wins over the reflex */
    step(&h);
    CHECK(msn_mode(&h.m) == MSN_FAULT && h.out.cmd == MC_STOP);
}

static void manual_and_bench(void)
{
    h_t h;
    h_init(&h);
    msn_request(&h.m, MSN_REQ_MANUAL);
    step(&h);
    msn_manual(&h.m, 1.0f, 5.0f, h.in.t_ms);                     /* clamped to msn_man_v_max / w_max */
    step(&h);
    CHECK(h.out.cmd == MC_VEL);
    CHECK_NEAR(h.out.v, 0.40, 1e-6); CHECK_NEAR(h.out.w, 1.5, 1e-6);
    msn_manual(&h.m, -1.0f, -5.0f, h.in.t_ms);
    step(&h);
    CHECK_NEAR(h.out.v, -0.40, 1e-6); CHECK_NEAR(h.out.w, -1.5, 1e-6);

    h.in.reflex = true;                                          /* blocks forward only */
    msn_manual(&h.m, 0.3f, 1.0f, h.in.t_ms);
    step(&h);
    CHECK(h.out.v == 0.0f && h.out.w == 1.0f);
    msn_manual(&h.m, -0.3f, -1.0f, h.in.t_ms);
    step(&h);
    CHECK_NEAR(h.out.v, -0.3, 1e-6); CHECK_NEAR(h.out.w, -1.0, 1e-6);
    h.in.reflex = false;
    h.in.fwd_stale = true;                                       /* F_FWD_STALE in MANUAL: R1, 0.15 m/s */
    msn_manual(&h.m, 0.3f, 0.0f, h.in.t_ms);
    step(&h);
    CHECK_NEAR(h.out.v, 0.15, 1e-6);
    CHECK(msn_mode(&h.m) == MSN_MANUAL);
    h.in.fwd_stale = false;

    h.in.interlock = true;                                       /* brake, then only motion away */
    h.in.gnd_r = GND_TOO_CLOSE;
    msn_manual(&h.m, 0.3f, -1.0f, h.in.t_ms);
    step(&h);
    CHECK(h.out.cmd == MC_BRAKE);
    msn_manual(&h.m, 0.3f, -1.0f, h.in.t_ms);
    step(&h);
    CHECK(h.out.cmd == MC_VEL && h.out.v == 0.0f && h.out.w == 0.0f);   /* towards the right: refused */
    msn_manual(&h.m, -0.2f, 1.0f, h.in.t_ms);
    step(&h);
    CHECK_NEAR(h.out.v, -0.2, 1e-6); CHECK_NEAR(h.out.w, 1.0, 1e-6);
    h.in.interlock = false;

    clr(&h);                                                     /* heartbeat: 0.5 s without a frame */
    steps(&h, 23);
    CHECK(!cmd(&h, MC_STOP));
    steps(&h, 4);
    CHECK(cmd(&h, MC_STOP) && h.n_ev[MSN_EV_MAN_TIMEOUT] == 1U);
    steps(&h, 20);
    CHECK(h.n_ev[MSN_EV_MAN_TIMEOUT] == 1U);                     /* once */
    msn_manual(&h.m, 0.2f, 0.0f, h.in.t_ms);                     /* the next valid command resumes */
    step(&h);
    CHECK(h.out.cmd == MC_VEL && h.out.v == 0.2f);
    msn_manual(&h.m, NAN, INFINITY, h.in.t_ms);                  /* never trusted */
    step(&h);
    CHECK(h.out.v == 0.0f && h.out.w == 0.0f);

    h_init(&h);                                                  /* BENCH: interlock only with gnd_interlock_bench */
    msn_request(&h.m, MSN_REQ_BENCH);
    step(&h);
    h.in.interlock = true;
    clr(&h);
    step(&h);
    CHECK(!cmd(&h, MC_BRAKE));
    h.in.interlock = false;
    step(&h);
    CHECK(par_set(620, 1.0f) == ST_OK);
    h.in.interlock = true;
    step(&h);
    CHECK(h.out.cmd == MC_BRAKE);
}

void t_msn(void)
{
    REQ("REQ-MSN-01: mode table (IDLE/BENCH/MANUAL/FAULT), STOP, fault reactions R2/R3, CLEAR");
    modes_and_stop();
    REQ("REQ-MSN-02: AUTO start conditions, first unmet one reported, bench overrides");
    start_conditions();
    REQ("REQ-MSN-01/06: AUTO_INIT converged, still, F_INIT; every state change emits EV_STATE");
    init_state();
    REQ("REQ-MSN-01/06: SEG dispatch, DRIVE, FINAL, HOLD, PRETURN, TURN, FINISHED on rect10");
    full_mission();
    REQ("REQ-MSN-01/05, REQ-GND-02: interlock before reflex before state logic; RECOVER, bias, F_GND_INTERLOCK");
    interlock_recover();
    REQ("REQ-MSN-05, REQ-FWD-01: reflex -> OBST_STOP -> OBST_WAIT -> resume, final re-planned");
    reflex();
    REQ("REQ-MSN-01: STOP -> AUTO_STOPPED, AUTO resumes the interrupted state, IDLE leaves");
    stop_resume();
    REQ("REQ-MSN-01: PAUSE, non-stop segment end keeps the speed, FINISHED -> IDLE");
    pause_and_nonstop();
    REQ("REQ-MSN-01: PRETURN waits msn_preturn_s of standstill");
    preturn_wait();
    REQ("REQ-MSN-01, §7.19: turn without IMU uses drv_turn_enc");
    turn_without_imu();
    REQ("REQ-MSN-04: degraded flag after 30 m without reference and sigma > 0.30 m");
    degraded();
    REQ("REQ-MSN-05, REQ-FWD-01: faults first in AUTO (R2 stop, R3 brake, F_FWD_STALE)");
    faults_in_auto();
    REQ("REQ-MSN-03, REQ-GND-02: MANUAL clamps, reflex, interlock, heartbeat; BENCH interlock parameter");
    manual_and_bench();
}
