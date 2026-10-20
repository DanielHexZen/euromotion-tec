#include "par.h"
#include "test.h"
#include "whl.h"

/* Steady state at the commanded speed unless a field is overridden. */
static whl_in_t in_speed(float v_cmd, float v_meas)
{
    whl_in_t in = { .v_ref_mps = v_cmd, .v_mps = v_meas, .u_bat_v = 0.0f, .dt_s = CTRL_DT_S };
    return in;
}

static void fresh(whl_t *w, uint8_t side)
{
    (void)par_init();
    whl_init(w, side);
}

void t_whl_law(void)
{
    whl_t w; whl_in_t in; whl_out_t o;

    REQ("REQ-WHL-01: feedforward equals the hand-computed value (rolling, zero error)");
    fresh(&w, WHL_L);
    in = in_speed(0.2f, 0.2f);
    whl_step(&w, &in, &o);
    CHECK_NEAR(o.ff_d, 0.098 + 1.275 * 0.2, 1e-6);
    CHECK_NEAR(o.duty_d, 0.098 + 1.275 * 0.2, 1e-6);
    in.v_ref_mps = -0.2f; in.v_mps = -0.2f;
    fresh(&w, WHL_L); whl_step(&w, &in, &o);
    CHECK_NEAR(o.duty_d, -(0.098 + 1.275 * 0.2), 1e-6);
    fresh(&w, WHL_R);
    in = in_speed(0.2f, 0.2f); whl_step(&w, &in, &o);
    CHECK_NEAR(o.duty_d, 0.041 + 1.361 * 0.2, 1e-6);
    in.a_ref_mps2 = 1.0f; fresh(&w, WHL_L); in.v_ref_mps = 0.2f; whl_step(&w, &in, &o);
    CHECK_NEAR(o.ff_d, 0.098 + 1.275 * 0.2 + 1.275 * 0.050, 1e-6);   /* + Kv tau_m a_ref */

    REQ("REQ-WHL-01: breakaway Ks while standing, P term on the speed error");
    fresh(&w, WHL_L);
    in = in_speed(0.2f, 0.0f);
    whl_step(&w, &in, &o);
    CHECK_NEAR(o.ff_d, 0.141 + 1.275 * 0.2, 1e-6);
    CHECK_NEAR(o.duty_d, 0.141 + 1.275 * 0.2 + 0.45 * 0.2, 1e-6);

    REQ("REQ-WHL-01: rolling flag has hysteresis (8 mm/s on, 3 mm/s off)");
    fresh(&w, WHL_L);
    const float vs[] = { 0.005f, 0.009f, 0.005f, 0.0025f, 0.005f };
    const bool  want[] = { false, true, true, false, false };
    for (int i = 0; i < 5; i++) {
        in = in_speed(0.2f, vs[i]); whl_step(&w, &in, &o);
        CHECK(((o.flags & WHL_ROLLING) != 0U) == want[i]);
    }

    REQ("REQ-WHL-01: PI integrates Ki e dt, clamps at I_max, command zero forgets it");
    fresh(&w, WHL_L);
    in = in_speed(0.1f, 0.0f);                                  /* e_v = 0.1 */
    whl_step(&w, &in, &o);
    CHECK_NEAR(o.integ_d, 8.98 * 0.1 * 0.02, 1e-6);
    for (int i = 0; i < 200; i++) { in = in_speed(0.1f, 0.05f); whl_step(&w, &in, &o); }
    CHECK_NEAR(o.integ_d, 0.30, 1e-6);
    in = in_speed(0.0f, 0.0f); whl_step(&w, &in, &o);
    CHECK(o.integ_d == 0.0f);

    REQ("REQ-WHL-01: conditional integration - frozen while |u| >= 1");
    fresh(&w, WHL_L);
    in = in_speed(0.45f, 0.0f);                                 /* ff 0.715 + P 0.2 + ... */
    whl_step(&w, &in, &o);
    const float i0 = o.integ_d;
    in = in_speed(0.45f, -0.5f);                                /* e_v = 0.95 -> u > 1 */
    whl_step(&w, &in, &o);
    CHECK(o.duty_d == 1.0f && (o.flags & WHL_SAT) != 0U);
    CHECK(o.integ_d == i0);

    REQ("REQ-WHL-01: position loop adds a clamped speed correction");
    fresh(&w, WHL_L);
    in = in_speed(0.0f, 0.0f); in.pos_loop = true; in.x_ref_m = 0.01f;
    whl_step(&w, &in, &o);
    CHECK_NEAR(o.v_cmd_mps, 2.0 * 0.01, 1e-6);
    in.x_ref_m = 1.0f; whl_step(&w, &in, &o);
    CHECK_NEAR(o.v_cmd_mps, 0.10, 1e-6);                        /* v_corr_max */

    REQ("REQ-WHL-01: battery normalisation only when enabled and the pack is present");
    fresh(&w, WHL_L);
    CHECK(par_set(128, 1.0f) == ST_OK);
    in = in_speed(0.2f, 0.2f); in.u_bat_v = 10.0f; whl_step(&w, &in, &o);
    CHECK_NEAR(o.ff_d, (0.098 + 1.275 * 0.2) * 12.0 / 10.0, 1e-5);
    fresh(&w, WHL_L);
    CHECK(par_set(128, 1.0f) == ST_OK);
    in.u_bat_v = 2.0f; whl_step(&w, &in, &o);
    CHECK_NEAR(o.ff_d, 0.098 + 1.275 * 0.2, 1e-6);

    REQ("REQ-WHL-04: detectors fire on the 30th consecutive tick (0.6 s) and not before");
    fresh(&w, WHL_L);
    in = in_speed(0.3f, 0.0f);                                  /* duty ~0.66, no motion */
    for (int i = 1; i <= 30; i++) {
        whl_step(&w, &in, &o);
        CHECK(((o.flags & WHL_STALL) != 0U) == (i == 30));
        (void)i;
    }
    fresh(&w, WHL_L);
    in = in_speed(0.3f, -0.03f);
    for (int i = 1; i <= 30; i++) { whl_step(&w, &in, &o); CHECK(((o.flags & WHL_RUNAWAY) != 0U) == (i == 30)); }
    fresh(&w, WHL_L);
    in = in_speed(0.0f, 0.0f); in.pos_loop = true; in.x_ref_m = 0.05f;
    for (int i = 1; i <= 30; i++) { whl_step(&w, &in, &o); CHECK(((o.flags & WHL_TRACK) != 0U) == (i == 30)); }
    fresh(&w, WHL_L);                                           /* interruption restarts the count */
    in = in_speed(0.3f, 0.0f);
    for (int i = 0; i < 29; i++) { whl_step(&w, &in, &o); }
    in = in_speed(0.3f, 0.3f); whl_step(&w, &in, &o);
    in = in_speed(0.3f, 0.0f); whl_step(&w, &in, &o);
    CHECK((o.flags & WHL_STALL) == 0U);

    REQ("REQ-WHL-05: dt outside [0.5, 2] x nominal uses the nominal period and is counted");
    fresh(&w, WHL_L);
    in = in_speed(0.1f, 0.0f); in.dt_s = 0.2f; whl_step(&w, &in, &o);
    CHECK_NEAR(o.integ_d, 8.98 * 0.1 * 0.02, 1e-6);
    CHECK(w.dt_glitch == 1U);
    in.dt_s = 0.03f; whl_step(&w, &in, &o);
    CHECK(w.dt_glitch == 1U);
}
