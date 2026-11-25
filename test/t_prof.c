#include "prof.h"
#include "test.h"

static void run(float D, float V, float ta, float td)
{
    prof_t p;
    CHECK(prof_plan(&p, D, V, ta, td) == ST_OK);
    float x, v, a, vp = 0.0f, xp = 0.0f;
    const float dt = 0.001f;
    for (float t = 0.0f; t < p.T + 0.1f; t += dt) {
        prof_eval(&p, t, &x, &v, &a);
        CHECK(fabsf(v - vp) <= p.V / fminf(p.ta, p.td) * dt * 1.01f + 1e-6f);          /* v continuous */
        CHECK((x - xp) * (D > 0.0f ? 1.0f : -1.0f) >= -1e-6f);                       /* no reversal  */
        vp = v; xp = x;
    }
    prof_eval(&p, p.T + 1.0f, &x, &v, &a);
    CHECK_NEAR(x, D, fabsf(D) * 1e-6f);
    CHECK(v == 0.0f && a == 0.0f);
    prof_eval(&p, p.T, &x, &v, &a);
    CHECK_NEAR(x, D, fabsf(D) * 1e-6f);
}

void t_prof(void)
{
    REQ("REQ-PRF-01: x(T) = D, v continuous - trapezoid");
    run(1.0f, 0.3f, 1.0f, 1.0f);
    run(-5.0f, 0.3f, 1.0f, 1.5f);
    REQ("REQ-PRF-01: triangle when |D| < V (ta + td) / 2");
    run(0.3f, 0.3f, 1.0f, 1.0f);
    run(-0.005f, 0.3f, 1.0f, 1.0f);
    prof_t p;
    CHECK(prof_plan(&p, 0.3f, 0.3f, 1.0f, 1.0f) == ST_OK);
    CHECK_NEAR(p.V, 0.3f, 1e-6);                     /* D = V (ta+td)/2 exactly: still trapezoid edge */
    CHECK(prof_plan(&p, 0.1f, 0.3f, 1.0f, 1.0f) == ST_OK);
    CHECK_NEAR(p.V, 0.1f, 1e-6); CHECK_NEAR(p.T, 2.0f, 1e-6);

    REQ("REQ-PRF-02: rejected inputs");
    CHECK(prof_plan(&p, 1.0f, 0.0f, 1.0f, 1.0f) == ST_RANGE);
    CHECK(prof_plan(&p, 1.0f, 0.3f, 0.04f, 1.0f) == ST_RANGE);
    CHECK(prof_plan(&p, 1.0f, 0.3f, 1.0f, 0.04f) == ST_RANGE);
    CHECK(prof_plan(&p, 20.01f, 0.3f, 1.0f, 1.0f) == ST_RANGE);
    CHECK(prof_plan(&p, 0.0f, 0.3f, 1.0f, 1.0f) == ST_RANGE);

    REQ("REQ-PRF-01 (prof_plan_v0, REQ-NAV-02): stops at D from v0, continuous speed, brakes early if it cannot stop");
    static const float v0s[3] = { 0.0f, 0.1f, 0.3f };
    for (int i = 0; i < 3; i++) {
        CHECK(prof_plan_v0(&p, 0.5f, v0s[i], 0.3f, 0.3f, 0.3f) == ST_OK);
        float x, v, a, xp, vp, ap;
        prof_eval(&p, 0.0f, &x, &v, &a);
        CHECK_NEAR(v, v0s[i], 1e-6);
        prof_eval(&p, p.T, &x, &v, &a);
        CHECK_NEAR(x, 0.5, 1e-6); CHECK_NEAR(v, 0.0, 1e-6);
        prof_eval(&p, 0.0f, &xp, &vp, &ap);
        float vmax = 0.0f;
        for (float t = 0.001f; t < p.T; t += 0.001f) {
            prof_eval(&p, t, &x, &v, &a);
            CHECK(fabsf(v - vp) < 0.3f * 0.001f * 1.01f + 1e-5f);        /* v continuous, |a| <= a_acc */
            CHECK(x >= xp - 1e-6f);                                      /* never backwards */
            vmax = fmaxf(vmax, v); xp = x; vp = v;
        }
        CHECK(vmax <= 0.3f + 1e-5f);
    }
    CHECK(prof_plan_v0(&p, 0.1f, 0.0f, 0.3f, 0.3f, 0.3f) == ST_OK);      /* triangle */
    CHECK(p.V < 0.3f && p.V > 0.15f);
    {
        float x, v, a;
        prof_eval(&p, p.T, &x, &v, &a);
        CHECK_NEAR(x, 0.1, 1e-6);
    }
    CHECK(prof_plan_v0(&p, 0.05f, 0.3f, 0.3f, 0.3f, 0.5f) == ST_OK);     /* needs 0.09 m: stops there */
    CHECK_NEAR(p.D, 0.09, 1e-6); CHECK_NEAR(p.T, 0.6, 1e-6);
    CHECK(prof_plan_v0(&p, 0.0f, 0.1f, 0.3f, 0.3f, 0.3f) == ST_RANGE);
    CHECK(prof_plan_v0(&p, 1.0f, -0.1f, 0.3f, 0.3f, 0.3f) == ST_RANGE);
    CHECK(prof_plan_v0(&p, 1.0f, 0.1f, 0.3f, 0.0f, 0.3f) == ST_RANGE);

    REQ("ramp_step: rates up/down, reverses through zero");
    ramp_t r = { 0.0f };
    CHECK_NEAR(ramp_step(&r, 1.0f, 2.0f, 4.0f, 0.1f), 0.2, 1e-6);
    r.y = 1.0f;
    CHECK_NEAR(ramp_step(&r, 0.0f, 2.0f, 4.0f, 0.1f), 0.6, 1e-6);
    r.y = 0.1f;
    CHECK_NEAR(ramp_step(&r, -1.0f, 2.0f, 4.0f, 0.1f), -0.3, 1e-6);   /* down-rate while |y| shrinks */
    r.y = 0.5f;
    CHECK(ramp_step(&r, 0.5f, 2.0f, 4.0f, 0.1f) == 0.5f);
}
