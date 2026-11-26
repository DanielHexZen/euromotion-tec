#include "fwd.h"
#include "par.h"
#include "test.h"

static void put(fwd_t *f, uint8_t id, float r, uint8_t st, uint32_t t_ms)
{
    const vl_meas_t m = { .range_m = r, .status = st, .t_us = t_ms * 1000U };
    fwd_on_meas(f, id, &m, 0U);
}

static uint32_t both(fwd_t *f, float r, uint32_t t_ms, uint32_t n, float v)
{
    for (uint32_t k = 0U; k < n; k++, t_ms += 20U) {
        put(f, VL_F1, r, VL_ST_VALID, t_ms);
        put(f, VL_F2, r, VL_ST_VALID, t_ms);
        fwd_step(f, v, t_ms * 1000U);
    }
    return t_ms;
}

void t_fwd(void)
{
    fwd_t f;
    (void)par_init();

    REQ("REQ-FWD-01: d_stop = |v| t_lat + v^2/(2 a) + margin, 0.27 m at 0.3 m/s");
    fwd_init(&f);
    fwd_step(&f, 0.3f, 0U);
    CHECK_NEAR(f.d_stop, 0.3f * 0.1f + 0.09f / 1.0f + 0.15f, 1e-6);
    CHECK_NEAR(f.d_stop, 0.27f, 1e-6);

    REQ("REQ-FWD-01: reflex sets below d_stop moving forward, not when reversing");
    fwd_init(&f);
    uint32_t t = both(&f, 2.0f, 0U, 3U, 0.3f);
    CHECK(!fwd_reflex(&f));
    CHECK(isinf(fwd_v_limit(&f)));
    t = both(&f, 1.8f, t, 3U, 0.3f);
    t = both(&f, 1.6f, t, 3U, 0.3f);
    t = both(&f, 1.4f, t, 3U, 0.3f);
    t = both(&f, 1.2f, t, 3U, 0.3f);
    t = both(&f, 0.9f, t, 3U, 0.3f);
    CHECK_NEAR(fwd_v_limit(&f), par()->fwd_v_slow, 1e-6);    /* governor below 1.0 m */
    t = both(&f, 0.6f, t, 3U, 0.3f);
    t = both(&f, 0.4f, t, 3U, 0.3f);
    t = both(&f, 0.25f, t, 3U, 0.3f);
    CHECK(fwd_reflex(&f));
    fwd_t r;
    fwd_init(&r);
    (void)both(&r, 0.2f, 0U, 3U, -0.3f);
    CHECK(!fwd_reflex(&r));

    REQ("REQ-FWD-01: reflex clears only after both > d_stop + 0.1 m for 0.5 s");
    t = both(&f, 0.40f, t, 3U, 0.0f);                          /* d_stop 0.15, 0.40 > 0.25 */
    t = both(&f, 0.45f, t, 20U, 0.0f);                         /* 0.4 s */
    CHECK(fwd_reflex(&f));
    for (uint32_t k = 0U; k < 2U; k++, t += 20U) {             /* F2 near twice: median follows, timer restarts */
        put(&f, VL_F1, 0.45f, VL_ST_VALID, t); put(&f, VL_F2, 0.20f, VL_ST_VALID, t);
        fwd_step(&f, 0.0f, t * 1000U);
    }
    t = both(&f, 0.45f, t, 20U, 0.0f);
    CHECK(fwd_reflex(&f));
    t = both(&f, 0.45f, t, 10U, 0.0f);
    CHECK(!fwd_reflex(&f));

    REQ("REQ-FWD-01: a single implausible jump is rejected, a real obstacle is accepted from its second sample");
    fwd_init(&f);
    t = both(&f, 2.0f, 0U, 3U, 0.2f);
    put(&f, VL_F1, 0.2f, VL_ST_VALID, t); put(&f, VL_F2, 2.0f, VL_ST_VALID, t);
    fwd_step(&f, 0.2f, t * 1000U); t += 20U;
    CHECK(!fwd_reflex(&f));
    CHECK_NEAR(fwd_range(&f, 0U), 2.0f, 1e-6);
    t = both(&f, 2.0f, t, 1U, 0.2f);                           /* crosstalk gone */
    put(&f, VL_F1, 0.2f, VL_ST_VALID, t); put(&f, VL_F2, 2.0f, VL_ST_VALID, t); fwd_step(&f, 0.2f, t * 1000U); t += 20U;
    put(&f, VL_F1, 0.2f, VL_ST_VALID, t); put(&f, VL_F2, 2.0f, VL_ST_VALID, t); fwd_step(&f, 0.2f, t * 1000U); t += 20U;
    put(&f, VL_F1, 0.2f, VL_ST_VALID, t); put(&f, VL_F2, 2.0f, VL_ST_VALID, t); fwd_step(&f, 0.2f, t * 1000U); t += 20U;
    CHECK(fwd_reflex(&f));                                     /* obstacle that stays is seen */

    REQ("REQ-FWD-01: signal fail counts as free, other invalid states as unknown; stale after vl_timeout");
    fwd_init(&f);
    t = 0U;
    for (uint32_t k = 0U; k < 3U; k++, t += 20U) {
        put(&f, VL_F1, 0.0f, VL_ST_SIGNAL_FAIL, t);
        put(&f, VL_F2, 0.0f, VL_ST_SIGNAL_FAIL, t);
        fwd_step(&f, 0.3f, t * 1000U);
    }
    CHECK(isinf(fwd_range(&f, 0U)));
    CHECK(!fwd_reflex(&f) && !fwd_stale(&f));
    for (uint32_t k = 0U; k < 15U; k++, t += 20U) {
        put(&f, VL_F1, 0.1f, 4U, t);
        put(&f, VL_F2, 0.1f, 4U, t);
        fwd_step(&f, 0.3f, t * 1000U);
    }
    CHECK(fwd_stale(&f));
    CHECK(!fwd_reflex(&f));

    REQ("REQ-FWD-01: rejected implausible samples age a sensor like unknown ones: stale after vl_timeout");
    fwd_init(&f);
    t = both(&f, 1.0f, 0U, 3U, 0.0f);
    CHECK(!fwd_stale(&f));
    for (uint32_t k = 0U; k < 15U; k++, t += 20U) {            /* F1 alternates 1.0 / 2.5 m: every sample a jump */
        put(&f, VL_F1, (k & 1U) ? 1.0f : 2.5f, VL_ST_VALID, t);
        put(&f, VL_F2, 0.0f, 4U, t);                           /* F2 unknown */
        fwd_step(&f, 0.0f, t * 1000U);
    }
    CHECK(fwd_stale(&f));
    CHECK_NEAR(fwd_range(&f, 0U), 1.0f, 1e-6);                 /* median frozen at the last accepted value */

    REQ("REQ-FWD-01: a plausible stream keeps its sensor fresh");
    fwd_init(&f);
    t = both(&f, 1.0f, 0U, 3U, 0.0f);
    for (uint32_t k = 0U; k < 15U; k++, t += 20U) {
        put(&f, VL_F1, 1.0f, VL_ST_VALID, t);
        put(&f, VL_F2, 0.0f, 4U, t);
        fwd_step(&f, 0.0f, t * 1000U);
    }
    CHECK(!fwd_stale(&f));
}
