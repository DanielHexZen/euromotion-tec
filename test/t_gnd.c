#include "gnd.h"
#include "par.h"
#include "test.h"

/* S2 sample whose vertical drop below the sensor is h (m), at time t (ms). */
static vl_meas_t s2(float h, uint8_t status, uint32_t t_ms)
{
    const float r = h / sinf(par()->lat_tilt);
    return (vl_meas_t){ .range_m = r, .status = status, .signal_kcps = 10.0f, .t_us = t_ms * 1000U };
}

static uint32_t feed(gnd_t *g, float h, uint32_t t_ms, uint32_t n)
{
    for (uint32_t k = 0U; k < n; k++, t_ms += 20U) {
        const vl_meas_t m = s2(h, VL_ST_VALID, t_ms);
        gnd_on_meas(g, VL_S2, &m, 0.0f, 0.0f);
    }
    return t_ms;
}

void t_gnd(void)
{
    gnd_t g;
    gnd_edge_ev_t ev;
    (void)par_init();
    const float h0 = par()->lat_h;

    REQ("REQ-GND-01: S2 classes from height and spread (D-70, REQ-LAT-07)");
    gnd_init(&g);
    uint32_t t = feed(&g, h0, 0U, 5U);
    CHECK(gnd_class(&g, VL_S2) == GND_CL_PATH);
    t = feed(&g, h0 + 0.05f, t, 5U);
    CHECK(gnd_class(&g, VL_S2) == GND_CL_DROP);
    gnd_init(&g);
    t = feed(&g, h0 - 0.04f, 0U, 6U);
    CHECK(gnd_class(&g, VL_S2) == GND_CL_EDGE_UP);
    gnd_init(&g);
    t = 0U;
    for (uint32_t k = 0U; k < 10U; k++, t += 20U) {           /* grass: 2 cm scatter */
        const vl_meas_t m = s2(h0 + ((k & 1U) ? 0.02f : -0.02f) * 0.5f, VL_ST_VALID, t);
        gnd_on_meas(&g, VL_S2, &m, 0.0f, 0.0f);
    }
    CHECK(gnd_class(&g, VL_S2) == GND_CL_GRASS);

    REQ("REQ-GND-01: debounce gnd_n_deb = 2, event time = first sample of the new class");
    gnd_init(&g);
    t = feed(&g, h0, 0U, 5U);
    while (gnd_pop_edge_event(&g, &ev)) {}
    const uint32_t t_first = t;
    t = feed(&g, h0 + 0.05f, t, 1U);
    CHECK(gnd_class(&g, VL_S2) == GND_CL_PATH);              /* one sample is not enough */
    t = feed(&g, h0 + 0.05f, t, 1U);
    CHECK(gnd_class(&g, VL_S2) == GND_CL_DROP);
    CHECK(gnd_pop_edge_event(&g, &ev) && ev.from == GND_CL_PATH && ev.to == GND_CL_DROP && ev.t_us == t_first * 1000U && ev.id == VL_S2);
    CHECK(!gnd_pop_edge_event(&g, &ev));

    REQ("REQ-GND-03: events both directions, none via UNKNOWN; staleness");
    t = feed(&g, h0, t, 8U);   /* the step raises the spread: DROP -> GRASS -> PATH (OQ-12) */
    CHECK(gnd_pop_edge_event(&g, &ev) && ev.from == GND_CL_GRASS && ev.to == GND_CL_PATH);
    CHECK(gnd_class(&g, VL_S2) == GND_CL_PATH);
    for (uint32_t k = 0U; k < 3U; k++, t += 20U) {
        const vl_meas_t m = s2(h0, 4U, t);                    /* invalid status */
        gnd_on_meas(&g, VL_S2, &m, 0.0f, 0.0f);
    }
    CHECK(gnd_class(&g, VL_S2) == GND_CL_UNKNOWN);
    t = feed(&g, h0 + 0.05f, t, 3U);
    CHECK(gnd_class(&g, VL_S2) == GND_CL_DROP);
    CHECK(!gnd_pop_edge_event(&g, &ev));                      /* PATH -> UNKNOWN -> DROP: no event */
    gnd_step(&g, (t + 300U) * 1000U);                         /* > vl_timeout without a sample */
    CHECK(gnd_class(&g, VL_S2) == GND_CL_UNKNOWN);
    CHECK(!gnd_pop_edge_event(&g, &ev));

    REQ("REQ-GND-01: signal fail looking far down is DROP, near is UNKNOWN");
    gnd_init(&g);
    t = 0U;
    for (uint32_t k = 0U; k < 3U; k++, t += 20U) {
        const vl_meas_t m = s2(h0 + 0.2f, VL_ST_SIGNAL_FAIL, t);
        gnd_on_meas(&g, VL_S2, &m, 0.0f, 0.0f);
    }
    CHECK(gnd_class(&g, VL_S2) == GND_CL_DROP);

    REQ("REQ-LAT-07: +2 deg roll is compensated (theta_eff = theta + roll for S2)");
    gnd_init(&g);
    const float roll = 2.0f * DEG2RAD_F;
    t = 0U;
    for (uint32_t k = 0U; k < 5U; k++, t += 20U) {
        const vl_meas_t m = { .range_m = h0 / sinf(par()->lat_tilt + roll), .status = VL_ST_VALID, .t_us = t * 1000U };
        gnd_on_meas(&g, VL_S2, &m, roll, 0.0f);
    }
    CHECK(gnd_class(&g, VL_S2) == GND_CL_PATH);

    REQ("REQ-GND-02 / D-70: right side state from S2, left unknown, no sensor interlock (OQ-12)");
    gnd_init(&g);
    t = feed(&g, h0, 0U, 5U);
    CHECK(gnd_side(&g, GND_RIGHT) == GND_FAR);
    CHECK(gnd_side(&g, GND_LEFT) == GND_UNKNOWN);
    t = feed(&g, h0 - 0.04f, t, 8U);
    CHECK(gnd_side(&g, GND_RIGHT) == GND_NOMINAL);
    CHECK(!gnd_interlock(&g));
    vl_meas_t other = s2(h0 + 0.1f, VL_ST_VALID, t);
    gnd_on_meas(&g, VL_F1, &other, 0.0f, 0.0f);               /* other sensors are ignored */
    CHECK(gnd_class(&g, VL_S2) == GND_CL_EDGE_UP);
}
