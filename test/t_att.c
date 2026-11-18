#include "att.h"
#include "par.h"
#include "test.h"

#define DT 0.0012f          /* ~833 Hz IMU rate */

/* Accelerometer reading of a body at roll phi, pitch th (rad), plus lateral extra (g). */
static void acc_of(float phi, float th, float lat_g, float a[3])
{
    a[0] = -G_MPS2 * sinf(th);
    a[1] = G_MPS2 * (sinf(phi) * cosf(th) + lat_g);
    a[2] = G_MPS2 * cosf(phi) * cosf(th);
}

static void run(att_t *at, float phi, float th, float lat_g, float p_radps, float secs)
{
    float a[3], w[3] = { p_radps, 0.0f, 0.0f };
    acc_of(phi, th, lat_g, a);
    for (float t = 0.0f; t < secs; t += DT) { att_update(at, w, a, DT); }
}

void t_att(void)
{
    att_t at;
    (void)par_init();

    REQ("REQ-ATT-01: level at rest, |roll|, |pitch| < 0.5 deg after 2 s");
    att_init(&at);
    run(&at, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f);
    CHECK(fabsf(att_roll(&at)) < 0.5f * DEG2RAD_F);
    CHECK(fabsf(att_pitch(&at)) < 0.5f * DEG2RAD_F);

    REQ("REQ-ATT-01: 6 deg ramp, pitch within 0.5 deg after 3 s (and roll 4 deg)");
    att_init(&at);
    run(&at, 4.0f * DEG2RAD_F, 6.0f * DEG2RAD_F, 0.0f, 0.0f, 3.0f);
    CHECK_NEAR(att_pitch(&at), 6.0f * DEG2RAD_F, 0.5f * DEG2RAD_F);
    CHECK_NEAR(att_roll(&at), 4.0f * DEG2RAD_F, 0.5f * DEG2RAD_F);

    REQ("REQ-ATT-01: lateral 0.5 g fails the |a| gate, the gyro carries the angle");
    att_init(&at);
    run(&at, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
    run(&at, 0.0f, 0.0f, 0.5f, 0.1f, 1.0f);       /* true roll rate 0.1 rad/s for 1 s */
    CHECK_NEAR(att_roll(&at), 0.1, 0.003);

    REQ("REQ-ATT-01: lateral 0.3 g is inside the 0.1 g gate: bounded effect, then recovers");
    att_init(&at);
    run(&at, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
    run(&at, 0.0f, 0.0f, 0.3f, 0.0f, 0.2f);
    CHECK(fabsf(att_roll(&at)) < 4.0f * DEG2RAD_F);
    run(&at, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f);
    CHECK(fabsf(att_roll(&at)) < 0.5f * DEG2RAD_F);
}
