/* Yaw-ring arithmetic of spec 7.11.3, built on the host without the SPI backend (REQ-IMU-05, D-40). */
#include "imu_yaw.h"
#include "test.h"

#define T_S_US 1200U       /* 833 Hz sample period, rounded */

void t_imu_yaw(void)
{
    REQ("REQ-IMU-05: 37 ms back is answered, 38 ms back and beyond imu_extrap_max ahead are ST_RANGE");
    imu_yaw_t y;
    imu_yaw_init(&y);
    float p;
    CHECK(imu_yaw_at(&y, 0U, 0.002f, &p) == ST_RANGE);               /* empty ring */
    uint32_t t = 5000U;
    for (uint32_t i = 0U; i < 40U; i++, t += T_S_US) { imu_yaw_push(&y, t, 1.0f); }
    const uint32_t newest = t - T_S_US;
    CHECK(imu_yaw_at(&y, newest - 37000U, 0.002f, &p) == ST_OK);
    CHECK(imu_yaw_at(&y, newest - 38000U, 0.002f, &p) == ST_RANGE);
    CHECK(imu_yaw_at(&y, newest + 2000U, 0.002f, &p) == ST_OK);
    CHECK(imu_yaw_at(&y, newest + 2100U, 0.002f, &p) == ST_RANGE);

    REQ("difference of two Psi = rate x dt (interpolation and extrapolation)");
    float a, b;
    CHECK(imu_yaw_at(&y, newest - 20600U, 0.002f, &a) == ST_OK);
    CHECK(imu_yaw_at(&y, newest + 1500U, 0.002f, &b) == ST_OK);
    CHECK_NEAR(b - a, 1.0 * 22100e-6, 1e-5);

    REQ("D-40: re-base above 8 rad, differences stay exact after 1000 rad of turning");
    imu_yaw_init(&y);
    t = 0xFFFFF000U;                                                 /* the timestamp wraps meanwhile */
    uint32_t last_rb = 0U;
    for (uint32_t i = 0U; i < 1000U * 833U / 2U; i++, t += T_S_US) {   /* rate 2 rad/s, ~500 s: 1000 rad */
        imu_yaw_push(&y, t, 2.0f);
        if (y.rebase_n == last_rb) { continue; }                      /* a re-base happened in this push */
        last_rb = y.rebase_n;
        CHECK_NEAR(y.pt[(y.head + IMU_YAW_N - 1U) % IMU_YAW_N].psi, 0.0, 1e-6);
    }
    CHECK(y.rebase_n > 100U);
    const uint32_t nt = t - T_S_US;
    CHECK(imu_yaw_at(&y, nt - 20000U, 0.002f, &a) == ST_OK);
    CHECK(imu_yaw_at(&y, nt, 0.002f, &b) == ST_OK);
    CHECK_NEAR(b - a, 2.0 * 20000e-6, 1e-5);

    REQ("REQ-IMU-07 / D-77: a single corrupted gyro sample is rejected, a real step is accepted after 3");
    {
        imu_glitch_t f = { 0 };
        const int16_t rest[3] = { 307, -275, -148 };
        const int16_t spike[3] = { 307, -275, (int16_t)(-148 + 17000) };
        CHECK(!imu_glitch_reject(&f, rest, 5700));           /* first sample: reference */
        CHECK(!imu_glitch_reject(&f, rest, 5700));
        CHECK(imu_glitch_reject(&f, spike, 5700));            /* +300 deg/s in one sample */
        CHECK(!imu_glitch_reject(&f, rest, 5700));            /* next normal sample passes */
        const int16_t small[3] = { 307, -275, (int16_t)(-148 + 5000) };
        CHECK(!imu_glitch_reject(&f, small, 5700));           /* below the limit: kept */
        const int16_t step[3] = { 307, -275, 20000 };
        CHECK(imu_glitch_reject(&f, step, 5700));
        CHECK(imu_glitch_reject(&f, step, 5700));
        CHECK(imu_glitch_reject(&f, step, 5700));
        CHECK(!imu_glitch_reject(&f, step, 5700));            /* 4th in a row: accepted, new reference */
        CHECK(!imu_glitch_reject(&f, step, 5700));
        const int16_t neg[3] = { 307, (int16_t)(-275 - 6000), 20000 };
        CHECK(imu_glitch_reject(&f, neg, 5700));              /* negative jumps on any axis count too */
    }

    REQ("REQ-IMU-03 / D-81: corrupted temperature readings are replaced by the last good one");
    {
        imu_tguard_t g = { 0 };
        float tc = 1279.3f;
        CHECK(imu_temp_reject(&g, &tc, 1.0f) && tc == 25.0f);       /* out of range before any good reading */
        tc = 17.5f;  CHECK(!imu_temp_reject(&g, &tc, 1.0f) && tc == 17.5f);
        tc = 17.6f;  CHECK(!imu_temp_reject(&g, &tc, 1.0f));
        tc = -32.9f; CHECK(imu_temp_reject(&g, &tc, 1.0f) && tc == 17.6f);
        tc = 1279.3f; CHECK(imu_temp_reject(&g, &tc, 1.0f) && tc == 17.6f);
        for (int k = 0; k < 5; k++) { tc = 99.0f; CHECK(imu_temp_reject(&g, &tc, 1.0f)); }   /* never accepted */
        tc = 20.0f;  CHECK(imu_temp_reject(&g, &tc, 1.0f));
        tc = 20.0f;  CHECK(imu_temp_reject(&g, &tc, 1.0f));
        tc = 20.0f;  CHECK(imu_temp_reject(&g, &tc, 1.0f));
        tc = 20.0f;  CHECK(!imu_temp_reject(&g, &tc, 1.0f) && tc == 20.0f);   /* a real step after 3 */
    }
}
