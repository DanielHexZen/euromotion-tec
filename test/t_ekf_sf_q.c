#include "t_ekf.h"

/* One 90 deg in-place turn at 0.8 rad/s with the given step; returns P_thth. */
static float turn(uint32_t dt_us, float sf)
{
    ekfsim_t s;
    sim_init(&s);
    (void)par_set(504, sf);
    CHECK(par_set(519, 0.0087f) == ST_OK);   /* bias already learnt (ZUPT at rest): isolate the sf term */
    ekf_init(&s.e);
    const float dt = (float)dt_us * 1.0e-6f, b = par()->odo_wheelbase;
    float left = 0.5f * PI_F;
    while (left > 1.0e-6f) {
        const float dth = fminf(0.8f * dt, left);
        sim_step(&s, dt_us, -0.5f * b * dth, 0.5f * b * dth, dth, false);
        left -= dth;
    }
    return s.e.P[2][2];
}

void t_ekf_sf_q(void)
{
    REQ("REQ-EKF-03: gyro scale-factor noise per radian, independent of the step rate (D-38)");
    const float a50 = turn(20000U, 0.003f), a200 = turn(5000U, 0.003f);
    CHECK_NEAR(a50 / a200, 1.0, 0.01);
    const float inc = (a50 - turn(20000U, 0.0f)) / (0.003f * 0.003f * par()->ekf_sf_corr * (0.5f * PI_F));
    CHECK_NEAR(inc, 1.0, 0.01);
    CHECK_NEAR((a200 - turn(5000U, 0.0f)) / (0.003f * 0.003f * par()->ekf_sf_corr * (0.5f * PI_F)), 1.0, 0.01);
}
