#include "t_ekf.h"

#define V_STEP 0.006f      /* 0.3 m/s at 50 Hz */

/* Straight step whose yaw rate disagrees with the wheels by 0.2 rad/s (> ekf_slip_thr). */
static void slip_step(ekfsim_t *s, uint32_t dt_us)
{
    const float dt = (float)dt_us * 1.0e-6f;
    sim_step(s, dt_us, V_STEP * dt / 0.02f, V_STEP * dt / 0.02f, -0.2f * dt, false);
}

void t_ekf_slip(void)
{
    ekfsim_t s;
    ekf_out_t o;

    REQ("REQ-EKF-04: one 20 ms step above the threshold: no flag; two: flag in the second, one episode");
    sim_init(&s);
    sim_step(&s, 20000U, V_STEP, V_STEP, 0.0f, false);
    slip_step(&s, 20000U);
    ekf_get(&s.e, &o);
    CHECK((o.flags & EKF_SLIP) == 0U);
    slip_step(&s, 20000U);
    ekf_get(&s.e, &o);
    CHECK((o.flags & EKF_SLIP) != 0U);
    CHECK(o.slip_episodes == 1U);
    slip_step(&s, 20000U);
    ekf_get(&s.e, &o);
    CHECK(o.slip_episodes == 1U);                         /* same episode */
    sim_step(&s, 20000U, V_STEP, V_STEP, 0.0f, false);
    ekf_get(&s.e, &o);
    CHECK((o.flags & EKF_SLIP) == 0U);

    REQ("REQ-EKF-04: same residual at 5 ms steps: flag in the 8th step (n = ceil(T/dt), D-43)");
    sim_init(&s);
    sim_step(&s, 5000U, V_STEP / 4.0f, V_STEP / 4.0f, 0.0f, false);
    for (int i = 1; i <= 8; i++) {
        slip_step(&s, 5000U);
        ekf_get(&s.e, &o);
        CHECK(((o.flags & EKF_SLIP) != 0U) == (i == 8));
    }

    REQ("REQ-EKF-03/04: in-place turn, slip inflates P_xx by kd*d*infl per step (D-45), none without slip");
    const float d = 0.00176f, b = par()->odo_wheelbase;
    sim_init(&s);
    for (int i = 0; i < 2; i++) { sim_step(&s, 20000U, -d, d, 0.0f, false); }   /* residual 0.8 rad/s */
    ekf_get(&s.e, &o);
    CHECK((o.flags & EKF_SLIP) != 0U);
    const float p0 = s.e.P[0][0];
    sim_step(&s, 20000U, -d, d, 0.0f, false);
    CHECK_NEAR((s.e.P[0][0] - p0) / (par()->ekf_kd * d * par()->ekf_slip_infl), 1.0, 0.01);
    sim_init(&s);
    for (int i = 0; i < 3; i++) { sim_step(&s, 20000U, -d, d, 2.0f * d / b, false); }
    ekf_get(&s.e, &o);
    CHECK((o.flags & EKF_SLIP) == 0U);
    const float p1 = s.e.P[0][0];
    sim_step(&s, 20000U, -d, d, 2.0f * d / b, false);
    CHECK_NEAR(s.e.P[0][0] - p1, 0.0, 1.0e-9);

    REQ("REQ-EKF-04: straight with slip: increase equals kd*|ds_e|*infl (D-45)");
    sim_init(&s);
    for (int i = 0; i < 2; i++) { slip_step(&s, 20000U); }
    const float p2 = s.e.P[0][0];
    slip_step(&s, 20000U);
    CHECK_NEAR((s.e.P[0][0] - p2) / (par()->ekf_kd * V_STEP * par()->ekf_slip_infl), 1.0, 0.01);
}
