#include <math.h>

#include "t_ekf.h"

void t_ekf_pred(void)
{
    ekfsim_t s;
    ekf_out_t o;
    (void)par_init();
    const float dt = 0.02f, b = par()->odo_wheelbase;

    REQ("REQ-EKF-03: straight line with the odometry scale state s");
    sim_init(&s);
    s.e.x[4] = 1.01f;
    for (int i = 0; i < 100; i++) { sim_step(&s, 20000U, 0.006f, 0.006f, 0.0f, false); }
    CHECK_NEAR(s.e.x[0], 0.6 * 1.01, 1.0e-4);
    CHECK_NEAR(s.e.x[1], 0.0, 1.0e-6);

    REQ("REQ-EKF-03: arc, 0.3 m/s and 1 rad/s, ends on the analytic circle; bias is subtracted");
    sim_init(&s);
    s.e.x[3] = 0.02f;
    for (int i = 0; i < 100; i++) {
        sim_step(&s, 20000U, (0.3f - 0.5f * b) * dt, (0.3f + 0.5f * b) * dt, (1.0f + 0.02f) * dt, false);
    }
    CHECK_NEAR(s.e.x[0], 0.3 * sin(2.0), 1.0e-3);
    CHECK_NEAR(s.e.x[1], 0.3 * (1.0 - cos(2.0)), 1.0e-3);
    CHECK_NEAR(wrap_pi(s.e.x[2] - 2.0f), 0.0, 1.0e-4);

    REQ("REQ-EKF-10: encoder-heading mode (dpsi = NAN) takes theta from the wheel difference");
    sim_init(&s);
    for (int i = 0; i < 100; i++) {
        sim_step(&s, 20000U, (0.3f - 0.5f * b) * dt, (0.3f + 0.5f * b) * dt, NAN, false);
    }
    CHECK_NEAR(s.e.x[0], 0.3 * sin(2.0), 1.0e-3);
    CHECK_NEAR(s.e.x[1], 0.3 * (1.0 - cos(2.0)), 1.0e-3);
    CHECK_NEAR(wrap_pi(s.e.x[2] - 2.0f), 0.0, 1.0e-4);
    ekf_get(&s.e, &o);
    CHECK((o.flags & EKF_SLIP) == 0U);                 /* slip detector off */
    CHECK_NEAR(s.e.x[3], 0.0, 0.0);                    /* b_g frozen ... */
    CHECK_NEAR(s.e.P[3][3], par()->ekf_p0_bg * par()->ekf_p0_bg, 1.0e-12);
    CHECK_NEAR(s.e.P[2][3], 0.0, 0.0);                 /* ... and uncorrelated */

    REQ("REQ-EKF-10: no ZUPT in encoder mode while standing; Q_thth grows by q_th_enc*|ds_r - ds_l|");
    sim_init(&s);
    for (int i = 0; i < 200; i++) { sim_step(&s, 20000U, 0.0f, 0.0f, NAN, true); }
    CHECK(s.e.n_acc[EKF_U_ZUPT] == 0U);
    sim_init(&s);
    const float p0 = s.e.P[2][2];
    sim_step(&s, 20000U, 0.002f, 0.004f, NAN, false);
    CHECK_NEAR(s.e.P[2][2] - p0, par()->ekf_q_th_enc * 0.002f, 1.0e-9);

    REQ("REQ-EKF-01: covariance stays symmetric after prediction and updates; reset_pose keeps b_g and s");
    sim_init(&s);
    s.e.x[3] = 0.01f; s.e.x[4] = 1.02f;
    for (int i = 0; i < 500; i++) { sim_step(&s, 20000U, 0.006f, 0.0061f, 0.0003f, false); }
    const ekf_back_t bk = { 0 };
    (void)ekf_update_heading(&s.e, 0.001f, 1.0e-4f);
    (void)ekf_update_point(&s.e, s.e.x[0] + 0.01f, s.e.x[1], 0.2f, 0.0f, 0.0f, &bk);
    for (int i = 0; i < 5; i++) { for (int j = 0; j < 5; j++) { CHECK(s.e.P[i][j] == s.e.P[j][i]); } }
    const float bg = s.e.x[3], sc = s.e.x[4];
    ekf_reset_pose(&s.e, 1.0f, 2.0f, 3.0f);
    CHECK(s.e.x[0] == 1.0f && s.e.x[1] == 2.0f && s.e.x[3] == bg && s.e.x[4] == sc);
    CHECK(s.e.hist_n == 0U);
}
