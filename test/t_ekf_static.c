#include <math.h>

#include "t_ekf.h"

/* Cholesky of the float P in double: true if symmetric and positive definite. */
static bool pd(const ekf_t *e)
{
    double L[5][5] = { { 0 } };
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j <= i; j++) {
            if (e->P[i][j] != e->P[j][i]) { return false; }
            double s = (double)e->P[i][j];
            for (int k = 0; k < j; k++) { s -= L[i][k] * L[j][k]; }
            if (i == j) { if (!(s > 0.0)) { return false; } L[i][i] = sqrt(s); }
            else { L[i][j] = s / L[j][j]; }
        }
    }
    return true;
}

void t_ekf_static(void)
{
    ekfsim_t s;
    const float bias = 0.5f * DEG2RAD_F, dt = 0.02f;

    REQ("REQ-EKF-09: still from the first step; first ZUPT in step 50 (integer us, D-41, D-37)");
    sim_init(&s);
    sim_step(&s, 20000U, 0.0f, 0.0f, bias * dt, true);
    ekf_out_t o;
    ekf_get(&s.e, &o);
    CHECK((o.flags & EKF_STILL) != 0U);
    for (int i = 2; i <= 49; i++) { sim_step(&s, 20000U, 0.0f, 0.0f, bias * dt, true); }
    CHECK(s.e.n_acc[EKF_U_ZUPT] == 0U);
    sim_step(&s, 20000U, 0.0f, 0.0f, bias * dt, true);
    CHECK(s.e.n_acc[EKF_U_ZUPT] == 1U);

    REQ("REQ-EKF-09: not still while the drive layer commands motion");
    sim_init(&s);
    sim_step(&s, 20000U, 0.0f, 0.0f, 0.0f, false);
    ekf_get(&s.e, &o);
    CHECK((o.flags & EKF_STILL) == 0U);

    REQ("REQ-EKF-01/05: 60 s stationary, bias 0.5 deg/s + noise: sigma_bg < 0.02 deg/s after 3 s, heading frozen");
    sim_init(&s);
    for (int i = 1; i <= 3000; i++) {
        sim_step(&s, 20000U, 0.0f, 0.0f, (bias + 7.0e-4f * gauss(&s.rng)) * dt, true);
        if (i == 150) { CHECK(sqrtf(s.e.P[3][3]) < 0.02f * DEG2RAD_F); }
    }
    CHECK(fabsf(s.e.x[2]) < 0.01f * DEG2RAD_F);          /* D-36: -dt in F(3,4) gives 0.45 deg */
    CHECK_NEAR(s.e.x[3], bias, 1.0e-4);

    REQ("REQ-EKF-01: P symmetric and positive definite after 10^6 steps");
    for (int i = 0; i < 1000000; i++) {
        sim_step(&s, 20000U, 0.0f, 0.0f, (bias + 7.0e-4f * gauss(&s.rng)) * dt, true);
    }
    CHECK(pd(&s.e));
    CHECK((s.e.degraded) == false);
}
