#include <math.h>

#include "t_ekf.h"

void t_ekf_line(void)
{
    ekfsim_t s;
    const float dt = 0.02f, v = 0.3f, sy = 0.15f, scale_err = 1.005f;
    int n_nis = 0, n_in = 0;
    float max_y = 0.0f;

    REQ("REQ-EKF-05: 40 m straight, scale error 0.5 %, gyro bias drift, edge every 0.5 m: cross-track < 3 cm, NIS in band");
    sim_init(&s);
    for (int i = 0; i < 250; i++) {                              /* 5 s standing: bias convergence */
        sim_step(&s, 20000U, 0.0f, 0.0f, (0.5f * DEG2RAD_F + 7.0e-4f * gauss(&s.rng)) * dt, true);
    }
    const int steps = (int)(40.0f / v / dt);
    for (int i = 1; i <= steps; i++) {
        const float bias = (0.5f + 0.02f * (float)i / (float)steps) * DEG2RAD_F;
        const float ds = v * dt * scale_err;
        sim_step(&s, 20000U, ds, ds, (bias + 7.0e-4f * gauss(&s.rng)) * dt, false);
        if (i % 83 == 0) {                                       /* 0.5 m at 0.3 m/s = 83 steps */
            const ekf_line_t ln = { 0.0f, 1.0f, sy + 0.02f * gauss(&s.rng) };   /* truth: y = 0, theta = 0 */
            const ekf_back_t bk = { 0 };
            const float nis0 = s.e.nis_sum[EKF_U_EDGE];
            if (ekf_update_edge(&s.e, &ln, 0.2f, sy, &bk) == ST_OK) {
                n_nis++;
                n_in += (s.e.nis_sum[EKF_U_EDGE] - nis0) < 3.84f;   /* chi2_1 95 % */
            }
        }
        if (i > 200) { max_y = fmaxf(max_y, fabsf(s.e.x[1])); }
    }
    CHECK(max_y < 0.03f);
    CHECK(n_nis > 70);
    CHECK((float)n_in >= 0.9f * (float)n_nis);
    CHECK(s.e.degraded == false);
}
