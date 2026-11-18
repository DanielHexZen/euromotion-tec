#include <math.h>

#include "t_ekf.h"

/* Arc at 0.3 m/s, 1 rad/s from the origin: exact pose at time t (s). */
static void truth(double t, double *x, double *y, double *th)
{
    const double R = 0.3;
    *x = R * sin(t); *y = R * (1.0 - cos(t)); *th = t;
}

void t_ekf_backshift(void)
{
    ekfsim_t s;
    const float dt = 0.02f, b = par_init() == ST_OK ? 0.22325f : 0.0f, v = 0.3f, w = 1.0f;

    sim_init(&s);
    s.e.P[1][1] = 0.01f;                       /* wide enough for a 20 mm correction to pass the gate */
    for (int i = 0; i < 100; i++) {
        sim_step(&s, 20000U, (v - 0.5f * w * b) * dt, (v + 0.5f * w * b) * dt, w * dt, false);
    }
    const uint32_t t_now = s.t_us;

    REQ("REQ-EKF-02: back-shift between two ring entries is within 2 mm of the true motion");
    const uint32_t tq = t_now - 530000U;       /* 0.53 s back: between two entries */
    ekf_back_t bk;
    CHECK(ekf_back_at(&s.e, tq, &bk) == ST_OK);
    double xn, yn, tn, xq, yq, tq_;
    truth((double)t_now * 1.0e-6, &xn, &yn, &tn);
    truth((double)tq * 1.0e-6, &xq, &yq, &tq_);
    CHECK_NEAR(bk.dx, xn - xq, 0.002);
    CHECK_NEAR(bk.dy, yn - yq, 0.002);
    CHECK_NEAR(bk.dth, tn - tq_, 0.002);
    float px, py, pth;
    CHECK(ekf_pose_at(&s.e, tq, &px, &py, &pth) == ST_OK);
    CHECK_NEAR(px, xq, 0.002);

    REQ("REQ-EKF-02: a 20 mm edge correction after the measurement time changes the back-shift by < 0.1 mm (D-39)");
    const ekf_line_t ln = { 0.0f, 1.0f, s.e.x[1] + 0.0208f };
    const ekf_back_t zero = { 0 };
    CHECK(ekf_update_edge(&s.e, &ln, 0.0f, 0.0f, &zero) == ST_OK);
    CHECK(fabsf(s.e.corr[1]) > 0.015f);
    ekf_back_t bk2;
    CHECK(ekf_back_at(&s.e, tq, &bk2) == ST_OK);
    CHECK(fabsf(bk2.dx - bk.dx) < 1.0e-4f && fabsf(bk2.dy - bk.dy) < 1.0e-4f);

    REQ("REQ-EKF-02: a measurement older than 1.0 s is discarded; one at or after the newest entry has no shift");
    CHECK(ekf_back_at(&s.e, t_now - 1100000U, &bk) == ST_RANGE);
    CHECK(ekf_back_at(&s.e, t_now, &bk) == ST_OK && bk.dx == 0.0f && bk.dy == 0.0f);
}
