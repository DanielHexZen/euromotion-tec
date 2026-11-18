#include <math.h>

#include "t_ekf.h"

/* Predict model on a state x: returns the new state. */
static void f_of(const float x0[5], bool imu, float dpsi, ekf_t *out)
{
    ekfsim_t s;
    sim_init(&s);
    for (int i = 0; i < 5; i++) { s.e.x[i] = x0[i]; }
    s.e.started = true;
    sim_step(&s, 20000U, 0.20f, 0.21f, imu ? dpsi : NAN, false);
    *out = s.e;
}

/* Analytic F column j: with P = e_j e_j', P' = F e_j e_j' F' + Q, so P'[i][j] = F[i][j] + Q[i][j]. */
static float f_col(const float x0[5], bool imu, bool idle, int j, int i)
{
    ekfsim_t s;
    sim_init(&s);
    (void)par_set(500, 1.0e-6f);
    for (int a = 0; a < 5; a++) {
        for (int b = 0; b < 5; b++) { s.e.P[a][b] = 0.0f; }
        s.e.x[a] = x0[a];
    }
    s.e.P[j][j] = 1.0f;
    s.e.started = true;
    sim_step(&s, 20000U, idle ? 0.0f : 0.20f, idle ? 0.0f : 0.21f, imu ? 0.3f : NAN, idle);
    return s.e.P[i][j];
}

/* Innovation z - h of the edge (m = 0) or face (m = 1) model at pose x[0..2], in double. */
static const ekf_back_t BK = { 0.05f, -0.02f, 0.1f };
static double nu_of(int m, const double x[3])
{
    const double th = x[2] - (double)BK.dth, c = cos(th), n = sin(th);
    const double rx = 0.3, ry = 0.1, ux = 0.8, uy = 0.5;
    const double nx = (m == 0) ? 0.8 : 0.6, ny = (m == 0) ? 0.6 : 0.8, d = (m == 0) ? 2.2 : 2.0;
    const double fx = x[0] - (double)BK.dx + c * rx - n * ry, fy = x[1] - (double)BK.dy + n * rx + c * ry;
    if (m == 0) { return -(nx * fx + ny * fy - d); }
    return -0.5 - (d - (nx * fx + ny * fy)) / (nx * (c * ux - n * uy) + ny * (n * ux + c * uy));
}

static void update_of(int m, ekf_t *e)
{
    const ekf_line_t l0 = { 0.8f, 0.6f, 2.2f }, l1 = { 0.6f, 0.8f, 2.0f };
    if (m == 0) { (void)ekf_update_edge(e, &l0, 0.3f, 0.1f, &BK); }
    else { (void)ekf_update_face_range(e, &l1, 0.3f, 0.1f, 0.8f, 0.5f, -0.5f, 1.0e-4f, &BK); }
}

void t_ekf_jacobians(void)
{
    const float x0[5] = { 1.0f, 2.0f, 0.7f, 0.1f, 1.01f };
    const float eps = 1.0e-2f;
    ekf_t a, b;

    REQ("REQ-EKF-03: F (IMU, moving) matches central finite differences");
    for (int j = 0; j < 5; j++) {
        float xp[5], xm[5];
        for (int k = 0; k < 5; k++) { xp[k] = xm[k] = x0[k]; }
        xp[j] += eps; xm[j] -= eps;
        f_of(xp, true, 0.3f, &a); f_of(xm, true, 0.3f, &b);
        for (int i = 0; i < 5; i++) {
            CHECK_NEAR(f_col(x0, true, false, j, i), (a.x[i] - b.x[i]) / (2.0f * eps), 1.0e-3);
        }
    }

    REQ("REQ-EKF-10: F in encoder mode matches finite differences");
    for (int j = 0; j < 5; j++) {
        float xp[5], xm[5];
        for (int k = 0; k < 5; k++) { xp[k] = xm[k] = x0[k]; }
        xp[j] += eps; xm[j] -= eps;
        f_of(xp, false, 0.0f, &a); f_of(xm, false, 0.0f, &b);
        for (int i = 0; i < 5; i++) {
            CHECK_NEAR(f_col(x0, false, false, j, i), (a.x[i] - b.x[i]) / (2.0f * eps), 1.0e-3);
        }
    }

    REQ("REQ-EKF-03: F(3,4) = 0 while still (D-36)");
    CHECK(f_col(x0, true, true, 3, 2) == 0.0f);
    CHECK_NEAR(f_col(x0, true, false, 3, 2), -0.02, 1.0e-6);

    REQ("REQ-EKF-05: H of the edge and face updates (x, y, theta, delayed pose) matches finite differences");
    /* P = p e_j e_j': the correction is dx = p H nu / (p H^2 + R); solve for H (large root). */
    const double p = 100.0, R[2] = { 4.0e-4, 1.0e-4 }, X[3] = { 1.0, 2.0, 0.7 };
    for (int m = 0; m < 2; m++) {
        for (int j = 0; j < 3; j++) {
            ekfsim_t s;
            sim_init(&s);
            for (int i = 0; i < 5; i++) { for (int k = 0; k < 5; k++) { s.e.P[i][k] = 0.0f; } s.e.x[i] = x0[i]; }
            s.e.P[j][j] = (float)p;
            update_of(m, &s.e);
            const double dx = (double)s.e.x[j] - (double)x0[j], nu = nu_of(m, X);
            const double dsc = sqrt(p * p * nu * nu - 4.0 * p * dx * dx * R[m]);
            const double H_upd = (p * nu + (nu > 0.0 ? dsc : -dsc)) / (2.0 * p * dx);
            double xp[3] = { X[0], X[1], X[2] }, xm[3] = { X[0], X[1], X[2] };
            xp[j] += 1.0e-4; xm[j] -= 1.0e-4;
            const double H_fd = -(nu_of(m, xp) - nu_of(m, xm)) / 2.0e-4;
            CHECK(fabs(H_fd) > 0.05);
            CHECK(fabs(H_upd - H_fd) < 1.0e-3 * fmax(1.0, fabs(H_fd)));
        }
    }
}
