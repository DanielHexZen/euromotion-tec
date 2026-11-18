#include "t_ekf.h"

/* Edge on the line y = d, sensor 0.2 m ahead, 0.1 m left; h = sy - d at the origin pose. */
static st_t edge(ekfsim_t *s, float h)
{
    const ekf_line_t ln = { 0.0f, 1.0f, 0.1f - h };
    const ekf_back_t bk = { 0 };
    return ekf_update_edge(&s->e, &ln, 0.2f, 0.1f, &bk);
}

void t_ekf_gate(void)
{
    ekfsim_t s;
    /* S = P_yy + R + (n'R'r)^2 P_thth = 1e-4 + 4e-4 + 3e-6 -> sigma 22.4 mm */
    REQ("REQ-EKF-05: a 10 sigma edge outlier is rejected and counted");
    sim_init(&s);
    CHECK(edge(&s, 0.224f) == ST_REJECT);
    CHECK(s.e.n_rej[EKF_U_EDGE] == 1U && s.e.n_acc[EKF_U_EDGE] == 0U);
    CHECK_NEAR(s.e.x[1], 0.0, 1.0e-9);

    REQ("REQ-EKF-05: a 2 sigma one is accepted, pulls the state and shrinks P");
    sim_init(&s);
    const float pyy = s.e.P[1][1];
    CHECK(edge(&s, 0.045f) == ST_OK);
    CHECK(s.e.n_acc[EKF_U_EDGE] == 1U);
    CHECK(s.e.x[1] < 0.0f);                    /* h > 0: the footprint is on the wrong side */
    CHECK(s.e.P[1][1] < pyy);

    REQ("REQ-EKF-05: 2-D post update gates on the joint d2 (chi2_2)");
    sim_init(&s);
    const ekf_back_t bk = { 0 };
    CHECK(ekf_update_point(&s.e, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, &bk) == ST_REJECT);   /* 1 m off */
    CHECK(ekf_update_point(&s.e, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, &bk) == ST_OK);
    CHECK(s.e.n_rej[EKF_U_POINT] == 1U && s.e.n_acc[EKF_U_POINT] == 1U);

    REQ("REQ-EKF-06: face range with an oblique ray (|g| < 0.3) is rejected without update");
    sim_init(&s);
    const ekf_line_t ln = { 1.0f, 0.0f, 1.0f };
    CHECK(ekf_update_face_range(&s.e, &ln, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0e-4f, &bk) == ST_REJECT);
    CHECK(s.e.n_acc[EKF_U_FACE] == 0U);
}
