/* ekf.c — spec §7.17 (REQ-EKF-01..10). Reference: Matlab/ekf_design/core. */
#include "ekf.h"

#include <string.h>

#include "par.h"

#define ZUPT_MIN_US  500000U     /* shortest partial window applied on leaving standstill (§7.17.4) */
#define MEAS_MAX_US  1000000     /* measurements older than this are discarded (§7.17.1)             */
#define FACE_G_MIN   0.3f        /* REQ-EKF-06 */
#define PXY_MAX      25.0f       /* REQ-EKF-05 step 5 */
#define PTH_MAX      1.0f

typedef float mat5_t[5][5];

static uint32_t us(float s) { return (uint32_t)(s * 1.0e6f + 0.5f); }

static void sym(mat5_t P)
{
    for (int i = 0; i < 5; i++) {
        for (int j = i + 1; j < 5; j++) { P[i][j] = P[j][i] = 0.5f * (P[i][j] + P[j][i]); }
    }
}

/* P = A P A' + Q */
static void congruence(mat5_t P, mat5_t A, mat5_t Q)
{
    mat5_t T, R;
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            float s = 0.0f;
            for (int k = 0; k < 5; k++) { s += A[i][k] * P[k][j]; }
            T[i][j] = s;
        }
    }
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            float s = Q[i][j];
            for (int k = 0; k < 5; k++) { s += T[i][k] * A[j][k]; }
            R[i][j] = s;
        }
    }
    memcpy(P, R, sizeof R);
    sym(P);
}

static void health(ekf_t *e)
{
    bool ok = e->P[0][0] < PXY_MAX && e->P[1][1] < PXY_MAX && e->P[2][2] < PTH_MAX;
    for (int i = 0; i < 5; i++) {
        ok = ok && isfinite(e->x[i]) && e->P[i][i] > 0.0f;
        for (int j = 0; j < 5; j++) { ok = ok && isfinite(e->P[i][j]); }
    }
    if (!ok) { e->degraded = true; e->n_fault++; }
}

void ekf_init(ekf_t *e)
{
    const par_t *p = par();
    memset(e, 0, sizeof *e);
    e->x[4] = 1.0f;
    e->P[0][0] = e->P[1][1] = sqf(p->ekf_p0_xy);
    e->P[2][2] = sqf(p->ekf_p0_th);
    e->P[3][3] = sqf(p->ekf_p0_bg);
    e->P[4][4] = sqf(p->ekf_p0_s);
    e->nochg_us = us(p->ekf_still_s);   /* the vehicle stands at start (D-37) */
}

void ekf_reset_pose(ekf_t *e, float x, float y, float th)
{
    const par_t *p = par();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 5; j++) { e->P[i][j] = e->P[j][i] = 0.0f; }
    }
    e->x[0] = x; e->x[1] = y; e->x[2] = wrap_pi(th);
    e->P[0][0] = e->P[1][1] = sqf(p->ekf_p0_xy);
    e->P[2][2] = sqf(p->ekf_p0_th);
    e->hist_n = 0U; e->hist_head = 0U;
    memset(e->corr, 0, sizeof e->corr);
}

void ekf_predict(ekf_t *e, const ekf_odo_t *u)
{
    const par_t *p = par();
    const bool imu = !isnan(u->dpsi);
    const float dt = u->dt;
    const uint32_t dt_us = e->started ? (u->t_us - e->t_us) : ((dt > 0.0f) ? us(dt) : 0U);
    e->started = true;
    e->t_us = u->t_us;

    /* standstill and ZUPT window, integer microseconds (D-41) */
    const uint32_t still_us = us(p->ekf_still_s), zt_us = us(p->ekf_zupt_T);
    if (u->ds_l != 0.0f || u->ds_r != 0.0f) { e->nochg_us = 0U; }
    else if (e->nochg_us < still_us) { e->nochg_us += dt_us; }
    const bool was_still = e->still;
    e->still = u->still && e->nochg_us >= still_us;
    bool  zupt_due = false;
    if (e->still) {
        if (imu) {
            e->z_sum += u->dpsi;
            e->z_T_us += dt_us;
            zupt_due = e->z_T_us >= zt_us;
        }
    } else if (was_still) {
        zupt_due = e->z_T_us >= ZUPT_MIN_US;
    }
    const float zt = (float)e->z_T_us * 1.0e-6f, z_rate = e->z_sum / zt;

    /* slip detector, time based (D-43) */
    if (imu && !e->still && dt > 0.0f && dt_us > 0U) {
        const float r = (u->ds_r - u->ds_l) / (p->odo_wheelbase * dt) - (u->dpsi / dt - e->x[3]);
        e->slip_cnt = (fabsf(r) > p->ekf_slip_thr) ? (uint16_t)(e->slip_cnt + (e->slip_cnt < UINT16_MAX)) : 0U;
        const bool was = e->slip;
        e->slip = e->slip_cnt >= (us(p->ekf_slip_T) + dt_us - 1U) / dt_us;
        if (e->slip && !was) { e->slip_episodes++; }
    } else {
        e->slip_cnt = 0U;
        e->slip = false;
    }

    /* state */
    const float s = e->x[4], dse = 0.5f * (u->ds_l + u->ds_r);
    float dth = 0.0f, ds = 0.0f;
    if (!e->still) {
        dth = imu ? (u->dpsi - e->x[3] * dt) : ((u->ds_r - u->ds_l) / p->odo_wheelbase);
        ds  = s * dse;
    }
    if (dt > 0.0f) { e->w_meas = imu ? (u->dpsi / dt - e->x[3]) : ((u->ds_r - u->ds_l) / (p->odo_wheelbase * dt)); }
    const float phi = e->x[2] + 0.5f * dth, c = cosf(phi), n = sinf(phi);
    e->x[0] += ds * c;
    e->x[1] += ds * n;
    e->x[2] = wrap_pi(e->x[2] + dth);

    /* covariance */
    mat5_t F = { {1,0,0,0,0}, {0,1,0,0,0}, {0,0,1,0,0}, {0,0,0,1,0}, {0,0,0,0,1} };
    mat5_t Q = { {0} };
    if (!e->still) {
        F[0][2] = -ds * n;  F[0][4] = dse * c;
        F[1][2] =  ds * c;  F[1][4] = dse * n;
        if (imu) {          /* F(3,4) stays 0 while still (D-36) */
            F[0][3] = 0.5f * ds * n * dt;
            F[1][3] = -0.5f * ds * c * dt;
            F[2][3] = -dt;
        }
    }
    const float d = e->slip ? 0.5f * (fabsf(u->ds_l) + fabsf(u->ds_r)) : fabsf(dse);   /* D-45 */
    const float sig2 = p->ekf_kd * d * (e->slip ? p->ekf_slip_infl : 1.0f);
    Q[0][0] = sqf(s * c) * sig2;
    Q[0][1] = Q[1][0] = s * s * c * n * sig2;
    Q[1][1] = sqf(s * n) * sig2;
    if (imu) {
        if (!e->still) { Q[2][2] += sqf(p->ekf_gyro_nd) * dt; }
        Q[2][2] += sqf(p->ekf_gyro_sf) * p->ekf_sf_corr * fabsf(dth);   /* per radian (D-38) */
        Q[3][3] = p->ekf_q_bg * dt;
    } else {
        Q[2][2] += p->ekf_q_th_enc * fabsf(u->ds_r - u->ds_l);
    }
    Q[4][4] = p->ekf_q_s * dt;
    congruence(e->P, F, Q);
    health(e);

    e->hist_head = (uint8_t)((e->hist_head + 1U) % EKF_HIST_N);
    e->hist_t[e->hist_head] = u->t_us;
    for (int i = 0; i < 3; i++) { e->hist_p[e->hist_head][i] = e->x[i] - e->corr[i]; }
    if (e->hist_n < EKF_HIST_N) { e->hist_n++; }

    if (zupt_due) { (void)ekf_update_zupt(e, z_rate, zt); }
    if (!e->still || zupt_due) { e->z_sum = 0.0f; e->z_T_us = 0U; }
}

/* ---- measurement updates (§7.17.3) ---- */

static float h_s(const ekf_t *e, const float *H, float R)
{
    float s = R;
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) { s += H[i] * e->P[i][j] * H[j]; }
    }
    return s;
}

/* Sequential scalar update, Joseph form. */
static void kal1(ekf_t *e, const float *H, float nu, float R, float dx[5])
{
    float K[5], PHt[5];
    const float S = h_s(e, H, R);
    for (int i = 0; i < 5; i++) {
        PHt[i] = 0.0f;
        for (int j = 0; j < 5; j++) { PHt[i] += e->P[i][j] * H[j]; }
        K[i] = PHt[i] / S;
        dx[i] = K[i] * nu;
        e->x[i] += dx[i];
    }
    for (int i = 0; i < 3; i++) { e->corr[i] += dx[i]; }
    e->x[2] = wrap_pi(e->x[2]);
    mat5_t A, Q;
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            A[i][j] = ((i == j) ? 1.0f : 0.0f) - K[i] * H[j];
            Q[i][j] = R * K[i] * K[j];
        }
    }
    congruence(e->P, A, Q);
}

static st_t accept(ekf_t *e, int type, float d2, float gate)
{
    if (!(d2 <= gate)) { e->n_rej[type]++; return ST_REJECT; }   /* also catches NaN */
    return ST_OK;
}

static void done(ekf_t *e, int type, float d2)
{
    health(e);
    e->n_acc[type]++;
    e->nis_sum[type] += d2;
}

static st_t upd1(ekf_t *e, int type, const float *H, float nu, float R)
{
    const float d2 = nu * nu / h_s(e, H, R);
    if (accept(e, type, d2, par()->ekf_gate1) != ST_OK) { return ST_REJECT; }
    float dx[5];
    kal1(e, H, nu, R, dx);
    done(e, type, d2);
    return ST_OK;
}

st_t ekf_update_zupt(ekf_t *e, float rate_mean_radps, float T_s)
{
    const par_t *p = par();
    const float H[5] = { 0, 0, 0, 1, 0 };
    return upd1(e, EKF_U_ZUPT, H, rate_mean_radps - e->x[3],
                sqf(p->ekf_gyro_nd) / T_s + sqf(p->ekf_zupt_floor));
}

st_t ekf_update_heading(ekf_t *e, float th_meas, float var)
{
    const float H[5] = { 0, 0, 1, 0, 0 };
    return upd1(e, EKF_U_HEADING, H, wrap_pi(th_meas - e->x[2]), var);
}

/* R(th) r and R'(th) r */
static void rot(float th, float rx, float ry, float *Rx, float *Ry, float *dRx, float *dRy)
{
    const float c = cosf(th), n = sinf(th);
    *Rx  = c * rx - n * ry;
    *Ry  = n * rx + c * ry;
    *dRx = -n * rx - c * ry;
    *dRy =  c * rx - n * ry;
}

/* Footprint at the delayed pose: p - dp + R(th - dth) r (REQ-EKF-02). */
static void foot(const ekf_t *e, const ekf_back_t *bk, float rx, float ry,
                 float *fx, float *fy, float *dRx, float *dRy)
{
    rot(e->x[2] - bk->dth, rx, ry, fx, fy, dRx, dRy);
    *fx += e->x[0] - bk->dx;
    *fy += e->x[1] - bk->dy;
}

st_t ekf_update_edge(ekf_t *e, const ekf_line_t *ln, float sx, float sy, const ekf_back_t *bk)
{
    float fx, fy, dx, dy;
    foot(e, bk, sx, sy, &fx, &fy, &dx, &dy);
    const float H[5] = { ln->nx, ln->ny, ln->nx * dx + ln->ny * dy, 0, 0 };
    return upd1(e, EKF_U_EDGE, H, -(ln->nx * fx + ln->ny * fy - ln->d), sqf(par()->ekf_sig_edge));
}

st_t ekf_update_face_range(ekf_t *e, const ekf_line_t *ln, float sx, float sy,
                           float ux, float uy, float r_m, float var, const ekf_back_t *bk)
{
    float fx, fy, dx, dy, gx, gy, dux, duy;
    foot(e, bk, sx, sy, &fx, &fy, &dx, &dy);
    rot(e->x[2] - bk->dth, ux, uy, &gx, &gy, &dux, &duy);
    const float g = ln->nx * gx + ln->ny * gy;
    if (!(fabsf(g) >= FACE_G_MIN)) { return ST_REJECT; }      /* REQ-EKF-06 */
    const float D = ln->d - (ln->nx * fx + ln->ny * fy);
    const float H[5] = { -ln->nx / g, -ln->ny / g,
                         (-(ln->nx * dx + ln->ny * dy) * g - D * (ln->nx * dux + ln->ny * duy)) / (g * g), 0, 0 };
    return upd1(e, EKF_U_FACE, H, r_m - D / g, var);
}

st_t ekf_update_point(ekf_t *e, float lx, float ly, float qx, float qy, float var,
                      const ekf_back_t *bk)
{
    float hx, hy, drx, dry;
    foot(e, bk, qx, qy, &hx, &hy, &drx, &dry);
    const float H[2][5] = { { 1, 0, drx, 0, 0 }, { 0, 1, dry, 0, 0 } };
    const float nu[2] = { lx - hx, ly - hy };
    const float r = sqf(par()->ekf_sig_post) + var;
    float S[2][2];
    for (int a = 0; a < 2; a++) {
        for (int b = 0; b < 2; b++) {
            S[a][b] = (a == b) ? r : 0.0f;
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 5; j++) { S[a][b] += H[a][i] * e->P[i][j] * H[b][j]; }
            }
        }
    }
    const float s12 = 0.5f * (S[0][1] + S[1][0]), det = S[0][0] * S[1][1] - s12 * s12;
    const float d2 = (det > 0.0f)
        ? (S[1][1] * nu[0] * nu[0] - 2.0f * s12 * nu[0] * nu[1] + S[0][0] * nu[1] * nu[1]) / det : INFINITY;
    if (accept(e, EKF_U_POINT, d2, par()->ekf_gate2) != ST_OK) { return ST_REJECT; }
    float dx[5];
    kal1(e, H[0], nu[0], r, dx);
    float dx2[5];                      /* the second row sees the correction of the first */
    kal1(e, H[1], nu[1] - (dx[1] + H[1][2] * dx[2]), r, dx2);
    done(e, EKF_U_POINT, d2);
    return ST_OK;
}

void ekf_get(const ekf_t *e, ekf_out_t *o)
{
    for (int i = 0; i < 5; i++) { o->x[i] = e->x[i]; o->P_diag[i] = e->P[i][i]; }
    o->P_xy = e->P[0][1];
    o->w_meas_radps = e->w_meas;
    o->slip_episodes = e->slip_episodes;
    for (int t = 0; t < EKF_U_N; t++) {
        o->n_acc[t] = e->n_acc[t];
        o->n_rej[t] = e->n_rej[t];
        o->nis_mean[t] = (e->n_acc[t] != 0U) ? e->nis_sum[t] / (float)e->n_acc[t] : 0.0f;
    }
    o->flags = (uint8_t)((e->still ? EKF_STILL : 0U) | (e->slip ? EKF_SLIP : 0U)
                         | (e->degraded ? EKF_DEGRADED : 0U));
}

st_t ekf_back_at(const ekf_t *e, uint32_t t_us, ekf_back_t *bk)
{
    memset(bk, 0, sizeof *bk);
    if (e->hist_n == 0U) { return ST_RANGE; }
    const uint32_t t_new = e->hist_t[e->hist_head];
    const int32_t age = (int32_t)(t_new - t_us);
    if (age > MEAS_MAX_US) { return ST_RANGE; }
    if (age <= 0) { return ST_OK; }
    uint8_t j = e->hist_head, i = j;
    bool found = false;
    for (uint8_t k = 1U; k < e->hist_n && !found; k++) {
        i = (uint8_t)((j + EKF_HIST_N - 1U) % EKF_HIST_N);
        found = (int32_t)(t_new - e->hist_t[i]) >= age;   /* entry i is at or before t_us */
        if (!found) { j = i; }
    }
    if (!found) { return ST_RANGE; }
    const float a = (float)(int32_t)(t_us - e->hist_t[i]) / (float)(uint32_t)(e->hist_t[j] - e->hist_t[i]);
    const float *mi = e->hist_p[i], *mj = e->hist_p[j];
    bk->dx = (e->x[0] - e->corr[0]) - (mi[0] + a * (mj[0] - mi[0]));
    bk->dy = (e->x[1] - e->corr[1]) - (mi[1] + a * (mj[1] - mi[1]));
    bk->dth = wrap_pi((e->x[2] - e->corr[2]) - (mi[2] + a * wrap_pi(mj[2] - mi[2])));
    return ST_OK;
}

st_t ekf_pose_at(const ekf_t *e, uint32_t t_us, float *x, float *y, float *th)
{
    ekf_back_t bk;
    const st_t st = ekf_back_at(e, t_us, &bk);
    *x = e->x[0] - bk.dx;
    *y = e->x[1] - bk.dy;
    *th = wrap_pi(e->x[2] - bk.dth);
    return st;
}
