/* imu_yaw.c — spec §7.11.3 (REQ-IMU-05, D-40). */
#include "imu_yaw.h"

static const imu_yaw_pt_t *back(const imu_yaw_t *y, uint32_t k)   /* k = 0 newest */
{
    return &y->pt[(y->head + IMU_YAW_N - 1U - k) % IMU_YAW_N];
}

void imu_yaw_init(imu_yaw_t *y) { *y = (imu_yaw_t){ 0 }; }

void imu_yaw_push(imu_yaw_t *y, uint32_t t_us, float rate_radps)
{
    float psi = 0.0f;
    if (y->n > 0U) {
        const imu_yaw_pt_t *p = back(y, 0U);
        psi = p->psi + rate_radps * (float)(uint32_t)(t_us - p->t_us) * 1.0e-6f;
    }
    y->pt[y->head] = (imu_yaw_pt_t){ .t_us = t_us, .psi = psi, .rate = rate_radps };
    y->head = (y->head + 1U) % IMU_YAW_N;
    if (y->n < IMU_YAW_N) { y->n++; }
    if (fabsf(psi) > IMU_YAW_REBASE_RAD) {
        for (uint32_t i = 0U; i < y->n; i++) { y->pt[i].psi -= psi; }
        y->rebase_n++;
    }
}

st_t imu_yaw_at(const imu_yaw_t *y, uint32_t t_us, float extrap_max_s, float *psi)
{
    if (y->n == 0U) { return ST_RANGE; }
    const imu_yaw_pt_t *b = back(y, 0U);
    const int32_t d = (int32_t)(t_us - b->t_us);
    if (d >= 0) {
        const float dt = (float)d * 1.0e-6f;
        if (dt > extrap_max_s) { return ST_RANGE; }
        *psi = b->psi + b->rate * dt;
        return ST_OK;
    }
    for (uint32_t k = 1U; k < y->n; k++) {                 /* REQ-COD-07: bounded by the ring */
        const imu_yaw_pt_t *a = back(y, k);
        if ((int32_t)(t_us - a->t_us) >= 0) {
            const uint32_t span = b->t_us - a->t_us;
            const float f = (span > 0U) ? (float)(uint32_t)(t_us - a->t_us) / (float)span : 0.0f;
            *psi = a->psi + (b->psi - a->psi) * f;
            return ST_OK;
        }
        b = a;
    }
    return ST_RANGE;
}
