/* imu_yaw.h — yaw-integral ring of spec §7.11.3 (REQ-IMU-05, D-40). Pure, no HAL: built on the host. */
#ifndef IMU_YAW_H
#define IMU_YAW_H

#include "units.h"

#define IMU_YAW_N          32U      /* 31 periods = 37.2 ms at 833 Hz (D-42) */
#define IMU_YAW_REBASE_RAD 8.0f     /* D-40 */

typedef struct { uint32_t t_us; float psi, rate; } imu_yaw_pt_t;

typedef struct {
    imu_yaw_pt_t pt[IMU_YAW_N];
    uint32_t     head;              /* next write index */
    uint32_t     n;                 /* valid entries    */
    uint32_t     rebase_n;          /* a consumer must not subtract two Psi read across a re-base */
} imu_yaw_t;

void imu_yaw_init(imu_yaw_t *y);
/* Psi_i = Psi_(i-1) + rate_i * (t_i - t_(i-1)); the first sample starts at 0. Re-bases at |Psi| > 8 rad. */
void imu_yaw_push(imu_yaw_t *y, uint32_t t_us, float rate_radps);
/* Psi at t_us: interpolated inside the ring, extrapolated with the newest rate up to extrap_max_s
 * beyond it. ST_RANGE for anything older or newer. */
st_t imu_yaw_at(const imu_yaw_t *y, uint32_t t_us, float extrap_max_s, float *psi);

/* Glitch filter (D-77): a gyro sample whose raw value jumps by more than lim_lsb on any axis
 * against the last accepted sample is rejected (returns true). After IMU_GLITCH_RUN rejections in a
 * row the sample is accepted and becomes the new reference, so a real change is never locked out. */
#define IMU_GLITCH_RUN 3U
typedef struct { int16_t g[3]; uint8_t run; bool have; } imu_glitch_t;
bool imu_glitch_reject(imu_glitch_t *f, const int16_t g[3], int32_t lim_lsb);

/* Temperature guard (D-81): a reading outside -40..85 C is always rejected; a step of more than
 * max_step_c against the last accepted reading is rejected up to IMU_GLITCH_RUN times in a row.
 * On rejection *t_c is replaced by the last accepted reading (or 25 C before the first one). */
typedef struct { float t; uint8_t run; bool have; } imu_tguard_t;
bool imu_temp_reject(imu_tguard_t *g, float *t_c, float max_step_c);

/* SPI1 prescaler (D-82): BR code 0..7 (divider 2..256) for a power-of-two divider, -1 otherwise. */
int imu_spi_br_bits(float div);

#endif
