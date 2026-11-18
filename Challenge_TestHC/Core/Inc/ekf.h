/* ekf.h — state estimation x = [x y theta b_g s] (spec §7.17). Pure. */
#ifndef EKF_H
#define EKF_H

#include "units.h"

#define EKF_HIST_N 64U          /* pose ring, 1.28 s at 50 Hz (§7.17.1) */

/* update types: index of the counters in ekf_out_t */
enum { EKF_U_ZUPT = 0, EKF_U_EDGE, EKF_U_FACE, EKF_U_HEADING, EKF_U_POINT, EKF_U_N };

#define EKF_STILL    0x01U
#define EKF_SLIP     0x02U
#define EKF_DEGRADED 0x04U      /* health check failed (P or x non-finite / out of bounds) -> F_EKF */

typedef struct {
    float    ds_l, ds_r;        /* m, after backlash                                         */
    float    dpsi;              /* rad, imu_yaw_integral_at difference; NAN = encoder mode   */
    float    dt;                /* s, one S_CTRL period                                      */
    uint32_t t_us;              /* encoder timestamp                                         */
    bool     still;             /* drive layer in HOLD/DISABLED/IDLE; the EKF adds the
                                 * "no count change for ekf_still_s" part itself (REQ-EKF-09) */
} ekf_odo_t;

typedef struct { float dx, dy, dth; } ekf_back_t;     /* motion since the measurement time  */
typedef struct { float nx, ny, d; } ekf_line_t;       /* n'q = d, n unit, pointing into the path */

typedef struct {
    float    x[5];
    float    P[5][5];
    float    corr[3];                       /* running sum of measurement corrections (D-39) */
    uint32_t hist_t[EKF_HIST_N];
    float    hist_p[EKF_HIST_N][3];         /* pose - corr                                    */
    uint8_t  hist_head, hist_n;
    uint32_t t_us;
    uint32_t nochg_us, z_T_us;              /* integer microseconds (D-41)                    */
    float    z_sum, w_meas;
    uint16_t slip_cnt;
    uint32_t slip_episodes;
    uint32_t n_acc[EKF_U_N], n_rej[EKF_U_N], n_fault;
    float    nis_sum[EKF_U_N];
    bool     started, still, slip, degraded;
} ekf_t;

typedef struct {
    float    x[5];
    float    P_diag[5], P_xy;               /* sigma_perp = sqrt(n'P_pp n) is left to the caller (needs the segment) */
    float    w_meas_radps;                  /* bias-corrected yaw rate of the last step       */
    uint32_t n_acc[EKF_U_N], n_rej[EKF_U_N], slip_episodes;
    float    nis_mean[EKF_U_N];
    uint8_t  flags;
} ekf_out_t;

void ekf_init(ekf_t *e);                                    /* P0, x = origin, b_g 0, s 1 */
void ekf_reset_pose(ekf_t *e, float x, float y, float th);  /* keeps b_g, s               */
/* Standstill, slip detector, state/covariance prediction and, when due, the ZUPT (REQ-EKF-09). */
void ekf_predict(ekf_t *e, const ekf_odo_t *u);
st_t ekf_update_zupt(ekf_t *e, float rate_mean_radps, float T_s);
st_t ekf_update_edge(ekf_t *e, const ekf_line_t *ln, float sx, float sy, const ekf_back_t *bk);
st_t ekf_update_face_range(ekf_t *e, const ekf_line_t *ln, float sx, float sy,
                           float ux, float uy, float r_m, float var, const ekf_back_t *bk);
st_t ekf_update_heading(ekf_t *e, float th_meas, float var);
st_t ekf_update_point(ekf_t *e, float lx, float ly, float qx, float qy, float var,
                      const ekf_back_t *bk);
void ekf_get(const ekf_t *e, ekf_out_t *out);
/* Back-shift for a measurement taken at t_us; ST_RANGE if older than 1.0 s or than the ring. */
st_t ekf_back_at(const ekf_t *e, uint32_t t_us, ekf_back_t *bk);
st_t ekf_pose_at(const ekf_t *e, uint32_t t_us, float *x, float *y, float *th);

#endif
