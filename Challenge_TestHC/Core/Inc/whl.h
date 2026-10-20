/* whl.h — per-wheel controller (spec §7.8). Pure. */
#ifndef WHL_H
#define WHL_H

#include "app_config.h"
#include "units.h"

#define WHL_L 0U
#define WHL_R 1U

#define WHL_ROLLING 0x01U
#define WHL_SAT     0x02U
#define WHL_STALL   0x04U
#define WHL_RUNAWAY 0x08U
#define WHL_TRACK   0x10U

/* Detector thresholds fixed by REQ-WHL-04; persistence is the parameter whl_fault_time. */
#define WHL_STALL_DUTY   0.5f
#define WHL_STALL_V_MPS  0.005f
#define WHL_RUN_DUTY     0.1f
#define WHL_RUN_V_MPS    0.020f

typedef struct {
    float x_ref_m, v_ref_mps, a_ref_mps2;   /* reference from drv                       */
    float v_sync_mps;                       /* L/R coupling term from drv               */
    float x_m, v_mps;                       /* measured                                 */
    float u_bat_v;                          /* filtered pack voltage, < BAT_ABSENT_V = none */
    float dt_s;                             /* from timestamps                          */
    bool  pos_loop;                         /* false: pure speed control                */
} whl_in_t;

typedef struct {
    float   duty_d;                         /* to mot_set()                             */
    float   v_cmd_mps, e_x_m, e_v_mps, integ_d, ff_d;
    uint8_t flags;
} whl_out_t;

typedef struct {
    uint8_t  side;
    float    integ_d;
    bool     rolling;
    uint16_t n_stall, n_runaway, n_track;   /* consecutive control ticks                */
    uint32_t dt_glitch;                     /* REQ-WHL-05 statistics                    */
} whl_t;

void whl_init(whl_t *w, uint8_t side);
void whl_reset(whl_t *w);                   /* integrator 0, detectors cleared          */
void whl_step(whl_t *w, const whl_in_t *in, whl_out_t *out);   /* pure, any context     */
/* dt outside [0.5, 2] x nominal -> nominal (REQ-WHL-05). */
float whl_dt_guard(float dt_s, bool *glitch);

#endif
