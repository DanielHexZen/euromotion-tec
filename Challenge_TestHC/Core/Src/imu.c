/* imu.c — spec §7.11 (REQ-IMU-01..07). Chain: DRDY edge -> TIM2 CC2 ISR (hardware timestamp,
 * starts the SPI DMA read with CS low) -> DMA complete ISR (CS high, publishes the raw slot) ->
 * imu_drain() in S_FAST (decode, axis map, att, yaw integral). */
#include "imu.h"

#include "imu_lsm6ds3.h"
#include "imu_yaw.h"
#include "par.h"
#include "sys_fault.h"
#include "sys_irq.h"
#include "sys_time.h"

#define RAW_N     8U             /* power of two; one slot always stays free for the DMA, so 7 usable */
#define COS_MIN   0.05f          /* 1 / cos(theta) cap: a ground vehicle never pitches to 87 deg */
#define SAT_RUN   3U             /* REQ-IMU-07 */
#define T_REF_C   25.0f          /* REQ-IMU-03 */
#define SETTLE_US 500000U        /* samples of the first 0.5 s after configure are dropped (D-67) */

typedef struct { uint32_t t_us; uint8_t rx[IMU_READ_BYTES]; } raw_t;

static const imu_backend_t *s_be;
static bool          s_present;
static raw_t         s_raw[RAW_N];
static uint8_t       s_tx[IMU_READ_BYTES];
static volatile uint32_t s_head, s_tail;     /* free-running; REQ-EXE-06 c. head: ISR, tail: main */
static volatile bool     s_busy;
static volatile uint32_t s_rx_us, s_rx_n;    /* last real DRDY edge (ISR); timeout reference */
static volatile uint32_t s_drop;

static imu_yaw_t s_yaw;
static uint32_t  s_prev_us, s_kick_us, s_init_us, s_sat_run, s_sat_n, s_n;
static float     s_sum_w[3];                     /* IMUS window, main loop only */
static uint32_t  s_sum_n;
static imu_glitch_t s_glitch;
static uint32_t  s_glitch_n;
static bool      s_have_prev, s_timed_out;
static float     s_temp_c = T_REF_C, s_w[3], s_acc[3];

/* ---- ISR side ------------------------------------------------------------------------- */
static void start_read(uint32_t t_us)
{
    if (s_busy) { s_drop++; return; }
    raw_t *r = &s_raw[s_head % RAW_N];                    /* never visible to the consumer yet */
    r->t_us = t_us;
    s_busy = true;
    IMU_CS_GPIO_Port->BSRR = (uint32_t)IMU_CS_Pin << 16U;
    s_be->start_read_dma(r->rx, s_tx, IMU_READ_BYTES);
}

void imu_drdy_capture_isr(uint32_t t_capture_us)
{
    s_rx_us = t_capture_us;
    s_rx_n++;
    start_read(t_capture_us);
}

void imu_dma_done_isr(void)
{
    IMU_CS_GPIO_Port->BSRR = (uint32_t)IMU_CS_Pin;
    const uint32_t h = s_head;
    if (h - s_tail < RAW_N - 1U) { s_head = h + 1U; } else { s_drop++; }
    s_busy = false;
}

void imu_error_isr(void)
{
    IMU_CS_GPIO_Port->BSRR = (uint32_t)IMU_CS_Pin;
    s_drop++;
    s_busy = false;
}

/* A latched DRDY only falls when the data are read, so after a lost read (SPI error, start-up)
 * there is no new edge. Reading once from main restarts the chain. */
static void kick(void)
{
    const uint32_t pm = sys_irq_save();
    start_read(sys_time_us());
    sys_irq_restore(pm);
}

/* ---- init ----------------------------------------------------------------------------- */
st_t imu_init(SPI_HandleTypeDef *hspi1, TIM_HandleTypeDef *htim2)
{
    imu_lsm6ds3_bind(hspi1);
    imu_yaw_init(&s_yaw);
    st_t s = IMU_LSM6DS3.probe();
    if (s == ST_OK) { s = IMU_LSM6DS3.configure(); }
    if (s != ST_OK) { return s; }
    s_be = &IMU_LSM6DS3;
    s_rx_us = sys_time_us();
    s_kick_us = s_rx_us;
    s_init_us = s_rx_us;
    if (HAL_TIM_IC_Start_IT(htim2, TIM_CHANNEL_2) != HAL_OK) { return ST_BUS; }
    s_present = true;
    kick();                                               /* clears a DRDY that rose before the capture ran */
    return ST_OK;
}

/* ---- main side ------------------------------------------------------------------------ */
static void process(const raw_t *r, att_t *att, const par_t *q)
{
    if ((uint32_t)(r->t_us - s_init_us) < SETTLE_US) { return; }   /* gyro turn-on transient (D-67) */
    int16_t g[3], a[3], tr;
    s_be->decode(r->rx, g, a, &tr);
    /* D-77: single corrupted samples (+~300 deg/s on one axis, H7 log 2026-11-25) never reach att/yaw */
    if (imu_glitch_reject(&s_glitch, g, (int32_t)(q->imu_jump_max / s_be->gyro_lsb_radps))) { s_glitch_n++; return; }

    bool sat = false;
    for (uint32_t i = 0U; i < 3U; i++) { sat = sat || g[i] == INT16_MAX || g[i] == INT16_MIN; }
    if (sat) {
        s_sat_n++;
        if (++s_sat_run == SAT_RUN) { sys_fault_raise(F_IMU_SAT, (int32_t)s_sat_n); }
    } else {
        s_sat_run = 0U;
    }

    static const int8_t MAP[3][3] = IMU_AXIS_MAP;
    for (uint32_t i = 0U; i < 3U; i++) {
        s_w[i] = 0.0f;
        s_acc[i] = 0.0f;
        for (uint32_t j = 0U; j < 3U; j++) {
            s_w[i]   += (float)MAP[i][j] * (float)g[j] * s_be->gyro_lsb_radps;
            s_acc[i] += (float)MAP[i][j] * (float)a[j] * s_be->acc_lsb_mps2;
        }
    }
    s_temp_c = (float)tr * s_be->temp_lsb_c + s_be->temp_off_c;
    const float dT = s_temp_c - T_REF_C;
    s_w[2] = s_w[2] * q->imu_gz_scale - (q->imu_tc1 * dT + q->imu_tc2 * dT * dT);   /* REQ-IMU-03 */

    const float dt = s_have_prev ? (float)(uint32_t)(r->t_us - s_prev_us) * 1.0e-6f : 1.0f / s_be->odr_hz;
    s_prev_us = r->t_us;
    s_have_prev = true;
    att_update(att, s_w, s_acc, dt);

    const float phi = att_roll(att), th = att_pitch(att);
    const float rate = ((s_w[1] - att_bias_y(att)) * sinf(phi) + s_w[2] * cosf(phi)) / fmaxf(cosf(th), COS_MIN);
    if (!isfinite(rate)) { sys_fault_raise(F_NUMERIC, 0); return; }                  /* REQ-COD-10 */
    imu_yaw_push(&s_yaw, r->t_us, rate);
    s_n++;
    for (uint32_t k = 0U; k < 3U; k++) { s_sum_w[k] += s_w[k]; }
    s_sum_n++;
}

void imu_drain(att_t *att)
{
    if (!s_present) { return; }
    const par_t *q = par();
    const uint32_t h = s_head;
    __DMB();                                              /* slot contents after the head index */
    uint32_t t = s_tail;
    for (uint32_t n = 0U; t != h && n < RAW_N; n++, t++) { process(&s_raw[t % RAW_N], att, q); }
    s_tail = t;

    const uint32_t rx_us = s_rx_us;                       /* before now: a DRDY in between must not wrap */
    const uint32_t now = sys_time_us();
    const uint32_t to_us = (uint32_t)(q->imu_timeout * 1.0e6f);
    if ((uint32_t)(now - rx_us) > to_us) {              /* REQ-IMU-06 */
        if (!s_timed_out) {
            s_timed_out = true;
            sys_fault_raise_as(F_IMU_TIMEOUT, 0, R1);     /* R3 only in AUTO (§11), which comes with msn */
        }
        if ((uint32_t)(now - s_kick_us) > to_us) { s_kick_us = now; kick(); }
    } else {
        s_timed_out = false;
    }
}

st_t imu_yaw_integral_at(uint32_t t_us, float *psi_rad)
{
    return imu_yaw_at(&s_yaw, t_us, par()->imu_extrap_max, psi_rad);
}

bool imu_alive(void)
{
    const uint32_t rx_us = s_rx_us;                       /* before now, as in imu_drain */
    return s_present && s_rx_n > 0U && (uint32_t)(sys_time_us() - rx_us) <= (uint32_t)(par()->imu_timeout * 1.0e6f);
}

float imu_temp_c(void) { return s_temp_c; }

imu_info_t imu_info(void)
{
    imu_info_t i = { .present = s_present, .alive = imu_alive(), .temp_c = s_temp_c,
                     .n_samples = s_n, .n_drop = s_drop, .n_sat = s_sat_n,
                     .n_glitch = s_glitch_n };
    for (uint32_t k = 0U; k < 3U; k++) { i.w_radps[k] = s_w[k]; i.acc_mps2[k] = s_acc[k]; }
    return i;
}

uint32_t imu_take_mean(float w_radps[3], uint32_t *t_last_us, uint32_t *n_total)
{
    const uint32_t n = s_sum_n;
    for (uint32_t k = 0U; k < 3U; k++) {
        w_radps[k] = (n > 0U) ? s_sum_w[k] / (float)n : NAN;
        s_sum_w[k] = 0.0f;
    }
    s_sum_n = 0U;
    *t_last_us = s_prev_us;
    *n_total = s_n;
    return n;
}
