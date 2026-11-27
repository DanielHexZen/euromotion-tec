/* imu.h — inertial sensor front end (spec §7.11). Device layer. The part-specific code is in
 * the backend (imu_lsm6ds3.c); a different part needs only a new backend. */
#ifndef IMU_H
#define IMU_H

#include "att.h"
#include "main.h"
#include "units.h"

typedef struct {
    st_t (*probe)(void);                       /* WHO_AM_I                        */
    st_t (*configure)(void);                   /* ODR, ranges, filters, DRDY      */
    /* From the DRDY ISR, CS is already low. rx/tx hold n bytes; the front end raises CS when the
     * transfer completes (imu_dma_done_isr) and must be told if it cannot start (imu_error_isr). */
    void (*start_read_dma)(uint8_t *rx, uint8_t *tx, uint16_t n);
    void (*decode)(const uint8_t *rx, int16_t g[3], int16_t a[3], int16_t *t);
    float gyro_lsb_radps, acc_lsb_mps2, temp_lsb_c, temp_off_c;
    float odr_hz;
} imu_backend_t;

#define IMU_READ_BYTES 15U    /* address byte + 14 data bytes, the backend decodes them */

/* REQ-IMU-02: body axis i = sum_j MAP[i][j] * sensor axis j; body x forward, y left, z up.
 * [MEASURE] after mounting: a CCW turn must give positive w_z, nose down positive pitch. */
#define IMU_AXIS_MAP { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } }

st_t  imu_init(SPI_HandleTypeDef *hspi1, TIM_HandleTypeDef *htim2);   /* init only, may block */
void  imu_drdy_capture_isr(uint32_t t_capture_us);  /* TIM2 CC2 ISR: start DMA              */
void  imu_dma_done_isr(void);                       /* SPI RX DMA complete: push to the ring */
void  imu_error_isr(void);                          /* SPI/DMA error or start failure        */
void  imu_drain(att_t *att);                        /* S_FAST: convert, att, yaw integral    */
st_t  imu_yaw_integral_at(uint32_t t_us, float *psi_rad);   /* Psi up to t (§7.11.3)         */
bool  imu_alive(void);                              /* sample within imu_timeout             */
float imu_temp_c(void);

typedef struct {
    bool     present;                               /* probe and configuration passed        */
    bool     alive;
    float    w_radps[3], acc_mps2[3];               /* last sample, body axes                */
    float    temp_c;
    uint32_t n_samples, n_drop, n_sat, n_glitch, n_temp_rej;   /* gyro glitches D-77, temperature rejections D-81 */
    uint32_t spi_div;                               /* applied SPI1 prescaler (D-82)         */
} imu_info_t;
imu_info_t imu_info(void);                          /* console IMU                           */
/* Mean body rate of the samples since the previous call, time of the last one and the total
 * sample count (console IMUS, H7). Returns the number of samples in the window. Main loop only. */
uint32_t imu_take_mean(float w_radps[3], uint32_t *t_last_us, uint32_t *n_total);

#endif
