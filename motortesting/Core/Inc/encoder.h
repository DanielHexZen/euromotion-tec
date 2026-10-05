/* encoder.h — quadrature acquisition, CONCEPT §5.3 / §5.4
 *
 * Timers run in hardware encoder mode (TI12, x4). Zero CPU load, no
 * missed pulses, direction from phase. We only ever read CNT.
 *
 * ARR is 0xFFFF on both timers and the counters are allowed to wrap
 * freely; the signed 16-bit difference recovers the delta correctly as
 * long as we sample faster than half a full period, which at 1 kHz we
 * do by four orders of magnitude.
 */
#ifndef ENCODER_H
#define ENCODER_H

#include "bench_config.h"

void encoder_init(TIM_HandleTypeDef *htim_left, TIM_HandleTypeDef *htim_right);

/* 1 kHz slot: read CNT, accumulate the signed delta. */
void encoder_sample_1khz(void);

/* 200 Hz slot: update the boxcar velocity estimate. */
void encoder_update_200hz(void);

/* Accumulated output-shaft counts since the last reset, sign-corrected. */
int32_t encoder_count(motor_id_t id);

/* Zero the accumulators. Used before the one-revolution hand-turn check. */
void    encoder_reset(void);

/* Output shaft speed, rpm. Boxcar over VEL_BOXCAR_N samples (100 ms). */
float   encoder_rpm_out(motor_id_t id);

/* Wheel surface speed, m/s. Display only — depends on WHEEL_DIA_M_F. */
float   encoder_speed_mps(motor_id_t id);

/* Measured counts per output revolution for this motor. */
float   encoder_cpr(motor_id_t id);

#endif /* ENCODER_H */
