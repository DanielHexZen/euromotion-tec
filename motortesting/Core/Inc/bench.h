/* bench.h — MR2006B drivetrain bench application
 *
 * A cyclic executive with the same harmonic rate structure the flight
 * firmware will use (CONCEPT §15.2), a serial console, and the test
 * sequences that produce the numbers §17.4 asks for.
 *
 * Deliberately absent, and this is not an omission:
 *   - no velocity PI loop      (layer 2 is commissioned in week 8)
 *   - no EKF, no path follower (§22 principle 12: one layer at a time)
 *   - no DMA                   (nothing here is timing-critical)
 * Week 5 is graded on "drives forward and turns in place under manual
 * duty". Everything beyond that is a way to fail two milestones at once.
 */
#ifndef BENCH_H
#define BENCH_H

#include "bench_config.h"

/* Call after all MX_*_Init(). Takes the peripheral handles it needs. */
void bench_init(TIM_HandleTypeDef *htim_pwm,
                TIM_HandleTypeDef *htim_enc_l,
                TIM_HandleTypeDef *htim_enc_r,
                TIM_HandleTypeDef *htim_timebase,
                TIM_HandleTypeDef *htim_tick,
                ADC_HandleTypeDef *hadc_batt,
                UART_HandleTypeDef *huart_console);

/* Never returns. */
void bench_run(void);

/* Call from HAL_TIM_PeriodElapsedCallback when htim is the 1 kHz tick. */
void bench_tick_isr(void);

/* Free-running 32-bit microsecond time base (§15.1). Available to
 * everything; the flight firmware timestamps every measurement from it. */
uint32_t bench_micros(void);

#endif /* BENCH_H */
