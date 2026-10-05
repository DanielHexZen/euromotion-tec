/* motor.h — actuation layer for the L298-style opto-isolated driver
 *
 * CONCEPT §5.2. The forbidden state (IN1 = IN2 = 1) is made
 * structurally unreachable: nothing outside this module ever writes the
 * direction GPIOs or the PWM compare registers. That is the whole point
 * of the module existing as a separate file.
 */
#ifndef MOTOR_H
#define MOTOR_H

#include "bench_config.h"

/* Bind the module to the PWM timer. Call once, after MX_TIM1_Init().
 * Starts both PWM channels at zero duty with both direction pins low. */
void  motor_init(TIM_HandleTypeDef *htim_pwm);

/* The single setter. u in [-1, +1]; sign is direction, magnitude is duty.
 *  - |u| below U_MIN_F is coerced to 0 (the driver cannot resolve it)
 *  - a direction reversal is never commanded directly: the channel is
 *    forced to coast for DIR_DEADTIME_MS first
 *  - IN1 and IN2 are never simultaneously high, in any transient */
void  motor_apply(motor_id_t id, float u);

void  motor_coast(motor_id_t id);
void  motor_coast_all(void);

/* Call from the 1 kHz slot. Runs the reversal dead-time counters. */
void  motor_service_1khz(void);

/* What is actually on the pins right now, after clamping and dead-time.
 * Telemetry must log this, not the requested value. */
float motor_get_applied(motor_id_t id);

/* ---- Bench-only: truth-table verification (§5.2 [RISK], §19.1) -------
 *
 * The vendor states the module reproduces L298 logic with discrete
 * gates. That claim is unverified. This function drives one channel's
 * logic inputs to an explicitly chosen state so the outputs can be
 * probed with a multimeter.
 *
 * PRECONDITIONS, all of them:
 *   - motors physically disconnected from the driver outputs
 *   - driver supply from a current-limited lab supply at 6 V, 100 mA
 *   - never called from the normal control path
 *
 * The (1,1) state is the one worth measuring and the one that can hurt.
 * The 100 mA limit is what makes measuring it safe.
 */
void  motor_logic_probe(motor_id_t id, bool in1, bool in2, float en_duty);

#endif /* MOTOR_H */
