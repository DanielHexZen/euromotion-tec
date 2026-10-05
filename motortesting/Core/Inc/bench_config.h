/* bench_config.h — MR2006B drivetrain bench firmware
 *
 * Week-5 milestone: "Chassis, motor, driver built and working".
 * Acceptance test (CONCEPT §19.2): drives forward and turns in place
 * under manual duty, open loop.
 *
 * Everything marked [VERIFY] must be checked against a datasheet,
 * the reference manual or a multimeter BEFORE it is relied on.
 *
 * Language: C, float discipline per TOOLCHAIN.md.
 * No doubles anywhere: every literal carries an 'f'.
 */
#ifndef BENCH_CONFIG_H
#define BENCH_CONFIG_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ======================================================================
 * 1. Clock tree
 * ==================================================================== */

/* [VERIFY] against the CubeMX clock configuration actually generated.
 * Assumed: HSE 8 MHz (ST-Link MCO) -> PLL -> SYSCLK 180 MHz,
 * AHB /1, APB1 /4 (45 MHz, timer clock 90 MHz), APB2 /2 (90 MHz,
 * timer clock 180 MHz). If the clock config differs, PWM_ARR and the
 * timebase prescaler below are wrong and every duty cycle and every
 * timestamp is wrong with them. */
#define APB2_TIM_CLK_HZ    180000000UL   /* TIM1, TIM8            */
#define APB1_TIM_CLK_HZ     90000000UL   /* TIM2, TIM3, TIM6      */

/* ======================================================================
 * 2. PWM (CONCEPT §5.2)
 * ==================================================================== */

/* 5 kHz - changed with the move to the L298N (week 5).
 *
 * The L298 is a bipolar Darlington bridge, not a MOSFET bridge. Its
 * switching times are microseconds, so every transition costs real
 * energy in a part that is already thermally marginal (see the current
 * note below). Halving the carrier halves the switching loss, and it
 * doubles the duty resolution, which sec.5.2 wanted anyway.
 *
 * Cost: audible whine, which is not graded. */
#define PWM_FREQ_HZ             5000UL
#define PWM_PERIOD_TICKS       (APB2_TIM_CLK_HZ / PWM_FREQ_HZ)   /* 36000 */
#define PWM_ARR                (PWM_PERIOD_TICKS - 1UL)          /* 35999 */

/* The 5-10 us minimum pulse width was a property of the optocoupled
 * module, which is no longer fitted. The L298N has no comparable floor,
 * so this drops to a nominal value that only suppresses meaningless
 * sub-percent commands.
 * [VERIFY] scope the output at 2 %, 5 %, 10 % and confirm. */
#define U_MIN_F                 0.02f
#define U_MAX_F                 1.00f

/* Duty slew rate, in units of full scale per second. 2.0 => 0 to 100 %
 * in 500 ms. Prevents current spikes and keeps the drivetrain from
 * jerking through its own backlash on every command. */
#define SLEW_PER_S_F            2.0f

/* Enforced coast time on every direction reversal. The bridge is never
 * commanded from forward to reverse directly. */
#define DIR_DEADTIME_MS           50U

/* ======================================================================
 * 3. Encoders (CONCEPT §5.3)
 * ==================================================================== */

#define ENC_CPR_MOTOR_F         64.0f    /* 16 PPR/channel x4 quadrature */

/* MEASURED, week 5 — mean of three output-shaft revolutions per motor.
 * Catalogue nominal is 56.3:1 -> 3603 counts. Both measured values land
 * within half a percent of it, and within 0.64 % of each other.
 *
 * The consequence worth recording: the 12 % speed difference between the
 * two motors is therefore NOT gearing. It is motor constant and friction
 * (§12.9). The gearboxes are effectively identical.
 *
 * Per-motor constants rather than one shared value, because 0.64 % sits
 * in the same order as the eps ~= 0.5 % scale error of the error budget
 * (§18.1) — over a 40 m segment that is 26 cm of along-track error. */
#define ENC_CPR_OUTPUT_L_F    3595.0f   /* -> gear ratio 56.17 */
#define ENC_CPR_OUTPUT_R_F    3618.0f   /* -> gear ratio 56.53 */

/* Nominal, kept for reference and for the hand-turn readout's expectation
 * line. Not used in any velocity computation. */
#define ENC_CPR_OUTPUT_NOM_F    3603.0f

/* Sign convention. Set these so that a POSITIVE u produces a POSITIVE
 * count on both motors. Determined by the DIRCHECK sequence ('1'), not
 * by guessing which way round the encoder wires went in.
 * Legal values: +1 or -1. */
#define ENC_SIGN_L               (+1)
#define ENC_SIGN_R               (-1)   /* mirrored motor -> usually -1 */

/* Velocity: M-method, delta over a 100 ms boxcar (20 samples at 200 Hz).
 * §5.4 — at 0.4 m/s a 5 ms window holds ~19 counts, so the boxcar is
 * what makes the bench readout stable enough to fit Kv against. */
#define VEL_BOXCAR_N              20U

/* ---- Open-loop feedforward, per motor (§5.5) -------------------------
 *
 * u_ff = Kv * |rpm| + Ks, sign applied afterwards.
 *
 * MEASURED, week 5, two sweeps per motor, linear fit over nine duty
 * steps from 20 % to 100 %. R^2 > 0.999 on both, so the linear model of
 * §5.5 is confirmed, not assumed.
 *
 * Ks here is the INTERCEPT of the running-speed fit, not the breakaway
 * duty from the deadband sweep. Those differ by ~0.03 on both motors
 * because static friction exceeds kinetic friction. The feedforward has
 * to hit the running case; the integrator (week 8) covers starting from
 * rest. Breakaway for the record: L 0.141, R 0.169.
 *
 * The gear ratio has since been measured (0.2-0.4 % off nominal), which
 * shifts Kv by the same fraction - well inside the scatter of the sweeps
 * themselves, so the constants below stand as measured.
 *
 * [VERIFY] Re-measure after the wheels are fitted and after any change to
 * supply voltage. */
#define KV_L_F                0.00492f   /* duty per rpm, output shaft */
#define KS_L_F                0.110f
#define KV_R_F                0.00507f
#define KS_R_F                0.127f

/* Full-scale manual command, in output-shaft rpm. 170 sits just under
 * the measured no-load speed so 100 % command stays achievable as the
 * supply sags. */
#define RPM_MAX_CMD_F         170.0f

/* Only for the m/s display; the decision itself is gated on the ramp lip
 * measurement (§12.5). Wrong value here changes nothing but the readout. */
#define WHEEL_DIA_M_F           0.120f

/* ======================================================================
 * 4. Battery sensing (CONCEPT §13.4)
 * ==================================================================== */

/* Divider 47k / 10k. Use 1 % metal film, then no calibration is needed
 * (§14.6). With 5 % parts, measure the real ratio once and put it here. */
#define BATT_R_TOP_F         47000.0f
#define BATT_R_BOT_F         10000.0f
#define ADC_VREF_MV_F         3300.0f
#define ADC_FULL_SCALE_F      4095.0f

#define V_BATT_WARN_MV          10500   /* telemetry warning  (§13.2) */
#define V_BATT_CUTOFF_MV        10000   /* soft stop          (§13.2) */

/* Bench work runs off the lab supply, not the pack (§13.2). Battery
 * supervision is then meaningless and is toggled off with 'b'. */
#define BATT_SUPERVISION_DEFAULT   false

/* ======================================================================
 * 5. Safety supervisor
 * ==================================================================== */

/* Stall detection from encoders, not current (§13.4): setpoint non-zero,
 * encoder not counting. Measures the actual condition, not a proxy. */
/* Stall detection is now THERMAL PROTECTION, not only diagnosis.
 *
 * The L298N is rated 2 A continuous per channel; the motors draw 4.3 A
 * at stall. There is no current headroom at all - the previous module
 * had a factor of 1.6. A stalled motor will cook this bridge, so the
 * detection window is shortened.
 *
 * [MEASURE] STALL_U_THRESH_F must sit above the largest breakaway duty,
 * which changes with the driver. 0.25 is carried over provisionally and
 * must be re-set after a new deadband sweep. */
#define STALL_U_THRESH_F        0.25f
#define STALL_RPM_THRESH_F      5.0f
#define STALL_TIME_MS            150U

/* Dead-man switch for manual duty. §15.4 uses 500 ms in flight; on the
 * bench 2 s is enough and less annoying while typing. */
#define CMD_TIMEOUT_MS          2000U

/* ======================================================================
 * 6. Pin and peripheral map
 * ======================================================================
 *
 * NOTE ON TIM8 — this is not arbitrary.
 * TIM1_CH1/CH2 exist only on PA8/PA9 in LQFP64, and PA9/PA10 are also
 * USART1's default pins. The remaining USART1 mapping is PB6/PB7, which
 * is also the only TIM4 encoder mapping. Putting the second encoder on
 * TIM4 would therefore kill USART1 outright — and §15.3 needs USART1 for
 * a forward TF-Luna. Second encoder goes on TIM8 (PC6/PC7) instead.
 * Put this in the pin reservation table in TOOLCHAIN.md before the first
 * CubeMX session, not after.
 *
 * | Function              | Pin       | Peripheral        |
 * |-----------------------|-----------|-------------------|
 * | Motor L enable (PWM)  | PA8  (D7) | TIM1_CH1          |
 * | Motor R enable (PWM)  | PA9  (D8) | TIM1_CH2          |
 * | Motor L IN1           | PB12      | GPIO out          |
 * | Motor L IN2           | PB13      | GPIO out          |
 * | Motor R IN1           | PB14      | GPIO out          |
 * | Motor R IN2           | PB15      | GPIO out          |
 * | Encoder L A/B         | PA6/PA7   | TIM3_CH1/CH2      |
 * | Encoder R A/B         | PC6/PC7   | TIM8_CH1/CH2      |
 * | Battery divider       | PA0  (A0) | ADC1_IN0          |
 * | Global time base      | -         | TIM2, 32 bit, 1MHz|
 * | Cyclic executive tick | -         | TIM6, 1 kHz       |
 * | Console               | PA2/PA3   | USART2 (ST-Link)  |
 * | Slot load probe       | PC8       | GPIO out          |
 *
 * NVIC, both required:
 *   TIM6 global interrupt   preemption priority 1
 *   USART2 global interrupt preemption priority 3
 * Without the USART2 interrupt, HAL_UART_Transmit_IT never completes,
 * gState stays BUSY_TX forever and the console goes permanently silent
 * after the first call. Without the TIM6 interrupt nothing runs at all.
 *
 * TIM6: PSC 8999, ARR 9 -> 90 MHz / 9000 / 10 = 1 kHz. CubeMX defaults
 * ARR to 65535 and restores it on every regeneration; check it.
 */

#define M_L_IN1_PORT   GPIOB
#define M_L_IN1_PIN    GPIO_PIN_12
#define M_L_IN2_PORT   GPIOB
#define M_L_IN2_PIN    GPIO_PIN_13
#define M_R_IN1_PORT   GPIOB
#define M_R_IN1_PIN    GPIO_PIN_14
#define M_R_IN2_PORT   GPIOB
#define M_R_IN2_PIN    GPIO_PIN_15

#define M_L_PWM_CHANNEL   TIM_CHANNEL_1
#define M_R_PWM_CHANNEL   TIM_CHANNEL_2

/* §15.2: CPU load meter, one pin. Moved off PA5 (LD1) because PA5 is
 * SPI1_SCK, and the LED's series resistor and capacitance smear the
 * edges of exactly the signal we want to time. PC8 is free and sits on
 * CN10 where a probe reaches it. */
#define LOAD_PROBE_PORT   GPIOC
#define LOAD_PROBE_PIN    GPIO_PIN_8

/* ======================================================================
 * 7. Types shared across the bench modules
 * ==================================================================== */

typedef enum {
    MOTOR_L = 0,
    MOTOR_R = 1,
    MOTOR_COUNT = 2
} motor_id_t;

#endif /* BENCH_CONFIG_H */
