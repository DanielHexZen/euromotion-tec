/* mot.h — BTS7960 actuation (spec §7.5). Device layer. */
#ifndef MOT_H
#define MOT_H

#include "main.h"
#include "units.h"

#define MOT_PWM_ARR 8999U       /* 180 MHz / 20 kHz - 1, set in CubeMX (§4.2) */
#define MOT_SIGN_L  (-1.0f)     /* REQ-MOT-04: harness, measured 8 Oct        */
#define MOT_SIGN_R  (1.0f)

/* Init only. EN stays low, CCR1..4 = 0, PWM started on all four channels.
 * ST_RANGE if TIM1 ARR is not 8999 (CubeMX not regenerated): nothing is started. */
st_t  mot_init(TIM_HandleTypeDef *htim1);
void  mot_enable(bool on);                  /* both EN lines                                */
/* Command for the next mot_apply(); wheel-forward positive. Non-finite -> F_NUMERIC, duty 0. */
void  mot_set(uint8_t side, float duty_d);
void  mot_apply(void);                      /* main loop; the single writer of CCR1..4 (REQ-MOT-01) */
float mot_duty(uint8_t side);               /* last applied, after limits, wheel-forward positive */

#endif
