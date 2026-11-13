/* calib.h — open-loop calibration firmware (APP_CALIB=1, spec §7.27). Device layer.
 * Harmless to include in the vehicle build: calib.c is empty there. */
#ifndef CALIB_H
#define CALIB_H

#include "main.h"

/* Init only. Starts PWM, encoders, TIM6 interrupt and both UART receivers; EN stays low. */
void Calib_Init(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim8,
                TIM_HandleTypeDef *htim6, UART_HandleTypeDef *huart2, UART_HandleTypeDef *huart1);
void Calib_Loop(void);   /* main loop, never blocks */
void Calib_Tick(void);   /* TIM6 ISR, 50 Hz: snapshots the encoders only */

#endif
