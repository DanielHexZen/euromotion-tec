/* app.h — composition: init sequence, main loop, slots (spec §5.4, §5.8, §7.26). */
#ifndef APP_H
#define APP_H

#include "main.h"

/* Handles that exist in the current .ioc. Phase-tagged peripherals (SPI1, I2C1/2, ADC1) are
 * added when CubeMX generates them (spec §16.2). */
typedef struct {
    TIM_HandleTypeDef  *tim1, *tim2, *tim3, *tim6, *tim8;
    UART_HandleTypeDef *uart1, *uart2;
    void               *adc1;       /* ADC_HandleTypeDef*, NULL until CubeMX generates ADC1 */
} app_hw_t;

void app_init(const app_hw_t *hw);   /* once, from main() USER CODE 2; blocking allowed here */
void app_loop(void);                 /* forever, from main() USER CODE 3; never blocks       */

#endif
