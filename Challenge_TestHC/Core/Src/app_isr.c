/* app_isr.c — every HAL callback lives here and only dispatches (spec §16.2, REQ-MIG-02). */
#include "app_config.h"
#include "main.h"

#if APP_CALIB
#include "calib.h"
#else
#include "com_uart.h"
#include "sys_sched.h"
#endif

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
#if APP_CALIB
        Calib_Tick();
#else
        sys_sched_tick_isr();
#endif
    }
}

#if !APP_CALIB
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { com_rx_cplt_isr(huart); }
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) { com_tx_cplt_isr(huart); }
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)  { com_error_isr(huart); }
#endif
