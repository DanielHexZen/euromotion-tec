/* app_isr.c — every HAL callback lives here and only dispatches (spec §16.2, REQ-MIG-02). */
#include "app_config.h"
#include "main.h"

#if APP_CALIB
#include "calib.h"
#else
#include "com_uart.h"
#include "sys_sched.h"
#if FEAT_IMU
#include "imu.h"
#endif
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

#if FEAT_IMU
/* IMU DRDY (PA1) on TIM2 CH2: the capture register holds the edge time, the ISR starts the SPI DMA. */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        imu_drdy_capture_isr(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2));
    }
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) { if (hspi->Instance == SPI1) { imu_dma_done_isr(); } }
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)    { if (hspi->Instance == SPI1) { imu_error_isr(); } }
#endif
#endif
