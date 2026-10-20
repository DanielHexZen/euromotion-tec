/* sys_time.c — spec §7.1 (REQ-TIM-01/02). */
#include "sys_time.h"

#define SYS_TIM2_PSC 89U     /* 90 MHz timer clock / 90 = 1 MHz */

static uint8_t s_reset_cause;

st_t sys_time_init(TIM_HandleTypeDef *htim2)
{
    const uint32_t csr = RCC->CSR;
    s_reset_cause = (uint8_t)(((csr & (RCC_CSR_PORRSTF | RCC_CSR_BORRSTF)) != 0U ? SYS_RST_POR_BOR : 0U)
                  | ((csr & RCC_CSR_PINRSTF)  != 0U ? SYS_RST_PIN  : 0U)
                  | ((csr & RCC_CSR_SFTRSTF)  != 0U ? SYS_RST_SOFT : 0U)
                  | ((csr & RCC_CSR_IWDGRSTF) != 0U ? SYS_RST_IWDG : 0U)
                  | ((csr & RCC_CSR_WWDGRSTF) != 0U ? SYS_RST_WWDG : 0U)
                  | ((csr & RCC_CSR_LPWRRSTF) != 0U ? SYS_RST_LPWR : 0U));
    __HAL_RCC_CLEAR_RESET_FLAGS();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    if (htim2->Instance->PSC != SYS_TIM2_PSC) { return ST_RANGE; }
    return (HAL_TIM_Base_Start(htim2) == HAL_OK) ? ST_OK : ST_BUS;
}

uint32_t sys_time_us(void) { return TIM2->CNT; }
uint32_t sys_cycles(void)  { return DWT->CYCCNT; }
uint32_t sys_ms(void)      { return HAL_GetTick(); }
uint8_t  sys_reset_cause(void) { return s_reset_cause; }
