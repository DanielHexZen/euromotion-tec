/* sys_time.h — 1 us time base (TIM2), CPU cycle counter (DWT), reset cause (spec §7.1). */
#ifndef SYS_TIME_H
#define SYS_TIME_H

#include "main.h"
#include "units.h"

#define SYS_RST_POR_BOR 0x01U
#define SYS_RST_PIN     0x02U
#define SYS_RST_SOFT    0x04U
#define SYS_RST_IWDG    0x08U
#define SYS_RST_WWDG    0x10U
#define SYS_RST_LPWR    0x20U

/* Init only. Starts TIM2, enables the DWT, reads and clears the reset cause.
 * ST_RANGE if TIM2 PSC is not 89 (REQ-TIM-01). */
st_t     sys_time_init(TIM_HandleTypeDef *htim2);
uint32_t sys_time_us(void);          /* any context; TIM2->CNT, wraps every 71.6 min */
uint32_t sys_cycles(void);           /* any context; DWT->CYCCNT                      */
uint32_t sys_ms(void);               /* any context; HAL_GetTick()                    */
uint8_t  sys_reset_cause(void);      /* SYS_RST_* bit field, valid after sys_time_init */

#endif
