/* sys_irq.h — short critical sections (REQ-EXE-07). Only for copying <= 64 bytes. */
#ifndef SYS_IRQ_H
#define SYS_IRQ_H

#include "stm32f4xx.h"

static inline uint32_t sys_irq_save(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void sys_irq_restore(uint32_t primask) { __set_PRIMASK(primask); }

#endif
