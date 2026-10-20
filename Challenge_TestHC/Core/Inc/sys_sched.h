/* sys_sched.h — 1 kHz tick and slot dispatch (spec §5, §7.2). */
#ifndef SYS_SCHED_H
#define SYS_SCHED_H

#include "main.h"
#include "units.h"

/* Slot bits. S_EST (200 Hz, phase 2) is added when the EKF exists; a slot nobody services
 * would count an overrun on every tick. */
typedef enum { SLOT_FAST = 0, SLOT_CTRL, SLOT_TLM, SLOT_SLOW, SLOT_N } slot_id_t;
#define S_FAST (1U << SLOT_FAST)   /* 1 kHz  */
#define S_CTRL (1U << SLOT_CTRL)   /* 50 Hz, phase 1 of 20 */
#define S_TLM  (1U << SLOT_TLM)    /* 10 Hz  */
#define S_SLOW (1U << SLOT_SLOW)   /* 1 Hz   */

typedef struct {
    uint32_t overrun[SLOT_N];
    uint32_t max_cycles[SLOT_N];     /* since start                          */
    uint32_t sum_cycles[SLOT_N];     /* since the previous sys_sched_stats() */
    uint32_t ticks;
} sys_sched_stats_t;

/* Init only. Verifies TIM6 PSC 89 / ARR 999 (ST_RANGE otherwise), then starts the interrupt. */
st_t     sys_sched_start(TIM_HandleTypeDef *htim6);
void     sys_sched_tick_isr(void);      /* ISR, called from HAL_TIM_PeriodElapsedCallback */
uint32_t sys_sched_take(void);          /* main: atomically fetch and clear the pending bits */
uint32_t sys_sched_tick(void);
/* Main: copies the statistics and restarts the per-second cycle sums. */
void     sys_sched_stats(sys_sched_stats_t *out);

void     sys_slot_begin(slot_id_t s);
void     sys_slot_end(slot_id_t s);

#endif
