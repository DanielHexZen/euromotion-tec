/* sys_sched.h — 1 kHz tick and slot dispatch (spec §5, §7.2). */
#ifndef SYS_SCHED_H
#define SYS_SCHED_H

#include <stddef.h>
#include <stdio.h>

#include "main.h"
#include "units.h"

/* Slot bits. There is no estimation slot: the EKF will run inside S_CTRL (D-30). A slot nobody
 * services would count an overrun on every tick. */
typedef enum { SLOT_FAST = 0, SLOT_CTRL, SLOT_TLM, SLOT_SLOW, SLOT_N } slot_id_t;
#define S_FAST (1U << SLOT_FAST)   /* 1 kHz  */
#define S_CTRL (1U << SLOT_CTRL)   /* 50 Hz, phase 1 of 20 */
#define S_TLM  (1U << SLOT_TLM)    /* 10 Hz  */
#define S_SLOW (1U << SLOT_SLOW)   /* 1 Hz   */

/* Interval statistics of a periodic event, integer microseconds. Main loop only, no ISR access.
 * Kept in the header (no HAL in it) so the host tests can run the arithmetic. */
typedef struct {
    uint32_t t_prev_us;
    uint32_t skip;                   /* starts still to ignore, see sys_per_init()       */
    uint32_t n;                      /* intervals counted                                */
    uint32_t min_us, max_us;         /* valid when n > 0                                 */
    uint64_t sum_us;
} sys_per_t;

static inline void sys_per_clear(sys_per_t *s)
{
    s->n = 0U;
    s->min_us = UINT32_MAX;
    s->max_us = 0U;
    s->sum_us = 0U;
}

/* The first event after start is late by the boot printing, so it only arms the measurement:
 * the interval that ends at the second event is not counted, the one ending at the third is. */
static inline void sys_per_init(sys_per_t *s)
{
    sys_per_clear(s);
    s->skip = 2U;
}

/* t_us: free-running 32-bit microsecond stamp of the event; the subtraction survives its wrap. */
static inline void sys_per_add(sys_per_t *s, uint32_t t_us)
{
    const uint32_t d = t_us - s->t_prev_us;
    s->t_prev_us = t_us;
    if (s->skip > 0U) { s->skip--; return; }
    if (d < s->min_us) { s->min_us = d; }
    if (d > s->max_us) { s->max_us = d; }
    s->sum_us += d;
    s->n++;
}

typedef struct {
    uint32_t overrun[SLOT_N];        /* since boot, never reset                              */
    uint32_t max_cycles[SLOT_N];     /* since boot or sys_sched_stats_reset()                */
    uint32_t sum_cycles[SLOT_N];     /* since the previous sys_sched_stats()                 */
    uint32_t ticks;
    /* H2 self-measurement (REQ-EXE-11), since boot or sys_sched_stats_reset(): */
    uint32_t ctrl_per_n;             /* S_CTRL start-to-start intervals counted              */
    uint32_t ctrl_per_min_us;        /* 0 while ctrl_per_n == 0                              */
    uint32_t ctrl_per_max_us;
    uint64_t ctrl_per_sum_us;
    uint32_t isr_max_cycles;         /* sys_sched_tick_isr() body, CPU cycles (REQ-NFR-01)   */
} sys_sched_stats_t;

/* The timing line of `I` (REQ-NFR-05, REQ-NFR-01):
 *   S_CTRL period min 19996 max 20004 mean 20000.0 us n 45000; tick isr max 0.6 us
 * Integer only; mean and ISR time are truncated to 0.1 us. cpu_hz >= 1 MHz. Returns the snprintf
 * length; 78 characters with typical values, console lines hold 95. */
static inline int sys_sched_format_timing(const sys_sched_stats_t *st, uint32_t cpu_hz, char *buf, size_t n)
{
    const uint32_t cyc_per_us = cpu_hz / 1000000U;
    const uint64_t mean_x10 = (st->ctrl_per_n > 0U) ? (st->ctrl_per_sum_us * 10U) / st->ctrl_per_n : 0U;
    return snprintf(buf, n, "S_CTRL period min %lu max %lu mean %lu.%lu us n %lu; tick isr max %lu.%lu us",
                    (unsigned long)st->ctrl_per_min_us, (unsigned long)st->ctrl_per_max_us,
                    (unsigned long)(mean_x10 / 10U), (unsigned long)(mean_x10 % 10U), (unsigned long)st->ctrl_per_n,
                    (unsigned long)(st->isr_max_cycles / cyc_per_us),
                    (unsigned long)(((st->isr_max_cycles % cyc_per_us) * 10U) / cyc_per_us));
}

/* Init only. Verifies TIM6 PSC 89 / ARR 999 (ST_RANGE otherwise), then starts the interrupt. */
st_t     sys_sched_start(TIM_HandleTypeDef *htim6);
void     sys_sched_tick_isr(void);      /* ISR, called from HAL_TIM_PeriodElapsedCallback */
uint32_t sys_sched_take(void);          /* main: atomically fetch and clear the pending bits */
/* Main, with interrupts masked: true if a slot is pending. The sleep gate of app_loop(). */
bool     sys_sched_pending(void);
uint32_t sys_sched_tick(void);
/* Main, S_SLOW only: copies the statistics and restarts the per-second cycle sums. */
void     sys_sched_stats(sys_sched_stats_t *out);
/* Main: copies the statistics and leaves the cycle sums alone (console). */
void     sys_sched_stats_peek(sys_sched_stats_t *out);
/* Main: restarts the slot maxima, the S_CTRL period statistics and the tick-ISR maximum.
 * Overrun counters and the tick count are kept. */
void     sys_sched_stats_reset(void);

void     sys_slot_begin(slot_id_t s);
void     sys_slot_end(slot_id_t s);

#endif
