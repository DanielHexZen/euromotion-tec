/* sys_sched.c — spec §5.2, §5.3, §5.6, §7.2 (REQ-EXE-02, REQ-SCH-01, REQ-EXE-11).
 *
 * Self-measurement for gate H2 (REQ-NFR-01, REQ-NFR-05), integer only:
 * - S_CTRL period: TIM2 (1 us) at every S_CTRL start. Not DWT->CYCCNT: the main loop sleeps in WFI
 *   between slots, and whether CYCCNT counts during sleep depends on the clock gating (and on an
 *   attached debugger keeping the core clock alive). TIM2 is a peripheral and runs in Sleep mode.
 *   1 us is ample against the 100 us jitter limit; the mean over n intervals resolves 1/n us.
 * - Tick-ISR time: DWT->CYCCNT at entry and exit (the CPU is awake there, and 2 us needs sub-us
 *   resolution). Exception entry/exit and the HAL dispatch in front of the callback are outside. */
#include "sys_sched.h"

#include "enc.h"
#include "sys_irq.h"

typedef struct { uint16_t div; uint16_t phase; } slot_cfg_t;

static const slot_cfg_t SLOT[SLOT_N] = {          /* REQ-SCH-01 */
    [SLOT_FAST] = { 1U, 0U },
    /* Phase 1 dates from the 200 Hz S_EST slot (control one tick after an EKF step, REQ-EXE-03).
     * The EKF moves into S_CTRL (D-30), so the phase no longer matters. It is kept: changing it
     * would shift the control instants for no gain. */
    [SLOT_CTRL] = { 20U, 1U },
    [SLOT_TLM]  = { 100U, 7U },
    [SLOT_SLOW] = { 1000U, 13U },
};

static volatile uint32_t s_tick;
static volatile uint32_t s_pending;
static volatile uint32_t s_overrun[SLOT_N];
static volatile uint32_t s_isr_max;      /* cycles; written by the ISR, zeroed by main (REQ-EXE-06 a) */
static uint32_t s_max[SLOT_N], s_sum[SLOT_N], s_t0[SLOT_N];
static sys_per_t s_ctrl_per;             /* main loop only */

st_t sys_sched_start(TIM_HandleTypeDef *htim6)
{
    if (htim6->Instance->PSC != 89U || htim6->Instance->ARR != 999U) { return ST_RANGE; }
    sys_per_init(&s_ctrl_per);
    return (HAL_TIM_Base_Start_IT(htim6) == HAL_OK) ? ST_OK : ST_BUS;
}

void sys_sched_tick_isr(void)
{
    const uint32_t c0 = DWT->CYCCNT;
    const uint32_t t = TIM2->CNT;
    enc_snapshot_isr(t);
    const uint32_t tk = s_tick + 1U;
    s_tick = tk;
    for (uint32_t i = 0U; i < (uint32_t)SLOT_N; i++) {
        if (tk >= SLOT[i].phase && ((tk - SLOT[i].phase) % SLOT[i].div) == 0U) {
            if ((s_pending & (1U << i)) != 0U) { s_overrun[i]++; }
            s_pending |= (1U << i);
        }
    }
    const uint32_t c = DWT->CYCCNT - c0;
    if (c > s_isr_max) { s_isr_max = c; }
}

uint32_t sys_sched_take(void)
{
    const uint32_t pm = sys_irq_save();
    const uint32_t p = s_pending;
    s_pending = 0U;
    sys_irq_restore(pm);
    return p;
}

bool sys_sched_pending(void) { return s_pending != 0U; }

uint32_t sys_sched_tick(void) { return s_tick; }

void sys_slot_begin(slot_id_t s)
{
    if (s == SLOT_CTRL) { sys_per_add(&s_ctrl_per, TIM2->CNT); }       /* REQ-NFR-05 */
    s_t0[s] = DWT->CYCCNT;
#if defined(PROBE_Pin) && defined(DEBUG)
    if (s == SLOT_CTRL) { PROBE_GPIO_Port->BSRR = PROBE_Pin; }
#endif
}

void sys_slot_end(slot_id_t s)
{
    const uint32_t c = DWT->CYCCNT - s_t0[s];
    if (c > s_max[s]) { s_max[s] = c; }
    s_sum[s] += c;
#if defined(PROBE_Pin) && defined(DEBUG)
    if (s == SLOT_CTRL) { PROBE_GPIO_Port->BSRR = (uint32_t)PROBE_Pin << 16; }
#endif
}

static void stats_copy(sys_sched_stats_t *out)
{
    for (uint32_t i = 0U; i < (uint32_t)SLOT_N; i++) {
        out->overrun[i] = s_overrun[i];
        out->max_cycles[i] = s_max[i];
        out->sum_cycles[i] = s_sum[i];
    }
    out->ticks = s_tick;
    out->ctrl_per_n = s_ctrl_per.n;
    out->ctrl_per_min_us = (s_ctrl_per.n > 0U) ? s_ctrl_per.min_us : 0U;
    out->ctrl_per_max_us = s_ctrl_per.max_us;
    out->ctrl_per_sum_us = s_ctrl_per.sum_us;
    out->isr_max_cycles = s_isr_max;
}

void sys_sched_stats_peek(sys_sched_stats_t *out) { stats_copy(out); }

void sys_sched_stats(sys_sched_stats_t *out)
{
    stats_copy(out);
    for (uint32_t i = 0U; i < (uint32_t)SLOT_N; i++) { s_sum[i] = 0U; }
}

void sys_sched_stats_reset(void)
{
    for (uint32_t i = 0U; i < (uint32_t)SLOT_N; i++) { s_max[i] = 0U; }
    sys_per_clear(&s_ctrl_per);
    s_isr_max = 0U;
}
