/* sys_sched.c — spec §5.2, §5.3, §5.6, §7.2 (REQ-EXE-02, REQ-SCH-01, REQ-EXE-11). */
#include "sys_sched.h"

#include "enc.h"
#include "sys_irq.h"

typedef struct { uint16_t div; uint16_t phase; } slot_cfg_t;

static const slot_cfg_t SLOT[SLOT_N] = {          /* REQ-SCH-01 */
    [SLOT_FAST] = { 1U, 0U },
    [SLOT_CTRL] = { 20U, 1U },     /* one tick after an EKF step, REQ-EXE-03 */
    [SLOT_TLM]  = { 100U, 7U },
    [SLOT_SLOW] = { 1000U, 13U },
};

static volatile uint32_t s_tick;
static volatile uint32_t s_pending;
static volatile uint32_t s_overrun[SLOT_N];
static uint32_t s_max[SLOT_N], s_sum[SLOT_N], s_t0[SLOT_N];

st_t sys_sched_start(TIM_HandleTypeDef *htim6)
{
    if (htim6->Instance->PSC != 89U || htim6->Instance->ARR != 999U) { return ST_RANGE; }
    return (HAL_TIM_Base_Start_IT(htim6) == HAL_OK) ? ST_OK : ST_BUS;
}

void sys_sched_tick_isr(void)
{
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
}

uint32_t sys_sched_take(void)
{
    const uint32_t pm = sys_irq_save();
    const uint32_t p = s_pending;
    s_pending = 0U;
    sys_irq_restore(pm);
    return p;
}

uint32_t sys_sched_tick(void) { return s_tick; }

void sys_slot_begin(slot_id_t s)
{
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

void sys_sched_stats(sys_sched_stats_t *out)
{
    for (uint32_t i = 0U; i < (uint32_t)SLOT_N; i++) {
        out->overrun[i] = s_overrun[i];
        out->max_cycles[i] = s_max[i];
        out->sum_cycles[i] = s_sum[i];
        s_sum[i] = 0U;
    }
    out->ticks = s_tick;
}
