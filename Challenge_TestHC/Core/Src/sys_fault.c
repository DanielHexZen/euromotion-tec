/* sys_fault.c — spec §7.3, §11 (REQ-FLT-01..05). */
#include "sys_fault.h"

#include "main.h"
#include "sys_irq.h"

#define HARD_MAGIC 0xFA170000UL
#define SRAM_BEGIN 0x20000000UL
#define SRAM_END   0x20020000UL

/* Base reaction per code (§11). Modal rows (IMU/VL/FWD) take their stricter value; the
 * mode-dependent relaxation arrives with the modules that raise them. */
static const uint8_t BASE_REACTION[F_COUNT] = {
    [F_CLOCK] = R4, [F_HARDFAULT] = R4, [F_WDG_RESET] = R0,
    [F_STALL_L] = R3, [F_STALL_R] = R3, [F_RUNAWAY_L] = R4, [F_RUNAWAY_R] = R4,
    [F_TRACK_L] = R3, [F_TRACK_R] = R3, [F_BAT_LOW] = R2,
    [F_IMU_TIMEOUT] = R3, [F_IMU_SAT] = R0, [F_VL_BUS1] = R3, [F_VL_BUS2] = R3,
    [F_GND_STALE] = R2, [F_FWD_STALE] = R2, [F_GND_INTERLOCK] = R3,
    [F_EKF] = R3, [F_NUMERIC] = R4, [F_OVERRUN] = R0, [F_INIT] = R2, [F_MAP] = R0,
    [F_COM] = R0, [F_ASSERT] = R3, [F_CSS] = R4, [F_STACK] = R0, [F_PARAM] = R4,
};

static volatile uint32_t s_latched;          /* bit n = fault n latched                */
static uint8_t  s_reaction[F_COUNT];         /* reaction under which it was latched    */
static volatile uint16_t s_first;
static sys_event_rec_t  s_log[SYS_EVENT_LOG_N];
static volatile uint32_t s_log_n;            /* total events ever logged               */

void sys_fault_hard_stop(void)
{
    GPIOB->BSRR = ((uint32_t)EN_L_Pin | (uint32_t)EN_R_Pin) << 16U;
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    TIM1->CCR4 = 0U;
}

static void log_event(uint16_t code, int32_t arg)
{
    const uint32_t pm = sys_irq_save();
    sys_event_rec_t *e = &s_log[s_log_n % SYS_EVENT_LOG_N];
    e->t_ms = HAL_GetTick();
    e->code = code;
    e->arg  = arg;
    s_log_n++;
    sys_irq_restore(pm);
}

void sys_event(event_t ev, int32_t arg) { log_event((uint16_t)ev, arg); }

void sys_fault_raise_as(fault_t code, int32_t arg, reaction_t reaction)
{
    if ((uint32_t)code == 0U || (uint32_t)code >= (uint32_t)F_COUNT) { return; }
    if (reaction == R4) { sys_fault_hard_stop(); }            /* first, before any bookkeeping */
    log_event((uint16_t)code, arg);
    if (reaction >= R2) {
        const uint32_t bit = 1UL << (uint32_t)code;
        const uint32_t pm = sys_irq_save();
        if (s_latched == 0U) { s_first = (uint16_t)code; }
        if ((s_latched & bit) == 0U || reaction > (reaction_t)s_reaction[code]) {
            s_reaction[code] = (uint8_t)reaction;
        }
        s_latched |= bit;
        sys_irq_restore(pm);
    }
}

void sys_fault_raise(fault_t code, int32_t arg)
{
    if ((uint32_t)code == 0U || (uint32_t)code >= (uint32_t)F_COUNT) { return; }
    sys_fault_raise_as(code, arg, (reaction_t)BASE_REACTION[code]);
}

void sys_fault_clear(void)
{
    const uint32_t keep = (1UL << (uint32_t)F_CLOCK) | (1UL << (uint32_t)F_PARAM);
    const uint32_t pm = sys_irq_save();
    s_latched &= keep;
    if (s_latched == 0U) { s_first = 0U; }
    sys_irq_restore(pm);
}

bool sys_fault_active(fault_t code)
{
    return ((uint32_t)code < (uint32_t)F_COUNT) && ((s_latched & (1UL << (uint32_t)code)) != 0U);
}

reaction_t sys_fault_reaction(void)
{
    reaction_t r = R0;
    const uint32_t m = s_latched;
    for (uint32_t c = 1U; c < (uint32_t)F_COUNT; c++) {
        if (((m >> c) & 1U) != 0U && (reaction_t)s_reaction[c] > r) { r = (reaction_t)s_reaction[c]; }
    }
    return r;
}

uint16_t sys_fault_first(void) { return s_first; }
uint32_t sys_fault_mask(void)  { return s_latched; }

uint32_t sys_event_count(void) { return (s_log_n < SYS_EVENT_LOG_N) ? s_log_n : SYS_EVENT_LOG_N; }

bool sys_event_at(uint32_t i, sys_event_rec_t *out)
{
    const uint32_t n = sys_event_count();
    if (i >= n) { return false; }
    const uint32_t pm = sys_irq_save();
    *out = s_log[(s_log_n - n + i) % SYS_EVENT_LOG_N];
    sys_irq_restore(pm);
    return true;
}

/* Hard-fault record in RTC backup registers 0.. (D-19). Register access needs the PWR clock
 * and DBP; whether the RTC clock must also run on the F446 is open (spec §11, F_HARDFAULT)
 * [VERIFY] - H5 reads the record back after a deliberate fault. */
void sys_fault_record_hard(fault_t code, uint32_t msp)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    PWR->CR |= PWR_CR_DBP;
    volatile uint32_t *bkp = &RTC->BKP0R;
    bkp[0] = HARD_MAGIC | (uint32_t)code;
    bkp[1] = msp;
    const bool ok = (msp >= SRAM_BEGIN) && (msp + 4UL * SYS_HARD_WORDS <= SRAM_END) && ((msp & 3UL) == 0UL);
    for (uint32_t i = 0U; i < SYS_HARD_WORDS; i++) {
        bkp[2U + i] = ok ? ((const uint32_t *)msp)[i] : 0U;
    }
}

bool sys_fault_take_hard_record(uint32_t *code, uint32_t *msp, uint32_t words[SYS_HARD_WORDS])
{
    __HAL_RCC_PWR_CLK_ENABLE();
    PWR->CR |= PWR_CR_DBP;
    volatile uint32_t *bkp = &RTC->BKP0R;
    if ((bkp[0] & 0xFFFF0000UL) != HARD_MAGIC) { return false; }
    *code = bkp[0] & 0xFFFFU;
    *msp = bkp[1];
    for (uint32_t i = 0U; i < SYS_HARD_WORDS; i++) { words[i] = bkp[2U + i]; }
    bkp[0] = 0U;
    return true;
}
