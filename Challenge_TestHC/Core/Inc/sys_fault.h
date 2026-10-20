/* sys_fault.h — fault codes, reactions, event log (spec §7.3, §11). Codes are
 * HAL-free so pure modules can name the faults they report. */
#ifndef SYS_FAULT_H
#define SYS_FAULT_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    F_NONE = 0x00,
    F_CLOCK = 0x01, F_HARDFAULT = 0x02, F_WDG_RESET = 0x03,
    F_STALL_L = 0x04, F_STALL_R = 0x05, F_RUNAWAY_L = 0x06, F_RUNAWAY_R = 0x07,
    F_TRACK_L = 0x08, F_TRACK_R = 0x09, F_BAT_LOW = 0x0A,
    F_IMU_TIMEOUT = 0x0B, F_IMU_SAT = 0x0C, F_VL_BUS1 = 0x0D, F_VL_BUS2 = 0x0E,
    F_GND_STALE = 0x0F, F_FWD_STALE = 0x10, F_GND_INTERLOCK = 0x11,
    F_EKF = 0x12, F_NUMERIC = 0x13, F_OVERRUN = 0x14, F_INIT = 0x15, F_MAP = 0x16,
    F_COM = 0x17, F_ASSERT = 0x18, F_CSS = 0x19, F_STACK = 0x1A, F_PARAM = 0x1B,
    F_COUNT = 0x1C
} fault_t;

typedef enum { R0 = 0, R1, R2, R3, R4 } reaction_t;   /* §11 */

typedef enum {
    EV_BOOT = 0x100, EV_PARAM, EV_BAT_WARN, EV_MODE, EV_DRV_DONE, EV_DRV_TIMEOUT, EV_DEADMAN
} event_t;

typedef struct { uint32_t t_ms; uint16_t code; int32_t arg; } sys_event_rec_t;

#define SYS_EVENT_LOG_N 64U

/* Any context. Logs, latches (reaction >= R2) and, for R4, stops the bridge at once. */
void        sys_fault_raise(fault_t code, int32_t arg);
/* Any context. Same, with an explicit reaction (escalation, e.g. REQ-EXE-12). */
void        sys_fault_raise_as(fault_t code, int32_t arg, reaction_t reaction);
/* Main loop. Drops latched faults; F_CLOCK and F_PARAM stay (boot stops). */
void        sys_fault_clear(void);
bool        sys_fault_active(fault_t code);
reaction_t  sys_fault_reaction(void);          /* highest latched reaction, R0 if none */
uint16_t    sys_fault_first(void);             /* first latched fault since CLEAR      */
uint32_t    sys_fault_mask(void);              /* bit n = fault code n latched         */
void        sys_event(event_t ev, int32_t arg);
uint32_t    sys_event_count(void);             /* entries held, <= SYS_EVENT_LOG_N     */
bool        sys_event_at(uint32_t i, sys_event_rec_t *out);   /* i = 0 oldest */
/* Register-level: EN low, CCR1..4 = 0. Safe from any ISR and fault handler (REQ-FLT-01). */
void        sys_fault_hard_stop(void);
/* Fault handlers: store the code, MSP and the first words of the stack in RTC backup
 * registers (D-19). The exception frame (r0-r3, r12, lr, pc, xpsr) is somewhere in those words:
 * the handler's own prologue shifts it, so the words are stored raw and read by eye. */
#define SYS_HARD_WORDS 12U
void        sys_fault_record_hard(fault_t code, uint32_t msp);
/* Init only: returns the previous hard-fault record and clears it. */
bool        sys_fault_take_hard_record(uint32_t *code, uint32_t *msp, uint32_t words[SYS_HARD_WORDS]);

#endif
