/* app_priv.h — state shared by app.c and app_cmd.c (both are the `app` module, REQ-APP-01). */
#ifndef APP_PRIV_H
#define APP_PRIV_H

#include "console.h"
#include "drv.h"

/* Phase-0 modes. msn (phase 3) takes this over and adds MANUAL and AUTO. */
typedef enum { APP_IDLE = 0, APP_BENCH, APP_FAULT } app_mode_t;

typedef struct {
    drv_t       drv;
    app_mode_t  mode;
    bool        stopping;        /* BENCH: a stop was requested, leave BENCH once the drive rests */
    bool        sched_ok;
    com_port_t  ol_port;         /* port that owns the open-loop dead-man (REQ-CON-03)             */
    uint32_t    fault_t0_ms;
    uint32_t    tick_hz_x10;     /* measured, REQ-SCH-02                                           */
    uint32_t    cpu_permille;
    uint32_t    boot_status;     /* bit n = init step n failed                                     */
} app_state_t;

extern app_state_t g_app;

void        app_cmd_exec(const con_cmd_t *c);                 /* app_cmd.c */
void        app_print_status(com_port_t p);                   /* app_cmd.c */
void        app_enter_fault(reaction_t r);                    /* app.c     */
const char *app_mode_name(void);                              /* app.c     */
bool        app_clear_faults(void);                           /* app.c: true if now out of FAULT */

#endif
