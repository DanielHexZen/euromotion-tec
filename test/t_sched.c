/* H2 self-measurement on the host: the interval statistics and the `I` timing line of sys_sched.h.
 * The scheduler itself (sys_sched.c) needs the target; only its header arithmetic is tested here. */
#include <string.h>

#include "sys_sched.h"
#include "test.h"

#define CPU_HZ   180000000U
#define LINE_MAX 95U                /* console.c OUT_LINE_MAX - 1; the radio TX buffer holds 128 */

static sys_sched_stats_t stats_of(const sys_per_t *p, uint32_t isr_cycles)
{
    sys_sched_stats_t st = { 0 };
    st.ctrl_per_n = p->n;
    st.ctrl_per_min_us = (p->n > 0U) ? p->min_us : 0U;
    st.ctrl_per_max_us = p->max_us;
    st.ctrl_per_sum_us = p->sum_us;
    st.isr_max_cycles = isr_cycles;
    return st;
}

void t_sched(void)
{
    char buf[128];
    sys_per_t p;
    memset(&p, 0, sizeof p);

    REQ("REQ-NFR-05: S_CTRL period statistics skip the late first start");
    sys_per_init(&p);
    sys_per_add(&p, 4200U);                     /* first start, 3.2 ms late after the boot printing */
    CHECK(p.n == 0U);
    sys_per_add(&p, 21000U);                    /* second start on time: the 16.8 ms interval is dropped */
    CHECK(p.n == 0U);
    sys_sched_stats_t st = stats_of(&p, 0U);
    CHECK(st.ctrl_per_min_us == 0U && st.ctrl_per_max_us == 0U);
    (void)sys_sched_format_timing(&st, CPU_HZ, buf, sizeof buf);
    CHECK(strcmp(buf, "S_CTRL period min 0 max 0 mean 0.0 us n 0; tick isr max 0.0 us") == 0);

    REQ("REQ-NFR-05: min, max and mean of the S_CTRL period in us");
    sys_per_add(&p, 41000U);                    /* 20000 */
    sys_per_add(&p, 60996U);                    /* 19996 */
    sys_per_add(&p, 81003U);                    /* 20007 */
    sys_per_add(&p, 101000U);                   /* 19997 */
    CHECK(p.n == 4U && p.min_us == 19996U && p.max_us == 20007U && p.sum_us == 80000U);
    sys_per_add(&p, 121003U);                   /* 20003: sum 100003 over 5 -> 20000.6 */
    st = stats_of(&p, 0U);
    (void)sys_sched_format_timing(&st, CPU_HZ, buf, sizeof buf);
    CHECK(strstr(buf, "min 19996 max 20007 mean 20000.6 us n 5;") != NULL);

    REQ("REQ-NFR-05: the 32-bit microsecond counter may wrap between two starts");
    sys_per_init(&p);
    sys_per_add(&p, 0xFFFFB000U);
    sys_per_add(&p, 0xFFFFB000U + 20000U);
    sys_per_add(&p, 0xFFFFB000U + 40000U);      /* wraps to 0x00004C40 */
    CHECK(p.n == 1U && p.min_us == 20000U && p.max_us == 20000U);

    REQ("REQ-NFR-05: a reset restarts the statistics without losing an interval; the sum does not overflow");
    sys_per_init(&p);
    uint32_t t = 0U;
    sys_per_add(&p, t);
    sys_per_add(&p, t += 20000U);
    sys_per_add(&p, t += 20950U);               /* outlier before the reset */
    sys_per_clear(&p);
    CHECK(p.n == 0U);
    for (uint32_t i = 0U; i < 250000U; i++) { sys_per_add(&p, t += 20000U); }   /* 83 min > 2^32 us */
    CHECK(p.n == 250000U && p.min_us == 20000U && p.max_us == 20000U);
    CHECK(p.sum_us == 5000000000ULL);
    st = stats_of(&p, 0U);
    (void)sys_sched_format_timing(&st, CPU_HZ, buf, sizeof buf);
    CHECK(strstr(buf, "mean 20000.0 us n 250000;") != NULL);

    REQ("REQ-NFR-01: tick-ISR maximum, CPU cycles to 0.1 us, truncated");
    st = stats_of(&p, 108U);                    /* 0.6 us at 180 MHz */
    (void)sys_sched_format_timing(&st, CPU_HZ, buf, sizeof buf);
    CHECK(strstr(buf, "; tick isr max 0.6 us") != NULL);
    st.isr_max_cycles = 360U;                   /* the 2 us limit exactly */
    (void)sys_sched_format_timing(&st, CPU_HZ, buf, sizeof buf);
    CHECK(strstr(buf, "; tick isr max 2.0 us") != NULL);
    st.isr_max_cycles = 377U;                   /* 2.094 us */
    (void)sys_sched_format_timing(&st, CPU_HZ, buf, sizeof buf);
    CHECK(strstr(buf, "; tick isr max 2.0 us") != NULL);
    st.isr_max_cycles = 378U;                   /* 2.1 us */
    (void)sys_sched_format_timing(&st, CPU_HZ, buf, sizeof buf);
    CHECK(strstr(buf, "; tick isr max 2.1 us") != NULL);

    REQ("REQ-EXE-11: the `I` timing line fits a console line and the radio buffer");
    memset(&st, 0, sizeof st);
    st.ctrl_per_n = 45000U; st.ctrl_per_min_us = 19996U; st.ctrl_per_max_us = 20004U;
    st.ctrl_per_sum_us = 900000000ULL; st.isr_max_cycles = 108U;
    int n = sys_sched_format_timing(&st, CPU_HZ, buf, sizeof buf);
    CHECK(strcmp(buf, "S_CTRL period min 19996 max 20004 mean 20000.0 us n 45000; tick isr max 0.6 us") == 0);
    CHECK(n == 78);
    st.ctrl_per_n = 99999999U;                  /* 23 days at 50 Hz, a 1 s stall, a 100 us ISR */
    st.ctrl_per_min_us = 1000000U; st.ctrl_per_max_us = 1000000U;
    st.ctrl_per_sum_us = 99999999000000ULL; st.isr_max_cycles = 18000U;
    n = sys_sched_format_timing(&st, CPU_HZ, buf, sizeof buf);
    CHECK(n > 0 && (unsigned)n <= LINE_MAX);
    memset(&st, 0xFF, sizeof st);               /* every field at its maximum: longer than a line, */
    char small[96];                             /* cut by snprintf, never written past the buffer  */
    n = sys_sched_format_timing(&st, CPU_HZ, small, sizeof small);
    CHECK(n > 0 && strlen(small) == sizeof small - 1U);
}
