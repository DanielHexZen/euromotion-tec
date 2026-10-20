/* con.c — spec §7.23, §9 (REQ-CON-01..05). */
#include "con.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fmt.h"
#include "sys_fault.h"
#include "sys_time.h"

#define LINE_MAX_CH   48U
#define CHUNK_BYTES   200U           /* REQ-CON-01 */
#define OUT_LINE_MAX  96U

#define DIST_MIN_MM   5L
#define DIST_MAX_MM   20000L
#define V_NOM_MIN_MM  20L
#define RAMP_MIN_MS   100L
#define RAMP_MAX_MS   5000L
#define TURN_MAX_DEG  3600.0f

typedef struct {
    char     line[LINE_MAX_CH + 1U];
    uint8_t  len;
    bool     ready;
    uint32_t last_rx_ms;
    bool     autoprint;
    con_job_t job;
    uint32_t  job_i;
} port_t;

static port_t P[COM_N];

void con_rx_byte(com_port_t p, uint8_t b)
{
    port_t *c = &P[p];
    c->last_rx_ms = sys_ms();
    if (b == '\r' || b == '\n') {
        if (c->len > 0U && !c->ready) { c->line[c->len] = '\0'; c->len = 0U; c->ready = true; }
    } else if (!c->ready && c->len < LINE_MAX_CH) {
        c->line[c->len++] = (char)b;
    }
}

uint32_t con_last_rx_ms(com_port_t p) { return P[p].last_rx_ms; }
bool con_autoprint(com_port_t p) { return P[p].autoprint; }
bool con_autoprint_toggle(com_port_t p) { P[p].autoprint = !P[p].autoprint; return P[p].autoprint; }

void con_printf(com_port_t p, const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0) { (void)com_write(p, (const uint8_t *)buf, (size_t)((n < (int)sizeof buf) ? n : (int)sizeof buf - 1)); }
}

/* ---- chunked multi-line output (REQ-CON-01) ------------------------------------------ */
static const char *const HELP[] = {
    "D<mm> drive, V<mm/s> speed, T<ms>[ <ms>] ramps, W<deg> turn, -100..100 open loop %",
    "PWM <l%> <r%>, s or STOP stop, r reset counters (CLEAR in FAULT), p print, a auto, k params",
    "F[L|R]<Ks%> <Kv>, G<Kp> <Ki>, Y<Ksync>, P<id|name>=<v> set, P<id|name>? get, L dump",
    "S status, I info, EV events, FAULTS, CLEAR, MODE IDLE|BENCH",
};

/* Writes the line for item i into buf; returns its length, or -1 when the list is over. */
static int job_line(con_job_t j, uint32_t i, char *buf, size_t n)
{
    switch (j) {
    case CON_JOB_HELP:
        return (i < sizeof HELP / sizeof HELP[0]) ? snprintf(buf, n, "%s", HELP[i]) : -1;
    case CON_JOB_PARAMS: {                                   /* control parameters, IDs 100..199 */
        const par_desc_t *d = par_desc_at(i);
        if (d == NULL) { return -1; }
        if (d->id >= 200U) { return -1; }
        float v = 0.0f;
        (void)par_get(d->id, &v);
        char num[20];
        (void)fmt_sci(num, sizeof num, v);
        return snprintf(buf, n, "%u %s=%s %s", (unsigned)d->id, d->name, num, d->unit);
    }
    case CON_JOB_DUMP:
        return (i < par_count()) ? par_format_define(i, buf, n) : -1;
    case CON_JOB_EVLOG: {
        sys_event_rec_t e;
        if (!sys_event_at(i, &e)) { return -1; }
        return snprintf(buf, n, "%lu ms  0x%03X  %ld", (unsigned long)e.t_ms, (unsigned)e.code, (long)e.arg);
    }
    case CON_JOB_FAULTS: {
        const uint32_t m = sys_fault_mask();
        if (i == 0U) { return snprintf(buf, n, "latched mask 0x%08lX, first 0x%02X", (unsigned long)m, (unsigned)sys_fault_first()); }
        return (i < (uint32_t)F_COUNT) ? ((((m >> i) & 1U) != 0U) ? snprintf(buf, n, "active 0x%02lX", (unsigned long)i) : 0) : -1;
    }
    case CON_JOB_NONE:
    default:
        return -1;
    }
}

static void job_advance(com_port_t p)
{
    port_t *c = &P[p];
    size_t budget = CHUNK_BYTES;
    while (c->job != CON_JOB_NONE && budget > 0U) {
        char line[OUT_LINE_MAX];
        const int n = job_line(c->job, c->job_i, line, sizeof line);
        if (n < 0) { c->job = CON_JOB_NONE; break; }
        if (n > 0) {
            const size_t len = (size_t)n + 2U;
            if (len > com_tx_free(p)) { break; }              /* try again next pass */
            con_printf(p, "%s\r\n", line);
            budget = (len < budget) ? budget - len : 0U;
        }
        c->job_i++;
    }
}

void con_job(com_port_t p, con_job_t j)
{
    P[p].job = j;
    P[p].job_i = 0U;
}

/* ---- parsing -------------------------------------------------------------------------- */
static const char *sp(const char *s) { while (*s == ' ') { s++; } return s; }
static bool at_end(const char *s) { return *sp(s) == '\0'; }

static bool kw(const char *s, const char *word, const char **rest)
{
    const size_t n = strlen(word);
    if (strncmp(s, word, n) != 0 || (s[n] != '\0' && s[n] != ' ')) { return false; }
    *rest = s + n;
    return true;
}

static bool num_l(const char *s, long *v, const char **end)
{
    char *e;
    const long x = strtol(s, &e, 10);
    if (e == s) { return false; }
    *v = x; *end = e;
    return true;
}

static bool num_f(const char *s, float *v, const char **end)
{
    char *e;
    const float x = strtof(s, &e);
    if (e == s) { return false; }
    *v = x; *end = e;
    return true;
}

static void err_range(com_port_t p, long lo, long hi) { con_printf(p, "ERR range %ld..%ld\r\n", lo, hi); }
static void err_syntax(com_port_t p, const char *usage) { con_printf(p, "ERR syntax, use %s\r\n", usage); }

void con_err_state(com_port_t p, const char *mode) { con_printf(p, "ERR state %s\r\n", mode); }

void con_err_range_par(com_port_t p, const par_desc_t *d)
{
    char lo[20], hi[20];
    (void)fmt_sci(lo, sizeof lo, d->min);
    (void)fmt_sci(hi, sizeof hi, d->max);
    con_printf(p, "ERR range %s %s..%s %s (SI)\r\n", d->name, lo, hi, d->unit);
}

static bool noarg(con_cmd_t *c, con_cmd_type_t t, const char *rest)
{
    if (!at_end(rest)) { return false; }
    c->type = t;
    return true;
}

/* Fills *c from the line. Replies to malformed input itself. */
static bool parse(com_port_t p, const char *s, con_cmd_t *c)
{
    const char *r, *e;
    long a, b;
    float x, y;
    memset(c, 0, sizeof *c);
    c->port = p;
    s = sp(s);

    /* multi-letter keywords first */
    if (kw(s, "FAULTINJ", &r)) {
        if (!num_l(sp(r), &a, &e) || !at_end(e) || a < 1 || a >= (long)F_COUNT) { err_range(p, 1, (long)F_COUNT - 1L); return false; }
        c->type = CMD_FAULTINJ; c->f[0] = (float)a; return true;
    }
    if (kw(s, "FAULTS", &r) && at_end(r)) { c->type = CMD_FAULTS; return true; }
    if (kw(s, "CLEAR", &r) && at_end(r))  { c->type = CMD_CLEAR; return true; }
    if (kw(s, "STOP", &r) && at_end(r))   { c->type = CMD_STOP; return true; }
    if (kw(s, "RUN", &r) && at_end(r))    { c->type = CMD_UNAVAILABLE; return true; }
    if (kw(s, "EV", &r) && at_end(r))     { c->type = CMD_EVLOG; return true; }
    if (kw(s, "MODE", &r)) {
        r = sp(r);
        if (strcmp(r, "IDLE") == 0)  { c->type = CMD_MODE; c->which = 0U; return true; }
        if (strcmp(r, "BENCH") == 0) { c->type = CMD_MODE; c->which = 1U; return true; }
        if (strcmp(r, "MANUAL") == 0 || strcmp(r, "AUTO") == 0) { c->type = CMD_UNAVAILABLE; return true; }
        err_syntax(p, "MODE IDLE|BENCH|MANUAL|AUTO"); return false;
    }
    if (kw(s, "PWM", &r)) {
        if (!num_l(sp(r), &a, &e) || !num_l(sp(e), &b, &e) || !at_end(e)) { err_syntax(p, "PWM <l %> <r %>"); return false; }
        if (a < -100 || a > 100 || b < -100 || b > 100) { err_range(p, -100, 100); return false; }
        c->type = CMD_PWM; c->f[0] = (float)a / 100.0f; c->f[1] = (float)b / 100.0f; return true;
    }

    /* single-letter commands: no-argument ones must end right after the letter, so "IMU" is
     * unknown rather than 'I'. */
    const char k = s[0];
    r = s + 1;
    switch (k) {
    case '?': if (noarg(c, CMD_HELP, r))      { return true; } break;
    case 'k': if (noarg(c, CMD_PARAMS, r))    { return true; } break;
    case 's': if (noarg(c, CMD_STOP, r))      { return true; } break;
    case 'r': if (noarg(c, CMD_RESET, r))     { return true; } break;
    case 'p': if (noarg(c, CMD_PRINT, r))     { return true; } break;
    case 'a': if (noarg(c, CMD_AUTOPRINT, r)) { return true; } break;
    case 'L': if (noarg(c, CMD_DUMP, r))      { return true; } break;
    case 'S': if (noarg(c, CMD_STATUS, r))    { return true; } break;
    case 'I': if (noarg(c, CMD_INFO, r))      { return true; } break;
    case 'M': if (r[0] == '\0' || r[0] == ' ') { c->type = CMD_UNAVAILABLE; return true; } break;
    case 'D': case 'd':
        if (!num_l(r, &a, &e) || !at_end(e)) { err_syntax(p, "D<mm>"); return false; }
        if (labs(a) < DIST_MIN_MM || labs(a) > DIST_MAX_MM) { err_range(p, DIST_MIN_MM, DIST_MAX_MM); return false; }
        c->type = CMD_MOVE; c->f[0] = (float)a * 1.0e-3f; return true;
    case 'V': case 'v': {
        const long vmax = lroundf(par()->drv_v_max * 1000.0f);
        if (!num_l(r, &a, &e) || !at_end(e)) { err_syntax(p, "V<mm/s>"); return false; }
        if (a < V_NOM_MIN_MM || a > vmax) { err_range(p, V_NOM_MIN_MM, vmax); return false; }
        c->type = CMD_SPEED; c->f[0] = (float)a * 1.0e-3f; return true;
    }
    case 'T': case 't':
        if (!num_l(r, &a, &e)) { err_syntax(p, "T<ms>[ <ms>]"); return false; }
        b = a;
        e = sp(e);
        if (*e != '\0' && (!num_l(e, &b, &e) || !at_end(e))) { err_syntax(p, "T<ms>[ <ms>]"); return false; }
        if (a < RAMP_MIN_MS || a > RAMP_MAX_MS || b < RAMP_MIN_MS || b > RAMP_MAX_MS) { err_range(p, RAMP_MIN_MS, RAMP_MAX_MS); return false; }
        c->type = CMD_RAMP; c->f[0] = (float)a * 1.0e-3f; c->f[1] = (float)b * 1.0e-3f; return true;
    case 'F':
        if (*r == 'L') { c->which = 1U; r++; } else if (*r == 'R') { c->which = 2U; r++; }
        if (!num_f(r, &x, &e) || !num_f(sp(e), &y, &e) || !at_end(e)) { err_syntax(p, "F[L|R]<Ks %> <Kv %/(mm/s)>"); return false; }
        c->type = CMD_FF; c->f[0] = x * 0.01f; c->f[1] = y * 10.0f; return true;   /* % -> 1; %/(mm/s) -> 1/(m/s) */
    case 'G':
        if (!num_f(r, &x, &e) || !num_f(sp(e), &y, &e) || !at_end(e)) { err_syntax(p, "G<Kp %/(mm/s)> <Ki %/(mm/s*s)>"); return false; }
        c->type = CMD_GAIN; c->f[0] = x * 10.0f; c->f[1] = y * 10.0f; return true;
    case 'Y':
        if (!num_f(r, &x, &e) || !at_end(e)) { err_syntax(p, "Y<K_sync (mm/s)/mm>"); return false; }
        c->type = CMD_SYNC; c->f[0] = x; return true;        /* (mm/s)/mm = 1/s */
    case 'W':
        if (!num_f(r, &x, &e) || !at_end(e)) { err_syntax(p, "W<deg>"); return false; }
        if (fabsf(x) < 1.0f || fabsf(x) > TURN_MAX_DEG) { err_range(p, 1, (long)TURN_MAX_DEG); return false; }
        c->type = CMD_TURN; c->f[0] = x * DEG2RAD_F; return true;
    case 'P': {
        char name[24];
        size_t n = 0U;
        while (r[n] != '\0' && r[n] != '=' && r[n] != '?' && r[n] != ' ' && n < sizeof name - 1U) { name[n] = r[n]; n++; }
        name[n] = '\0';
        e = r + n;
        const bool is_id = (n > 0U) && (name[0] >= '0' && name[0] <= '9');
        const par_desc_t *d = is_id ? par_desc((uint16_t)atoi(name)) : par_find(name);
        if (n == 0U || (*e != '=' && *e != '?')) { err_syntax(p, "P<id|name>=<value> or P<id|name>?"); return false; }
        if (d == NULL) { con_printf(p, "ERR unknown parameter %s\r\n", name); return false; }
        c->par = d;
        if (*e == '?') { c->type = CMD_PGET; return at_end(e + 1) ? true : (err_syntax(p, "P<id|name>?"), false); }
        if (!num_f(e + 1, &x, &e) || !at_end(e)) { err_syntax(p, "P<id|name>=<value>"); return false; }
        c->type = CMD_PSET; c->f[0] = x; return true;
    }
    default:
        break;
    }
    if (k == '-' || k == '+' || (k >= '0' && k <= '9')) {
        if (!num_l(s, &a, &e) || !at_end(e)) { err_syntax(p, "-100..100"); return false; }
        if (a < -100 || a > 100) { err_range(p, -100, 100); return false; }
        c->type = CMD_DUTY; c->f[0] = (float)a / 100.0f; return true;
    }
    con_printf(p, "ERR unknown, ? for help\r\n");
    return false;
}

bool con_poll(con_cmd_t *cmd)
{
    for (uint32_t i = 0U; i < (uint32_t)COM_N; i++) { job_advance((com_port_t)i); }
    for (uint32_t i = 0U; i < (uint32_t)COM_N; i++) {
        port_t *c = &P[i];
        if (!c->ready) { continue; }
        char line[LINE_MAX_CH + 1U];
        memcpy(line, c->line, sizeof line);
        c->ready = false;
        con_printf((com_port_t)i, "> %s\r\n", line);                 /* echo: corrupted radio input is visible */
        if (parse((com_port_t)i, line, cmd)) { return true; }
    }
    return false;
}

/* ---- replies in legacy units ---------------------------------------------------------- */
static long mm(float m)  { return lroundf(m * 1000.0f); }

void con_print_status(com_port_t p, const con_status_t *s)
{
    char xl[16], xr[16];
    (void)fmt_fixed(xl, sizeof xl, s->x_m[0] * 1000.0f, 1);
    (void)fmt_fixed(xr, sizeof xr, s->x_m[1] * 1000.0f, 1);
    con_printf(p, "L=%ld (%s mm, %ld mm/s)  R=%ld (%s mm, %ld mm/s)  %s %s\r\n",
               (long)s->cnt[0], xl, mm(s->v_mps[0]), (long)s->cnt[1], xr, mm(s->v_mps[1]), s->mode, s->phase);
}

void con_print_move(com_port_t p, float dist_m, float v_mps, float t_s)
{
    con_printf(p, "Move %ld mm: Vmax %ld mm/s, T %ld ms\r\n", mm(dist_m), mm(v_mps), mm(t_s));
}

void con_print_done(com_port_t p, bool timeout, const float d_m[2], float sync_m, float t_s)
{
    char l[16], r[16], sy[16];
    (void)fmt_fixed(l, sizeof l, d_m[0] * 1000.0f, 1);
    (void)fmt_fixed(r, sizeof r, d_m[1] * 1000.0f, 1);
    (void)fmt_fixed(sy, sizeof sy, sync_m * 1000.0f, 1);
    con_printf(p, "%s L=%s R=%s sync_max=%s t=%ld\r\n", timeout ? "TIMEOUT" : "DONE", l, r, sy, mm(t_s));
}

void con_print_param(com_port_t p, const par_desc_t *d, float v)
{
    char num[20];
    (void)fmt_sci(num, sizeof num, v);
    con_printf(p, "P%u %s=%s %s\r\n", (unsigned)d->id, d->name, num, d->unit);
}
