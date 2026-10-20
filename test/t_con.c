/* Console parser and chunked output on the host. com_uart / sys_* are stubbed below. */
#include <string.h>

#include "con.h"
#include "sys_fault.h"
#include "sys_time.h"
#include "test.h"

static char   g_out[2][8192];
static size_t g_out_n[2];
static size_t g_free = 4096;

size_t com_write(com_port_t p, const uint8_t *d, size_t n)
{
    if (g_out_n[p] + n < sizeof g_out[p]) { memcpy(&g_out[p][g_out_n[p]], d, n); g_out_n[p] += n; g_out[p][g_out_n[p]] = '\0'; }
    return n;
}
size_t   com_tx_free(com_port_t p) { (void)p; return g_free; }
uint32_t sys_ms(void) { return 1234U; }
bool     sys_event_at(uint32_t i, sys_event_rec_t *o) { (void)i; (void)o; return false; }
uint32_t sys_fault_mask(void) { return 0U; }
uint16_t sys_fault_first(void) { return 0U; }

static void feed(com_port_t p, const char *s) { while (*s != '\0') { con_rx_byte(p, (uint8_t)*s++); } }
static void clear_out(void) { memset(g_out_n, 0, sizeof g_out_n); g_out[0][0] = g_out[1][0] = '\0'; }

/* Sends one line to the USB port; returns the parsed command type (CMD_NONE if rejected). */
static con_cmd_t line(const char *s)
{
    con_cmd_t c = { 0 };
    clear_out();
    feed(COM_USB, s);
    feed(COM_USB, "\r");
    if (!con_poll(&c)) { c.type = CMD_NONE; }
    return c;
}

static bool says(const char *needle) { return strstr(g_out[COM_USB], needle) != NULL; }

void t_con(void)
{
    REQ("REQ-CON-05: legacy commands keep syntax and meaning (converted to SI)");
    (void)par_init();
    con_cmd_t c;
    c = line("D1000");   CHECK(c.type == CMD_MOVE);  CHECK_NEAR(c.f[0], 1.0, 1e-6);  CHECK(says("> D1000\r\n"));
    c = line("D-500");   CHECK(c.type == CMD_MOVE);  CHECK_NEAR(c.f[0], -0.5, 1e-6);
    c = line("d300");    CHECK(c.type == CMD_MOVE);  CHECK_NEAR(c.f[0], 0.3, 1e-6);
    c = line("V300");    CHECK(c.type == CMD_SPEED); CHECK_NEAR(c.f[0], 0.3, 1e-6);
    c = line("T1000");   CHECK(c.type == CMD_RAMP);  CHECK_NEAR(c.f[0], 1.0, 1e-6); CHECK_NEAR(c.f[1], 1.0, 1e-6);
    c = line("T1000 1500"); CHECK(c.type == CMD_RAMP); CHECK_NEAR(c.f[1], 1.5, 1e-6);
    c = line("F9.8 0.1275"); CHECK(c.type == CMD_FF && c.which == 0U);
    CHECK_NEAR(c.f[0], 0.098, 1e-6); CHECK_NEAR(c.f[1], 1.275, 1e-5);
    c = line("FL4.1 0.1361"); CHECK(c.type == CMD_FF && c.which == 1U);
    c = line("FR4.1 0.1361"); CHECK(c.type == CMD_FF && c.which == 2U);
    c = line("G0.045 0.898"); CHECK(c.type == CMD_GAIN); CHECK_NEAR(c.f[0], 0.45, 1e-6); CHECK_NEAR(c.f[1], 8.98, 1e-5);
    c = line("Y3");      CHECK(c.type == CMD_SYNC);  CHECK_NEAR(c.f[0], 3.0, 1e-6);
    c = line("-100");    CHECK(c.type == CMD_DUTY);  CHECK_NEAR(c.f[0], -1.0, 1e-6);
    c = line("+50");     CHECK(c.type == CMD_DUTY);  CHECK_NEAR(c.f[0], 0.5, 1e-6);
    c = line("0");       CHECK(c.type == CMD_DUTY);  CHECK(c.f[0] == 0.0f);
    c = line("s");       CHECK(c.type == CMD_STOP);
    c = line("r");       CHECK(c.type == CMD_RESET);
    c = line("p");       CHECK(c.type == CMD_PRINT);
    c = line("a");       CHECK(c.type == CMD_AUTOPRINT);
    c = line("?");       CHECK(c.type == CMD_HELP);
    c = line("k");       CHECK(c.type == CMD_PARAMS);

    REQ("spec 9: new commands");
    c = line("STOP");    CHECK(c.type == CMD_STOP);
    c = line("CLEAR");   CHECK(c.type == CMD_CLEAR);
    c = line("W90");     CHECK(c.type == CMD_TURN);  CHECK_NEAR(c.f[0], 1.5707963, 1e-5);
    c = line("PWM 50 -50"); CHECK(c.type == CMD_PWM); CHECK_NEAR(c.f[0], 0.5, 1e-6); CHECK_NEAR(c.f[1], -0.5, 1e-6);
    c = line("MODE IDLE");  CHECK(c.type == CMD_MODE && c.which == 0U);
    c = line("MODE BENCH"); CHECK(c.type == CMD_MODE && c.which == 1U);
    c = line("MODE AUTO");  CHECK(c.type == CMD_UNAVAILABLE);
    c = line("RUN");        CHECK(c.type == CMD_UNAVAILABLE);
    c = line("M 100 200");  CHECK(c.type == CMD_UNAVAILABLE);
    c = line("S");       CHECK(c.type == CMD_STATUS);
    c = line("I");       CHECK(c.type == CMD_INFO);
    c = line("L");       CHECK(c.type == CMD_DUMP);
    c = line("EV");      CHECK(c.type == CMD_EVLOG);
    c = line("FAULTS");  CHECK(c.type == CMD_FAULTS);
    c = line("FAULTINJ 4"); CHECK(c.type == CMD_FAULTINJ); CHECK(c.f[0] == 4.0f);
    c = line("P113=3.5");   CHECK(c.type == CMD_PSET && c.par->id == 113U); CHECK_NEAR(c.f[0], 3.5, 1e-6);
    c = line("Pwhl_kp_x=3"); CHECK(c.type == CMD_PSET && c.par->id == 113U);
    c = line("P113?");      CHECK(c.type == CMD_PGET && c.par->id == 113U);
    c = line("Pwhl_kp_x?"); CHECK(c.type == CMD_PGET);

    REQ("REQ-CON-04: unknown, range and syntax errors");
    c = line("IMU");     CHECK(c.type == CMD_NONE && says("ERR unknown, ? for help"));
    c = line("x1");      CHECK(c.type == CMD_NONE && says("ERR unknown"));
    c = line("D3");      CHECK(c.type == CMD_NONE && says("ERR range 5..20000"));
    c = line("D20001");  CHECK(c.type == CMD_NONE && says("ERR range"));
    c = line("Dabc");    CHECK(c.type == CMD_NONE && says("ERR syntax"));
    c = line("V500");    CHECK(c.type == CMD_NONE && says("ERR range 20..450"));
    c = line("T50");     CHECK(c.type == CMD_NONE && says("ERR range 100..5000"));
    c = line("101");     CHECK(c.type == CMD_NONE && says("ERR range -100..100"));
    c = line("PWM 200 0"); CHECK(c.type == CMD_NONE && says("ERR range"));
    c = line("P999=1");  CHECK(c.type == CMD_NONE && says("ERR unknown parameter"));
    c = line("FAULTINJ 99"); CHECK(c.type == CMD_NONE && says("ERR range"));
    c = line("MODE FOO"); CHECK(c.type == CMD_NONE && says("ERR syntax"));

    REQ("REQ-CON-02: each port has its own line buffer");
    clear_out();
    feed(COM_USB, "D10");                       /* unfinished on USB ... */
    feed(COM_RADIO, "s\r");                     /* ... complete on the radio */
    CHECK(con_poll(&c) && c.type == CMD_STOP && c.port == COM_RADIO);
    feed(COM_USB, "00\r");
    CHECK(con_poll(&c) && c.type == CMD_MOVE && c.port == COM_USB);
    CHECK_NEAR(c.f[0], 1.0, 1e-6);                /* "D10" + "00" = D1000 */

    REQ("line length limit 48: the rest is dropped, the line still runs");
    c = line("D1000                                                  junk");
    CHECK(c.type == CMD_MOVE);

    REQ("REQ-CON-03: every byte is a sign of life");
    feed(COM_RADIO, "x");
    CHECK(con_last_rx_ms(COM_RADIO) == 1234U);

    REQ("REQ-CON-01: long output in chunks of <= 200 bytes per pass");
    clear_out();
    con_job(COM_USB, CON_JOB_DUMP);
    size_t total = 0, passes = 0, worst = 0;
    while (passes < 200) {
        const size_t before = g_out_n[COM_USB];
        (void)con_poll(&c);
        const size_t n = g_out_n[COM_USB] - before;
        if (n == 0) { break; }
        total += n; passes++;
        if (n > worst) { worst = n; }
    }
    CHECK(worst <= 200U + 96U);                 /* budget is checked per line: last line may cross it */
    CHECK(passes > 1U);
    size_t lines = 0;
    for (const char *s = g_out[COM_USB]; (s = strstr(s, "#define PAR_")) != NULL; s++) { lines++; }
    CHECK(lines == par_count());
    (void)total;

    REQ("a full TX buffer postpones the job instead of dropping lines");
    clear_out();
    g_free = 10;
    con_job(COM_USB, CON_JOB_HELP);
    (void)con_poll(&c);
    CHECK(g_out_n[COM_USB] == 0U);
    g_free = 4096;
    for (int i = 0; i < 5; i++) { (void)con_poll(&c); }
    CHECK(says("D<mm> drive") && says("MODE IDLE|BENCH"));
}
