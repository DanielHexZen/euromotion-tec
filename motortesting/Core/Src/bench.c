/* bench.c — see bench.h */

#include "bench.h"
#include "motor.h"
#include "encoder.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ======================================================================
 * Handles
 * ==================================================================== */

static TIM_HandleTypeDef  *s_htim_timebase;
static ADC_HandleTypeDef  *s_hadc;
static UART_HandleTypeDef *s_huart;

/* ======================================================================
 * Cyclic executive
 * ==================================================================== */

static volatile uint32_t s_tick_ms;   /* incremented in the 1 kHz ISR */
static uint32_t          s_tick_seen;

void bench_tick_isr(void)
{
    s_tick_ms++;
}

uint32_t bench_micros(void)
{
    return __HAL_TIM_GET_COUNTER(s_htim_timebase);
}

/* ======================================================================
 * Console output — ring buffer, never blocks (§16.4 discipline)
 * ==================================================================== */

#define TXBUF_SZ    4096U
#define TXCHUNK_SZ   128U

static uint8_t  s_txbuf[TXBUF_SZ];
static uint16_t s_tx_head;
static uint16_t s_tx_tail;

static void out_str(const char *s)
{
    while (*s != '\0') {
        uint16_t next = (uint16_t)((s_tx_head + 1U) % TXBUF_SZ);
        if (next == s_tx_tail) {
            return;   /* full: drop the rest. Stale console text is worthless. */
        }
        s_txbuf[s_tx_head] = (uint8_t)*s++;
        s_tx_head = next;
    }
}

static void out(const char *fmt, ...)
{
    char line[192];
    va_list ap;

    va_start(ap, fmt);
    (void)vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    out_str(line);
}

static void tx_pump(void)
{
    /* Two buffers in alternation. HAL clears gState when the last byte
     * reaches the data register, not when it has left the shift register,
     * so a single buffer can be overwritten while the hardware still
     * reads it. Same construction as the telemetry TX of sec.16.4. */
    static uint8_t chunk[2][TXCHUNK_SZ];
    static uint8_t which = 0U;
    uint16_t n;

    if (s_tx_head == s_tx_tail) {
        return;
    }
    if (s_huart->gState != HAL_UART_STATE_READY) {
        return;
    }

    n = (s_tx_head > s_tx_tail)
        ? (uint16_t)(s_tx_head - s_tx_tail)
        : (uint16_t)(TXBUF_SZ - s_tx_tail);
    if (n > TXCHUNK_SZ) {
        n = TXCHUNK_SZ;
    }

    which ^= 1U;
    (void)memcpy(chunk[which], &s_txbuf[s_tx_tail], n);
    s_tx_tail = (uint16_t)((s_tx_tail + n) % TXBUF_SZ);
    (void)HAL_UART_Transmit_IT(s_huart, chunk[which], n);
}

/* Integer-only formatting throughout. One printf("%f") pulls in the
 * float formatter, +10-15 KB (§15.6). Not a problem here, but the habit
 * is what keeps it out of the flight build. */
static int32_t milli(float v)  { return (int32_t)lrintf(v * 1000.0f); }
static int32_t centi(float v)  { return (int32_t)lrintf(v * 100.0f);  }

/* ======================================================================
 * State
 * ==================================================================== */

typedef enum {
    MODE_DISARMED = 0,
    MODE_MANUAL,
    MODE_SEQ_DIRCHECK,
    MODE_SEQ_DEADBAND,
    MODE_SEQ_KV,
    MODE_SEQ_STRAIGHT,
    MODE_SEQ_SPIN,
    MODE_HANDTURN,
    MODE_TRUTHTABLE,
    MODE_FAULT
} mode_t;

#define FAULT_NONE        0x00U
#define FAULT_STALL_L     0x01U
#define FAULT_STALL_R     0x02U
#define FAULT_UNDERVOLT   0x04U
#define FAULT_TIMEOUT     0x08U
#define FAULT_ESTOP       0x10U

static mode_t   s_mode;
static uint8_t  s_faults;

static float    s_target[MOTOR_COUNT];   /* what we want on the pins   */
static float    s_ramped[MOTOR_COUNT];   /* slew-limited, what we send */

static float    s_cmd_common;
static float    s_cmd_diff;
static motor_id_t s_sel = MOTOR_L;       /* motor under test           */
static bool     s_sel_both = true;

static uint32_t s_last_cmd_ms;
static uint32_t s_stall_ms[MOTOR_COUNT];

static bool     s_telem_on = true;
static bool     s_ff_on    = false;  /* sec.5.5 feedforward, 'f' toggles.
                                      * OFF by default since the L298N swap:
                                      * Kv/Ks are from the old driver. */
static bool     s_batt_sup = BATT_SUPERVISION_DEFAULT;
static uint16_t s_vbatt_mv;
static bool     s_adc_pending;

/* sequence scratch */
static uint32_t s_seq_ms;
static uint8_t  s_seq_step;
static int32_t  s_seq_c0[MOTOR_COUNT];
static float    s_seq_u;
static bool     s_seq_found;
static bool     s_truth_confirm;

/* ======================================================================
 * Open-loop feedforward (§5.5, feedforward term only)
 * ==================================================================== */

/* Converts a speed request into a duty cycle using the per-motor
 * constants measured in week 5. This is NOT a control loop: there is no
 * feedback, so a load difference between the two sides still produces a
 * speed difference. What it removes is the SYSTEMATIC motor-to-motor
 * difference — different Kv and different friction — which on this pair
 * is about 6 % in speed and 20 % in breakaway duty.
 *
 * Applies to manual duty and to the two demo sequences only. The
 * deadband and Kv sweeps must stay on raw duty, or they would measure
 * this compensation instead of the motor. */
static float ff_duty(motor_id_t id, float rpm_set)
{
    const float kv = (id == MOTOR_L) ? KV_L_F : KV_R_F;
    const float ks = (id == MOTOR_L) ? KS_L_F : KS_R_F;
    float u;

    if (fabsf(rpm_set) < 1.0f) {
        return 0.0f;
    }

    u = (kv * fabsf(rpm_set)) + ks;
    if (u > 1.0f) {
        u = 1.0f;
    }
    return (rpm_set >= 0.0f) ? u : -u;
}

/* Maps a command pair in [-1,+1] onto s_target, through the feedforward
 * when it is enabled. Joint saturation (§5.5): both are scaled by the
 * same factor rather than clipped individually, so the commanded ratio
 * and therefore the driven curvature survive. */
static void set_cmd_pair(float cL, float cR)
{
    float peak = fmaxf(fabsf(cL), fabsf(cR));

    if (peak > 1.0f) {
        cL /= peak;
        cR /= peak;
    }

    if (s_ff_on) {
        s_target[MOTOR_L] = ff_duty(MOTOR_L, cL * RPM_MAX_CMD_F);
        s_target[MOTOR_R] = ff_duty(MOTOR_R, cR * RPM_MAX_CMD_F);
    } else {
        s_target[MOTOR_L] = cL;
        s_target[MOTOR_R] = cR;
    }
}

/* ======================================================================
 * Safety
 * ==================================================================== */

static void all_stop(void)
{
    s_target[MOTOR_L] = 0.0f;
    s_target[MOTOR_R] = 0.0f;
    s_ramped[MOTOR_L] = 0.0f;
    s_ramped[MOTOR_R] = 0.0f;
    s_cmd_common      = 0.0f;
    s_cmd_diff        = 0.0f;
    motor_coast_all();
}

static void raise_fault(uint8_t flag, const char *why)
{
    all_stop();
    s_faults |= flag;
    s_mode = MODE_FAULT;
    out("# FAULT: %s  (flags 0x%02X). 'r' to clear.\r\n", why, (unsigned)s_faults);
}

static void enter_mode(mode_t m)
{
    all_stop();
    s_seq_ms   = 0U;
    s_seq_step = 0U;
    s_seq_found = false;
    s_truth_confirm = false;
    s_mode = m;
}

/* ======================================================================
 * Console input
 * ==================================================================== */

static void print_help(void)
{
    out_str(
      "\r\n=== MR2006B drivetrain bench ===============================\r\n"
      " SAFETY, before the motors are ever connected:\r\n"
      "   1. verify the driver truth table  -> '0'  (motors OFF, 6 V/100 mA)\r\n"
      "   2. verify the motor connector pinout with a multimeter\r\n"
      "   3. commissioning order per CONCEPT sec.13.7 step 1..6\r\n"
      "\r\n"
      " o  arm            x  E-STOP / disarm      r  clear faults\r\n"
      " w/s common duty +/-5%%    a/d differential +/-5%%   SPACE stop\r\n"
      " L/R/B  select motor left / right / both\r\n"
      " c  zero encoder counts   p  telemetry on/off   b  batt supervision\r\n"
      " f  feedforward on/off  (on: w/s command speed, not duty)\r\n"
      "\r\n"
      " Sequences (any key aborts):\r\n"
      "   0  driver truth table probe   (disarmed, motors disconnected)\r\n"
      "   1  direction + encoder sign check\r\n"
      "   2  deadband sweep  -> Ks      (selected motor)\r\n"
      "   3  duty/rpm sweep  -> Kv      (selected motor)\r\n"
      "   4  straight demo   35%%, 3 s   (week-5 acceptance)\r\n"
      "   5  turn in place   35%%, 3 s   (week-5 acceptance)\r\n"
      "   6  hand-turn readout          (counts per output revolution)\r\n"
      " h  this help\r\n"
      " Telemetry CSV: T,ms,mode,uL,uR,rpmL,rpmR,cntL,cntR,mV,flags\r\n"
      "                (u in 1/1000, rpm in 1/100)\r\n"
      "============================================================\r\n");
}

static void cmd_bump(float *v, float delta)
{
    *v += delta;
    if (*v >  1.0f) { *v =  1.0f; }
    if (*v < -1.0f) { *v = -1.0f; }
}

static void handle_char(uint8_t c)
{
    s_last_cmd_ms = s_tick_ms;

    /* Any key aborts a running sequence, except the truth-table stepper
     * which has its own keys. */
    if ((s_mode >= MODE_SEQ_DIRCHECK) && (s_mode <= MODE_HANDTURN)) {
        if ((c != 'p') && (c != 'h')) {
            out("# sequence aborted\r\n");
            enter_mode(MODE_DISARMED);
            return;
        }
    }

    switch (c) {
    case 'h': case '?':
        print_help();
        break;

    case 'o':
        if (s_faults != FAULT_NONE) {
            out("# cannot arm: faults 0x%02X pending, press 'r'\r\n", (unsigned)s_faults);
        } else {
            enter_mode(MODE_MANUAL);
            out("# ARMED. manual duty. SPACE stops, x is E-STOP.\r\n");
        }
        break;

    case 'x':
        all_stop();
        s_faults |= FAULT_ESTOP;
        s_mode = MODE_FAULT;
        out("# E-STOP\r\n");
        break;

    case 'r':
        all_stop();
        s_faults = FAULT_NONE;
        s_mode = MODE_DISARMED;
        out("# faults cleared, disarmed\r\n");
        break;

    case ' ':
        s_cmd_common = 0.0f;
        s_cmd_diff   = 0.0f;
        break;

    case 'w': cmd_bump(&s_cmd_common, +0.05f); break;
    case 's': cmd_bump(&s_cmd_common, -0.05f); break;
    case 'a': cmd_bump(&s_cmd_diff,   -0.05f); break;
    case 'd': cmd_bump(&s_cmd_diff,   +0.05f); break;

    case 'L': case 'l': s_sel = MOTOR_L; s_sel_both = false; out("# motor L\r\n"); break;
    case 'R':           s_sel = MOTOR_R; s_sel_both = false; out("# motor R\r\n"); break;
    case 'B': case 'n': /* 'n' doubles as truth-table advance below */
        if (s_mode != MODE_TRUTHTABLE) {
            s_sel_both = true;
            out("# both motors\r\n");
        }
        break;

    case 'c':
        encoder_reset();
        out("# encoder counts zeroed\r\n");
        break;

    case 'p':
        s_telem_on = !s_telem_on;
        out("# telemetry %s\r\n", s_telem_on ? "on" : "off");
        break;

    case 'f':
        s_ff_on = !s_ff_on;
        out("# feedforward %s%s\r\n", s_ff_on ? "on" : "off",
            s_ff_on ? " (command is speed)" : " (command is raw duty)");
        break;

    case 'b':
        s_batt_sup = !s_batt_sup;
        out("# battery supervision %s\r\n", s_batt_sup ? "on" : "off");
        break;

    case '0':
        if (s_mode != MODE_DISARMED) {
            out("# truth-table probe only from disarmed state\r\n");
        } else {
            enter_mode(MODE_TRUTHTABLE);
            out("\r\n# TRUTH TABLE PROBE\r\n"
                "# Motors MUST be disconnected. Driver supply 6 V, limit 100 mA.\r\n"
                "# Probe OUT1/OUT2 of the selected channel at each step.\r\n"
                "# 'n' = next step, 'y' = also test the (1,1) state, any other key = abort\r\n");
        }
        break;

    case '1': enter_mode(MODE_SEQ_DIRCHECK);
              out("# DIRCHECK: each motor fwd/rev at 30%%. Expect positive counts on fwd.\r\n");
              break;
    case '2': enter_mode(MODE_SEQ_DEADBAND);
              out("# DEADBAND sweep on motor %c, from %ld/1000 upward\r\n",
                  (s_sel == MOTOR_L) ? 'L' : 'R', (long)milli(U_MIN_F));
              break;
    case '3': enter_mode(MODE_SEQ_KV);
              out("# KV sweep on motor %c\r\n", (s_sel == MOTOR_L) ? 'L' : 'R');
              break;
    case '4': enter_mode(MODE_SEQ_STRAIGHT); out("# STRAIGHT 35%%, 3 s\r\n"); break;
    case '5': enter_mode(MODE_SEQ_SPIN);     out("# SPIN 35%%, 3 s\r\n");     break;
    case '6': enter_mode(MODE_HANDTURN);
              encoder_reset();
              out("# HANDTURN: motors coasting. Turn the OUTPUT (front, 6 mm D)\r\n"
                  "# shaft exactly one revolution and read the count. The rear\r\n"
                  "# encoder shaft gives 64 - that is the motor, not the output.\r\n"
                  "# Measured: L %ld, R %ld. Nominal %ld.\r\n",
                  (long)lrintf(encoder_cpr(MOTOR_L)),
                  (long)lrintf(encoder_cpr(MOTOR_R)),
                  (long)lrintf(ENC_CPR_OUTPUT_NOM_F));
              break;

    case 'y':
        if (s_mode == MODE_TRUTHTABLE) {
            s_truth_confirm = true;
            out("# (1,1) state enabled for this run\r\n");
        }
        break;

    default:
        break;
    }

    /* 'n' advances the truth-table stepper */
    if ((c == 'n') && (s_mode == MODE_TRUTHTABLE)) {
        s_seq_step++;
        s_seq_ms = 0U;
    }
}

static void console_poll(void)
{
    /* Overrun clears by reading SR then DR; do it unconditionally so a
     * paused terminal cannot wedge the console. */
    if (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_OREFLAG(s_huart);
    }
    while (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_RXNE)) {
        uint8_t c = (uint8_t)(s_huart->Instance->DR & 0xFFU);
        handle_char(c);
    }
}

/* ======================================================================
 * Battery (non-blocking: start in one 20 Hz slot, read in the next)
 * ==================================================================== */

static void battery_service_20hz(void)
{
    if (s_adc_pending) {
        if (HAL_ADC_PollForConversion(s_hadc, 0U) == HAL_OK) {
            uint32_t raw = HAL_ADC_GetValue(s_hadc);
            float    v   = ((float)raw / ADC_FULL_SCALE_F) * ADC_VREF_MV_F;
            v *= (BATT_R_TOP_F + BATT_R_BOT_F) / BATT_R_BOT_F;
            s_vbatt_mv = (uint16_t)lrintf(v);
            s_adc_pending = false;
        }
    }
    if (!s_adc_pending) {
        (void)HAL_ADC_Start(s_hadc);
        s_adc_pending = true;
    }

    if (s_batt_sup && (s_vbatt_mv > 5000U)) {   /* >5 V => a pack is present */
        if (s_vbatt_mv < (uint16_t)V_BATT_CUTOFF_MV) {
            raise_fault(FAULT_UNDERVOLT, "battery below cutoff");
        }
    }
}

/* ======================================================================
 * Supervisor — stall and dead-man, 200 Hz
 * ==================================================================== */

static void supervisor_200hz(void)
{
    /* During the deadband sweep, not turning IS the measurement: the ramp
     * deliberately dwells above STALL_U_THRESH_F until the shaft breaks
     * away. Motor R breaks away near 17 %, so without this exception the
     * supervisor aborts the very sequence it is meant to protect. */
    bool stall_check = (s_mode != MODE_SEQ_DEADBAND);

    for (uint8_t i = 0U; i < (uint8_t)MOTOR_COUNT; i++) {
        float u   = motor_get_applied((motor_id_t)i);
        float rpm = encoder_rpm_out((motor_id_t)i);

        if (stall_check &&
            (fabsf(u) > STALL_U_THRESH_F) &&
            (fabsf(rpm) < STALL_RPM_THRESH_F)) {
            s_stall_ms[i] += 5U;
            if (s_stall_ms[i] >= STALL_TIME_MS) {
                raise_fault((i == 0U) ? FAULT_STALL_L : FAULT_STALL_R,
                            (i == 0U) ? "motor L stalled" : "motor R stalled");
                s_stall_ms[i] = 0U;
            }
        } else {
            s_stall_ms[i] = 0U;
        }
    }

    /* Dead-man: only in manual mode, and only while actually driving. */
    if (s_mode == MODE_MANUAL) {
        bool moving = (fabsf(s_ramped[MOTOR_L]) > 0.0f) ||
                      (fabsf(s_ramped[MOTOR_R]) > 0.0f);
        if (moving && ((s_tick_ms - s_last_cmd_ms) > CMD_TIMEOUT_MS)) {
            raise_fault(FAULT_TIMEOUT, "no command for 2 s");
        }
    }
}

/* ======================================================================
 * Sequences — 100 Hz
 * ==================================================================== */

static void seq_dircheck(void)
{
    /* 6 s per motor: +30 % 2 s, coast 1 s, -30 % 2 s, coast 1 s */
    uint8_t   m   = (s_seq_step < 4U) ? 0U : 1U;
    uint8_t   sub = (uint8_t)(s_seq_step % 4U);
    motor_id_t id = (motor_id_t)m;

    if (s_seq_ms == 0U) {
        s_seq_c0[0] = encoder_count(MOTOR_L);
        s_seq_c0[1] = encoder_count(MOTOR_R);
    }

    s_target[MOTOR_L] = 0.0f;
    s_target[MOTOR_R] = 0.0f;

    switch (sub) {
    case 0U: s_target[id] = +0.30f; break;
    case 1U: s_target[id] =  0.00f; break;
    case 2U: s_target[id] = -0.30f; break;
    default: s_target[id] =  0.00f; break;
    }

    uint32_t dur = ((sub == 0U) || (sub == 2U)) ? 2000U : 1000U;
    s_seq_ms += 10U;

    if (s_seq_ms >= dur) {
        int32_t d = encoder_count(id) - s_seq_c0[m];
        if (sub == 0U) {
            out("# %c forward  30%%: delta %ld counts, %ld rpm/100 %s\r\n",
                (m == 0U) ? 'L' : 'R', (long)d, (long)centi(encoder_rpm_out(id)),
                (d > 0) ? "OK" : "<< SIGN WRONG: flip ENC_SIGN in bench_config.h");
        } else if (sub == 2U) {
            out("# %c reverse  30%%: delta %ld counts %s\r\n",
                (m == 0U) ? 'L' : 'R', (long)d, (d < 0) ? "OK" : "<< SIGN WRONG");
        } else {
            /* coast phase, nothing to report */
        }
        s_seq_ms = 0U;
        s_seq_step++;
        if (s_seq_step >= 8U) {
            out("# DIRCHECK done\r\n");
            enter_mode(MODE_DISARMED);
        }
    }
}

static void seq_deadband(void)
{
    /* Ramp from the duty floor to 40 % over 20 s and report the duty at
     * which the shaft first turns. That is Ks in sec.5.5.
     * If it moves immediately at U_MIN_F, the deadband is below what a
     * 10 kHz carrier can resolve -> drop the carrier to 5 kHz (sec.5.2). */
    const float u_end = 0.40f;
    const float t_s   = 20.0f;

    s_target[MOTOR_L] = 0.0f;
    s_target[MOTOR_R] = 0.0f;

    s_seq_u = U_MIN_F + ((u_end - U_MIN_F) * ((float)s_seq_ms / (t_s * 1000.0f)));
    s_target[s_sel] = s_seq_u;
    s_seq_ms += 10U;

    if (!s_seq_found && (fabsf(encoder_rpm_out(s_sel)) > 3.0f)) {
        s_seq_found = true;
        out("# breakaway at duty %ld/1000%s\r\n", (long)milli(s_seq_u),
            (s_seq_u <= (U_MIN_F + 0.005f))
                ? "  << at the duty floor: deadband unresolvable at 10 kHz" : "");
    }

    if (s_seq_ms >= (uint32_t)(t_s * 1000.0f)) {
        if (!s_seq_found) {
            out("# no motion up to 40%% duty. Check wiring, supply, mechanics.\r\n");
        }
        out("# DEADBAND done\r\n");
        enter_mode(MODE_DISARMED);
    }
}

static void seq_kv(void)
{
    /* Duty steps 20..100 %, 2 s each, report steady-state rpm.
     * Fit rpm = (u - Ks) / Kv offline; also a no-load sanity check
     * against the catalogue 178 rpm scaled by the actual supply. */
    const float duties[9] = { 0.20f, 0.30f, 0.40f, 0.50f, 0.60f,
                              0.70f, 0.80f, 0.90f, 1.00f };

    s_target[MOTOR_L] = 0.0f;
    s_target[MOTOR_R] = 0.0f;
    s_target[s_sel]   = duties[s_seq_step];
    s_seq_ms += 10U;

    if (s_seq_ms >= 2000U) {
        out("# u %ld/1000 -> %ld rpm/100 (%ld mm/s), Vbat %u mV\r\n",
            (long)milli(duties[s_seq_step]),
            (long)centi(encoder_rpm_out(s_sel)),
            (long)milli(encoder_speed_mps(s_sel)),
            (unsigned)s_vbatt_mv);
        s_seq_ms = 0U;
        s_seq_step++;
        if (s_seq_step >= 9U) {
            out("# KV sweep done\r\n");
            enter_mode(MODE_DISARMED);
        }
    }
}

static void seq_fixed(float cL, float cR, uint32_t dur_ms, const char *name)
{
    set_cmd_pair(cL, cR);
    s_seq_ms += 10U;
    if (s_seq_ms >= dur_ms) {
        out("# %s done: L %ld counts, R %ld counts (ff %s)\r\n", name,
            (long)encoder_count(MOTOR_L), (long)encoder_count(MOTOR_R),
            s_ff_on ? "on" : "off");
        enter_mode(MODE_DISARMED);
    }
}

static void seq_truthtable(void)
{
    /* Steps: EN/IN1/IN2 combinations, held until 'n'.
     * Step 4 (1,1) only after 'y'. */
    static const struct { bool in1; bool in2; float en; const char *label; } tt[5] = {
        { false, false, 0.00f, "EN=0   IN1=0 IN2=0  -> expect both outputs floating/low" },
        { true,  false, 0.50f, "EN=PWM IN1=1 IN2=0  -> expect OUT1 ~ +Vm, OUT2 ~ 0" },
        { false, true,  0.50f, "EN=PWM IN1=0 IN2=1  -> expect OUT1 ~ 0, OUT2 ~ +Vm" },
        { false, false, 0.50f, "EN=PWM IN1=0 IN2=0  -> expect both low (brake) or floating" },
        { true,  true,  0.50f, "EN=PWM IN1=1 IN2=1  -> the unverified state. Watch the current limit." },
    };

    uint8_t step = s_seq_step;

    if (step >= 4U) {
        if ((step == 4U) && !s_truth_confirm) {
            out("# step 5 is the (1,1) state. Press 'y' to enable, any other key to abort.\r\n");
            s_seq_step = 4U;
            /* hold here until 'y' sets s_truth_confirm */
            motor_logic_probe(s_sel, false, false, 0.0f);
            s_seq_ms = 1U;   /* suppress the repeat print below */
            return;
        }
        if (step > 4U) {
            out("# truth-table probe finished. Record the results in CONCEPT sec.5.2.\r\n");
            motor_coast_all();
            enter_mode(MODE_DISARMED);
            return;
        }
    }

    if (s_seq_ms == 0U) {
        out("# step %u/5, channel %c: %s\r\n", (unsigned)(step + 1U),
            (s_sel == MOTOR_L) ? 'L' : 'R', tt[step].label);
        out("#   probe now, then 'n'\r\n");
    }
    s_seq_ms += 10U;

    motor_logic_probe(s_sel, tt[step].in1, tt[step].in2, tt[step].en);
}

/* ======================================================================
 * Slew limiter + output, 100 Hz
 * ==================================================================== */

static void output_100hz(void)
{
    const float step = SLEW_PER_S_F / 100.0f;

    if (s_mode == MODE_MANUAL) {
        float cL = s_cmd_common - s_cmd_diff;
        float cR = s_cmd_common + s_cmd_diff;

        if (!s_sel_both) {
            if (s_sel == MOTOR_L) { cR = 0.0f; } else { cL = 0.0f; }
        }
        set_cmd_pair(cL, cR);
    } else if ((s_mode == MODE_DISARMED) || (s_mode == MODE_FAULT) ||
               (s_mode == MODE_HANDTURN) || (s_mode == MODE_TRUTHTABLE)) {
        s_target[MOTOR_L] = 0.0f;
        s_target[MOTOR_R] = 0.0f;
    } else {
        /* sequences set s_target themselves */
    }

    for (uint8_t i = 0U; i < (uint8_t)MOTOR_COUNT; i++) {
        float d = s_target[i] - s_ramped[i];
        if (d >  step) { d =  step; }
        if (d < -step) { d = -step; }
        s_ramped[i] += d;
    }

    if (s_mode != MODE_TRUTHTABLE) {
        motor_apply(MOTOR_L, s_ramped[MOTOR_L]);
        motor_apply(MOTOR_R, s_ramped[MOTOR_R]);
    }
}

/* ======================================================================
 * Telemetry, 10 Hz
 * ==================================================================== */

static void telemetry_10hz(void)
{
    if (!s_telem_on) {
        return;
    }
    out("T,%lu,%u,%ld,%ld,%ld,%ld,%ld,%ld,%u,%u\r\n",
        (unsigned long)s_tick_ms,
        (unsigned)s_mode,
        (long)milli(motor_get_applied(MOTOR_L)),
        (long)milli(motor_get_applied(MOTOR_R)),
        (long)centi(encoder_rpm_out(MOTOR_L)),
        (long)centi(encoder_rpm_out(MOTOR_R)),
        (long)encoder_count(MOTOR_L),
        (long)encoder_count(MOTOR_R),
        (unsigned)s_vbatt_mv,
        (unsigned)s_faults);
}

static void handturn_5hz(void)
{
    out("# handturn: L %ld  R %ld  (measured L %ld / R %ld per output rev)\r\n",
        (long)encoder_count(MOTOR_L), (long)encoder_count(MOTOR_R),
        (long)lrintf(encoder_cpr(MOTOR_L)), (long)lrintf(encoder_cpr(MOTOR_R)));
}

/* ======================================================================
 * Init and main loop
 * ==================================================================== */

void bench_init(TIM_HandleTypeDef *htim_pwm,
                TIM_HandleTypeDef *htim_enc_l,
                TIM_HandleTypeDef *htim_enc_r,
                TIM_HandleTypeDef *htim_timebase,
                TIM_HandleTypeDef *htim_tick,
                ADC_HandleTypeDef *hadc_batt,
                UART_HandleTypeDef *huart_console)
{
    s_htim_timebase = htim_timebase;
    s_hadc          = hadc_batt;
    s_huart         = huart_console;

    motor_init(htim_pwm);
    encoder_init(htim_enc_l, htim_enc_r);

    (void)HAL_TIM_Base_Start(htim_timebase);   /* 1 MHz free run, no IRQ */
    (void)HAL_TIM_Base_Start_IT(htim_tick);    /* 1 kHz executive tick   */

    s_mode        = MODE_DISARMED;
    s_faults      = FAULT_NONE;
    s_last_cmd_ms = 0U;
    s_tick_seen   = 0U;

    all_stop();
    print_help();
}

void bench_run(void)
{
    for (;;) {
        uint32_t now = s_tick_ms;

        if (now == s_tick_seen) {
            continue;                    /* nothing due */
        }
        if ((now - s_tick_seen) > 1U) {
            /* Overrun: a slot took longer than a millisecond. On the
             * bench this is only interesting; in the flight firmware it
             * would be a timestamp error (sec.15.1). */
            out("# WARN: %lu ms overrun\r\n", (unsigned long)(now - s_tick_seen - 1U));
        }
        s_tick_seen = now;

        HAL_GPIO_WritePin(LOAD_PROBE_PORT, LOAD_PROBE_PIN, GPIO_PIN_SET);

        /* ---- 1 kHz ---------------------------------------------- */
        encoder_sample_1khz();
        motor_service_1khz();
        console_poll();   /* 1 kHz: the USART has a one-byte holding
                           * register, so a slower poll silently drops
                           * characters as soon as anything is pasted. */
        tx_pump();

        /* ---- 200 Hz --------------------------------------------- */
        if ((now % 5U) == 0U) {
            encoder_update_200hz();
            supervisor_200hz();
        }

        /* ---- 100 Hz --------------------------------------------- */
        if ((now % 10U) == 0U) {
            switch (s_mode) {
            case MODE_SEQ_DIRCHECK: seq_dircheck(); break;
            case MODE_SEQ_DEADBAND: seq_deadband(); break;
            case MODE_SEQ_KV:       seq_kv();       break;
            case MODE_SEQ_STRAIGHT: seq_fixed(+0.30f, +0.30f, 3000U, "STRAIGHT"); break;
            case MODE_SEQ_SPIN:     seq_fixed(-0.30f, +0.30f, 3000U, "SPIN");     break;
            case MODE_TRUTHTABLE:   seq_truthtable(); break;
            default: break;
            }
            output_100hz();
        }

        /* ---- 20 Hz ---------------------------------------------- */
        if ((now % 50U) == 0U) {
            battery_service_20hz();
        }

        /* ---- 10 Hz ---------------------------------------------- */
        if ((now % 100U) == 0U) {
            telemetry_10hz();
        }

        /* ---- 5 Hz ----------------------------------------------- */
        if (((now % 200U) == 0U) && (s_mode == MODE_HANDTURN)) {
            handturn_5hz();
        }

        HAL_GPIO_WritePin(LOAD_PROBE_PORT, LOAD_PROBE_PIN, GPIO_PIN_RESET);
    }
}
