/* motor.c — see motor.h */

#include "motor.h"
#include <math.h>

typedef struct {
    GPIO_TypeDef *in1_port;
    uint16_t      in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t      in2_pin;
    uint32_t      channel;
} motor_pins_t;

static const motor_pins_t s_pins[MOTOR_COUNT] = {
    [MOTOR_L] = { M_L_IN1_PORT, M_L_IN1_PIN, M_L_IN2_PORT, M_L_IN2_PIN, M_L_PWM_CHANNEL },
    [MOTOR_R] = { M_R_IN1_PORT, M_R_IN1_PIN, M_R_IN2_PORT, M_R_IN2_PIN, M_R_PWM_CHANNEL },
};

typedef struct {
    int8_t   dir;          /* -1, 0, +1 — what is on the pins now */
    uint16_t deadtime_ms;  /* > 0 => channel forced to coast       */
    float    u_applied;
} motor_state_t;

static motor_state_t      s_state[MOTOR_COUNT];
static TIM_HandleTypeDef *s_htim;

/* ---------------------------------------------------------------- */

static void set_pwm(motor_id_t id, float magnitude)
{
    uint32_t ccr;

    if (magnitude <= 0.0f) {
        ccr = 0U;
    } else {
        if (magnitude > U_MAX_F) {
            magnitude = U_MAX_F;
        }
        ccr = (uint32_t)(magnitude * (float)PWM_PERIOD_TICKS);
        if (ccr > PWM_ARR) {
            ccr = PWM_ARR;
        }
    }
    __HAL_TIM_SET_COMPARE(s_htim, s_pins[id].channel, ccr);
}

/* Deactivate first, then activate. There is no ordering of these two
 * writes in which both pins are high, not even for one instruction. */
static void set_dir(motor_id_t id, int8_t dir)
{
    const motor_pins_t *p = &s_pins[id];

    if (dir > 0) {
        HAL_GPIO_WritePin(p->in2_port, p->in2_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(p->in1_port, p->in1_pin, GPIO_PIN_SET);
    } else if (dir < 0) {
        HAL_GPIO_WritePin(p->in1_port, p->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(p->in2_port, p->in2_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(p->in1_port, p->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(p->in2_port, p->in2_pin, GPIO_PIN_RESET);
    }
}

/* ---------------------------------------------------------------- */

void motor_init(TIM_HandleTypeDef *htim_pwm)
{
    s_htim = htim_pwm;

    for (uint8_t i = 0U; i < (uint8_t)MOTOR_COUNT; i++) {
        s_state[i].dir         = 0;
        s_state[i].deadtime_ms = 0U;
        s_state[i].u_applied   = 0.0f;
        set_dir((motor_id_t)i, 0);
    }

    __HAL_TIM_SET_COMPARE(s_htim, M_L_PWM_CHANNEL, 0U);
    __HAL_TIM_SET_COMPARE(s_htim, M_R_PWM_CHANNEL, 0U);

    /* TIM1 is an advanced-control timer: HAL_TIM_PWM_Start also sets MOE. */
    (void)HAL_TIM_PWM_Start(s_htim, M_L_PWM_CHANNEL);
    (void)HAL_TIM_PWM_Start(s_htim, M_R_PWM_CHANNEL);
}

void motor_apply(motor_id_t id, float u)
{
    motor_state_t *m = &s_state[id];
    int8_t dir;
    float  mag;

    if (u > 1.0f)  { u = 1.0f;  }
    if (u < -1.0f) { u = -1.0f; }

    mag = fabsf(u);
    dir = (u > 0.0f) ? (int8_t)1 : ((u < 0.0f) ? (int8_t)-1 : (int8_t)0);

    /* Below the driver's minimum pulse width the output jumps between
     * off and the minimum pulse. Commanding it is worse than not. */
    if (mag < U_MIN_F) {
        dir = 0;
        mag = 0.0f;
    }

    /* Reversal: force coast, arm the dead-time, return. The caller's
     * slew limiter will re-request the new direction on the next cycle. */
    if ((dir != 0) && (m->dir != 0) && (dir != m->dir)) {
        set_pwm(id, 0.0f);
        set_dir(id, 0);
        m->dir         = 0;
        m->deadtime_ms = DIR_DEADTIME_MS;
        m->u_applied   = 0.0f;
        return;
    }

    if (m->deadtime_ms > 0U) {
        set_pwm(id, 0.0f);
        m->u_applied = 0.0f;
        return;
    }

    if (dir != m->dir) {
        set_dir(id, dir);
        m->dir = dir;
    }

    set_pwm(id, mag);
    m->u_applied = (float)dir * mag;
}

void motor_coast(motor_id_t id)
{
    set_pwm(id, 0.0f);
    set_dir(id, 0);
    s_state[id].dir       = 0;
    s_state[id].u_applied = 0.0f;
}

void motor_coast_all(void)
{
    for (uint8_t i = 0U; i < (uint8_t)MOTOR_COUNT; i++) {
        motor_coast((motor_id_t)i);
    }
}

void motor_service_1khz(void)
{
    for (uint8_t i = 0U; i < (uint8_t)MOTOR_COUNT; i++) {
        if (s_state[i].deadtime_ms > 0U) {
            s_state[i].deadtime_ms--;
        }
    }
}

float motor_get_applied(motor_id_t id)
{
    return s_state[id].u_applied;
}

void motor_logic_probe(motor_id_t id, bool in1, bool in2, float en_duty)
{
    const motor_pins_t *p = &s_pins[id];

    HAL_GPIO_WritePin(p->in1_port, p->in1_pin, in1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(p->in2_port, p->in2_pin, in2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    set_pwm(id, en_duty);

    /* Deliberately does not touch s_state.dir. Leaving the probe mode
     * goes through motor_coast(), which resynchronises it. */
}
