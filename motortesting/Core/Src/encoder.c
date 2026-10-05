/* encoder.c — see encoder.h */

#include "encoder.h"

#define PI_F   3.14159265f

typedef struct {
    TIM_HandleTypeDef *htim;
    int8_t   sign;
    uint16_t last_cnt;
    int32_t  count;             /* accumulated, sign-corrected     */
    int32_t  count_at_last_vel; /* snapshot at the last 200 Hz call */
    int32_t  ring[VEL_BOXCAR_N];
    uint8_t  ring_idx;
    int32_t  ring_sum;
    float    rpm;
    float	 cpr;
} encoder_t;

static encoder_t s_enc[MOTOR_COUNT];

/* ---------------------------------------------------------------- */

void encoder_init(TIM_HandleTypeDef *htim_left, TIM_HandleTypeDef *htim_right)
{
    s_enc[MOTOR_L].htim = htim_left;
    s_enc[MOTOR_L].sign = (int8_t)ENC_SIGN_L;
    s_enc[MOTOR_L].cpr  = ENC_CPR_OUTPUT_L_F;
    s_enc[MOTOR_R].htim = htim_right;
    s_enc[MOTOR_R].sign = (int8_t)ENC_SIGN_R;
    s_enc[MOTOR_R].cpr  = ENC_CPR_OUTPUT_R_F;

    for (uint8_t i = 0U; i < (uint8_t)MOTOR_COUNT; i++) {
        (void)HAL_TIM_Encoder_Start(s_enc[i].htim, TIM_CHANNEL_ALL);
        __HAL_TIM_SET_COUNTER(s_enc[i].htim, 0U);
        s_enc[i].last_cnt          = 0U;
        s_enc[i].count             = 0;
        s_enc[i].count_at_last_vel = 0;
        s_enc[i].ring_idx          = 0U;
        s_enc[i].ring_sum          = 0;
        s_enc[i].rpm               = 0.0f;
        for (uint8_t k = 0U; k < (uint8_t)VEL_BOXCAR_N; k++) {
            s_enc[i].ring[k] = 0;
        }
    }
}

void encoder_sample_1khz(void)
{
    for (uint8_t i = 0U; i < (uint8_t)MOTOR_COUNT; i++) {
        encoder_t *e = &s_enc[i];

        uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(e->htim);
        /* Modular subtraction, then reinterpret as signed. Correct across
         * the 0xFFFF -> 0x0000 wrap in both directions. */
        int16_t  d   = (int16_t)((uint16_t)(now - e->last_cnt));

        e->last_cnt = now;
        e->count   += (int32_t)e->sign * (int32_t)d;
    }
}

void encoder_update_200hz(void)
{
    for (uint8_t i = 0U; i < (uint8_t)MOTOR_COUNT; i++) {
        encoder_t *e = &s_enc[i];

        int32_t delta = e->count - e->count_at_last_vel;
        e->count_at_last_vel = e->count;

        e->ring_sum -= e->ring[e->ring_idx];
        e->ring[e->ring_idx] = delta;
        e->ring_sum += delta;
        e->ring_idx = (uint8_t)((e->ring_idx + 1U) % (uint8_t)VEL_BOXCAR_N);

        /* ring_sum is counts over VEL_BOXCAR_N * 5 ms = 100 ms.
         * counts/s = ring_sum * (200 / N); rpm = counts/s * 60 / CPR. */
        e->rpm = ((float)e->ring_sum * (200.0f / (float)VEL_BOXCAR_N) * 60.0f)
                 / e->cpr;
    }
}

int32_t encoder_count(motor_id_t id)
{
    return s_enc[id].count;
}

void encoder_reset(void)
{
    for (uint8_t i = 0U; i < (uint8_t)MOTOR_COUNT; i++) {
        s_enc[i].count             = 0;
        s_enc[i].count_at_last_vel = 0;
        s_enc[i].ring_sum          = 0;
        s_enc[i].rpm               = 0.0f;
        for (uint8_t k = 0U; k < (uint8_t)VEL_BOXCAR_N; k++) {
            s_enc[i].ring[k] = 0;
        }
    }
}

float encoder_rpm_out(motor_id_t id)
{
    return s_enc[id].rpm;
}

float encoder_speed_mps(motor_id_t id)
{
    return (s_enc[id].rpm / 60.0f) * PI_F * WHEEL_DIA_M_F;
}

float encoder_cpr(motor_id_t id)
{
    return s_enc[id].cpr;
}
