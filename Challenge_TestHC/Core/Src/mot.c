/* mot.c — spec §7.5 (REQ-MOT-01..04). */
#include "mot.h"

#include "par.h"
#include "sys_fault.h"

static TIM_HandleTypeDef *s_t1;
static float s_cmd[2], s_applied[2];

static const uint32_t CH_FWD[2] = { TIM_CHANNEL_1, TIM_CHANNEL_3 };
static const uint32_t CH_REV[2] = { TIM_CHANNEL_2, TIM_CHANNEL_4 };
static const float    SIGN[2]   = { MOT_SIGN_L, MOT_SIGN_R };

st_t mot_init(TIM_HandleTypeDef *htim1)
{
    s_t1 = htim1;
    mot_enable(false);
    if (htim1->Instance->ARR != MOT_PWM_ARR) { return ST_RANGE; }
    for (uint8_t i = 0U; i < 2U; i++) {
        __HAL_TIM_SET_COMPARE(htim1, CH_FWD[i], 0U);
        __HAL_TIM_SET_COMPARE(htim1, CH_REV[i], 0U);
        if (HAL_TIM_PWM_Start(htim1, CH_FWD[i]) != HAL_OK || HAL_TIM_PWM_Start(htim1, CH_REV[i]) != HAL_OK) {
            return ST_BUS;
        }
    }
    return ST_OK;
}

void mot_enable(bool on)
{
    const GPIO_PinState s = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(EN_L_GPIO_Port, EN_L_Pin, s);
    HAL_GPIO_WritePin(EN_R_GPIO_Port, EN_R_Pin, s);
}

void mot_set(uint8_t side, float duty_d)
{
    if (!isfinite(duty_d)) {                                    /* REQ-COD-10 */
        sys_fault_raise(F_NUMERIC, (int32_t)side);
        duty_d = 0.0f;
    }
    s_cmd[side] = duty_d;
}

void mot_apply(void)
{
    const par_t *p = par();
    for (uint8_t i = 0U; i < 2U; i++) {
        float d = clampf(s_cmd[i], -p->mot_d_max, p->mot_d_max);            /* REQ-MOT-03 */
        if (fabsf(d) < p->mot_d_min) { d = 0.0f; }
        s_applied[i] = d;
        const float m = d * SIGN[i];                                        /* REQ-MOT-02 */
        const uint32_t ccr = (uint32_t)lroundf(fabsf(m) * (float)(MOT_PWM_ARR + 1U));
        if (m >= 0.0f) {                                    /* inactive channel first */
            __HAL_TIM_SET_COMPARE(s_t1, CH_REV[i], 0U);
            __HAL_TIM_SET_COMPARE(s_t1, CH_FWD[i], ccr);
        } else {
            __HAL_TIM_SET_COMPARE(s_t1, CH_FWD[i], 0U);
            __HAL_TIM_SET_COMPARE(s_t1, CH_REV[i], ccr);
        }
    }
}

float mot_duty(uint8_t side) { return s_applied[side]; }
