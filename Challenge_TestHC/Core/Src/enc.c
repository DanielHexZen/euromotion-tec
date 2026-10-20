/* enc.c — spec §7.6 (REQ-ENC-01..03, REQ-COD-13). */
#include "enc.h"

typedef struct {
    volatile uint32_t seq;          /* odd while the ISR writes (REQ-EXE-06 b) */
    uint16_t cnt[2];
    uint32_t t_us;
} snap_t;

static snap_t s_snap;
static uint16_t s_last[2];
static enc_reading_t s_reading;

st_t enc_init(TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim8)
{
    if (HAL_TIM_Encoder_Start(htim3, TIM_CHANNEL_ALL) != HAL_OK
        || HAL_TIM_Encoder_Start(htim8, TIM_CHANNEL_ALL) != HAL_OK) {
        return ST_BUS;
    }
    s_last[0] = (uint16_t)TIM3->CNT;
    s_last[1] = (uint16_t)TIM8->CNT;
    s_snap.cnt[0] = s_last[0];                  /* until the first tick snapshot */
    s_snap.cnt[1] = s_last[1];
    s_snap.t_us = TIM2->CNT;
    s_reading = (enc_reading_t){ .t_us = s_snap.t_us };
    return ST_OK;
}

void enc_snapshot_isr(uint32_t t_us)
{
    s_snap.seq++;
    __DMB();
    s_snap.cnt[0] = (uint16_t)TIM3->CNT;
    s_snap.cnt[1] = (uint16_t)TIM8->CNT;
    s_snap.t_us = t_us;
    __DMB();
    s_snap.seq++;
}

void enc_update(void)
{
    uint16_t cnt[2] = { s_last[0], s_last[1] };
    uint32_t t_us = s_reading.t_us;
    for (uint32_t try = 0U; try < 4U; try++) {          /* REQ-COD-07: bounded retry */
        const uint32_t s0 = s_snap.seq;
        __DMB();
        const uint16_t c0 = s_snap.cnt[0], c1 = s_snap.cnt[1];
        const uint32_t t = s_snap.t_us;
        __DMB();
        if ((s0 & 1U) == 0U && s0 == s_snap.seq) {
            cnt[0] = c0; cnt[1] = c1; t_us = t;
            break;
        }
    }
    const int32_t sign[2] = { ENC_SIGN_L, ENC_SIGN_R };
    for (uint32_t i = 0U; i < 2U; i++) {
        const int16_t d = (int16_t)(uint16_t)(cnt[i] - s_last[i]);   /* wrap-safe, REQ-ENC-02 */
        s_reading.cnt[i] += sign[i] * (int32_t)d;
        s_last[i] = cnt[i];
    }
    s_reading.t_us = t_us;
}

enc_reading_t enc_read(void) { return s_reading; }

void enc_zero(void)
{
    s_reading.cnt[0] = 0;
    s_reading.cnt[1] = 0;
}
