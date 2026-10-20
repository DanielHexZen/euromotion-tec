/* enc.h — encoders and odometry increments (spec §7.6). Device layer. */
#ifndef ENC_H
#define ENC_H

#include "main.h"
#include "units.h"

#define ENC_SIGN_L (1)      /* REQ-ENC-01: forward gives positive counts on both wheels */
#define ENC_SIGN_R (-1)

typedef struct {
    int32_t  cnt[2];        /* accumulated counts, wheel-forward positive */
    uint32_t t_us;          /* TIM2 timestamp of the snapshot             */
} enc_reading_t;

st_t          enc_init(TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim8);   /* init only */
void          enc_snapshot_isr(uint32_t t_us);   /* ISR (tick): latch the 16-bit counters + time */
void          enc_update(void);                  /* S_FAST: accumulate int32 totals              */
enc_reading_t enc_read(void);                    /* main loop: latest accumulated reading        */
void          enc_zero(void);                    /* only when stationary; consumers must resync  */

#endif
