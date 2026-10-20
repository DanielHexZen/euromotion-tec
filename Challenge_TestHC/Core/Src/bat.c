/* bat.c — spec §7.10 (REQ-BAT-01..04). */
#include "bat.h"

#include "par.h"
#include "sys_fault.h"

#define BAT_PERIOD_MS   20U
#define BAT_ADC_FS      4095.0f
#define BAT_VREF_V      3.3f
#define BAT_HOLD_MS     2000U     /* REQ-BAT-03: 2 s filter against stall sag */

static float    s_v;
static bool     s_present;
static bool     s_low;

#ifdef HAL_ADC_MODULE_ENABLED
static ADC_HandleTypeDef *s_adc;
static uint32_t s_t_last, s_t_warn, s_t_cut;
static bool     s_warn_sent;

static float raw_to_v(uint32_t raw)
{
    return (float)raw / BAT_ADC_FS * BAT_VREF_V * par()->bat_ratio;
}

st_t bat_init(void *hadc1)
{
    s_adc = (ADC_HandleTypeDef *)hadc1;
    if (s_adc == NULL) { return ST_NODEV; }
    if (HAL_ADC_Start(s_adc) != HAL_OK || HAL_ADC_PollForConversion(s_adc, 10U) != HAL_OK) {
        return ST_TIMEOUT;                                       /* init only: timeout allowed */
    }
    s_v = raw_to_v(HAL_ADC_GetValue(s_adc));
    s_present = (s_v >= BAT_ABSENT_V);
    s_t_last = HAL_GetTick();
    (void)HAL_ADC_Start(s_adc);
    return ST_OK;
}

void bat_service(void)
{
    if (s_adc == NULL) { return; }
    const uint32_t now = HAL_GetTick();
    if (now - s_t_last < BAT_PERIOD_MS) { return; }
    s_t_last = now;
    if (!__HAL_ADC_GET_FLAG(s_adc, ADC_FLAG_EOC)) { return; }   /* no timeout: try again later */
    const float raw_v = raw_to_v(HAL_ADC_GetValue(s_adc));
    (void)HAL_ADC_Start(s_adc);

    const float dt = (float)BAT_PERIOD_MS * 1.0e-3f;
    s_v += (dt / (par()->bat_tau + dt)) * (raw_v - s_v);
    s_present = (s_v >= BAT_ABSENT_V);
    if (!s_present) { s_t_warn = now; s_t_cut = now; s_warn_sent = false; return; }

    if (s_v >= par()->bat_warn_v) { s_t_warn = now; s_warn_sent = false; }
    else if (!s_warn_sent && now - s_t_warn >= BAT_HOLD_MS) {
        s_warn_sent = true;
        sys_event(EV_BAT_WARN, (int32_t)(s_v * 1000.0f));
    }
    if (s_v >= par()->bat_cut_v) { s_t_cut = now; s_low = false; }
    else if (!s_low && now - s_t_cut >= BAT_HOLD_MS) {
        s_low = true;
        sys_fault_raise(F_BAT_LOW, (int32_t)(s_v * 1000.0f));
    }
}
#else
st_t bat_init(void *hadc1) { (void)hadc1; return ST_NODEV; }
void bat_service(void) {}
#endif

float bat_v(void)       { return s_present ? s_v : 0.0f; }
bool  bat_present(void) { return s_present; }
bool  bat_low(void)     { return s_low; }
