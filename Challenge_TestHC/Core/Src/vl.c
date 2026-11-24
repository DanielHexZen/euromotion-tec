/* vl.c — VL53L1X manager (spec §7.13), sensor set D-70: F1, F2, S2 and the optional L1 on I2C2.
 * Init (blocking, §7.13.2) uses the ST ULD; run time (§7.13.3) is an interrupt-driven state
 * machine with one transaction in flight: ready? -> 17-byte result -> clear. All sensors are
 * polled (D-73: the EXTI lines are not enabled in the .ioc). */
#include "vl.h"

#include "VL53L1X_api.h"
#include "main.h"
#include "par.h"
#include "sys_fault.h"
#include "sys_time.h"

#define ADDR_DEFAULT   0x29U
#define REG_MODEL_ID   0x010FU
#define MODEL_ID       0xEACCU
#define REG_READY      0x0031U    /* GPIO__TIO_HV_STATUS                              */
#define REG_RESULT     0x0089U    /* VL53L1_RESULT__RANGE_STATUS, 17 bytes             */
#define REG_CLEAR      0x0086U    /* SYSTEM__INTERRUPT_CLEAR                           */
#define I2C_TIMEOUT_MS 5U
#define BOOT_POLL_MS   2U
#define BOOT_MAX_MS    100U
#define F2_DELAY_MS    10U        /* F1 and F2 emitters out of phase (§7.13.2 step 3)  */
#define TXN_TIMEOUT_US 5000U      /* REQ-VL-04                                         */
#define REC_WINDOW_US  1000000U
#define REC_MAX        3U
#define DM_SHORT       1U

typedef struct {
    const char   *name;
    GPIO_TypeDef *xshut_port;
    uint16_t      xshut_pin;
    uint8_t       addr;            /* 7-bit, §7.13.1   */
    uint8_t       roi;             /* square ROI side  */
    bool          fitted;          /* D-70             */
} vl_cfg_t;

static const vl_cfg_t VL_CFG[VL_N] = {
    [VL_G1] = { "vl G1", XSHUT_G1_GPIO_Port, XSHUT_G1_Pin, 0x30U, 4U,  false },
    [VL_G2] = { "vl G2", XSHUT_G2_GPIO_Port, XSHUT_G2_Pin, 0x31U, 4U,  false },
    [VL_G3] = { "vl G3", XSHUT_G3_GPIO_Port, XSHUT_G3_Pin, 0x32U, 4U,  false },
    [VL_G4] = { "vl G4", XSHUT_G4_GPIO_Port, XSHUT_G4_Pin, 0x33U, 4U,  false },
    [VL_F1] = { "vl F1", XSHUT_F1_GPIO_Port, XSHUT_F1_Pin, 0x34U, 16U, true  },
    [VL_F2] = { "vl F2", XSHUT_F2_GPIO_Port, XSHUT_F2_Pin, 0x35U, 16U, true  },
    [VL_L1] = { "vl L1", XSHUT_S1_GPIO_Port, XSHUT_S1_Pin, 0x36U, 4U,  true  },   /* optional; ROI [DECIDE] OQ-12 */
    [VL_S2] = { "vl S2", XSHUT_S2_GPIO_Port, XSHUT_S2_Pin, 0x37U, 4U,  true  },
};

/* ULD range status mapping (VL53L1X_api.c status_rtn, STSW-IMG009 v3.5.5). */
static const uint8_t STATUS_RTN[24] = { 255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
                                        255, 255, 9, 13, 255, 255, 255, 255, 10, 6, 255, 255, 11, 12 };

typedef enum { B_IDLE, B_READY, B_RESULT, B_CLEAR } bus_step_t;

static I2C_HandleTypeDef *s_i2c;
static st_t      s_state[VL_N];
static uint8_t   s_pol[VL_N];                  /* data-ready level of GPIO__TIO_HV_STATUS bit 0 */
static uint32_t  s_next_poll_us[VL_N];
static vl_meas_t s_meas[VL_N];

static bus_step_t s_step;
static uint8_t    s_cur, s_rr;
static uint8_t    s_buf[17];
static uint8_t    s_clear_val = 0x01U;
static uint32_t   s_txn_us, s_ready_us;
static volatile bool s_done, s_err;
static uint32_t   s_rec_t0, s_rec_n, s_rec_total;

/* ---- init (blocking) ------------------------------------------------------------------- */
static st_t hal_st(HAL_StatusTypeDef h) { return (h == HAL_OK) ? ST_OK : ((h == HAL_TIMEOUT) ? ST_TIMEOUT : ST_BUS); }

static st_t model_ok(uint8_t addr)
{
    uint8_t id[2] = { 0U, 0U };
    const st_t s = hal_st(HAL_I2C_Mem_Read(s_i2c, (uint16_t)(addr << 1U), REG_MODEL_ID, I2C_MEMADD_SIZE_16BIT, id, 2U, I2C_TIMEOUT_MS));
    if (s != ST_OK) { return s; }
    return (((uint16_t)id[0] << 8U | id[1]) == MODEL_ID) ? ST_OK : ST_RANGE;
}

static st_t bring_up(uint8_t id)
{
    const vl_cfg_t *c = &VL_CFG[id];
    const par_t *q = par();
    HAL_GPIO_WritePin(c->xshut_port, c->xshut_pin, GPIO_PIN_SET);
    uint8_t booted = 0U;
    for (uint32_t t = 0U; t <= BOOT_MAX_MS && booted == 0U; t += BOOT_POLL_MS) {
        HAL_Delay(BOOT_POLL_MS);
        if (VL53L1X_BootState(ADDR_DEFAULT << 1U, &booted) != 0) { booted = 0U; }
    }
    st_t s = (booted != 0U) ? model_ok(ADDR_DEFAULT) : ST_NODEV;
    const uint16_t dev = (uint16_t)(c->addr << 1U);
    if (s == ST_OK) { s = (VL53L1X_SetI2CAddress(ADDR_DEFAULT << 1U, (uint8_t)dev) == 0) ? model_ok(c->addr) : ST_BUS; }
    if (s == ST_OK) {
        int8_t e = VL53L1X_SensorInit(dev);
        e |= VL53L1X_SetDistanceMode(dev, DM_SHORT);
        e |= VL53L1X_SetTimingBudgetInMs(dev, (uint16_t)q->vl_tb_ms);
        e |= VL53L1X_SetInterMeasurementInMs(dev, (uint32_t)q->vl_imp_ms);
        e |= VL53L1X_SetROI(dev, c->roi, c->roi);
        e |= VL53L1X_GetInterruptPolarity(dev, &s_pol[id]);
        if (e != 0) { s = ST_BUS; }
    }
    if (s != ST_OK) { HAL_GPIO_WritePin(c->xshut_port, c->xshut_pin, GPIO_PIN_RESET); }   /* keep 0x29 free */
    return s;
}

st_t vl_init(void *hi2c2)
{
    s_i2c = (I2C_HandleTypeDef *)hi2c2;
    vl53l1_platform_bind(hi2c2);
    for (uint8_t i = 0U; i < VL_N; i++) {            /* step 1: all off (also the GPIO init state) */
        HAL_GPIO_WritePin(VL_CFG[i].xshut_port, VL_CFG[i].xshut_pin, GPIO_PIN_RESET);
        s_state[i] = ST_NODEV;
    }
    HAL_Delay(2U);
    uint32_t n = 0U;
    for (uint8_t i = 0U; i < VL_N; i++) {            /* step 2, in ID order */
        if (!VL_CFG[i].fitted) { continue; }
        s_state[i] = bring_up(i);
        if (s_state[i] == ST_OK) { n++; }
    }
    for (uint8_t i = 0U; i < VL_N; i++) {            /* step 3 */
        if (s_state[i] != ST_OK) { continue; }
        if (VL53L1X_StartRanging((uint16_t)(VL_CFG[i].addr << 1U)) != 0) { s_state[i] = ST_BUS; continue; }
        if (i == VL_F1) { HAL_Delay(F2_DELAY_MS); }
    }
    return (n > 0U) ? ST_OK : ST_NODEV;
}

st_t vl_state(uint8_t id)        { return (id < VL_N) ? s_state[id] : ST_NODEV; }
const char *vl_name(uint8_t id)  { return (id < VL_N) ? VL_CFG[id].name : "vl ?"; }
const vl_meas_t *vl_get(uint8_t id) { return (id < VL_N) ? &s_meas[id] : NULL; }

/* ---- run time (non-blocking) ------------------------------------------------------------ */
void vl_i2c_done_isr(void)  { s_done = true; }
void vl_i2c_error_isr(void) { s_err = true; }

static bool start_rd(uint16_t reg, uint16_t n)
{
    s_done = false; s_err = false; s_txn_us = sys_time_us();
    return HAL_I2C_Mem_Read_IT(s_i2c, (uint16_t)(VL_CFG[s_cur].addr << 1U), reg, I2C_MEMADD_SIZE_16BIT, s_buf, n) == HAL_OK;
}

static bool start_clear(void)
{
    s_done = false; s_err = false; s_txn_us = sys_time_us();
    return HAL_I2C_Mem_Write_IT(s_i2c, (uint16_t)(VL_CFG[s_cur].addr << 1U), REG_CLEAR, I2C_MEMADD_SIZE_16BIT, &s_clear_val, 1U) == HAL_OK;
}

/* REQ-VL-04: de-init, 9 clocks on SCL (PB10) with SDA (PC12) released, STOP, re-init. ~100 us. */
static void bus_recover(void)
{
    (void)HAL_I2C_DeInit(s_i2c);
    GPIO_InitTypeDef g = { .Mode = GPIO_MODE_OUTPUT_OD, .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_FREQ_HIGH };
    g.Pin = GPIO_PIN_10; HAL_GPIO_Init(GPIOB, &g);
    g.Pin = GPIO_PIN_12; HAL_GPIO_Init(GPIOC, &g);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
    for (uint32_t k = 0U; k < 18U; k++) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
        const uint32_t t0 = sys_time_us();
        while ((uint32_t)(sys_time_us() - t0) < 5U) {}
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);                  /* STOP: SDA low -> high, SCL high */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
    const uint32_t t0 = sys_time_us();
    while ((uint32_t)(sys_time_us() - t0) < 5U) {}
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
    (void)HAL_I2C_Init(s_i2c);                                            /* MSP init restores the AF pins */

    const uint32_t now = sys_time_us();
    if ((uint32_t)(now - s_rec_t0) > REC_WINDOW_US) { s_rec_t0 = now; s_rec_n = 0U; }
    s_rec_total++;
    if (++s_rec_n == REC_MAX) { sys_fault_raise_as(F_VL_BUS2, (int32_t)s_rec_total, R1); }   /* R3 only in AUTO (msn) */
}

static void fail(void)
{
    (void)HAL_I2C_Master_Abort_IT(s_i2c, (uint16_t)(VL_CFG[s_cur].addr << 1U));
    bus_recover();
    s_step = B_IDLE;
}

static void publish(uint32_t t_ready_us)
{
    const par_t *q = par();
    const uint8_t rs = s_buf[0] & 0x1FU;
    vl_meas_t *m = &s_meas[s_cur];
    m->status = (rs < 24U) ? STATUS_RTN[rs] : 255U;
    m->range_m = (float)((uint16_t)s_buf[13] << 8 | s_buf[14]) * 1.0e-3f;
    m->ambient_kcps = (float)((uint16_t)s_buf[7] << 8 | s_buf[8]) * 8.0f;      /* as VL53L1X_GetResult */
    m->signal_kcps = (float)((uint16_t)s_buf[15] << 8 | s_buf[16]) * 8.0f;
    m->t_us = t_ready_us - (uint32_t)(q->vl_tb_ms * 500.0f);                    /* REQ-VL-03 */
    m->seq++;
}

void vl_service(void)
{
    const uint32_t now = sys_time_us();
    if (s_step != B_IDLE) {
        if (s_err) { fail(); return; }
        if (!s_done) {
            if ((uint32_t)(now - s_txn_us) > TXN_TIMEOUT_US) { fail(); }
            return;
        }
        s_done = false;
        switch (s_step) {
        case B_READY:
            if ((s_buf[0] & 0x01U) == s_pol[s_cur]) {
                s_ready_us = s_txn_us - (uint32_t)(par()->vl_poll_ms * 500.0f);   /* REQ-VL-03, polled */
                s_step = start_rd(REG_RESULT, 17U) ? B_RESULT : B_IDLE;
            } else {
                s_step = B_IDLE;
            }
            return;
        case B_RESULT:
            publish(s_ready_us);
            s_step = start_clear() ? B_CLEAR : B_IDLE;
            return;
        case B_CLEAR:
        case B_IDLE:
        default:
            s_step = B_IDLE;
            return;
        }
    }
    /* REQ-VL-02: next due sensor, round robin */
    const uint32_t poll_us = (uint32_t)(par()->vl_poll_ms * 1000.0f);
    for (uint8_t k = 0U; k < VL_N; k++) {
        const uint8_t i = (uint8_t)((s_rr + 1U + k) % VL_N);
        if (s_state[i] != ST_OK || (int32_t)(now - s_next_poll_us[i]) < 0) { continue; }
        s_rr = i;
        s_cur = i;
        s_next_poll_us[i] = now + poll_us;
        s_step = start_rd(REG_READY, 1U) ? B_READY : B_IDLE;
        return;
    }
}
