/* vl.c — VL53L1X manager (spec §7.13), D-70 sensor set. So far only the init part that needs no
 * ST ULD: XSHUT sequence, boot wait, model ID, address assignment (§7.13.2 steps 1-2). It also
 * serves as the wiring check: the boot log shows every sensor present or absent. */
#include "vl.h"

#include "main.h"

#define ADDR_DEFAULT   0x29U
#define REG_ADDR       0x0001U    /* I2C_SLAVE__DEVICE_ADDRESS, 7-bit value (ULD SetI2CAddress) */
#define REG_BOOT       0x00E5U    /* FIRMWARE__SYSTEM_STATUS, bit 0 = booted (ULD BootState)     */
#define REG_MODEL_ID   0x010FU    /* IDENTIFICATION__MODEL_ID, 16 bit                            */
#define MODEL_ID       0xEACCU
#define I2C_TIMEOUT_MS 5U         /* §7.13.2: blocking platform calls, init only */
#define BOOT_POLL_MS   2U
#define BOOT_MAX_MS    100U

typedef struct {
    const char   *name;
    GPIO_TypeDef *xshut_port;
    uint16_t      xshut_pin;
    uint8_t       addr;            /* 7-bit, §7.13.1 */
    bool          fitted;          /* D-70 */
} vl_cfg_t;

static const vl_cfg_t VL_CFG[VL_N] = {
    [VL_G1] = { "vl G1", XSHUT_G1_GPIO_Port, XSHUT_G1_Pin, 0x30U, false },
    [VL_G2] = { "vl G2", XSHUT_G2_GPIO_Port, XSHUT_G2_Pin, 0x31U, false },
    [VL_G3] = { "vl G3", XSHUT_G3_GPIO_Port, XSHUT_G3_Pin, 0x32U, false },
    [VL_G4] = { "vl G4", XSHUT_G4_GPIO_Port, XSHUT_G4_Pin, 0x33U, false },
    [VL_F1] = { "vl F1", XSHUT_F1_GPIO_Port, XSHUT_F1_Pin, 0x34U, true  },
    [VL_F2] = { "vl F2", XSHUT_F2_GPIO_Port, XSHUT_F2_Pin, 0x35U, true  },
    [VL_L1] = { "vl L1", XSHUT_S1_GPIO_Port, XSHUT_S1_Pin, 0x36U, true  },   /* optional: absent is fine */
    [VL_S2] = { "vl S2", XSHUT_S2_GPIO_Port, XSHUT_S2_Pin, 0x37U, true  },
};

static I2C_HandleTypeDef *s_i2c;
static st_t s_state[VL_N];

static st_t hal_st(HAL_StatusTypeDef h)
{
    if (h == HAL_OK) { return ST_OK; }
    return (h == HAL_TIMEOUT) ? ST_TIMEOUT : ST_BUS;
}

static st_t rd(uint8_t addr, uint16_t reg, uint8_t *buf, uint16_t n)
{
    return hal_st(HAL_I2C_Mem_Read(s_i2c, (uint16_t)(addr << 1U), reg, I2C_MEMADD_SIZE_16BIT, buf, n, I2C_TIMEOUT_MS));
}

static st_t wr8(uint8_t addr, uint16_t reg, uint8_t v)
{
    return hal_st(HAL_I2C_Mem_Write(s_i2c, (uint16_t)(addr << 1U), reg, I2C_MEMADD_SIZE_16BIT, &v, 1U, I2C_TIMEOUT_MS));
}

static st_t model_ok(uint8_t addr)
{
    uint8_t id[2] = { 0U, 0U };
    const st_t s = rd(addr, REG_MODEL_ID, id, 2U);
    if (s != ST_OK) { return s; }
    return (((uint16_t)id[0] << 8U | id[1]) == MODEL_ID) ? ST_OK : ST_RANGE;
}

static st_t bring_up(const vl_cfg_t *c)
{
    HAL_GPIO_WritePin(c->xshut_port, c->xshut_pin, GPIO_PIN_SET);
    uint8_t b = 0U;
    st_t s = ST_NODEV;
    for (uint32_t t = 0U; t <= BOOT_MAX_MS; t += BOOT_POLL_MS) {
        HAL_Delay(BOOT_POLL_MS);
        if (rd(ADDR_DEFAULT, REG_BOOT, &b, 1U) == ST_OK && (b & 0x01U) != 0U) { s = ST_OK; break; }
    }
    if (s == ST_OK) { s = model_ok(ADDR_DEFAULT); }
    if (s == ST_OK) { s = wr8(ADDR_DEFAULT, REG_ADDR, c->addr); }
    if (s == ST_OK) { s = model_ok(c->addr); }       /* answers at its new address */
    if (s != ST_OK) { HAL_GPIO_WritePin(c->xshut_port, c->xshut_pin, GPIO_PIN_RESET); }   /* keep 0x29 free */
    return s;
}

st_t vl_init(void *hi2c2)
{
    s_i2c = (I2C_HandleTypeDef *)hi2c2;
    for (uint8_t i = 0U; i < VL_N; i++) {            /* step 1: all off (also the GPIO init state) */
        HAL_GPIO_WritePin(VL_CFG[i].xshut_port, VL_CFG[i].xshut_pin, GPIO_PIN_RESET);
        s_state[i] = ST_NODEV;
    }
    HAL_Delay(2U);
    uint32_t n = 0U;
    for (uint8_t i = 0U; i < VL_N; i++) {            /* step 2, in ID order */
        if (!VL_CFG[i].fitted) { continue; }
        s_state[i] = bring_up(&VL_CFG[i]);
        if (s_state[i] == ST_OK) { n++; }
    }
    return (n > 0U) ? ST_OK : ST_NODEV;
}

st_t vl_state(uint8_t id)        { return (id < VL_N) ? s_state[id] : ST_NODEV; }
const char *vl_name(uint8_t id)  { return (id < VL_N) ? VL_CFG[id].name : "vl ?"; }
