/* vl53l1_platform.c — ST VL53L1X ULD platform layer (STSW-IMG009 v3.5.5 template, filled in).
 * Blocking HAL I2C with a 5 ms timeout: used ONLY inside vl_init() (spec §7.13.2, exception to
 * REQ-EXE-09). Run time uses the interrupt-driven state machine in vl.c. dev = 8-bit address. */
#include "vl53l1_platform.h"

#include "main.h"

#define TIMEOUT_MS 5U

static I2C_HandleTypeDef *s_i2c;

void vl53l1_platform_bind(void *hi2c) { s_i2c = (I2C_HandleTypeDef *)hi2c; }

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    return (HAL_I2C_Mem_Write(s_i2c, dev, index, I2C_MEMADD_SIZE_16BIT, pdata, (uint16_t)count, TIMEOUT_MS) == HAL_OK) ? 0 : -1;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    return (HAL_I2C_Mem_Read(s_i2c, dev, index, I2C_MEMADD_SIZE_16BIT, pdata, (uint16_t)count, TIMEOUT_MS) == HAL_OK) ? 0 : -1;
}

/* The sensor is big-endian. */
int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) { return VL53L1_WriteMulti(dev, index, &data, 1U); }

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
    uint8_t b[2] = { (uint8_t)(data >> 8), (uint8_t)data };
    return VL53L1_WriteMulti(dev, index, b, 2U);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
    uint8_t b[4] = { (uint8_t)(data >> 24), (uint8_t)(data >> 16), (uint8_t)(data >> 8), (uint8_t)data };
    return VL53L1_WriteMulti(dev, index, b, 4U);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata) { return VL53L1_ReadMulti(dev, index, pdata, 1U); }

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata)
{
    uint8_t b[2] = { 0U, 0U };
    const int8_t s = VL53L1_ReadMulti(dev, index, b, 2U);
    *pdata = (uint16_t)((uint16_t)b[0] << 8 | b[1]);
    return s;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *pdata)
{
    uint8_t b[4] = { 0U, 0U, 0U, 0U };
    const int8_t s = VL53L1_ReadMulti(dev, index, b, 4U);
    *pdata = (uint32_t)b[0] << 24 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 | b[3];
    return s;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
    (void)dev;
    HAL_Delay((uint32_t)wait_ms);
    return 0;
}
