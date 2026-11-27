/* imu_lsm6ds3.c — spec §7.11.1. Every register value is [VERIFY] against the LSM6DS3 datasheet
 * (OQ-1: the breakout board is not wired yet). probe/configure block and run in app_init only. */
#include "imu_lsm6ds3.h"

#define REG_INT1_CTRL  0x0DU
#define REG_WHO_AM_I   0x0FU
#define REG_CTRL1_XL   0x10U
#define REG_CTRL2_G    0x11U
#define REG_CTRL3_C    0x12U
#define REG_CTRL4_C    0x13U
#define REG_CTRL6_C    0x15U
#define REG_CTRL7_G    0x16U
#define REG_OUT_TEMP_L 0x20U

#define WHO_AM_I_LSM6DS3 0x69U     /* the TR-C reads 0x6A and is rejected on purpose */
#define SPI_TIMEOUT_MS   10U

static SPI_HandleTypeDef *s_spi;

void imu_lsm6ds3_bind(SPI_HandleTypeDef *hspi) { s_spi = hspi; }

void imu_lsm6ds3_set_spi_br(uint32_t br)
{
    s_spi->Instance->CR1 &= ~SPI_CR1_SPE;                  /* HAL_SPI_TransmitReceive_DMA sets it again */
    s_spi->Instance->CR1 = (s_spi->Instance->CR1 & ~SPI_CR1_BR) | (br << SPI_CR1_BR_Pos);
    s_spi->Init.BaudRatePrescaler = br << SPI_CR1_BR_Pos;
}

static void cs(bool low)
{
    IMU_CS_GPIO_Port->BSRR = low ? ((uint32_t)IMU_CS_Pin << 16U) : (uint32_t)IMU_CS_Pin;
}

static st_t rd(uint8_t reg, uint8_t *v)
{
    uint8_t tx[2] = { (uint8_t)(0x80U | reg), 0U };
    uint8_t rx[2];
    cs(true);
    const HAL_StatusTypeDef s = HAL_SPI_TransmitReceive(s_spi, tx, rx, 2U, SPI_TIMEOUT_MS);
    cs(false);
    *v = rx[1];
    return (s == HAL_OK) ? ST_OK : ST_BUS;
}

static st_t wr(uint8_t reg, uint8_t v)
{
    uint8_t tx[2] = { reg, v };
    uint8_t rx[2];
    cs(true);
    const HAL_StatusTypeDef s = HAL_SPI_TransmitReceive(s_spi, tx, rx, 2U, SPI_TIMEOUT_MS);
    cs(false);
    return (s == HAL_OK) ? ST_OK : ST_BUS;
}

static st_t probe(void)
{
    uint8_t id = 0U;
    const st_t s = rd(REG_WHO_AM_I, &id);
    if (s != ST_OK) { return s; }
    return (id == WHO_AM_I_LSM6DS3) ? ST_OK : ST_NODEV;
}

static const struct { uint8_t reg, val; } CFG[] = {
    { REG_CTRL3_C,   0x44U },   /* BDU = 1, IF_INC = 1, PP_OD = 0, H_LACTIVE = 0, 4-wire SPI */
    { REG_CTRL4_C,   0x00U },   /* LPF1_SEL_G = 0 (reset value, written to be explicit)       */
    { REG_CTRL6_C,   0x00U },   /* XL_HM_MODE = 0: high performance                           */
    { REG_CTRL7_G,   0x00U },   /* G_HM_MODE = 0, HP_EN_G = 0                                 */
    { REG_CTRL1_XL,  0x78U },   /* 833 Hz, +-4 g, BW_XL at reset value                        */
    { REG_CTRL2_G,   0x74U },   /* 833 Hz, FS_G = 01: +-500 dps (0x78 would be +-1000 dps)    */
    { REG_INT1_CTRL, 0x02U },   /* INT1_DRDY_G; last, so the sensors already run              */
};

static st_t configure(void)
{
    st_t s = wr(REG_CTRL3_C, 0x01U);                       /* SW_RESET, self-clearing */
    uint8_t v = 0xFFU;
    for (uint32_t i = 0U; i < 20U && s == ST_OK; i++) {
        HAL_Delay(1U);
        s = rd(REG_CTRL3_C, &v);
        if ((v & 0x01U) == 0U) { break; }
    }
    if (s == ST_OK && (v & 0x01U) != 0U) { s = ST_TIMEOUT; }
    for (uint32_t i = 0U; i < sizeof CFG / sizeof CFG[0] && s == ST_OK; i++) {
        s = wr(CFG[i].reg, CFG[i].val);
        if (s == ST_OK) { s = rd(CFG[i].reg, &v); }
        if (s == ST_OK && v != CFG[i].val) { s = ST_BUS; } /* read-back */
    }
    return s;
}

static void start_read_dma(uint8_t *rx, uint8_t *tx, uint16_t n)
{
    tx[0] = (uint8_t)(0x80U | REG_OUT_TEMP_L);             /* the other bytes stay 0 */
    if (HAL_SPI_TransmitReceive_DMA(s_spi, tx, rx, n) != HAL_OK) { imu_error_isr(); }
}

static int16_t s16(const uint8_t *p) { return (int16_t)(uint16_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8U)); }

/* rx[0] is the byte clocked in while the address went out. */
static void decode(const uint8_t *rx, int16_t g[3], int16_t a[3], int16_t *t)
{
    *t = s16(&rx[1]);
    for (uint32_t i = 0U; i < 3U; i++) {
        g[i] = s16(&rx[3U + 2U * i]);
        a[i] = s16(&rx[9U + 2U * i]);
    }
}

const imu_backend_t IMU_LSM6DS3 = {
    .probe = probe, .configure = configure, .start_read_dma = start_read_dma, .decode = decode,
    .gyro_lsb_radps = 17.5e-3f * DEG2RAD_F,      /* 17.50 mdps/LSB at +-500 dps */
    .acc_lsb_mps2   = 0.122e-3f * G_MPS2,        /* 0.122 mg/LSB at +-4 g       */
    .temp_lsb_c     = 1.0f / 16.0f,              /* 16 LSB/degC [VERIFY]        */
    .temp_off_c     = 25.0f,                     /* 0 LSB at 25 degC [VERIFY]   */
    .odr_hz         = 833.0f,
};
