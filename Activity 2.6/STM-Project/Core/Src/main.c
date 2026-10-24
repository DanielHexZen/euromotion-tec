/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   Complete Vehicle Control Firmware (Points 8 to 16)
  *          Board : NUCLEO-F446RE (STM32F446RETx, SYSCLK 180 MHz)
  *
  *  Pinout Configuration (matching CubeMX & wiring table):
  *   Motor driver: L298N double H-bridge (replaces 2x BTS7960 for the interim test)
  *   PA8        TIM1_CH1    L298N ENA  (PWM motor L, 5 kHz)
  *   PA9        TIM1_CH2    L298N ENB  (PWM motor R, 5 kHz)
  *   PB12, PB13 GPIO_Output L298N IN1, IN2 (direction motor L)
  *   PB14, PB15 GPIO_Output L298N IN3, IN4 (direction motor R)
  *   PA10, PA11 TIM1_CH3/4  not used (CCR = 0, leave unconnected)
  *   Truth table per motor: IN_a=1,IN_b=0 fwd | 0,1 rev | 0,0 + EN=1 brake
  *                          EN=0 coast | 1,1 is NEVER written by this code
  *   PA6, PA7   TIM3_CH1/2  Left  Encoder A, B
  *   PC6, PC7   TIM8_CH1/2  Right Encoder A, B
  *   PB8, PB9   I2C1        OLED Display SSD1306 (SCL = PB8, SDA = PB9)
  *   PA15       TIM2_CH1    HC-SR04 Echo (Input Capture, 1 MHz timebase)
  *   PC11       GPIO_Output HC-SR04 Trig (US_TRIG)
  *   PB6, PB7   USART1      Wireless module HC-12 / Bluetooth (9600 Bd)
  *   PA2, PA3   USART2      ST-Link Virtual COM Port (115200 Bd)
  *   TIM6       Internal    50 Hz (Ts = 20 ms) discrete PID control interrupt
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
typedef enum {
  PH_IDLE = 0,
  PH_MOVE,        /* following the trapezoidal speed profile           */
  PH_SETTLE,      /* profile finished, trimming final millimetres      */
  PH_FAULT,       /* motor stalled or encoder sign error               */
  PH_OBSTACLE     /* PAUSED: obstacle detected at <= 30 cm (Point 15)  */
} Phase_t;

typedef struct {
  TIM_HandleTypeDef *enc;
  uint32_t      pwm_ch;          /* TIM1 channel on the L298N EN input    */
  GPIO_TypeDef *in_a_port;       /* IN1 / IN3: high = forward             */
  uint16_t      in_a_pin;
  GPIO_TypeDef *in_b_port;       /* IN2 / IN4: high = reverse             */
  uint16_t      in_b_pin;
  int8_t   enc_sign;
  int8_t   mot_sign;
  float    mmpc;
  float    ks;
  float    kv;
  float    ks_break;
  uint16_t last_raw;
  int32_t  total;
  int32_t  start_total;
  float    speed;
  float    integ;
  float    duty;
  uint8_t  moved;
  uint8_t  rolling;
  uint8_t  fault_cnt;
} Wheel_t;

/* Private define ------------------------------------------------------------*/
#define WHEEL_DIAMETER_MM     72.6f
#define COUNTS_PER_WHEEL_REV  3584.0f
#define PI_F                  3.14159265f
#define MM_PER_COUNT          ((PI_F * WHEEL_DIAMETER_MM) / COUNTS_PER_WHEEL_REV)

#define ENC_L_SIGN            1
#define ENC_R_SIGN            -1
#define MOT_L_SIGN            1
#define MOT_R_SIGN            -1

/* ---- L298N ---- */
#define MOT_L_PWM_CH          TIM_CHANNEL_1   /* PA8  -> ENA */
#define MOT_R_PWM_CH          TIM_CHANNEL_2   /* PA9  -> ENB */
#define L298_IN1_PORT         GPIOB           /* motor L, high = forward */
#define L298_IN1_PIN          GPIO_PIN_12
#define L298_IN2_PORT         GPIOB           /* motor L, high = reverse */
#define L298_IN2_PIN          GPIO_PIN_13
#define L298_IN3_PORT         GPIOB           /* motor R, high = forward */
#define L298_IN3_PIN          GPIO_PIN_14
#define L298_IN4_PORT         GPIOB           /* motor R, high = reverse */
#define L298_IN4_PIN          GPIO_PIN_15
#define L298_IN_ALL_PINS      (GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)
/* 5 kHz: the opto-isolated L298 module cannot follow 20 kHz (duty floor).
   PWM_Setup() gives ARR = 180 MHz / 5 kHz - 1 = 35999, same as the .ioc. */
#define PWM_FREQ_HZ           5000U

#define CTRL_PERIOD_S         0.02f           /* 50 Hz (Ts = 20 ms) */
#define CTRL_HZ_EXPECTED      50U
#define SPEED_FILTER_ALPHA    0.5f

/* Feed-forward for the L298N: bench measurement week 5 (L298, no load).
   Kv [duty/rpm] -> % per mm/s: Kv * 100 * 60 / (pi * D).
   The BTS7960 values (9.8/4.1 %) do NOT apply: the L298 drops ~2 V.      */
#define KV_L_DUTY_RPM         0.00492f
#define KV_R_DUTY_RPM         0.00507f
#define RPM_PER_MMS           (60.0f / (PI_F * WHEEL_DIAMETER_MM))
#define FF_KS_L_PCT           11.0f     /* running intercept, left        */
#define FF_KV_L               (KV_L_DUTY_RPM * 100.0f * RPM_PER_MMS)  /* 0.129 */
#define FF_KS_R_PCT           12.7f
#define FF_KV_R               (KV_R_DUTY_RPM * 100.0f * RPM_PER_MMS)  /* 0.133 */
#define KS_BREAK_L_PCT        14.1f     /* breakaway from standstill      */
#define KS_BREAK_R_PCT        16.9f
#define TAU_PLANT_S           0.050f
#define V_MOVING_MM_S         8.0f
#define V_STOPPED_MM_S        3.0f
#define V_ZERO_MM_S           2.0f

/* Tuned PI Controller Gains (SIMC Balanced: Kp = 0.045, Ki = 0.898) */
#define KP_SPEED              0.045f
#define KI_SPEED              0.898f
#define INTEG_MAX_PCT         20.0f

#define KP_POS                2.0f
#define V_CORR_MAX_MM_S       100.0f
#define K_SYNC                3.0f
#define SETTLE_V_MM_S         20.0f
#define POS_TOL_MM            1.5f
#define SETTLE_TICKS          50U
#define FAULT_TICKS           30U

/* Obstacle Avoidance (HC-SR04 on PA15 = TIM2_CH1 Echo, PC11 = Trig) */
#define OBSTACLE_STOP_CM      30.0f
#define OBSTACLE_RESUME_CM    33.0f
#define US_FAILSAFE           0       /* 1: no valid echo for 300 ms = obstacle */
#define US_TRIG_PORT          GPIOC
#define US_TRIG_PIN           GPIO_PIN_11

/* User Defaults */
#define V_NOM_DEFAULT         300.0f
#define V_NOM_MIN             20L
#define V_NOM_MAX             450L
#define ACC_MS_DEFAULT        1000U
#define DEC_MS_DEFAULT        1000U
#define MIN_DIST_MM           5L
#define MAX_DIST_MM           20000L

#define EVT_DONE              1U
#define EVT_FAULT             2U
#define EVT_TIMEOUT           3U
#define EVT_OBS_STOP          4U
#define EVT_OBS_RESUME        5U

#define RX_LINE_MAX           24
#define AUTO_PRINT_MS         250U
#define DEADMAN_MS            2000U

#define OBS_LEAD_S            0.23f   /* Vorhalt [s]: 7 cm bei 300 mm/s gemessen */

/* Peripheral Handles --------------------------------------------------------*/
I2C_HandleTypeDef  hi2c1;
TIM_HandleTypeDef  htim1;
TIM_HandleTypeDef  htim2;
TIM_HandleTypeDef  htim3;
TIM_HandleTypeDef  htim6;
TIM_HandleTypeDef  htim8;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Global Variables for STM32CubeMonitor (Point 14) --------------------------*/
volatile float position_l_mm    = 0.0f;
volatile float position_r_mm    = 0.0f;
volatile float speed_l_mms      = 0.0f;
volatile float speed_r_mms      = 0.0f;
volatile float target_speed_mms = 0.0f;
volatile float target_pos_mm    = 0.0f;
volatile float duty_l_pct       = 0.0f;
volatile float duty_r_pct       = 0.0f;
volatile float sync_err_mm      = 0.0f;
volatile float obstacle_dist_cm = 100.0f;
volatile float position_mm       = 0.0f;
volatile float speed_mms         = 0.0f;
volatile float accel_mms2        = 0.0f;
volatile float target_accel_mms2 = 0.0f;

/* RAM drive logger, dumped with command 'l' */
#define LOG_N 1500U   /* 30 s at 50 Hz */
typedef struct { int32_t x_x10; int32_t xs_x10; int16_t v; int16_t vs; } LogS_t;
static LogS_t log_buf[LOG_N];
static volatile uint16_t log_n = 0;
static volatile uint8_t  log_dump = 0;

/* Private Variables ---------------------------------------------------------*/
static Wheel_t wl = { .enc = &htim3, .pwm_ch = MOT_L_PWM_CH,
                      .in_a_port = L298_IN1_PORT, .in_a_pin = L298_IN1_PIN,
                      .in_b_port = L298_IN2_PORT, .in_b_pin = L298_IN2_PIN,
                      .enc_sign = ENC_L_SIGN, .mot_sign = MOT_L_SIGN, .mmpc = MM_PER_COUNT,
                      .ks = FF_KS_L_PCT, .kv = FF_KV_L, .ks_break = KS_BREAK_L_PCT };
static Wheel_t wr = { .enc = &htim8, .pwm_ch = MOT_R_PWM_CH,
                      .in_a_port = L298_IN3_PORT, .in_a_pin = L298_IN3_PIN,
                      .in_b_port = L298_IN4_PORT, .in_b_pin = L298_IN4_PIN,
                      .enc_sign = ENC_R_SIGN, .mot_sign = MOT_R_SIGN, .mmpc = MM_PER_COUNT,
                      .ks = FF_KS_R_PCT, .kv = FF_KV_R, .ks_break = KS_BREAK_R_PCT };

static volatile Phase_t phase = PH_IDLE;
static volatile uint8_t evt   = 0;
static uint32_t pwm_arr       = 8999U;

static float kp_speed = KP_SPEED;
static float ki_speed = KI_SPEED;
static float k_sync   = K_SYNC;

static float    v_nom  = V_NOM_DEFAULT;
static uint32_t acc_ms = ACC_MS_DEFAULT;
static uint32_t dec_ms = DEC_MS_DEFAULT;

static float    mv_D = 0, mv_Sn = 0, mv_a = 1, mv_d = 1, mv_T = 0, mv_t1 = 0, mv_t2 = 0;
static int8_t   mv_dir = 1;
static long     mv_target_mm = 0;
static uint32_t mv_ticks = 0;
static uint32_t settle_ticks = 0;
static volatile float mv_sync_max = 0.0f;

static volatile char    rx_line[RX_LINE_MAX];
static volatile uint8_t rx_len     = 0;
static volatile uint8_t line_ready = 0;
static uint8_t rx_byte;
static uint8_t rx_byte1;
static uint8_t auto_print = 0;
static volatile uint32_t t_last_rx = 0;
static uint8_t open_loop = 0;

/* HC-SR04 Input Capture state on TIM2_CH1 (PA15) */
static uint32_t ic_t1 = 0;
static uint8_t  ic_edge = 0;
static volatile uint32_t t_echo_ok = 0;   /* last valid echo [ms]          */
static volatile uint8_t  fault_why = 0;   /* bit 4: wheel R, low: 1 stall, 2 runaway */

/* Function Prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

static void Car_Init(void);
static void Car_Loop(void);
static void PWM_Setup(void);
static void Serial_Print(const char *txt);
static void Serial_HandleCommand(const char *cmd);
static void Serial_PrintStatus(void);
static void Serial_PrintParams(void);
static void Wheel_SetDuty(Wheel_t *w, float duty);
static void Motors_Stop(void);
static int  Move_Start(long dist_mm);
static void OLED_Init(void);
static void OLED_FlushPage(uint8_t page);
static void OLED_UpdateStatus(void);
static void Ultrasonic_Trigger(void);

/* ===========================================================================
   BUILT-IN MINIMAL SSD1306 I2C OLED DRIVER (PB8=SCL, PB9=SDA, Addr 0x78)
   =========================================================================== */
#define OLED_I2C_ADDR  0x78U
static uint8_t oled_buf[1024];
static uint8_t oled_ok = 0;

static const uint8_t font5x7[][5] = {
  {0x00,0x00,0x00,0x00,0x00}, /* ' ' 32 */
  {0x00,0x00,0x5F,0x00,0x00}, /* '!' 33 */
  {0x00,0x07,0x00,0x07,0x00}, /* '"' 34 */
  {0x14,0x7F,0x14,0x7F,0x14}, /* '#' 35 */
  {0x24,0x2A,0x7F,0x2A,0x12}, /* '$' 36 */
  {0x23,0x13,0x08,0x64,0x62}, /* '%' 37 */
  {0x36,0x49,0x55,0x22,0x50}, /* '&' 38 */
  {0x00,0x05,0x03,0x00,0x00}, /* ''' 39 */
  {0x00,0x1C,0x22,0x41,0x00}, /* '(' 40 */
  {0x00,0x41,0x22,0x1C,0x00}, /* ')' 41 */
  {0x14,0x08,0x3E,0x08,0x14}, /* '*' 42 */
  {0x08,0x08,0x3E,0x08,0x08}, /* '+' 43 */
  {0x00,0x50,0x30,0x00,0x00}, /* ',' 44 */
  {0x08,0x08,0x08,0x08,0x08}, /* '-' 45 */
  {0x00,0x60,0x60,0x00,0x00}, /* '.' 46 */
  {0x20,0x10,0x08,0x04,0x02}, /* '/' 47 */
  {0x3E,0x51,0x49,0x45,0x3E}, /* '0' 48 */
  {0x00,0x42,0x7F,0x40,0x00}, /* '1' 49 */
  {0x42,0x61,0x51,0x49,0x46}, /* '2' 50 */
  {0x21,0x41,0x45,0x4B,0x31}, /* '3' 51 */
  {0x18,0x14,0x12,0x7F,0x10}, /* '4' 52 */
  {0x27,0x45,0x45,0x45,0x39}, /* '5' 53 */
  {0x3C,0x4A,0x49,0x49,0x30}, /* '6' 54 */
  {0x01,0x71,0x09,0x05,0x03}, /* '7' 55 */
  {0x36,0x49,0x49,0x49,0x36}, /* '8' 56 */
  {0x06,0x49,0x49,0x29,0x1E}, /* '9' 57 */
  {0x00,0x36,0x36,0x00,0x00}, /* ':' 58 */
  {0x00,0x56,0x36,0x00,0x00}, /* ';' 59 */
  {0x08,0x14,0x22,0x41,0x00}, /* '<' 60 */
  {0x14,0x14,0x14,0x14,0x14}, /* '=' 61 */
  {0x00,0x41,0x22,0x14,0x08}, /* '>' 62 */
  {0x02,0x01,0x51,0x09,0x06}, /* '?' 63 */
  {0x32,0x49,0x79,0x41,0x3E}, /* '@' 64 */
  {0x7E,0x11,0x11,0x11,0x7E}, /* 'A' 65 */
  {0x7F,0x49,0x49,0x49,0x36}, /* 'B' 66 */
  {0x3E,0x41,0x41,0x41,0x22}, /* 'C' 67 */
  {0x7F,0x41,0x41,0x22,0x1C}, /* 'D' 68 */
  {0x7F,0x49,0x49,0x49,0x41}, /* 'E' 69 */
  {0x7F,0x09,0x09,0x09,0x01}, /* 'F' 70 */
  {0x3E,0x41,0x49,0x49,0x7A}, /* 'G' 71 */
  {0x7F,0x08,0x08,0x08,0x7F}, /* 'H' 72 */
  {0x00,0x41,0x7F,0x41,0x00}, /* 'I' 73 */
  {0x20,0x40,0x41,0x3F,0x01}, /* 'J' 74 */
  {0x7F,0x08,0x14,0x22,0x41}, /* 'K' 75 */
  {0x7F,0x40,0x40,0x40,0x40}, /* 'L' 76 */
  {0x7F,0x02,0x0C,0x02,0x7F}, /* 'M' 77 */
  {0x7F,0x04,0x08,0x10,0x7F}, /* 'N' 78 */
  {0x3E,0x41,0x41,0x41,0x3E}, /* 'O' 79 */
  {0x7F,0x09,0x09,0x09,0x06}, /* 'P' 80 */
  {0x3E,0x41,0x51,0x21,0x5E}, /* 'Q' 81 */
  {0x7F,0x09,0x19,0x29,0x46}, /* 'R' 82 */
  {0x46,0x49,0x49,0x49,0x31}, /* 'S' 83 */
  {0x01,0x01,0x7F,0x01,0x01}, /* 'T' 84 */
  {0x3F,0x40,0x40,0x40,0x3F}, /* 'U' 85 */
  {0x1F,0x20,0x40,0x20,0x1F}, /* 'V' 86 */
  {0x3F,0x40,0x38,0x40,0x3F}, /* 'W' 87 */
  {0x63,0x14,0x08,0x14,0x63}, /* 'X' 88 */
  {0x07,0x08,0x70,0x08,0x07}, /* 'Y' 89 */
  {0x61,0x51,0x49,0x45,0x43}  /* 'Z' 90 */
};

static void OLED_Cmd(uint8_t cmd)
{
  uint8_t d[2] = { 0x00, cmd };
  HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR, d, 2, 10);
}

static void OLED_Init(void)
{
  if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_I2C_ADDR, 2, 20) != HAL_OK) {
    oled_ok = 0;
    return;
  }
  oled_ok = 1;
  const uint8_t init_cmds[] = {
    0xAE, 0x20, 0x02, 0xB0, 0xC8, 0x00, 0x10, 0x40,
    0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
    0x00, 0xD5, 0xF0, 0xD9, 0x22, 0xDA, 0x12, 0xDB,
    0x20, 0x8D, 0x14, 0xAF
  };
  for (unsigned i = 0; i < sizeof(init_cmds); i++) OLED_Cmd(init_cmds[i]);
  memset(oled_buf, 0, sizeof(oled_buf));
  for (uint8_t p = 0; p < 8; p++) OLED_FlushPage(p);   /* GDDRAM is random after power-up */
}

static void OLED_DrawStr(uint8_t page, uint8_t col, const char *s)
{
  if (page > 7) return;
  while (*s && col + 6 <= 128) {
    char c = *s++;
    if (c >= 'a' && c <= 'z') c -= 32; /* convert lowercase to uppercase for 5x7 font */
    if (c < 32 || c > 90) c = ' ';
    memcpy(&oled_buf[page * 128 + col], font5x7[c - 32], 5);
    oled_buf[page * 128 + col + 5] = 0x00;
    col += 6;
  }
}

static void OLED_FlushPage(uint8_t page)
{
  if (!oled_ok) return;
  OLED_Cmd(0xB0 + page);
  OLED_Cmd(0x00);
  OLED_Cmd(0x10);
  uint8_t pkt[129];
  pkt[0] = 0x40;
  memcpy(&pkt[1], &oled_buf[page * 128], 128);
  HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR, pkt, 129, 15);
}

/* ===========================================================================
   MAIN APPLICATION ENTRY POINT
   =========================================================================== */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  Car_Init();

  while (1)
  {
    Car_Loop();
  }
}

/* ===========================================================================
   CLOCK & PERIPHERAL INITIALIZATION (MATCHING CUBEMX SCREENSHOTS)
   =========================================================================== */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
  /* PB8 = I2C1_SCL, PB9 = I2C1_SDA (OLED Display) */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);
}

static void MX_TIM1_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 35999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) Error_Handler();

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();

  HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM2_Init(void)
{
  /* PA15 = TIM2_CH1 Input Capture (HC-SR04 Echo, 1 MHz timebase -> 1 us/tick) */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 89; /* 90 MHz / (89 + 1) = 1 MHz (1 us resolution) */
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
}

static void MX_TIM3_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 199; /* 50 Hz (Ts = 20 ms) */
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM8_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, L298_IN_ALL_PINS, GPIO_PIN_RESET);   /* all IN low */
  HAL_GPIO_WritePin(HC12_Set_GPIO_Port, HC12_Set_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* PB12..PB15 = L298N IN1..IN4, push-pull, pull-down, low after reset */
  GPIO_InitStruct.Pin = L298_IN_ALL_PINS;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = HC12_Set_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HC12_Set_GPIO_Port, &GPIO_InitStruct);

  /* PC11 = HC-SR04 Trigger Output (US_TRIG) */
  GPIO_InitStruct.Pin = US_TRIG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(US_TRIG_PORT, &GPIO_InitStruct);
}

/* ===========================================================================
   CONTROL LOOP, H-BRIDGE, ENCODERS, ULTRASONIC & OLED LOGIC
   =========================================================================== */
static float clampf(float x, float lo, float hi)
{
  return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static uint8_t Moving(void)
{
  return (phase == PH_MOVE || phase == PH_SETTLE || phase == PH_OBSTACLE);
}

static void PWM_Setup(void)
{
  uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
  uint32_t tclk  = ((RCC->CFGR & RCC_CFGR_PPRE2) == 0U) ? pclk2 : 2U * pclk2;
  pwm_arr = (tclk / PWM_FREQ_HZ) - 1U;
  __HAL_TIM_SET_PRESCALER(&htim1, 0U);
  __HAL_TIM_SET_AUTORELOAD(&htim1, pwm_arr);
  htim1.Instance->EGR = TIM_EGR_UG;
}

static uint32_t Ctrl_RateHz(void)
{
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  uint32_t tclk  = ((RCC->CFGR & RCC_CFGR_PPRE1) == 0U) ? pclk1 : 2U * pclk1;
  return tclk / ((TIM6->PSC + 1U) * (TIM6->ARR + 1U));
}

/* L298N: EN carries the PWM, IN_a/IN_b select the direction.
   duty > 0 : IN_a = 1, IN_b = 0, EN = PWM          (forward)
   duty < 0 : IN_a = 0, IN_b = 1, EN = PWM          (reverse)
   duty = 0 : IN_a = 0, IN_b = 0, EN = 100 %        (brake: both low sides on)
   The inactive input is always released BEFORE the active one is set, so
   IN_a = IN_b = 1 never occurs, not even for one instruction.              */
static void Wheel_SetDuty(Wheel_t *w, float duty)
{
  duty = clampf(duty, -100.0f, 100.0f);
  w->duty = duty;
  float s = duty * (float)w->mot_sign;
  uint32_t ccr = (uint32_t)(fabsf(s) * (float)(pwm_arr + 1U) / 100.0f + 0.5f);
  if (ccr > pwm_arr + 1U) ccr = pwm_arr + 1U;

  if (s > 0.0f) {
    w->in_b_port->BSRR = (uint32_t)w->in_b_pin << 16U;   /* IN_b low  */
    w->in_a_port->BSRR = (uint32_t)w->in_a_pin;          /* IN_a high */
    __HAL_TIM_SET_COMPARE(&htim1, w->pwm_ch, ccr);
  } else if (s < 0.0f) {
    w->in_a_port->BSRR = (uint32_t)w->in_a_pin << 16U;   /* IN_a low  */
    w->in_b_port->BSRR = (uint32_t)w->in_b_pin;          /* IN_b high */
    __HAL_TIM_SET_COMPARE(&htim1, w->pwm_ch, ccr);
  } else {
    w->in_a_port->BSRR = (uint32_t)w->in_a_pin << 16U;
    w->in_b_port->BSRR = (uint32_t)w->in_b_pin << 16U;
    __HAL_TIM_SET_COMPARE(&htim1, w->pwm_ch, pwm_arr + 1U);   /* brake */
  }
}

static void Motors_Stop(void)
{
  Wheel_SetDuty(&wl, 0.0f);
  Wheel_SetDuty(&wr, 0.0f);
  wl.integ = 0.0f;
  wr.integ = 0.0f;
}

static void Wheel_UpdateEncoder(Wheel_t *w)
{
  uint16_t now   = (uint16_t)__HAL_TIM_GET_COUNTER(w->enc);
  int16_t  raw   = (int16_t)(now - w->last_raw);
  w->last_raw    = now;
  int32_t  delta = (int32_t)w->enc_sign * (int32_t)raw;
  w->total      += delta;
  float v_raw    = (float)delta * w->mmpc / CTRL_PERIOD_S;
  w->speed      += SPEED_FILTER_ALPHA * (v_raw - w->speed);

  float as = fabsf(w->speed);
  if (as >= V_MOVING_MM_S)       w->rolling = 1U;
  else if (as < V_STOPPED_MM_S)  w->rolling = 0U;
}

static void Profile_Eval(float t, float *v, float *x, float *a)
{
  if (t < mv_t1) {
    *v = mv_Sn * t / mv_a;
    *x = 0.5f * mv_Sn * t * t / mv_a;
    *a = mv_Sn / mv_a;
  } else if (t < mv_t2) {
    *v = mv_Sn;
    *x = 0.5f * mv_Sn * mv_a + mv_Sn * (t - mv_t1);
    *a = 0.0f;
  } else if (t < mv_T) {
    float tau = mv_T - t;
    *v = mv_Sn * tau / mv_d;
    *x = mv_D - 0.5f * mv_Sn * tau * tau / mv_d;
    *a = -mv_Sn / mv_d;
  } else {
    *v = 0.0f;
    *x = mv_D;
    *a = 0.0f;
  }
}

static void Wheel_Control(Wheel_t *w, float v_cmd, float a_cmd)
{
  float av = fabsf(v_cmd);

  float ff = 0.0f;
  if (av > V_ZERO_MM_S) {
    float ks = w->rolling ? w->ks : w->ks_break;
    ff = ks + w->kv * av;
    if (v_cmd < 0.0f) ff = -ff;
    ff += w->kv * TAU_PLANT_S * a_cmd;
  } else {
    w->integ = 0.0f;
  }

  float err = v_cmd - w->speed;
  float u   = ff + kp_speed * err + w->integ;

  if (fabsf(w->speed) >= V_MOVING_MM_S) w->moved = 1;

  if (fabsf(u) < 100.0f) {
    w->integ += ki_speed * err * CTRL_PERIOD_S;
    w->integ  = clampf(w->integ, -INTEG_MAX_PCT, INTEG_MAX_PCT);
  }

  Wheel_SetDuty(w, u);
}

static uint8_t Wheel_Faulty(const Wheel_t *w)
{
  if (fabsf(w->duty) >= 50.0f && fabsf(w->speed) < 5.0f) return 1;   /* stall   */
  if (w->duty >  10.0f && w->speed < -20.0f) return 2;               /* runaway */
  if (w->duty < -10.0f && w->speed >  20.0f) return 2;
  return 0;
}

static void Control_Tick(void)
{
  Wheel_UpdateEncoder(&wl);
  Wheel_UpdateEncoder(&wr);

  position_l_mm = (float)(wl.total - wl.start_total) * wl.mmpc;
  position_r_mm = (float)(wr.total - wr.start_total) * wr.mmpc;
  speed_l_mms   = wl.speed;
  speed_r_mms   = wr.speed;
  duty_l_pct    = wl.duty;
  duty_r_pct    = wr.duty;

  static float v_prev = 0.0f;
  float v_mean = 0.5f * (wl.speed + wr.speed);
  float a_raw  = (v_mean - v_prev) / CTRL_PERIOD_S;
  v_prev        = v_mean;
  accel_mms2    = 0.8f * accel_mms2 + 0.2f * a_raw;
  speed_mms     = v_mean;
  position_mm   = 0.5f * (position_l_mm + position_r_mm);
  target_accel_mms2 = 0.0f;

  if (Moving() && log_n < LOG_N) {
    LogS_t *s = &log_buf[log_n];
    s->x_x10  = lroundf(5.0f * (position_l_mm + position_r_mm));
    s->xs_x10 = lroundf(10.0f * target_pos_mm);
    s->v      = (int16_t)lroundf(v_mean);
    s->vs     = (int16_t)lroundf(target_speed_mms);
    log_n++;
  }

  /* Point 15: Stop when obstacle <= 30 cm, continue once removed */
  if (phase == PH_MOVE || phase == PH_OBSTACLE) {
    float v_now   = 0.5f * (fabsf(wl.speed) + fabsf(wr.speed));          /* mm/s */
    float stop_cm = OBSTACLE_STOP_CM + v_now * OBS_LEAD_S / 10.0f;       /* cm   */
    if (obstacle_dist_cm <= stop_cm && mv_dir > 0) {
      if (phase != PH_OBSTACLE) {
        phase = PH_OBSTACLE;
        evt   = EVT_OBS_STOP;
      }
      Motors_Stop();
      target_speed_mms = 0.0f;
      return; /* Freeze mv_ticks so trapezoid profile pauses cleanly */
    } else if (phase == PH_OBSTACLE && obstacle_dist_cm >= OBSTACLE_RESUME_CM) {
      wl.fault_cnt = 0;
      wr.fault_cnt = 0;
      phase = PH_MOVE;
      evt   = EVT_OBS_RESUME;
    }
  }

  if (phase != PH_MOVE && phase != PH_SETTLE) {
    target_speed_mms = 0.0f;
    return;
  }

  mv_ticks++;
  float t = (float)mv_ticks * CTRL_PERIOD_S;
  float v_prof, x_prof, a_prof;
  Profile_Eval(t, &v_prof, &x_prof, &a_prof);
  target_accel_mms2 = (float)mv_dir * a_prof;

  target_speed_mms = (float)mv_dir * v_prof;
  target_pos_mm    = (float)mv_dir * x_prof;

  if (phase == PH_MOVE && t >= mv_T) {
    phase = PH_SETTLE;
    settle_ticks = 0;
  }

  Wheel_t *ws[2] = { &wl, &wr };
  float perr[2];
  for (int i = 0; i < 2; i++) {
    float x_meas = (float)(ws[i]->total - ws[i]->start_total) * ws[i]->mmpc;
    perr[i] = (float)mv_dir * x_prof - x_meas;
  }

  float e_sync = 0.5f * (perr[0] - perr[1]);
  sync_err_mm  = perr[0] - perr[1];
  if (fabsf(sync_err_mm) > mv_sync_max) mv_sync_max = fabsf(sync_err_mm);

  uint8_t reached[2];
  for (int i = 0; i < 2; i++) {
    reached[i] = ((float)mv_dir * perr[i] < POS_TOL_MM);
  }

  if (phase == PH_MOVE) {
    float v_ff = (float)mv_dir * v_prof;
    float a_ff = (float)mv_dir * a_prof;
    float v_l  = v_ff + clampf(KP_POS * perr[0], -V_CORR_MAX_MM_S, V_CORR_MAX_MM_S) + k_sync * e_sync;
    float v_r  = v_ff + clampf(KP_POS * perr[1], -V_CORR_MAX_MM_S, V_CORR_MAX_MM_S) - k_sync * e_sync;
    Wheel_Control(&wl, v_l, a_ff);
    Wheel_Control(&wr, v_r, a_ff);
  } else {
    for (int i = 0; i < 2; i++) {
      Wheel_t *w = ws[i];
      if (reached[i]) {
        Wheel_SetDuty(w, 0.0f);
        w->integ = 0.0f;
      } else {
        float v_cmd = clampf(KP_POS * perr[i], -SETTLE_V_MM_S, SETTLE_V_MM_S);
        if ((float)mv_dir * v_cmd < 0.0f) v_cmd = 0.0f;
        Wheel_Control(w, v_cmd, 0.0f);
      }
    }
  }

  for (int i = 0; i < 2; i++) {
    uint8_t f = Wheel_Faulty(ws[i]);
    if (f) {
      if (++ws[i]->fault_cnt >= FAULT_TICKS) {
        fault_why = (uint8_t)((i << 4) | f);
        Motors_Stop();
        phase = PH_FAULT;
        evt   = EVT_FAULT;
        return;
      }
    } else {
      ws[i]->fault_cnt = 0;
    }
  }

  if (phase == PH_SETTLE) {
    settle_ticks++;
    uint8_t done = reached[0] && reached[1]
                && (fabsf(wl.speed) < 10.0f) && (fabsf(wr.speed) < 10.0f);
    if (done || settle_ticks >= SETTLE_TICKS) {
      Motors_Stop();
      phase = PH_IDLE;
      evt   = done ? EVT_DONE : EVT_TIMEOUT;
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    Control_Tick();
  }
}

/* ==================== HC-SR04 Ultrasonic (Trig=PC11, Echo=PA15/TIM2_CH1) == */
static void Ultrasonic_Trigger(void)
{
  /* re-arm for a rising edge: a missed falling edge must not swap the edges */
  ic_edge = 0;
  __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
  HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_SET);
  uint32_t t0 = __HAL_TIM_GET_COUNTER(&htim2);
  while ((__HAL_TIM_GET_COUNTER(&htim2) - t0) < 12U) { __NOP(); } /* 12 us pulse */
  HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_RESET);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    if (ic_edge == 0) {
      ic_t1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      ic_edge = 1;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    } else {
      uint32_t ic_t2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      uint32_t us = ic_t2 - ic_t1;                    /* wrap-safe in uint32_t */
      float cm = (us >= 23000U) ? 400.0f : (float)us / 58.0f;   /* timeout pulse = free */
      if (cm >= 2.0f) {
        static float h[3] = { 400.0f, 400.0f, 400.0f };
        static uint8_t k = 0;
        h[k] = cm;  k = (uint8_t)((k + 1U) % 3U);
        float lo = fminf(h[0], h[1]), hi = fmaxf(h[0], h[1]);
        obstacle_dist_cm = fmaxf(lo, fminf(hi, h[2]));      /* Median aus 3 */
        t_echo_ok = HAL_GetTick();
      }
      ic_edge = 0;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    }
  }
}

/* ==================== OLED Display Update (PB8/PB9 I2C1) ================== */
static void OLED_UpdateStatus(void)
{
  static const char *const st_names[] = { "IDLE", "MOVE", "SETTLE", "FAULT", "OBSTACLE!" };
  char l0[22], l1[22], l2[22], l3[22];
  long d_avg = (long)(0.5f * (position_l_mm + position_r_mm));

  snprintf(l0, sizeof(l0), "ST: %-12s", st_names[(int)phase]);
  snprintf(l1, sizeof(l1), "D: %4ld/%4ld MM", d_avg, mv_target_mm);
  snprintf(l2, sizeof(l2), "V: %3ld / %3ld MMS", (long)speed_l_mms, (long)speed_r_mms);
  snprintf(l3, sizeof(l3), "OBS:%3ldCM KP.045", (long)obstacle_dist_cm);

  if (oled_ok) {
    memset(oled_buf, 0, sizeof(oled_buf));
    OLED_DrawStr(0, 2, l0);
    OLED_DrawStr(2, 2, l1);
    OLED_DrawStr(4, 2, l2);
    OLED_DrawStr(6, 2, l3);
    OLED_FlushPage(0);
    OLED_FlushPage(2);
    OLED_FlushPage(4);
    OLED_FlushPage(6);
  }
}

/* ============================ Start / Abort Move ========================== */
static int Move_Start(long dist_mm)
{
  log_n = 0;
  float D = (float)labs(dist_mm);
  float a  = (float)acc_ms / 1000.0f;
  float d  = (float)dec_ms / 1000.0f;
  float Sn = v_nom;
  float T;

  if (D >= Sn * (a + d) * 0.5f) {
    T = D / Sn + 0.5f * (a + d);
  } else {
    Sn = 2.0f * D / (a + d);
    T  = a + d;
  }

  __disable_irq();
  open_loop = 0;
  mv_D = D;  mv_Sn = Sn;  mv_a = a;  mv_d = d;  mv_T = T;
  mv_t1 = a; mv_t2 = T - d;
  mv_dir = (dist_mm > 0) ? 1 : -1;
  mv_target_mm = dist_mm;
  mv_ticks = 0;
  settle_ticks = 0;
  mv_sync_max = 0.0f;
  wl.start_total = wl.total;   wr.start_total = wr.total;
  position_l_mm = 0.0f;        position_r_mm = 0.0f;
  target_pos_mm = 0.0f;        target_speed_mms = 0.0f;
  wl.integ = 0.0f;             wr.integ = 0.0f;
  wl.moved = 0;                wr.moved = 0;
  wl.fault_cnt = 0;            wr.fault_cnt = 0;
  evt = 0;
  phase = PH_MOVE;
  __enable_irq();

  char msg[80];
  snprintf(msg, sizeof msg, "Move %ld mm: Vmax %ld mm/s, T %ld ms\r\n",
           dist_mm, (long)Sn, (long)(T * 1000.0f));
  Serial_Print(msg);
  return 0;
}

static void Move_Abort(void)
{
  __disable_irq();
  open_loop = 0;
  phase = PH_IDLE;
  target_speed_mms = 0.0f;
  Motors_Stop();
  __enable_irq();
}

/* ============================== Serial & Wireless ========================= */
static void Rx_Char(char c)
{
  t_last_rx = HAL_GetTick();
  if (line_ready) return;
  if (c == '\r' || c == '\n') {
    if (rx_len > 0) { rx_line[rx_len] = '\0'; rx_len = 0; line_ready = 1; }
  } else if (rx_len < RX_LINE_MAX - 1) {
    rx_line[rx_len++] = c;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    Rx_Char((char)rx_byte);
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  } else if (huart->Instance == USART1) {
    Rx_Char((char)rx_byte1);
    HAL_UART_Receive_IT(&huart1, &rx_byte1, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  } else if (huart->Instance == USART1) {
    HAL_UART_Receive_IT(&huart1, &rx_byte1, 1);
  }
}

static void Serial_Print(const char *txt)
{
  uint16_t n = (uint16_t)strlen(txt);
  HAL_UART_Transmit(&huart2, (const uint8_t *)txt, n, 10U + n / 10U);
  HAL_UART_Transmit(&huart1, (const uint8_t *)txt, n, 10U + 2U * n);
}

static void fmt_fix(char *out, size_t n, float v, int dec)
{
  static const char *const fmts[] = { "%s%ld.%01ld", "%s%ld.%02ld",
                                      "%s%ld.%03ld", "%s%ld.%04ld" };
  static const long scale[] = { 10L, 100L, 1000L, 10000L };
  if (dec < 1) dec = 1;
  if (dec > 4) dec = 4;
  long s = scale[dec - 1];
  long x = (long)(v * (float)s + ((v >= 0.0f) ? 0.5f : -0.5f));
  snprintf(out, n, fmts[dec - 1], (x < 0) ? "-" : "", labs(x) / s, labs(x) % s);
}

/* tenths as "123,4" (decimal comma); sign handled separately so -0,x keeps its minus */
static void fmt_dec1(char *out, size_t n, int32_t v_x10)
{
  long a = labs((long)v_x10);
  snprintf(out, n, "%s%ld,%ld", (v_x10 < 0) ? "-" : "", a / 10L, a % 10L);
}

static void fmt_x10(char *out, size_t n, float v)
{
  fmt_fix(out, n, v, 1);
}

static void Serial_PrintStatus(void)
{
  static const char *const names[] = { "IDLE", "MOVE", "SETTLE", "FAULT", "OBSTACLE" };
  char pl[16], pr[16], ob[16], msg[140];
  fmt_x10(pl, sizeof pl, position_l_mm);
  fmt_x10(pr, sizeof pr, position_r_mm);
  fmt_x10(ob, sizeof ob, obstacle_dist_cm);
  snprintf(msg, sizeof msg, "L=%s mm (%ld mm/s) R=%s mm (%ld mm/s) Obs=%s cm [%s]\r\n",
           pl, (long)speed_l_mms, pr, (long)speed_r_mms, ob, names[(int)phase]);
  Serial_Print(msg);
}

static void Serial_PrintParams(void)
{
  char a[16], b[16], c[16], msg[96];
  fmt_fix(a, sizeof a, wl.ks, 1);
  fmt_fix(b, sizeof b, wl.kv, 4);
  fmt_fix(c, sizeof c, wl.ks_break, 1);
  snprintf(msg, sizeof msg, "FF L: Ks %s %%  Kv %s %%/(mm/s)  Ks_break %s %%\r\n", a, b, c);
  Serial_Print(msg);
  fmt_fix(a, sizeof a, wr.ks, 1);
  fmt_fix(b, sizeof b, wr.kv, 4);
  fmt_fix(c, sizeof c, wr.ks_break, 1);
  snprintf(msg, sizeof msg, "FF R: Ks %s %%  Kv %s %%/(mm/s)  Ks_break %s %%\r\n", a, b, c);
  Serial_Print(msg);
  fmt_fix(a, sizeof a, kp_speed, 3);
  fmt_fix(b, sizeof b, ki_speed, 3);
  fmt_fix(c, sizeof c, k_sync, 1);
  snprintf(msg, sizeof msg, "PI: Kp %s  Ki %s   K_sync %s   tau_ff %ld ms\r\n",
           a, b, c, (long)(TAU_PLANT_S * 1000.0f + 0.5f));
  Serial_Print(msg);
}

static uint8_t parse_two_floats(const char *s, float *f1, float *f2)
{
  char *end;
  *f1 = strtof(s, &end);
  if (end == s) return 0;
  while (*end == ' ' || *end == ',') end++;
  const char *s2 = end;
  *f2 = strtof(s2, &end);
  if (end == s2) return 0;
  while (*end == ' ') end++;
  return (*end == '\0');
}

static void Serial_HandleCommand(const char *cmd)
{
  char  c = cmd[0];
  char *end;

  if (c == 's' || c == 'S') {
    Move_Abort();
    Serial_Print("STOP\r\n");

  } else if (c == 'r' || c == 'R') {
    if (Moving()) { Serial_Print("Busy, send s first\r\n"); return; }
    __disable_irq();
    wl.total = 0; wr.total = 0;
    wl.start_total = 0; wr.start_total = 0;
    position_l_mm = 0.0f; position_r_mm = 0.0f;
    target_pos_mm = 0.0f; target_speed_mms = 0.0f;
    __enable_irq();
    Serial_Print("Counters reset\r\n");

  } else if (c == 'p' || c == 'P') {
    Serial_PrintStatus();

  } else if (c == 'a' || c == 'A') {
    auto_print = !auto_print;
    Serial_Print(auto_print ? "Auto print ON\r\n" : "Auto print OFF\r\n");

  } else if (c == 'k' || c == 'K') {
    Serial_PrintParams();

  } else if (c == 'o' || c == 'O') {
    long cm = strtol(cmd + 1, &end, 10);
    if (end != cmd + 1 && cm >= 0 && cm <= 400) {
      obstacle_dist_cm = (float)cm;
      char msg[48];
      snprintf(msg, sizeof msg, "Obstacle distance set to %ld cm\r\n", cm);
      Serial_Print(msg);
    }

  } else if (c == 'e' || c == 'E') {
    char msg[96];
    snprintf(msg, sizeof msg, "ENC raw TIM3=%lu TIM8=%lu  total L=%ld R=%ld\r\n",
             (unsigned long)TIM3->CNT, (unsigned long)TIM8->CNT,
             (long)wl.total, (long)wr.total);
    Serial_Print(msg);

  } else if (c == 'l' || c == 'L') {
    if (Moving()) { Serial_Print("Busy, send s first\r\n"); return; }
    log_dump = 1;

  } else if (c == '?') {
    Serial_Print("D<mm> drive, V<mm/s> speed, T<acc> [<dec>] ramps, G<Kp> <Ki> PI,\r\n"
                 "F[L|R]<Ks> <Kv> FF, O<cm> obstacle, e encoders, s stop, r reset,\r\n"
                 "p print, a auto, l log dump, -100..100 open loop duty (L298N)\r\n");

  } else if (c == 'd' || c == 'D') {
    if (Moving()) { Serial_Print("Busy, send s to stop\r\n"); return; }
    long mm = strtol(cmd + 1, &end, 10);
    if (end == cmd + 1 || *end != '\0' || labs(mm) < MIN_DIST_MM || labs(mm) > MAX_DIST_MM) {
      Serial_Print("Invalid distance, use D<5..20000> or D-<5..20000> [mm]\r\n");
      return;
    }
    Move_Start(mm);

  } else if (c == 'v' || c == 'V') {
    long v = strtol(cmd + 1, &end, 10);
    if (end == cmd + 1 || *end != '\0' || v < V_NOM_MIN || v > V_NOM_MAX) {
      char msg[48];
      snprintf(msg, sizeof msg, "Invalid speed, use V%ld..%ld [mm/s]\r\n",
               (long)V_NOM_MIN, (long)V_NOM_MAX);
      Serial_Print(msg);
      return;
    }
    v_nom = (float)v;
    char msg[40];
    snprintf(msg, sizeof msg, "Vnom %ld mm/s\r\n", v);
    Serial_Print(msg);

  } else if (c == 't' || c == 'T') {
    long a = strtol(cmd + 1, &end, 10);
    long d = a;
    uint8_t ok = (end != cmd + 1);
    while (*end == ' ') end++;
    if (ok && *end != '\0') {
      char *e2;
      d  = strtol(end, &e2, 10);
      ok = (e2 != end) && (*e2 == '\0');
    }
    if (!ok || a < 100 || a > 5000 || d < 100 || d > 5000) {
      Serial_Print("Invalid ramp, use T<100..5000> [<100..5000>] [ms]\r\n");
      return;
    }
    acc_ms = (uint32_t)a;
    dec_ms = (uint32_t)d;
    char msg[48];
    snprintf(msg, sizeof msg, "Ramps: accel %ld ms, decel %ld ms\r\n", a, d);
    Serial_Print(msg);

  } else if (c == 'f' || c == 'F') {
    if (Moving()) { Serial_Print("Busy, send s first\r\n"); return; }
    const char *p = cmd + 1;
    uint8_t setL = 1, setR = 1;
    if (*p == 'L' || *p == 'l') { setR = 0; p++; }
    else if (*p == 'R' || *p == 'r') { setL = 0; p++; }
    float ks, kv;
    if (!parse_two_floats(p, &ks, &kv) || ks < 0.0f || ks > 40.0f || kv <= 0.0f || kv > 1.0f) {
      Serial_Print("Invalid, use F[L|R]<Ks 0..40> <Kv 0..1>\r\n");
      return;
    }
    if (setL) { wl.ks = ks; wl.kv = kv; }
    if (setR) { wr.ks = ks; wr.kv = kv; }
    Serial_PrintParams();

  } else if (c == 'g' || c == 'G') {
    if (Moving()) { Serial_Print("Busy, send s first\r\n"); return; }
    float kp, ki;
    if (!parse_two_floats(cmd + 1, &kp, &ki) || kp < 0.0f || kp > 1.0f || ki < 0.0f || ki > 10.0f) {
      Serial_Print("Invalid, use G<Kp 0..1> <Ki 0..10>, e.g. G0.045 0.898\r\n");
      return;
    }
    kp_speed = kp;
    ki_speed = ki;
    Serial_PrintParams();

  } else if (c == 'y' || c == 'Y') {
    if (Moving()) { Serial_Print("Busy, send s first\r\n"); return; }
    float ky = strtof(cmd + 1, &end);
    if (end == cmd + 1 || *end != '\0' || ky < 0.0f || ky > 20.0f) {
      Serial_Print("Invalid, use Y<0..20>\r\n");
      return;
    }
    k_sync = ky;
    Serial_PrintParams();

  } else if (c == '-' || c == '+' || (c >= '0' && c <= '9')) {
    if (Moving()) { Serial_Print("Busy, send s first\r\n"); return; }
    long v = strtol(cmd, &end, 10);
    if (*end != '\0' || v < -100 || v > 100) {
      Serial_Print("Invalid value, use -100..100\r\n");
      return;
    }
    phase = PH_IDLE;
    Wheel_SetDuty(&wl, (float)v);
    Wheel_SetDuty(&wr, (float)v);
    open_loop = (v != 0);
    char msg[40];
    snprintf(msg, sizeof msg, "Open loop duty %ld %%\r\n", v);
    Serial_Print(msg);

  } else {
    Serial_Print("Unknown command, type ?\r\n");
  }
}

/* ============================ Init + Main Loop ============================ */
static void Car_Init(void)
{
  PWM_Setup();
  Motors_Stop();
  /* L298N: ENA = CH1 (PA8), ENB = CH2 (PA9). CH3/CH4 stay off (CCR 0). */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);
  HAL_TIM_PWM_Start(&htim1, MOT_L_PWM_CH);
  HAL_TIM_PWM_Start(&htim1, MOT_R_PWM_CH);

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  wl.last_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  wr.last_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);

  /* Start TIM2_CH1 Input Capture on PA15 for HC-SR04 Echo */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

  /* Initialize SSD1306 OLED on I2C1 (PB8/PB9) */
  OLED_Init();
  OLED_UpdateStatus();

  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  HAL_UART_Receive_IT(&huart1, &rx_byte1, 1);
  Serial_Print("\r\nCar ready. Type ? for help.\r\n");

  uint32_t hz = Ctrl_RateHz();
  char msg[64];
  snprintf(msg, sizeof msg, "Control loop %lu Hz (expected %u)\r\n",
           (unsigned long)hz, (unsigned)CTRL_HZ_EXPECTED);
  Serial_Print(msg);
  Serial_PrintParams();

  HAL_TIM_Base_Start_IT(&htim6);
}

static void Car_Loop(void)
{
  static uint32_t t_print = 0;
  static uint32_t t_oled  = 0;
  static uint32_t t_sonar = 0;

  if (line_ready) {
    char cmd[RX_LINE_MAX];
    char echo[RX_LINE_MAX + 6];
    memcpy(cmd, (const char *)rx_line, RX_LINE_MAX);
    line_ready = 0;
    snprintf(echo, sizeof echo, "> %s\r\n", cmd);
    Serial_Print(echo);
    Serial_HandleCommand(cmd);
  }

  if (evt) {
    uint8_t e = evt;
    evt = 0;
    if (e == EVT_DONE || e == EVT_TIMEOUT) {
      char a[16], b[16], s[16], msg[160];
      fmt_x10(a, sizeof a, position_l_mm);
      fmt_x10(b, sizeof b, position_r_mm);
      fmt_x10(s, sizeof s, mv_sync_max);
      snprintf(msg, sizeof msg, "%s: target %ld mm, L=%s mm, R=%s mm, sync max %s mm\r\n",
               (e == EVT_DONE) ? "DONE" : "TIMEOUT", mv_target_mm, a, b, s);
      Serial_Print(msg);
    } else if (e == EVT_OBS_STOP) {
      Serial_Print("OBSTACLE <= 30 cm! Vehicle paused, waiting for obstacle removal...\r\n");
    } else if (e == EVT_OBS_RESUME) {
      Serial_Print("OBSTACLE removed! Resuming trajectory...\r\n");
    } else if (e == EVT_FAULT) {
      char msg[96];
      snprintf(msg, sizeof msg, "FAULT: wheel %c %s, counts L=%ld R=%ld\r\n",
               (fault_why >> 4) ? 'R' : 'L',
               ((fault_why & 0x0FU) == 1U) ? "stall" : "runaway (sign)",
               (long)(wl.total - wl.start_total), (long)(wr.total - wr.start_total));
      Serial_Print(msg);
    }
  }

  /* Trigger HC-SR04 on PC11 every 60 ms */
  if (HAL_GetTick() - t_sonar >= 60U) {
    t_sonar = HAL_GetTick();
    Ultrasonic_Trigger();
  }

#if US_FAILSAFE
  /* no valid echo for 300 ms: treat as obstacle (sensor dead = not free) */
  if (HAL_GetTick() - t_echo_ok > 300U) obstacle_dist_cm = 0.0f;
#endif

  /* Update OLED Display every 200 ms */
  if (HAL_GetTick() - t_oled >= 200U) {
    t_oled = HAL_GetTick();
    OLED_UpdateStatus();
  }

  if (auto_print && (HAL_GetTick() - t_print >= AUTO_PRINT_MS)) {
    t_print = HAL_GetTick();
    Serial_PrintStatus();
  }

  if (open_loop && (HAL_GetTick() - t_last_rx > DEADMAN_MS)) {
    Move_Abort();
    Serial_Print("DEADMAN: no input for 2 s, motors stopped\r\n");
  }

  if (log_dump) {
    log_dump = 0;
    uint16_t n = log_n;
    char line[64];
    snprintf(line, sizeof line, "LOG %u samples, dt 20 ms\r\n", (unsigned)n);
    Serial_Print(line);
    Serial_Print("t_ms;x_mm;x_soll_mm;v_mms;v_soll_mms\r\n");
    for (uint16_t i = 0; i < n; i++) {
      char x[16], xs[16];
      fmt_dec1(x,  sizeof x,  log_buf[i].x_x10);
      fmt_dec1(xs, sizeof xs, log_buf[i].xs_x10);
      snprintf(line, sizeof line, "%lu;%s;%s;%d;%d\r\n", (unsigned long)i * 20UL, x, xs,
               (int)log_buf[i].v, (int)log_buf[i].vs);
      Serial_Print(line);
    }
    Serial_Print("LOG END\r\n");
  }
}

void Error_Handler(void)
{
  __disable_irq();
  /* L298N: all IN low, EN (PWM) 0 -> both motors coast */
  GPIOB->BSRR = (uint32_t)(GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15) << 16U;
  TIM1->CCR1 = 0U; TIM1->CCR2 = 0U; TIM1->CCR3 = 0U; TIM1->CCR4 = 0U;
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
