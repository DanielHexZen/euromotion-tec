/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   Car: 2x encoder (TIM3 = left, TIM8 = right), 2x BTS7960,
  *          trapezoid speed profile + PI speed control, command D<mm>.
  *          Board : NUCLEO-F446RE (STM32F446RE), SYSCLK 180 MHz (HSE bypass)
  *
  *  Hardware (according to MR2006B_Pinout.xlsx)
  *   PA2/PA3    USART2     ST-LINK virtual COM port (115200)
  *   PA6, PA7   TIM3_CH1/2 left encoder  A, B   (3.3 V supply for encoders!)
  *   PC6, PC7   TIM8_CH1/2 right encoder A, B
  *   PA8        TIM1_CH1   LEFT  BTS7960  RPWM  (forward)
  *   PA9        TIM1_CH2   LEFT  BTS7960  LPWM  (reverse)
  *   PA10       TIM1_CH3   RIGHT BTS7960  RPWM  (forward)
  *   PA11       TIM1_CH4   RIGHT BTS7960  LPWM  (reverse)
  *   PB12 EN_L  GPIO out   LEFT  BTS7960  R_EN + L_EN (tied together, 10k pull-down)
  *   PB13 EN_R  GPIO out   RIGHT BTS7960  R_EN + L_EN (tied together, 10k pull-down)
  *   TIM6       internal 50 Hz timer interrupt = control loop
  *
  *  Serial monitor commands (CR or LF at the end):
  *     D1000 / D-1000   drive 1000 mm forward / backward
  *     V300             nominal speed [mm/s]            (default 300)
  *     T1000            acceleration = deceleration ramp [ms]
  *     T1000 1500       acceleration ramp, deceleration ramp [ms]
  *     -100 ... 100     OPEN LOOP duty in % (for measuring, wheels in the air)
  *     s                stop / abort
  *     r                reset both counters (only when stopped)
  *     p                print once
  *     a                auto print on/off (every 250 ms)
  *     ?                help
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  PH_IDLE = 0,
  PH_MOVE,        /* following the speed profile                       */
  PH_SETTLE,      /* profile finished, trimming the last millimetres   */
  PH_FAULT        /* stalled or wrong encoder/motor sign               */
} Phase_t;

typedef struct {
  TIM_HandleTypeDef *enc;       /* encoder timer                        */
  uint32_t ch_fwd, ch_rev;      /* TIM1 PWM channels (RPWM, LPWM)       */
  int8_t   enc_sign;            /* +1/-1: forward must give +counts     */
  int8_t   mot_sign;            /* +1/-1: +duty must drive forward      */
  float    ks;                  /* feed-forward offset, % duty          */
  float    kv;
  float    ks_break;
  uint16_t last_raw;
  int32_t  total;               /* counts                               */
  int32_t  start_total;         /* counts at the start of the move      */
  float    speed;               /* filtered speed [mm/s]                */
  float    integ;               /* PI integrator, already in % duty     */
  float    duty;                /* output, % (wheel-forward positive)   */
  uint8_t  kick_ticks;
  uint8_t  moved;               /* wheel has broken away in this move   */
  uint8_t  fault_cnt;
} Wheel_t;
                /* feed-forward slope, % per mm/s       */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---- mechanics ---- */
#define WHEEL_DIAMETER_MM     72.6f
#define WHEEL_BASE_MM         223.25f   /* not used yet, for odometry later  */
#define COUNTS_PER_WHEEL_REV  3584.0f   /* measured average: 3584.5          */
#define PI_F                  3.14159265f
#define MM_PER_COUNT          ((PI_F * WHEEL_DIAMETER_MM) / COUNTS_PER_WHEEL_REV)

/* Encoder signs: forward motion must give POSITIVE counts */
#define ENC_L_SIGN            1
#define ENC_R_SIGN            -1
/* Motor signs: positive duty must drive the wheel FORWARD */
#define MOT_L_SIGN            -1
#define MOT_R_SIGN            1

/* ---- BTS7960 / PWM ---- */
#define MOT_L_FWD_CH          TIM_CHANNEL_1   /* PA8  */
#define MOT_L_REV_CH          TIM_CHANNEL_2   /* PA9  */
#define MOT_R_FWD_CH          TIM_CHANNEL_3   /* PA10 */
#define MOT_R_REV_CH          TIM_CHANNEL_4   /* PA11 */
#define PWM_FREQ_HZ           20000U          /* BTS7960 allows up to 25 kHz */

/* ---- control loop (TIM6) ---- */
#define CTRL_PERIOD_S         0.02f           /* 50 Hz, must match TIM6      */
#define SPEED_FILTER_ALPHA    0.5f            /* 1 = no filtering            */

/* ---- feed-forward and start-up (TUNE THESE FIRST) ---- */
/* ---- feed-forward, measured per motor (CONCEPT.md §5.5) ---- */
#define KV_L                  0.00492f  /* duty per output rpm             */
#define KS_L                  0.110f    /* duty, running-fit intercept     */
#define KV_R                  0.00507f
#define KS_R                  0.127f
#define RPM_PER_MMS           (60.0f / (PI_F * WHEEL_DIAMETER_MM))
#define KV_PCT_PER_MMS(kv)    ((kv) * 100.0f * RPM_PER_MMS)
#define KS_PCT(ks)            ((ks) * 100.0f)
#define KICK_DUTY_PCT         40.0f   /* short boost to break static friction*/
#define KICK_TICKS            5U      /* max 5 x 20 ms = 100 ms of boost     */
#define V_MOVING_MM_S         8.0f    /* above this the wheel is "moving"    */
#define V_ZERO_MM_S           2.0f    /* below this the command is "zero"    */

/* ---- PI speed controller (per wheel) ---- */
#define KP_SPEED              0.10f   /* % duty per mm/s of error            */
#define KI_SPEED              0.50f   /* % duty per (mm/s * s)               */
#define INTEG_MAX_PCT         30.0f   /* anti-windup clamp                   */

/* ---- position trim (keeps both wheels on the profile = straight line) ---- */
#define KP_POS                2.0f    /* mm/s per mm of position error       */
#define V_CORR_MAX_MM_S       100.0f
#define SETTLE_V_MM_S         20.0f
#define POS_TOL_MM            2.0f
#define SETTLE_TICKS          50U     /* 1 s                                 */

/* ---- protection ---- */
#define FAULT_TICKS           30U     /* 0.6 s                               */

/* ---- user limits / defaults ---- */
#define V_NOM_DEFAULT         300.0f
#define V_NOM_MIN             20L
#define V_NOM_MAX             800L
#define ACC_MS_DEFAULT        1000U
#define DEC_MS_DEFAULT        1000U
#define MIN_DIST_MM           5L
#define MAX_DIST_MM           20000L

#define EVT_DONE              1U
#define EVT_FAULT             2U

#define RX_LINE_MAX           16
#define AUTO_PRINT_MS         250U
#define DEADMAN_MS            2000U   /* open loop: stop without input */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static Wheel_t wl = { .enc = &htim3, .ch_fwd = MOT_L_FWD_CH, .ch_rev = MOT_L_REV_CH,
                      .enc_sign = ENC_L_SIGN, .mot_sign = MOT_L_SIGN,
					  .ks = KS_PCT(KS_L), .kv = KV_PCT_PER_MMS(KV_L), .ks_break = 14.1f };
static Wheel_t wr = { .enc = &htim8, .ch_fwd = MOT_R_FWD_CH, .ch_rev = MOT_R_REV_CH,
                      .enc_sign = ENC_R_SIGN, .mot_sign = MOT_R_SIGN,
					  .ks = KS_PCT(KS_R), .kv = KV_PCT_PER_MMS(KV_R), .ks_break = 16.9f };

/* for STM32CubeMonitor / debugger */
volatile float position_l_mm = 0.0f;
volatile float position_r_mm = 0.0f;
volatile float speed_l_mms   = 0.0f;
volatile float speed_r_mms   = 0.0f;

static volatile Phase_t phase = PH_IDLE;
static volatile uint8_t evt   = 0;
static uint32_t pwm_arr       = 8999U;          /* recomputed in PWM_Setup() */

/* user settings */
static float    v_nom  = V_NOM_DEFAULT;
static uint32_t acc_ms = ACC_MS_DEFAULT;
static uint32_t dec_ms = DEC_MS_DEFAULT;

/* current move (written with interrupts disabled) */
static float    mv_D = 0, mv_Sn = 0, mv_a = 1, mv_d = 1, mv_T = 0, mv_t1 = 0, mv_t2 = 0;
static int8_t   mv_dir = 1;
static long     mv_target_mm = 0;
static uint32_t mv_ticks = 0;
static uint32_t settle_ticks = 0;

/* serial */
static volatile char    rx_line[RX_LINE_MAX];
static volatile uint8_t rx_len     = 0;
static volatile uint8_t line_ready = 0;
static uint8_t rx_byte;
static uint8_t auto_print = 0;
static uint8_t           rx_byte1;        /* HC-12 on USART1 */
static volatile uint32_t t_last_rx = 0;   /* last byte from either UART */
static uint8_t           open_loop = 0;   /* open-loop duty is active */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Car_Init(void);
static void Car_Loop(void);
static void PWM_Setup(void);
static void Serial_Print(const char *txt);
static void Serial_HandleCommand(const char *cmd);
static void Serial_PrintStatus(void);
static void Wheel_SetDuty(Wheel_t *w, float duty);
static void Motors_Stop(void);
static int  Move_Start(long dist_mm);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Car_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Car_Loop();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 35999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
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
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 199;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
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
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_L_Pin|EN_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HC12_Set_GPIO_Port, HC12_Set_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_L_Pin EN_R_Pin */
  GPIO_InitStruct.Pin = EN_L_Pin|EN_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : HC12_Set_Pin */
  GPIO_InitStruct.Pin = HC12_Set_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HC12_Set_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ============================== helpers =================================== */
static float clampf(float x, float lo, float hi)
{
  return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static uint8_t Moving(void)
{
  return (phase == PH_MOVE || phase == PH_SETTLE);
}

/* ============================== BTS7960 =================================== */
/* TIM1 sits on APB2 (90 MHz, timer clock 180 MHz because APB2 prescaler /2).
   Prescaler 0, ARR = 180 MHz / 20 kHz - 1 = 8999 -> 20 kHz PWM, 9000 steps.  */
static void PWM_Setup(void)
{
  uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
  uint32_t tclk  = ((RCC->CFGR & RCC_CFGR_PPRE2) == 0U) ? pclk2 : 2U * pclk2;
  pwm_arr = (tclk / PWM_FREQ_HZ) - 1U;
  __HAL_TIM_SET_PRESCALER(&htim1, 0U);
  __HAL_TIM_SET_AUTORELOAD(&htim1, pwm_arr);
  htim1.Instance->EGR = TIM_EGR_UG;
}

/* BTS7960: one half-bridge per PWM input. RPWM = PWM, LPWM = 0 -> one
   direction, RPWM = 0, LPWM = PWM -> the other. Both 0 with EN = 1 -> both
   outputs low = motor shorted = BRAKE. EN = 0 -> coast.                     */
static void Wheel_SetDuty(Wheel_t *w, float duty)
{
  duty = clampf(duty, -100.0f, 100.0f);
  w->duty = duty;
  float s = duty * (float)w->mot_sign;
  uint32_t ccr = (uint32_t)(fabsf(s) * (float)(pwm_arr + 1U) / 100.0f + 0.5f);
  if (ccr > pwm_arr + 1U) ccr = pwm_arr + 1U;

  if (s >= 0.0f) {                       /* switch the unused side off first */
    __HAL_TIM_SET_COMPARE(&htim1, w->ch_rev, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, w->ch_fwd, ccr);
  } else {
    __HAL_TIM_SET_COMPARE(&htim1, w->ch_fwd, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, w->ch_rev, ccr);
  }
}

static void Motors_Stop(void)
{
  Wheel_SetDuty(&wl, 0.0f);
  Wheel_SetDuty(&wr, 0.0f);
  wl.integ = 0.0f;
  wr.integ = 0.0f;
}

/* ============================== encoders ================================== */
/* (int16_t)(now - last) is correct even when the 16-bit counter wraps.      */
static void Wheel_UpdateEncoder(Wheel_t *w)
{
  uint16_t now   = (uint16_t)__HAL_TIM_GET_COUNTER(w->enc);
  int16_t  raw   = (int16_t)(now - w->last_raw);
  w->last_raw    = now;
  int32_t  delta = (int32_t)w->enc_sign * (int32_t)raw;
  w->total      += delta;
  float v_raw    = (float)delta * MM_PER_COUNT / CTRL_PERIOD_S;
  w->speed      += SPEED_FILTER_ALPHA * (v_raw - w->speed);
}

/* ============================== profile =================================== */
/* Trapezoid from the slides: A1 (accel), A2 (cruise), A3 (decel).
   v = speed, x = distance travelled since the start (area under the curve). */
static void Profile_Eval(float t, float *v, float *x)
{
  if (t < mv_t1) {
    *v = mv_Sn * t / mv_a;
    *x = 0.5f * mv_Sn * t * t / mv_a;
  } else if (t < mv_t2) {
    *v = mv_Sn;
    *x = 0.5f * mv_Sn * mv_a + mv_Sn * (t - mv_t1);
  } else if (t < mv_T) {
    float tau = mv_T - t;
    *v = mv_Sn * tau / mv_d;
    *x = mv_D - 0.5f * mv_Sn * tau * tau / mv_d;
  } else {
    *v = 0.0f;
    *x = mv_D;
  }
}

/* ============================ PI controller =============================== */
/* duty = feed-forward (min duty + k*speed) + Kp*err + integrator
   + short "kick" while the wheel is still standing.                         */
static void Wheel_Control(Wheel_t *w, float v_cmd)
{
  float av = fabsf(v_cmd);

  float ff = 0.0f;
  if (av > V_ZERO_MM_S) {
	ff = w->ks + w->kv * av;
    if (v_cmd < 0.0f) ff = -ff;
  } else {
    w->integ = 0.0f;           /* command is zero: forget the old correction */
  }

  float err = v_cmd - w->speed;
  float u   = ff + KP_SPEED * err + w->integ;

  if (fabsf(w->speed) >= V_MOVING_MM_S) w->moved = 1;

  /* kick only for the break-away at the start, never again in this move */
  uint8_t kicking = 0;
  if (!w->moved && av > V_ZERO_MM_S && w->kick_ticks < KICK_TICKS) {
    w->kick_ticks++;
    kicking = 1;
    float ku = (v_cmd > 0.0f) ? KICK_DUTY_PCT : -KICK_DUTY_PCT;
    if (fabsf(u) < KICK_DUTY_PCT) u = ku;
  }

  /* anti-windup: integrate only when not saturated and not kicking */
  if (!kicking && fabsf(u) < 100.0f) {
    w->integ += KI_SPEED * err * CTRL_PERIOD_S;
    w->integ  = clampf(w->integ, -INTEG_MAX_PCT, INTEG_MAX_PCT);
  }

  Wheel_SetDuty(w, u);
}

static uint8_t Wheel_Faulty(const Wheel_t *w)
{
  /* stalled: lots of power, no movement */
  if (fabsf(w->duty) >= 50.0f && fabsf(w->speed) < 5.0f) return 1;
  /* runaway: wheel moves against the command (wrong ENC/MOT sign) */
  if (w->duty >  10.0f && w->speed < -20.0f) return 1;
  if (w->duty < -10.0f && w->speed >  20.0f) return 1;
  return 0;
}

/* ============================ 50 Hz control tick ========================== */
static void Control_Tick(void)
{
  Wheel_UpdateEncoder(&wl);
  Wheel_UpdateEncoder(&wr);
  position_l_mm = (float)wl.total * MM_PER_COUNT;
  position_r_mm = (float)wr.total * MM_PER_COUNT;
  speed_l_mms   = wl.speed;
  speed_r_mms   = wr.speed;

  if (!Moving()) return;

  mv_ticks++;
  float t = (float)mv_ticks * CTRL_PERIOD_S;
  float v_prof, x_prof;
  Profile_Eval(t, &v_prof, &x_prof);

  if (phase == PH_MOVE && t >= mv_T) {
    phase = PH_SETTLE;
    settle_ticks = 0;
  }

  Wheel_t *ws[2] = { &wl, &wr };
  float perr[2];
  for (int i = 0; i < 2; i++) {
    Wheel_t *w = ws[i];
    float x_meas = (float)(w->total - w->start_total) * MM_PER_COUNT;
    perr[i] = (float)mv_dir * x_prof - x_meas;       /* position error [mm] */

    float v_cmd;
    if (phase == PH_MOVE) {
      v_cmd = (float)mv_dir * v_prof
            + clampf(KP_POS * perr[i], -V_CORR_MAX_MM_S, V_CORR_MAX_MM_S);
    } else {
      v_cmd = clampf(KP_POS * perr[i], -SETTLE_V_MM_S, SETTLE_V_MM_S);
      if (fabsf(perr[i]) < POS_TOL_MM) v_cmd = 0.0f;
    }
    Wheel_Control(w, v_cmd);
  }

  /* protection */
  for (int i = 0; i < 2; i++) {
    if (Wheel_Faulty(ws[i])) {
      if (++ws[i]->fault_cnt >= FAULT_TICKS) {
        Motors_Stop();
        phase = PH_FAULT;
        evt   = EVT_FAULT;
        return;
      }
    } else {
      ws[i]->fault_cnt = 0;
    }
  }

  /* end of the move */
  if (phase == PH_SETTLE) {
    settle_ticks++;
    uint8_t in_tol = (fabsf(perr[0]) < POS_TOL_MM) && (fabsf(perr[1]) < POS_TOL_MM)
                  && (fabsf(wl.speed) < 10.0f) && (fabsf(wr.speed) < 10.0f);
    if (in_tol || settle_ticks >= SETTLE_TICKS) {
      Motors_Stop();                    /* brake */
      phase = PH_IDLE;
      evt   = EVT_DONE;
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    Control_Tick();
  }
}

/* ============================ start of a move ============================= */
/* Distance -> profile. Slides: D = Sn * [ (a+d)/2 + (T-d-a) ]
   -> T = D/Sn + (a+d)/2. If D is too short for the full trapezoid the
   speed is lowered and the profile becomes a triangle (T = a + d).          */
static int Move_Start(long dist_mm)
{
  float D  = (float)labs(dist_mm);
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
  wl.start_total = wl.total;   wr.start_total = wr.total;
  wl.integ = 0.0f;             wr.integ = 0.0f;
  wl.kick_ticks = 0;           wr.kick_ticks = 0;
  wl.moved = 0;                wr.moved = 0;
  wl.fault_cnt = 0;            wr.fault_cnt = 0;
  evt = 0;
  phase = PH_MOVE;
  __enable_irq();

  char msg[72];
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
  Motors_Stop();
  __enable_irq();
}

/* ============================== serial ==================================== */
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
  HAL_UART_Transmit(&huart2, (const uint8_t *)txt, n, 10U + n / 10U);  /* 115200 */
  HAL_UART_Transmit(&huart1, (const uint8_t *)txt, n, 10U + 2U * n);   /* 9600   */
}

/* Integer-only formatting (float printf is not enabled in newlib-nano) */
static void fmt_x10(char *out, size_t n, float v)
{
  long x = (long)(v * 10.0f + ((v >= 0.0f) ? 0.5f : -0.5f));
  snprintf(out, n, "%s%ld.%ld", (x < 0) ? "-" : "", labs(x) / 10, labs(x) % 10);
}

static void Serial_PrintStatus(void)
{
  static const char *const names[] = { "IDLE", "MOVE", "SETTLE", "FAULT" };
  char pl[16], pr[16], msg[128];
  fmt_x10(pl, sizeof pl, position_l_mm);
  fmt_x10(pr, sizeof pr, position_r_mm);
  snprintf(msg, sizeof msg, "L=%ld (%s mm, %ld mm/s)  R=%ld (%s mm, %ld mm/s)  %s\r\n",
           (long)wl.total, pl, (long)speed_l_mms,
           (long)wr.total, pr, (long)speed_r_mms,
           names[(int)phase]);
  Serial_Print(msg);
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
    __enable_irq();
    Serial_Print("Counters reset\r\n");

  } else if (c == 'p' || c == 'P') {
    Serial_PrintStatus();

  } else if (c == 'a' || c == 'A') {
    auto_print = !auto_print;
    Serial_Print(auto_print ? "Auto print ON\r\n" : "Auto print OFF\r\n");

  } else if (c == '?') {
    Serial_Print("D<mm> / D-<mm> drive, V<mm/s> speed, T<ms> [<ms>] ramps,\r\n"
                 "-100..100 open loop duty, s stop, r reset, p print, a auto\r\n");

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
      Serial_Print("Invalid speed, use V20..800 [mm/s]\r\n");
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

/* ============================ init + main loop ============================ */
static void Car_Init(void)
{
  PWM_Setup();
  Motors_Stop();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  /* enable both BTS7960 only after the PWM outputs are at 0 */
  HAL_GPIO_WritePin(EN_L_GPIO_Port, EN_L_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(EN_R_GPIO_Port, EN_R_Pin, GPIO_PIN_SET);

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  wl.last_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  wr.last_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);

  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  HAL_UART_Receive_IT(&huart1, &rx_byte1, 1);
  Serial_Print("\r\nCar ready. Type ? for help.\r\n");

  HAL_TIM_Base_Start_IT(&htim6);          /* start the 50 Hz control loop */
}

static void Car_Loop(void)
{
  static uint32_t t_print = 0;

  /* 1) command from the serial monitor */
  /* 1) command from the serial monitor or the radio */
  if (line_ready) {
    char cmd[RX_LINE_MAX];
    char echo[RX_LINE_MAX + 6];
    memcpy(cmd, (const char *)rx_line, RX_LINE_MAX);
    line_ready = 0;
    snprintf(echo, sizeof echo, "> %s\r\n", cmd);
    Serial_Print(echo);
    Serial_HandleCommand(cmd);
  }

  /* 2) events from the control loop */
  if (evt) {
    uint8_t e = evt;
    evt = 0;
    if (e == EVT_DONE) {
      char a[16], b[16], msg[96];
      fmt_x10(a, sizeof a, (float)(wl.total - wl.start_total) * MM_PER_COUNT);
      fmt_x10(b, sizeof b, (float)(wr.total - wr.start_total) * MM_PER_COUNT);
      snprintf(msg, sizeof msg, "DONE: target %ld mm, L=%s mm, R=%s mm\r\n",
               mv_target_mm, a, b);
      Serial_Print(msg);
    } else {
      Serial_Print("FAULT: wheel stalled or wrong ENC_x_SIGN / MOT_x_SIGN. Motors stopped.\r\n");
    }
  }

  /* 3) periodic print */
  if (auto_print && (HAL_GetTick() - t_print >= AUTO_PRINT_MS)) {
    t_print = HAL_GetTick();
    Serial_PrintStatus();
  }

  /* 4) dead-man: open-loop duty needs input at least every 2 s */
  if (open_loop && (HAL_GetTick() - t_last_rx > DEADMAN_MS)) {
    Move_Abort();
    Serial_Print("DEADMAN: no input for 2 s, motors stopped\r\n");
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
