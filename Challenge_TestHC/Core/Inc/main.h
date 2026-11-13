/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void MX_IWDG_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define XSHUT_G1_Pin GPIO_PIN_0
#define XSHUT_G1_GPIO_Port GPIOC
#define XSHUT_G2_Pin GPIO_PIN_1
#define XSHUT_G2_GPIO_Port GPIOC
#define XSHUT_G3_Pin GPIO_PIN_2
#define XSHUT_G3_GPIO_Port GPIOC
#define XSHUT_G4_Pin GPIO_PIN_3
#define XSHUT_G4_GPIO_Port GPIOC
#define hadc1_Pin GPIO_PIN_0
#define hadc1_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define IMU_CS_Pin GPIO_PIN_4
#define IMU_CS_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define XSHUT_S1_Pin GPIO_PIN_0
#define XSHUT_S1_GPIO_Port GPIOB
#define XSHUT_S2_Pin GPIO_PIN_1
#define XSHUT_S2_GPIO_Port GPIOB
#define EN_L_Pin GPIO_PIN_12
#define EN_L_GPIO_Port GPIOB
#define EN_R_Pin GPIO_PIN_13
#define EN_R_GPIO_Port GPIOB
#define XSHUT_F1_Pin GPIO_PIN_14
#define XSHUT_F1_GPIO_Port GPIOB
#define XSHUT_F2_Pin GPIO_PIN_15
#define XSHUT_F2_GPIO_Port GPIOB
#define PROBE_Pin GPIO_PIN_8
#define PROBE_GPIO_Port GPIOC
#define HC12_Set_Pin GPIO_PIN_9
#define HC12_Set_GPIO_Port GPIOC
#define VL_S2_INT_Pin GPIO_PIN_12
#define VL_S2_INT_GPIO_Port GPIOA
#define VL_S2_INT_EXTI_IRQn EXTI15_10_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define VL_S1_INT_Pin GPIO_PIN_15
#define VL_S1_INT_GPIO_Port GPIOA
#define VL_S1_INT_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
