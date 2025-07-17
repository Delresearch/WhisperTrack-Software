/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
// ─── Configuration ────────────────────────────────────────────────────────────
/* Definitions ---------------------------------------------------------------*/
/* Each half holds 128 x 16‑bit samples  →  128 × 2 B = 256 B            */



/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUCK_10V_EN_Pin GPIO_PIN_2
#define BUCK_10V_EN_GPIO_Port GPIOE
#define BUCK_10V_MODE_Pin GPIO_PIN_3
#define BUCK_10V_MODE_GPIO_Port GPIOE
#define BOOST_EN_Pin GPIO_PIN_5
#define BOOST_EN_GPIO_Port GPIOE
#define BOOST_PGOOD_Pin GPIO_PIN_6
#define BOOST_PGOOD_GPIO_Port GPIOE
#define GATE_DRIVE_EN_N_Pin GPIO_PIN_1
#define GATE_DRIVE_EN_N_GPIO_Port GPIOB
#define OUTPUT_OD_N_Pin GPIO_PIN_8
#define OUTPUT_OD_N_GPIO_Port GPIOE
#define SMPS_V1_Pin GPIO_PIN_10
#define SMPS_V1_GPIO_Port GPIOE
#define SMPS_EN_Pin GPIO_PIN_11
#define SMPS_EN_GPIO_Port GPIOE
#define SMPS_PG_Pin GPIO_PIN_12
#define SMPS_PG_GPIO_Port GPIOE
#define SMPS_SW_Pin GPIO_PIN_13
#define SMPS_SW_GPIO_Port GPIOE
#define SAI1_EXTCLK_EN_N_Pin GPIO_PIN_14
#define SAI1_EXTCLK_EN_N_GPIO_Port GPIOE
#define STATUS_LED_Pin GPIO_PIN_15
#define STATUS_LED_GPIO_Port GPIOE
#define AMP_GS_Pin GPIO_PIN_12
#define AMP_GS_GPIO_Port GPIOD
#define VIN_DIV_EN_Pin GPIO_PIN_13
#define VIN_DIV_EN_GPIO_Port GPIOD
#define BOOST_48V_Pin GPIO_PIN_15
#define BOOST_48V_GPIO_Port GPIOD
#define BUCK_1V95_MODE_Pin GPIO_PIN_0
#define BUCK_1V95_MODE_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
