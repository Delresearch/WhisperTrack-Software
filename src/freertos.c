/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern void SysTick_Handler     (void);
/* FreeRTOS tick timer interrupt handler prototype */
extern void xPortSysTickHandler (void);

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* USER CODE BEGIN PREPOSTSLEEP */
void PreSleepProcessing(uint32_t ulExpectedIdleTime) {
  (void)ulExpectedIdleTime;
  //// __HAL_RCC_GPIOC_CLK_DISABLE();
  ////__HAL_RCC_GPIOH_CLK_DISABLE();
  ////__HAL_RCC_GPIOB_CLK_DISABLE();
  ////__HAL_RCC_GPIOG_CLK_DISABLE();
  ////__HAL_RCC_GPIOA_CLK_DISABLE();
  ////  HAL_PWREx_DisableVddIO2();
  /*Enter to sleep Mode using the HAL function HAL_PWR_EnterSLEEPMode with WFI
   * instruction*/
  ////// HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
}

void PostSleepProcessing(uint32_t ulExpectedIdleTime) {
  /* Re-enable clocks */

  // __HAL_RCC_GPIOB_CLK_ENABLE();
  //  HAL_PWREx_EnableVddIO2();
  (void)ulExpectedIdleTime;
}

/* USER CODE END PREPOSTSLEEP */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* Provide memory for the Idle task. */

void SysTick_Handler (void) {
  /* Clear overflow flag */
  #ifdef STM32L496xx
  SysTick->CTRL;
  #endif
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
    /* Call tick handler */
    xPortSysTickHandler();
  }
}

/* USER CODE END Application */
