/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file         stm32l4xx_hal_msp.c
 * @brief        This file provides code for the MSP Initialization
 *               and de-Initialization codes.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_dac_ch1;
extern DMA_HandleTypeDef hdma_usart3_rx;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void) {

    /* USER CODE BEGIN MspInit 0 */

    /* USER CODE END MspInit 0 */

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    /* System interrupt init*/
    /* PendSV_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

    /** Disable the Internal Voltage Reference buffer
     */
    HAL_SYSCFG_DisableVREFBUF();

    /** Configure the internal voltage reference buffer high impedance mode
     */
    HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_ENABLE);

    /* USER CODE BEGIN MspInit 1 */

    /* USER CODE END MspInit 1 */
}

/**
 * @brief COMP MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hcomp: COMP handle pointer
 * @retval None
 */
void HAL_COMP_MspInit(COMP_HandleTypeDef *hcomp) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hcomp->Instance == COMP1) {
        /* USER CODE BEGIN COMP1_MspInit 0 */

        /* USER CODE END COMP1_MspInit 0 */
        
        /* Enable COMP peripheral clock */
        __HAL_RCC_SYSCFG_CLK_ENABLE();
        
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**COMP1 GPIO Configuration
        PC5     ------> COMP1_INP
        PB10     ------> COMP1_OUT
        */
        GPIO_InitStruct.Pin = GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF12_COMP1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* USER CODE BEGIN COMP1_MspInit 1 */

        /* USER CODE END COMP1_MspInit 1 */
    } else if (hcomp->Instance == COMP2) {
        /* USER CODE BEGIN COMP2_MspInit 0 */

        /* USER CODE END COMP2_MspInit 0 */
        
        /* Enable COMP peripheral clock */
        __HAL_RCC_SYSCFG_CLK_ENABLE();
        
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**COMP2 GPIO Configuration
        PB4 (NJTRST)     ------> COMP2_INP
        PB5     ------> COMP2_OUT
        */
        GPIO_InitStruct.Pin = GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF12_COMP2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* USER CODE BEGIN COMP2_MspInit 1 */

        /* USER CODE END COMP2_MspInit 1 */
    }
}

/**
 * @brief COMP MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hcomp: COMP handle pointer
 * @retval None
 */
void HAL_COMP_MspDeInit(COMP_HandleTypeDef *hcomp) {
    if (hcomp->Instance == COMP1) {
        /* USER CODE BEGIN COMP1_MspDeInit 0 */

        /* USER CODE END COMP1_MspDeInit 0 */

        /**COMP1 GPIO Configuration
        PC5     ------> COMP1_INP
        PB10     ------> COMP1_OUT
        */
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_5);

        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

        /* USER CODE BEGIN COMP1_MspDeInit 1 */

        /* USER CODE END COMP1_MspDeInit 1 */
    } else if (hcomp->Instance == COMP2) {
        /* USER CODE BEGIN COMP2_MspDeInit 0 */

        /* USER CODE END COMP2_MspDeInit 0 */

        /**COMP2 GPIO Configuration
        PB4 (NJTRST)     ------> COMP2_INP
        PB5     ------> COMP2_OUT
        PB7     ------> COMP2_INM
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7);

        /* USER CODE BEGIN COMP2_MspDeInit 1 */

        /* USER CODE END COMP2_MspDeInit 1 */
    }
}

/**
 * @brief DAC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hdac: DAC handle pointer
 * @retval None
 */
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hdac->Instance==DAC1)
  {
  /* USER CODE BEGIN DAC1_MspInit 0 */

  /* USER CODE END DAC1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_DAC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**DAC1 GPIO Configuration
    PA4     ------> DAC1_OUT1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* DAC1 DMA Init */
    /* DAC_CH1 Init */
    hdma_dac_ch1.Instance = DMA2_Channel4;
    hdma_dac_ch1.Init.Request = DMA_REQUEST_3;
    hdma_dac_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_dac_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dac_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dac_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dac_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_dac_ch1.Init.Mode = DMA_CIRCULAR;
    hdma_dac_ch1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_dac_ch1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hdac,DMA_Handle1,hdma_dac_ch1);

    /* DAC1 interrupt Init */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN DAC1_MspInit 1 */

  /* USER CODE END DAC1_MspInit 1 */

  }

}
/**
 * @brief DAC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hdac: DAC handle pointer
 * @retval None
 */
void HAL_DAC_MspDeInit(DAC_HandleTypeDef *hdac) {
    if (hdac->Instance == DAC1) {
        /* USER CODE BEGIN DAC1_MspDeInit 0 */

        /* USER CODE END DAC1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_DAC1_CLK_DISABLE();

        /**DAC1 GPIO Configuration
        PA4     ------> DAC1_OUT1
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

        /* DAC1 interrupt DeInit */
        HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
        /* USER CODE BEGIN DAC1_MspDeInit 1 */

        /* USER CODE END DAC1_MspDeInit 1 */
    }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */

  }
  else if(htim_base->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM8_MspInit 0 */

  /* USER CODE END TIM8_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM8_CLK_ENABLE();
  /* USER CODE BEGIN TIM8_MspInit 1 */

  /* USER CODE END TIM8_MspInit 1 */

  }

}


/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /* TIM1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM8_MspDeInit 0 */

  /* USER CODE END TIM8_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM8_CLK_DISABLE();
  /* USER CODE BEGIN TIM8_MspDeInit 1 */

  /* USER CODE END TIM8_MspDeInit 1 */
  }

}
/**
 * @brief UART MSP Initialization
 * This function configures the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    if (huart->Instance == USART3) {
        /* USER CODE BEGIN USART3_MspInit 0 */

        /* USER CODE END USART3_MspInit 0 */

        /** Initializes the peripherals clock
         */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
        PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_HSI;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
            Error_Handler();
        }

        /* Peripheral clock enable */
        __HAL_RCC_USART3_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        /**USART3 GPIO Configuration
        PC10     ------> USART3_TX
        PC11     ------> USART3_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* USART3 DMA Init */
        /* USART3_TX Init */
        hdma_usart3_tx.Instance = DMA1_Channel2;
        hdma_usart3_tx.Init.Request = DMA_REQUEST_2;
        hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart3_tx.Init.Mode = DMA_NORMAL;
        hdma_usart3_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
        if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(huart, hdmatx, hdma_usart3_tx);

        /* USART3_RX Init */
        hdma_usart3_rx.Instance = DMA1_Channel3;
        hdma_usart3_rx.Init.Request = DMA_REQUEST_2;
        hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart3_rx.Init.Mode = DMA_NORMAL;
        hdma_usart3_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
        if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(huart, hdmarx, hdma_usart3_rx);

        /* USER CODE BEGIN USART3_MspInit 1 */

        /* USER CODE END USART3_MspInit 1 */
    }
}

/**
 * @brief UART MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        /* USER CODE BEGIN USART3_MspDeInit 0 */

        /* USER CODE END USART3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USART3_CLK_DISABLE();

        /**USART3 GPIO Configuration
        PC10     ------> USART3_TX
        PC11     ------> USART3_RX
        */
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10 | GPIO_PIN_11);

        /* USART3 DMA DeInit */
        HAL_DMA_DeInit(huart->hdmatx);
        HAL_DMA_DeInit(huart->hdmarx);
        /* USER CODE BEGIN USART3_MspDeInit 1 */

        /* USER CODE END USART3_MspDeInit 1 */
    }
}

extern DMA_HandleTypeDef hdma_sai2_a;

static uint32_t SAI1_client = 0;
static uint32_t SAI2_client = 0;

void HAL_SAI_MspInit(SAI_HandleTypeDef *hsai) {

    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    /* SAI1 */
    if (hsai->Instance == SAI1_Block_A) {
        /* Peripheral clock enable */

        /** Initializes the peripherals clock
         */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
        PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PIN;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
            Error_Handler();
        }

        if (SAI1_client == 0) {
            __HAL_RCC_SAI1_CLK_ENABLE();
        }
        SAI1_client++;

        /**SAI1_A_Block_A GPIO Configuration
        PC3     ------> SAI1_SD_A
        PA8     ------> SAI1_SCK_A
        PB9     ------> SAI1_FS_A
        */
        GPIO_InitStruct.Pin = GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_8;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* Add DMA for SAI1 TX */
        extern DMA_HandleTypeDef hdma_sai1_a;
        hdma_sai1_a.Instance = DMA2_Channel1;
        hdma_sai1_a.Init.Request = DMA_REQUEST_1;
        hdma_sai1_a.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_sai1_a.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_sai1_a.Init.MemInc = DMA_MINC_ENABLE;
        hdma_sai1_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma_sai1_a.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
        hdma_sai1_a.Init.Mode = DMA_CIRCULAR;
        hdma_sai1_a.Init.Priority = DMA_PRIORITY_HIGH;
        if (HAL_DMA_Init(&hdma_sai1_a) != HAL_OK) {
            Error_Handler();
        }
        __HAL_LINKDMA(hsai, hdmatx, hdma_sai1_a);
    }
    /* SAI2 */
    if (hsai->Instance == SAI2_Block_A) {
        /* Peripheral clock enable */

        /** Initializes the peripherals clock
         */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
        PeriphClkInit.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PIN;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
            Error_Handler();
        }

        if (SAI2_client == 0) {
            __HAL_RCC_SAI2_CLK_ENABLE();
        }
        SAI2_client++;

        /**SAI2_A_Block_A GPIO Configuration
        PB12     ------> SAI2_FS_A
        PB13     ------> SAI2_SCK_A
        PB15     ------> SAI2_SD_A
        */
        GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* Peripheral DMA init*/

        hdma_sai2_a.Instance = DMA1_Channel6;
        hdma_sai2_a.Init.Request = DMA_REQUEST_1;
        hdma_sai2_a.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_sai2_a.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_sai2_a.Init.MemInc = DMA_MINC_ENABLE;
        hdma_sai2_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma_sai2_a.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
        hdma_sai2_a.Init.Mode = DMA_CIRCULAR;
        hdma_sai2_a.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&hdma_sai2_a) != HAL_OK) {
            Error_Handler();
        }

        /* Several peripheral DMA handle pointers point to the same DMA handle.
         Be aware that there is only one channel to perform all the requested DMAs. */
        __HAL_LINKDMA(hsai, hdmarx, hdma_sai2_a);

        __HAL_LINKDMA(hsai, hdmatx, hdma_sai2_a);
    }
}

void HAL_SAI_MspDeInit(SAI_HandleTypeDef *hsai) {
    /* SAI1 */
    if (hsai->Instance == SAI1_Block_A) {
        SAI1_client--;
        if (SAI1_client == 0) {
            /* Peripheral clock disable */
            __HAL_RCC_SAI1_CLK_DISABLE();
        }

        /**SAI1_A_Block_A GPIO Configuration
        PC3     ------> SAI1_SD_A
        PA8     ------> SAI1_SCK_A
        PB9     ------> SAI1_FS_A
        */
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_3);

        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);
    }
    /* SAI2 */
    if (hsai->Instance == SAI2_Block_A) {
        SAI2_client--;
        if (SAI2_client == 0) {
            /* Peripheral clock disable */
            __HAL_RCC_SAI2_CLK_DISABLE();
        }

        /**SAI2_A_Block_A GPIO Configuration
        PB12     ------> SAI2_FS_A
        PB13     ------> SAI2_SCK_A
        PB15     ------> SAI2_SD_A
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15);

        /* SAI2 DMA Deinit */
        HAL_DMA_DeInit(hsai->hdmarx);
        HAL_DMA_DeInit(hsai->hdmatx);
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
