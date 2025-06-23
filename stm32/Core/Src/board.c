/* Includes ------------------------------------------------------------------*/
#include "board.h"

#include <string.h>  // for memcpy

#include "FreeRTOS.h"  // for BaseType_t, portYIELD_FROM_ISR, portMAX_DELAY
#include "SEGGER_RTT.h"
#include "main.h"
#include "common.h"
#include "semphr.h"  // for SemaphoreHandle_t, xSemaphoreGiveFromISR, xSemaphoreTake
/* one DMA interrupt = one 128‑byte *half* (32 × 4 bytes)                 // ⇦ */

__attribute__((section(".ram_d2"))) uint32_t dmaBuffer[DMA_RING_SAMPLES];  
static volatile uint32_t halfIdx = 0;         /* 0 = first half full, 1 = second */
extern SemaphoreHandle_t filledSem;
uint8_t xrun_count = 0;
/* Global Peripheral Handles ------------------------------------------------*/
COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;
DAC_HandleTypeDef hdac1;
SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockA2;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

DMA_HandleTypeDef hdma_sai2_a;
#define UART_RX_DMA_SIZE  256

/* circular DMA target buffer */
static uint8_t  uart_rx_dma_buf[UART_RX_DMA_SIZE];
/* software “tail” pointer */
static volatile size_t uart_rx_tail = 0;
/* Board Initialization -----------------------------------------------------*/
void Board_Init(void) {
  HAL_Init();
  SystemClock_Config();
  MX_DMA_Init();
  MX_GPIO_Init();
  MX_COMP1_Init();
  MX_COMP2_Init();
  MX_DAC1_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_SAI1_Init();
  MX_SAI2_Init();
  EnableDebugFeatures();
}

static void Console_UART_DMA_Init(void)
{
  /* enable IDLE‐line IRQ so we can recalc the DMA head if needed */
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
  /* start RX in circular DMA mode */
  HAL_UART_Receive_DMA(&huart3,
                       uart_rx_dma_buf,
                       UART_RX_DMA_SIZE);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}
/**
 * @brief COMP1 Initialization Function
 * @param None
 * @retval None
 */
void MX_COMP1_Init(void) {
  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InvertingInput = COMP_INPUT_MINUS_1_2VREFINT;
  hcomp1.Init.NonInvertingInput = COMP_INPUT_PLUS_IO1;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.Mode = COMP_POWERMODE_HIGHSPEED;
  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */
}

/**
 * @brief COMP2 Initialization Function
 * @param None
 * @retval None
 */
void MX_COMP2_Init(void) {
  /* USER CODE BEGIN COMP2_Init 0 */

  /* USER CODE END COMP2_Init 0 */

  /* USER CODE BEGIN COMP2_Init 1 */

  /* USER CODE END COMP2_Init 1 */
  hcomp2.Instance = COMP2;
  hcomp2.Init.InvertingInput = COMP_INPUT_MINUS_IO2;
  hcomp2.Init.NonInvertingInput = COMP_INPUT_PLUS_IO1;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp2.Init.Mode = COMP_POWERMODE_HIGHSPEED;
  hcomp2.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP2_Init 2 */

  /* USER CODE END COMP2_Init 2 */
}
/**
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
void MX_DAC1_Init(void) {
  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
   */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK) {
    Error_Handler();
  }

  /** DAC channel OUT1 config
   */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */
}

/**
 * @brief SAI1 Initialization Function
 * @param None
 * @retval None
 */
void MX_SAI1_Init(void) {
  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA1.FrameInit.FrameLength = 8;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA1.SlotInit.SlotNumber = 1;
  hsai_BlockA1.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */
}

/**
 * @brief SAI2 Initialization Function
 * @param None
 * @retval None
 */
void MX_SAI2_Init(void) {
  /* USER CODE BEGIN SAI2_Init 0 */

  /* USER CODE END SAI2_Init 0 */

  /* USER CODE BEGIN SAI2_Init 1 */

  /* USER CODE END SAI2_Init 1 */
  /* USER CODE END SAI2_Init 1 */
  hsai_BlockA2.Instance = SAI2_Block_A;
  hsai_BlockA2.Init.AudioMode = SAI_MODEMASTER_RX;
  hsai_BlockA2.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_MCKDIV;
  hsai_BlockA2.Init.Mckdiv = 4;
  hsai_BlockA2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;

  if (HAL_SAI_InitProtocol(&hsai_BlockA2, SAI_I2S_STANDARD,
                           SAI_PROTOCOL_DATASIZE_32BIT, 2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI2_Init 2 */
  // Start the SAI2 RX DMA into dmaRingBuffer continuously:
  // Start continuous, circular DMA on SAI2 → dmaRingBuffer
  // Start continuous, circular DMA → 128 B chunks
    HAL_SAI_Receive_DMA(&hsai_BlockA2,
        (uint8_t *)dmaBuffer,
        DMA_RING_SAMPLES );      // 128 half‑words
  __HAL_DMA_ENABLE_IT(hsai_BlockA2.hdmarx, DMA_IT_HT | DMA_IT_TC);

  /* USER CODE END SAI2_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
void MX_TIM1_Init(void) {
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
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
void MX_USART3_UART_Init(void) {
  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  Console_UART_DMA_Init();
  /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
void MX_DMA_Init() {
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE,
                    BUCK_10V_EN_Pin | BUCK_10V_MODE_Pin | BOOST_EN_Pin |
                        GATE_DRIVE_EN_N_Pin | OUTPUT_OD_N_Pin | SMPS_V1_Pin |
                        SMPS_EN_Pin | SMPS_SW_Pin | SAI1_EXTCLK_EN_N_Pin |
                        BUCK_1V95_MODE_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AMP_GS_GPIO_Port, AMP_GS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, VIN_DIV_EN_Pin | GPIO_PIN_14 | BOOST_48V_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pins : BUCK_10V_EN_Pin BUCK_10V_MODE_Pin BOOST_EN_Pin
     GATE_DRIVE_EN_N_Pin OUTPUT_OD_N_Pin SMPS_V1_Pin SMPS_EN_Pin SMPS_SW_Pin
                           SAI1_EXTCLK_EN_N_Pin BUCK_1V95_MODE_Pin */
  GPIO_InitStruct.Pin = BUCK_10V_EN_Pin | BUCK_10V_MODE_Pin | BOOST_EN_Pin |
                        GATE_DRIVE_EN_N_Pin | OUTPUT_OD_N_Pin | SMPS_V1_Pin |
                        SMPS_EN_Pin | SMPS_SW_Pin | SAI1_EXTCLK_EN_N_Pin |
                        BUCK_1V95_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 BOOST_PGOOD_Pin SMPS_PG_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | BOOST_PGOOD_Pin | SMPS_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : STATUS_LED_Pin */
  GPIO_InitStruct.Pin = STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(STATUS_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AMP_GS_Pin VIN_DIV_EN_Pin PD14 BOOST_48V_Pin */
  GPIO_InitStruct.Pin =
      AMP_GS_Pin | VIN_DIV_EN_Pin | GPIO_PIN_14 | BOOST_48V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

void EnableDebugFeatures(void) {
  // Optional: Enable fault handlers for UsageFault, BusFault, and
  // MemManageFault
  SCB->SHCSR |= (SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk |
                 SCB_SHCSR_MEMFAULTENA_Msk);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
}
void osCOMP2_Start() { HAL_COMP_Start(&hcomp2); }

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
  if (hsai->Instance == SAI2_Block_A) {
    halfIdx = 0;
    BaseType_t woken = pdFALSE;
    // xrun count++;
    if (xSemaphoreGiveFromISR(filledSem, &woken) != pdTRUE) {
      xrun_count++;
  }
    portYIELD_FROM_ISR(woken);
  }
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai) {
  if (hsai->Instance == SAI2_Block_A) {
    halfIdx = 1;
    BaseType_t woken = pdFALSE;
    if (xSemaphoreGiveFromISR(filledSem, &woken) != pdTRUE) {
      xrun_count++;
  }

    portYIELD_FROM_ISR(woken);
  }
}
// make a test gpio function at can be on or off use PC4
void test_gpio(uint8_t state) {
  if (state) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
  }
}

size_t osReceive(uint32_t *dst, size_t len) {
#ifdef STM32L496xx
#ifdef USE_RTT
  int rttChannel = 0;                   // Specify the RTT channel to read from
  static uint8_t rttbuffer[16 * 1024];  // Static buffer for storing RTT data
  static size_t bufferIndex =
      0;  // Tracks the current read position in the buffer
  static size_t bufferSize = 0;  // Tracks the size of valid data in the buffer

  // If the buffer has no data left to serve, perform a single SEGGER RTT read
  if (bufferIndex >= bufferSize) {
    bufferSize = SEGGER_RTT_Read(rttChannel, rttbuffer, sizeof(rttbuffer));
    bufferIndex = 0;  // Reset the index for the new data
  }

  // Calculate how much data we can actually copy
  size_t remainingData = bufferSize - bufferIndex;
  size_t bytesToCopy = (Size < remainingData) ? Size : remainingData;

  // Copy the data to the destination buffer
  memcpy(pData, &rttbuffer[bufferIndex], bytesToCopy);

  // Update the buffer index to track what we've consumed
  bufferIndex += bytesToCopy;

  return bytesToCopy;  // Return the actual number of bytes copied
#endif

  /* Wait until DMA ISR signals one half ready */


xSemaphoreTake(filledSem, portMAX_DELAY);

  /* Clear the xrun count */
               /* timeout */
                 memcpy(dst,
                    &dmaBuffer[halfIdx * CHUNK_SAMPLES],
                    CHUNK_BYTES);
return CHUNK_SAMPLES;  
#endif
}
int osConsole_ReadByte(void)
{
#if defined(STM32L496xx) && defined(USE_RTT)
  int ch = SEGGER_RTT_GetKey();
  return (ch >= 0) ? ch : -1;
#else
  /* how many bytes have DMA written so far? */
  uint32_t ndtr = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
  size_t head = UART_RX_DMA_SIZE - ndtr;

  if (head != uart_rx_tail) {
    uint8_t b = uart_rx_dma_buf[uart_rx_tail];
    uart_rx_tail = (uart_rx_tail + 1) % UART_RX_DMA_SIZE;
    return b;
  }
  return -1;
#endif
}

