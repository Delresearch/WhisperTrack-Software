#ifndef BOARD_H
#define BOARD_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Global Peripheral Handles ------------------------------------------------*/
extern COMP_HandleTypeDef hcomp1;
extern COMP_HandleTypeDef hcomp2;
extern DAC_HandleTypeDef hdac1;
extern SAI_HandleTypeDef hsai_BlockA1;
extern SAI_HandleTypeDef hsai_BlockA2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart3;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Function Prototypes ------------------------------------------------------*/

/**
 * @brief Initialize the board, including peripherals and system clock.
 */
void Board_Init(void);

/**
 * @brief Configure the system clock.
 */
void SystemClock_Config(void);

/* GPIO Initialization */
 void MX_GPIO_Init(void);

/* Peripheral Initializations */
 void MX_COMP1_Init(void);
 void MX_COMP2_Init(void);
 void MX_DAC1_Init(void);
 void MX_TIM1_Init(void);
 void MX_TIM8_Init(void);
 void MX_USART3_UART_Init(void);
 void MX_USB_OTG_FS_PCD_Init(void);
 void MX_SAI1_Init(void);
 void MX_SAI2_Init(void);
 void MX_DMA_Init(void);
 void EnableDebugFeatures(void);
/* Error Handler */
void Error_Handler(void);

size_t osReceive(uint32_t*, size_t);
size_t osTransmit(int16_t*, size_t);
void test_gpio(uint8_t state);
void osCOMP2_Start();
void osCOMP1_Start();
void osStartTIM(void);
void osStopTIM(void);
GPIO_PinState osGPIORead(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void osGPIOWrite(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
int osConsole_ReadByte(void);
void GateEnable(uint8_t state);
void BoostEnable(uint8_t state);
void Buck10VEnable(uint8_t state);
void fontus_tx_start(void);
void fontus_tx_stop(void);
#endif /* BOARD_H */
