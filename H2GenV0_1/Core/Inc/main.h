/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f1xx_hal.h"

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

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define A_IN_4_20_Pin GPIO_PIN_1
#define A_IN_4_20_GPIO_Port GPIOA
#define A_IN_MAP_Pin GPIO_PIN_2
#define A_IN_MAP_GPIO_Port GPIOA
#define A_IN_WQ_Pin GPIO_PIN_3
#define A_IN_WQ_GPIO_Port GPIOA
#define WQ_Active_Pin GPIO_PIN_4
#define WQ_Active_GPIO_Port GPIOA
#define A_IN_TEMP_Pin GPIO_PIN_5
#define A_IN_TEMP_GPIO_Port GPIOA
#define D_IN_1_Pin GPIO_PIN_4
#define D_IN_1_GPIO_Port GPIOC
#define D_IN_2_Pin GPIO_PIN_5
#define D_IN_2_GPIO_Port GPIOC
#define D_IN_3_Pin GPIO_PIN_0
#define D_IN_3_GPIO_Port GPIOB
#define D_IN_4_Pin GPIO_PIN_1
#define D_IN_4_GPIO_Port GPIOB
#define D_IN_6_Pin GPIO_PIN_10
#define D_IN_6_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_11
#define LED_GPIO_Port GPIOB
#define FaultPin1_Pin GPIO_PIN_12
#define FaultPin1_GPIO_Port GPIOB
#define FaultPin2_Pin GPIO_PIN_13
#define FaultPin2_GPIO_Port GPIOB
#define BTN2_Pin GPIO_PIN_15
#define BTN2_GPIO_Port GPIOB
#define BTN1_Pin GPIO_PIN_6
#define BTN1_GPIO_Port GPIOC
#define D_OUT_1_Pin GPIO_PIN_7
#define D_OUT_1_GPIO_Port GPIOC
#define D_OUT_2_Pin GPIO_PIN_8
#define D_OUT_2_GPIO_Port GPIOC
#define DI1_Pin GPIO_PIN_8
#define DI1_GPIO_Port GPIOA
#define RS485_TX1_Pin GPIO_PIN_9
#define RS485_TX1_GPIO_Port GPIOA
#define RS485_RX1_Pin GPIO_PIN_10
#define RS485_RX1_GPIO_Port GPIOA
#define RS485_TX2_Pin GPIO_PIN_10
#define RS485_TX2_GPIO_Port GPIOC
#define RS485_RX2_Pin GPIO_PIN_11
#define RS485_RX2_GPIO_Port GPIOC
#define UART_TX_Pin GPIO_PIN_12
#define UART_TX_GPIO_Port GPIOC
#define UART_RX_Pin GPIO_PIN_2
#define UART_RX_GPIO_Port GPIOD
#define DI2_Pin GPIO_PIN_5
#define DI2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
