/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DMX_TX_Pin GPIO_PIN_2
#define DMX_TX_GPIO_Port GPIOA
#define DMX_RX_Pin GPIO_PIN_3
#define DMX_RX_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_4
#define SD_CS_GPIO_Port GPIOA
#define LAD_LED_Pin GPIO_PIN_0
#define LAD_LED_GPIO_Port GPIOB
#define CLK_LED_Pin GPIO_PIN_1
#define CLK_LED_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_12
#define D1_GPIO_Port GPIOB
#define D2_Pin GPIO_PIN_13
#define D2_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_14
#define D3_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_15
#define D4_GPIO_Port GPIOB
#define BT_CONNECT_Pin GPIO_PIN_15
#define BT_CONNECT_GPIO_Port GPIOA
#define BT_CONNECT_EXTI_IRQn EXTI15_10_IRQn
#define DATA_LED_Pin GPIO_PIN_3
#define DATA_LED_GPIO_Port GPIOB
#define BT_UP_Pin GPIO_PIN_4
#define BT_UP_GPIO_Port GPIOB
#define BT_UP_EXTI_IRQn EXTI4_IRQn
#define BT_DOWN_Pin GPIO_PIN_5
#define BT_DOWN_GPIO_Port GPIOB
#define BT_DOWN_EXTI_IRQn EXTI9_5_IRQn
#define BT_1_Pin GPIO_PIN_6
#define BT_1_GPIO_Port GPIOB
#define BT_1_EXTI_IRQn EXTI9_5_IRQn
#define BT_2_Pin GPIO_PIN_7
#define BT_2_GPIO_Port GPIOB
#define BT_2_EXTI_IRQn EXTI9_5_IRQn
#define BT_3_Pin GPIO_PIN_8
#define BT_3_GPIO_Port GPIOB
#define BT_3_EXTI_IRQn EXTI9_5_IRQn
#define BT4_Pin GPIO_PIN_9
#define BT4_GPIO_Port GPIOB
#define BT4_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */
#define DMX_UART_INIT_SEND_ADD huart2.Init.StopBits = UART_STOPBITS_1
#define DMX_UART_INIT_SEND_DATA huart2.Init.StopBits = UART_STOPBITS_2
extern void delay_us(uint32_t us);
extern void delay_ms(uint32_t u32DelayInMs);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
