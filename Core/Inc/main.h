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
#include "stm32g4xx_hal.h"

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
#define DECPL0_Pin GPIO_PIN_0
#define DECPL0_GPIO_Port GPIOA
#define DECPL1_Pin GPIO_PIN_1
#define DECPL1_GPIO_Port GPIOA
#define DECPL2_Pin GPIO_PIN_2
#define DECPL2_GPIO_Port GPIOA
#define UST_RX_Pin GPIO_PIN_10
#define UST_RX_GPIO_Port GPIOB
#define UST_TX_Pin GPIO_PIN_11
#define UST_TX_GPIO_Port GPIOB
#define RST_HDC_Pin GPIO_PIN_12
#define RST_HDC_GPIO_Port GPIOB
#define INT_AS3935_Pin GPIO_PIN_13
#define INT_AS3935_GPIO_Port GPIOB
#define INT_AS3935_EXTI_IRQn EXTI15_10_IRQn
#define BMP_INT_Pin GPIO_PIN_14
#define BMP_INT_GPIO_Port GPIOB
#define BMP_INT_EXTI_IRQn EXTI15_10_IRQn
#define INT_ES160_Pin GPIO_PIN_15
#define INT_ES160_GPIO_Port GPIOB
#define INT_ES160_EXTI_IRQn EXTI15_10_IRQn
#define ESP_RX_Pin GPIO_PIN_9
#define ESP_RX_GPIO_Port GPIOA
#define ESP_TX_Pin GPIO_PIN_10
#define ESP_TX_GPIO_Port GPIOA
#define SET_ESP_MSG_Pin GPIO_PIN_11
#define SET_ESP_MSG_GPIO_Port GPIOA
#define GET_ESP_MSG_Pin GPIO_PIN_12
#define GET_ESP_MSG_GPIO_Port GPIOA
#define GET_ESP_MSG_EXTI_IRQn EXTI15_10_IRQn
#define RST_HDC3020_Pin GPIO_PIN_15
#define RST_HDC3020_GPIO_Port GPIOA
#define INT_TCS34717_Pin GPIO_PIN_4
#define INT_TCS34717_GPIO_Port GPIOB
#define INT_TSL25911_Pin GPIO_PIN_6
#define INT_TSL25911_GPIO_Port GPIOB
#define INT_TSL25911_EXTI_IRQn EXTI9_5_IRQn
#define INT_AS7331_Pin GPIO_PIN_7
#define INT_AS7331_GPIO_Port GPIOB
#define INT_AS7331_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
