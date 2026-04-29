/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
// Data-ready flag set by HAL_GPIO_EXTI_Callback on XXXX_INT_Pin
volatile uint8_t AS3935_Ready			= 0;
volatile uint8_t AS7331_Ready			= 0;
volatile uint8_t ENS160_Ready			= 0;
volatile uint8_t BMP_Ready				= 0;
volatile uint8_t TCS34003_Ready		= 0;
volatile uint8_t TSL25911_Ready		= 0;
volatile uint8_t ESP_Transmitting	= 0;
/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SGN_Pin|SET_ESP_MSG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_HDC_GPIO_Port, RST_HDC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : INT_AS3935_Pin */
  /* IRQ mode: plain EXTI rising-edge.
   * CALIB mode: PA3 is claimed by TIM2 CH4 (AF1) inside MX_TIM2_Init — skip EXTI. */
#ifdef AS3935_MODE_IRQ
  GPIO_InitStruct.Pin = INT_AS3935_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_AS3935_GPIO_Port, &GPIO_InitStruct);
#endif /* AS3935_MODE_IRQ */

  /*Configure GPIO pin : SGN_Pin */
  GPIO_InitStruct.Pin = SGN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SGN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_HDC_Pin */
  GPIO_InitStruct.Pin = RST_HDC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_HDC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BMP_INT_Pin INT_AS7331_Pin */
  GPIO_InitStruct.Pin = BMP_INT_Pin|INT_AS7331_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_ES160_Pin INT_TCS34003_Pin INT_TSL25911_Pin */
  GPIO_InitStruct.Pin = INT_ES160_Pin|INT_TCS34003_Pin|INT_TSL25911_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SET_ESP_MSG_Pin */
  GPIO_InitStruct.Pin = SET_ESP_MSG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SET_ESP_MSG_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
#ifdef AS3935_MODE_IRQ
  HAL_NVIC_SetPriority(EXTI3_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
#endif /* AS3935_MODE_IRQ */

  HAL_NVIC_SetPriority(EXTI4_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */
/* In AS3935_MODE_CALIB PA3 belongs to TIM2 CH4 (AF1).
 * Enable/Disable are no-ops in that mode; the pin is managed by tim.c. */
#ifdef AS3935_MODE_IRQ
void Enable_EXTI_AS3935(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  /*Configure GPIO pins : INT_AS3935_Pin */
  GPIO_InitStruct.Pin = INT_AS3935_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(INT_AS3935_GPIO_Port, &GPIO_InitStruct);	
	
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

void Disable_EXTI_AS3935(void) {
	HAL_GPIO_DeInit(INT_AS3935_GPIO_Port, INT_AS3935_Pin);	
  HAL_NVIC_DisableIRQ(EXTI3_IRQn);
}
#else /* AS3935_MODE_CALIB */
void Enable_EXTI_AS3935(void)  { /* no-op: PA3 is TIM2 CH4 in CALIB mode */ }
void Disable_EXTI_AS3935(void) { /* no-op: PA3 is TIM2 CH4 in CALIB mode */ }
#endif /* AS3935_MODE_IRQ */

// EXTI callback sets sensor ready flags
// NOTE: HAL_GPIO_EXTI_IRQHandler already clears the pending bit. Do NOT call
// __HAL_GPIO_EXTI_CLEAR_IT here; it can re-trigger the interrupt on some STM32F3 silicon.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// PA3 — only active as EXTI in AS3935_MODE_IRQ; in CALIB mode PA3 is TIM2 CH4.
#ifdef AS3935_MODE_IRQ
	if (GPIO_Pin == INT_AS3935_Pin) {
		AS3935_Ready = 1;
	}
#endif /* AS3935_MODE_IRQ */
	// PB4
	if (GPIO_Pin == INT_TCS34003_Pin) {
		TCS34003_Ready = 1;
	}
	// PB6
	if (GPIO_Pin == INT_TSL25911_Pin) {
		TSL25911_Ready = 1;
	}
	// PB7
	if (GPIO_Pin == INT_AS7331_Pin) {
		AS7331_Ready = 1;
	}
	// PB14	
	if (GPIO_Pin == BMP_INT_Pin) {
		BMP_Ready = 1;
	}
	// PB15
	if (GPIO_Pin == INT_ES160_Pin) {
		ENS160_Ready = 1;
	}
}
/* USER CODE END 2 */