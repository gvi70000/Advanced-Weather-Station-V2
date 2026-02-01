/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>

/* Per-bus DMA transfer completion flags (used when USE_I2C_DMA is enabled) */
#ifdef USE_I2C_DMA
	volatile uint8_t i2c1_transfer_complete = 0;
	volatile uint8_t i2c3_transfer_complete = 0;
#endif
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x40621236;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}
/* I2C3 init function */
void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x40B285C2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB8-BOOT0     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
  else if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspInit 0 */

  /* USER CODE END I2C3_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C3;
    PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C3 GPIO Configuration
    PA8     ------> I2C3_SCL
    PB5     ------> I2C3_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_I2C3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_I2C3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C3 clock enable */
    __HAL_RCC_I2C3_CLK_ENABLE();
  /* USER CODE BEGIN I2C3_MspInit 1 */

  /* USER CODE END I2C3_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB8-BOOT0     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
  else if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspDeInit 0 */

  /* USER CODE END I2C3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C3_CLK_DISABLE();

    /**I2C3 GPIO Configuration
    PA8     ------> I2C3_SCL
    PB5     ------> I2C3_SDA
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5);

  /* USER CODE BEGIN I2C3_MspDeInit 1 */

  /* USER CODE END I2C3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
static volatile uint8_t* get_flag_for_bus(I2C_HandleTypeDef* hi2c) {
  if (hi2c->Instance == I2C1) return &i2c1_transfer_complete;
  if (hi2c->Instance == I2C3) return &i2c3_transfer_complete;
  return NULL;
}

void myI2C_Scan(I2C_HandleTypeDef* hi2c) {
  if (!hi2c) return;
  printf("Scanning I2C bus (instance %s)...\r\n", (hi2c->Instance == I2C1) ? "I2C1" : (hi2c->Instance == I2C2) ? "I2C2" : "UNKNOWN");

  for (uint16_t addr7 = 0; addr7 < 128; addr7++)
  {
    uint16_t addr8 = (addr7 << 1); /* HAL expects 8-bit addr in some APIs */
    HAL_StatusTypeDef res = HAL_I2C_IsDeviceReady(hi2c, addr8, 1, I2C_TIMEOUT);
    if (res == HAL_OK) {
      printf("Found device at 7-bit 0x%02X (8-bit 0x%02X)\r\n", addr7, addr8);
    }
  }
}
// Common I2C write function with optional DMA support
HAL_StatusTypeDef WriteRegister(uint8_t device_address_7bit, uint8_t reg_address, uint8_t* data, uint16_t len, I2C_HandleTypeDef* hi2c) {
  if (!hi2c) return HAL_ERROR;
  //uint16_t dev8 = ((uint16_t)device_address_7bit) << 1;

#ifdef USE_I2C_DMA
  volatile uint8_t* flag = get_flag_for_bus(hi2c);
  if (!flag) return HAL_ERROR;
  *flag = 0;

  HAL_StatusTypeDef st = HAL_I2C_Mem_Write_DMA(hi2c, device_address_7bit, reg_address, I2C_MEMADD_SIZE_8BIT, data, len);
  if (st != HAL_OK) return st;

  uint32_t start = HAL_GetTick();
  while (!(*flag))
  {
    if ((HAL_GetTick() - start) > I2C_TIMEOUT) return HAL_TIMEOUT;
  }
  return HAL_OK;
#else
  return HAL_I2C_Mem_Write(hi2c, device_address_7bit, reg_address, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
#endif
}

// Common I2C read function with optional DMA support
HAL_StatusTypeDef ReadRegister(uint8_t device_address_7bit, uint8_t reg_address, uint8_t* data, uint16_t len, I2C_HandleTypeDef* hi2c) {
  if (!hi2c) return HAL_ERROR;
  //uint16_t dev8 = ((uint16_t)device_address_7bit) << 1;
#ifdef USE_I2C_DMA
  volatile uint8_t* flag = get_flag_for_bus(hi2c);
  if (!flag) return HAL_ERROR;
  *flag = 0;

  HAL_StatusTypeDef st = HAL_I2C_Mem_Read_DMA(hi2c, device_address_7bit, reg_address, I2C_MEMADD_SIZE_8BIT, data, len);
  if (st != HAL_OK) return st;

  uint32_t start = HAL_GetTick();
  while (!(*flag))
  {
    if ((HAL_GetTick() - start) > I2C_TIMEOUT) return HAL_TIMEOUT;
  }
  return HAL_OK;
#else
  return HAL_I2C_Mem_Read(hi2c, device_address_7bit, reg_address, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
#endif
}

#ifdef USE_I2C_DMA
// DMA transfer complete callback for use in interrupt service routines
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c) {
  volatile uint8_t* flag = get_flag_for_bus(hi2c);
  if (flag) *flag = 1;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c) {
  volatile uint8_t* flag = get_flag_for_bus(hi2c);
  if (flag) *flag = 1;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {
  /* On error, release waiters as well */
  volatile uint8_t* flag = get_flag_for_bus(hi2c);
  if (flag) *flag = 1;
}
#endif
/* USER CODE END 1 */
