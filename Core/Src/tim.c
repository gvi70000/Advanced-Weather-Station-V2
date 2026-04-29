/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
#include <string.h>

volatile DecplTimes_t  ToF_Result[3] = {{0, 0}, {0, 0}, {0, 0}};
volatile uint8_t       Decpl_RDY     = 0;

/* Internal bitmask — shared between the callback and Wind_TIM2_Arm so that
 * Arm() can reset it atomically before re-enabling the channels.            */
static volatile uint8_t s_channel_done = 0;
/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim2_ch2;
DMA_HandleTypeDef hdma_tim2_ch3;

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* TIM2 clock = 170 MHz, PSC = 16  →  tick period = 1/(170MHz/17) = 100 ns
   * (TICKS_PER_US = 10, i.e. 10 ticks per microsecond).
   * Period = 0xFFFFFFFF (32-bit free-run, wrap ~429 s — no overflow risk).   */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
#ifdef AS3935_MODE_CALIB
  /* PA3 = TIM2_CH4 used for antenna frequency measurement */
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
#endif /* AS3935_MODE_CALIB */
  /* USER CODE END TIM2_Init 2 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA0     ------> TIM2_CH1  (DECPL0)
    PA1     ------> TIM2_CH2  (DECPL1)
    PA2     ------> TIM2_CH3  (DECPL2)
    PA3     ------> TIM2_CH4  (INT_AS3935 — CALIB mode only)
    */
#ifdef AS3935_MODE_CALIB
    GPIO_InitStruct.Pin = DECPL0_Pin|DECPL1_Pin|DECPL2_Pin|INT_AS3935_Pin;
#else
    GPIO_InitStruct.Pin = DECPL0_Pin|DECPL1_Pin|DECPL2_Pin;
#endif /* AS3935_MODE_CALIB */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM2 DMA Init */
    /* TIM2_CH1 Init */
    hdma_tim2_ch1.Instance = DMA1_Channel3;
    hdma_tim2_ch1.Init.Request = DMA_REQUEST_TIM2_CH1;
    hdma_tim2_ch1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim2_ch1.Init.Mode = DMA_NORMAL;
    hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_tim2_ch1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC1],hdma_tim2_ch1);

    /* TIM2_CH2 Init */
    hdma_tim2_ch2.Instance = DMA1_Channel4;
    hdma_tim2_ch2.Init.Request = DMA_REQUEST_TIM2_CH2;
    hdma_tim2_ch2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_tim2_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_ch2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim2_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim2_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim2_ch2.Init.Mode = DMA_NORMAL;
    hdma_tim2_ch2.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_tim2_ch2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC2],hdma_tim2_ch2);

    /* TIM2_CH3 Init */
    hdma_tim2_ch3.Instance = DMA1_Channel5;
    hdma_tim2_ch3.Init.Request = DMA_REQUEST_TIM2_CH3;
    hdma_tim2_ch3.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_tim2_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_ch3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim2_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim2_ch3.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim2_ch3.Init.Mode = DMA_NORMAL;
    hdma_tim2_ch3.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_tim2_ch3) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC3],hdma_tim2_ch3);

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */
	
  /* USER CODE END TIM2_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */
  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /**TIM2 GPIO Configuration
    PA0     ------> TIM2_CH1  (DECPL0)
    PA1     ------> TIM2_CH2  (DECPL1)
    PA2     ------> TIM2_CH3  (DECPL2)
    PA3     ------> TIM2_CH4  (INT_AS3935 — CALIB mode only)
    */
#ifdef AS3935_MODE_CALIB
    HAL_GPIO_DeInit(GPIOA, DECPL0_Pin|DECPL1_Pin|DECPL2_Pin|INT_AS3935_Pin);
#else
    HAL_GPIO_DeInit(GPIOA, DECPL0_Pin|DECPL1_Pin|DECPL2_Pin);
#endif /* AS3935_MODE_CALIB */

    /* TIM2 DMA DeInit */
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC1]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC2]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC3]);

    /* TIM2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* =========================================================================
 * DECPL channels bitmask (HAL_TIM_ActiveChannel values, not TIM_CHANNEL_x).
 *   CH1 = 0x01,  CH2 = 0x02,  CH3 = 0x04,  CH4 = 0x08
 * CH4 (AS3935 in CALIB mode) is excluded by not being in the 0x07 mask.
 * ======================================================================= */
#define DECPL_CHANNELS_MASK \
    ((uint8_t)(HAL_TIM_ACTIVE_CHANNEL_1 | HAL_TIM_ACTIVE_CHANNEL_2 | HAL_TIM_ACTIVE_CHANNEL_3))

/* -------------------------------------------------------------------------
 * HAL_TIM_IC_CaptureCallback
 *
 * Called by the HAL DMA-TC ISR for each TIM2 input-capture channel.
 * Accumulates completed channels in s_channel_done; when all three DECPL
 * channels (CH1/2/3) have fired their DMA Transfer Complete, sets Decpl_RDY.
 *
 * s_channel_done is also reset in Wind_TIM2_Arm() before each round to
 * discard any partial accumulation left by an aborted previous round.
 * ---------------------------------------------------------------------- */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance != TIM2) { return; }
    if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) ||
        (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) ||
        (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3))
    {
        s_channel_done |= (uint8_t)htim->Channel;
        if ((s_channel_done & DECPL_CHANNELS_MASK) == DECPL_CHANNELS_MASK) {
            s_channel_done = 0;
            Decpl_RDY      = 1;
        }
    }
}

/* -------------------------------------------------------------------------
 * Wind_TIM2_Arm
 *
 * Prepares TIM2 CH1/2/3 for one measurement round.  Must be called
 * immediately before the PGA460 UART commands for each transmitter turn.
 *
 * Sequence (RM0440 §12.6.3/12.6.4 compliant):
 *   1. Stop timer counter -- no captures during setup.
 *   2. Stop DMA via HAL (disables CCxDE and clears HAL state).
 *   3. Per channel: clear CCR.EN, poll until EN=0, clear DMA flags,
 *      reload CNDTR=2 (E0 reference + E1 echo).
 *   4. Reset s_channel_done, Decpl_RDY, and ToF_Result[].
 *   5. Re-arm DMA via HAL_TIM_IC_Start_DMA.
 *   6. Zero counter and start timer -- all channels capture from t=0.
 * ---------------------------------------------------------------------- */
void Wind_TIM2_Arm(void) {
    static const uint8_t  ch_ids[3] = { TIM_DMA_ID_CC1, TIM_DMA_ID_CC2, TIM_DMA_ID_CC3 };
    static const uint32_t ch_sel[3] = { TIM_CHANNEL_1,  TIM_CHANNEL_2,  TIM_CHANNEL_3  };

    /* 1. Stop counter */
    __HAL_TIM_DISABLE(&htim2);

    /* 2+3. Stop, reload, clear flags for each DMA channel */
    for (uint8_t i = 0; i < 3u; i++) {
        DMA_HandleTypeDef *hdma = htim2.hdma[ch_ids[i]];

        /* 2. Stop DMA (disables CCxDE, clears HAL gState) */
        HAL_TIM_IC_Stop_DMA(&htim2, ch_sel[i]);

        /* 3a. Belt-and-braces EN clear + mandatory poll (RM0440 §12.6.4) */
        hdma->Instance->CCR &= ~DMA_CCR_EN;
        while (hdma->Instance->CCR & DMA_CCR_EN) {}

        /* 3b. Clear TC/HT/TE flags -- prevents a false immediate TC on re-enable */
        __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
        __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma));
        __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TE_FLAG_INDEX(hdma));

        /* 3c. Reload transfer count (CNDTR write only valid while EN=0) */
        hdma->Instance->CNDTR = 2u;
    }

    /* 4. Reset sync flags and capture buffers (DMA disabled -- no race) */
    s_channel_done = 0;
    Decpl_RDY      = 0;
    memset((void *)ToF_Result, 0, sizeof(ToF_Result));

    /* 5. Re-arm all three DMA channels */
    HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)&ToF_Result[0].E0, 2);
    HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, (uint32_t *)&ToF_Result[1].E0, 2);
    HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_3, (uint32_t *)&ToF_Result[2].E0, 2);

    /* 6. Zero counter then start -- all channels capture from t = 0 */
    __HAL_TIM_SET_COUNTER(&htim2, 0u);
    __HAL_TIM_ENABLE(&htim2);
}

/* -------------------------------------------------------------------------
 * Wind_TIM2_Disarm
 *
 * Stops the timer and all three DMA channels after a measurement round
 * (whether successful or timed-out).
 * ---------------------------------------------------------------------- */
void Wind_TIM2_Disarm(void) {
    __HAL_TIM_DISABLE(&htim2);
    HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_2);
    HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_3);
}

/* USER CODE END 1 */