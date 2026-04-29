/* USER CODE BEGIN Header */
/**
  ****
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ****
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ****
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
typedef struct __attribute__((packed)) {
    uint32_t E0;  /* Rising edge captured on BROADCAST_LISTEN_ONLY_P2 (reference, discarded) */
    uint32_t E1;  /* Rising edge captured on echo threshold crossing (acoustic arrival time)  */
} DecplTimes_t;

extern volatile DecplTimes_t ToF_Result[3];
extern volatile uint8_t      Decpl_RDY;

/* -----------------------------------------------------------------------
 * Wind_TIM2_Arm  -- stop, RM0440-compliant DMA reload, re-arm, start.
 * Wind_TIM2_Disarm -- stop timer and all three DMA channels.
 * Call Arm() before each PGA460 transmitter round; Disarm() after.
 * -------------------------------------------------------------------- */
void Wind_TIM2_Arm(void);
void Wind_TIM2_Disarm(void);
/* USER CODE END Includes */

extern TIM_HandleTypeDef htim2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM2_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */