/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cordic.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
// I2C3 sensors
#include "BMP581.h"
#include "HDC302x.h"
#include "TCS34003.h"
#include "ENS160.h"
// I2C1 sensors
#include "TSL25911.h"
#include "AS7331.h"
#include "AS3935.h"
#include "PGA460.h"
#include "Wind.h"
#include "debug.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Data-ready flags set by HAL_GPIO_EXTI_Callback
extern volatile uint8_t AS3935_Ready;
extern volatile uint8_t AS7331_Ready;
extern volatile uint8_t ENS160_Ready;
extern volatile uint8_t BMP_Ready;
extern volatile uint8_t TCS34003_Ready;
extern volatile uint8_t TSL25911_Ready;

// Data to store the Height, Temperature, RH, Pressure - sound speed is calculated in Wind.c
//extern Wind_EnvData_t externalData;

// BMP581 pressure and temperature result
BMP581_sensor_data_t BMP581_Data;
// HDC302x sensor handles Address set before HDC302x_Init()
HDC302x_t Sensor1;
HDC302x_t Sensor2;

AS3935_LightningData_t AS3935_Data;
AS7331_DataOut_t AS7331_Data;
ENS160_Data_t ENS160_Data;
ENS160_Resistance_t ENS160_Resistance;
TCS34003_LightData_t TCS34003_Data;
TSL25911_LightData_t TSL25911_Data;

Wind_t windData;

uint32_t time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_CORDIC_Init();
  /* USER CODE BEGIN 2 */

	//myI2C_Scan();
	//PGA460_Init(0);
	//PGA460_WriteUARTAddr(1);
	//PGA460_ScanAndReport();
	
	// Initialize HDC302x Sensors
	Sensor1.Address = HDC302X_SENSOR_1_ADDR;
	if (HDC302x_Init(&Sensor1) != HAL_OK) {
			DEBUG("Sensor 1 initialization failed!\n");
	} else {
			DEBUG("Sensor 1 initialized successfully.\n");
	}

	Sensor2.Address = HDC302X_SENSOR_2_ADDR;
	if (HDC302x_Init(&Sensor2) != HAL_OK) {
			DEBUG("Sensor 2 initialization failed!\n");
	} else {
			DEBUG("Sensor 2 initialized successfully.\n");
	}

	// Start HDC302x auto-measurement at 0.5 Hz LPM0 sensor measures continuously;
	// HDC302x_ReadData() fetches the latest result from the output register
	HDC302x_StartAutoMeasurement(&Sensor1, HDC302X_CMD_AUTO_MEASUREMENT_1_EVERY_2S_LPM0);
	HDC302x_StartAutoMeasurement(&Sensor2, HDC302X_CMD_AUTO_MEASUREMENT_1_EVERY_2S_LPM0);
	// Initialize BMP581 0.5Hz
	if (BMP581_Init() == HAL_OK) {
			DEBUG("BMP_INIT_OK!\n");
	} else {
			DEBUG("BMP_INIT_FAIL!\n");
	}
	// Air quality sensor
	if (ENS160_Init() != HAL_OK) {
			DEBUG("ENS160 initialization failed!\n");
	}
	// Lightning sensor
	if (AS3935_Init() != HAL_OK) {
			DEBUG("AS3935 initialization failed!\n");
//	} else {
//		HAL_TIM_Base_Start(&htim2);
//		DEBUG("_______________________\n");
//		AS3935_TuneAntenna();
	}
	
	// Light sensors
	if (AS7331_Init() != HAL_OK) {
			DEBUG("AS7331 initialization failed!\n");
	}
	if (TCS34003_Init() != HAL_OK) {
			DEBUG("TCS34003 initialization failed!\n");
	}
	if (TSL25911_Init() != HAL_OK) {
			DEBUG("TSL25911 initialization failed!\n");
	}
	//PGA460_Init(0); // No EERPOM burning
	//Wind_Init();
	time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // BMP581 update pressure and temperature (1 Hz, interrupt-driven)
			// The data is sent to Wind library to calculate the speed of sound
			if (BMP_Ready) {
					BMP_Ready				= 0;
					//float temp1			= 0.0f;
					//float temp2			= 0.0f;
					float rh1				= 0.0f;
					float rh2				= 0.0f;
					BMP581_Get_TempPressData(&BMP581_Data);
					if (HDC302x_ReadData(&Sensor1) == HAL_OK) {
							//temp1 = Sensor1.Data.Temperature; // HDC is reporting higher temperature - not used
							rh1   = Sensor1.Data.Humidity;
					}
					if (HDC302x_ReadData(&Sensor2) == HAL_OK) {
							//temp2 = Sensor2.Data.Temperature;
							rh2   = Sensor2.Data.Humidity;
					}
					
					// Update sound speed with latest sensor data (called ~1 Hz with BMP_Ready)
					externalData.Temperature	= BMP581_Data.temperature;
					externalData.RH						= (rh1   + rh2)   / 2.0f;
					externalData.Pressure			= BMP581_Data.pressure;
					ENS160_SetEnvCompensation(externalData.Temperature, externalData.RH);
					Wind_ComputeSoundSpeed(&externalData);
			}
			
			if (AS3935_Ready) {
				AS3935_Ready = 0;
				AS3935_ReadLightningData(&AS3935_Data);
			}
			
			if (AS7331_Ready) {
				AS7331_Ready = 0;
				AS7331_ReadUVData(&AS7331_Data);
			}
			
			if (ENS160_Ready) {
				ENS160_Ready = 0;
				//ENS160_ReadDeviceStatus();
				ENS160_ReadAllData(&ENS160_Data, &ENS160_Resistance);
			}
			
			// Once every 1009ms
			if (TCS34003_Ready) {
				TCS34003_Ready = 0;
        if (TCS34003_GetLightData(&TCS34003_Data) != HAL_OK) {
					DEBUG("TCS34003 read failed!\n");
        }
			}
			
			// Once every 586ms
			if (TSL25911_Ready) {
				TSL25911_Ready = 0;
				HAL_Delay(2);
        if (TSL25911_ReadLightData(&TSL25911_Data) != HAL_OK) {
					DEBUG("TSL25911 read failed!\n");
        }
				DEBUG("Period: %d ms \n", HAL_GetTick() - time);
				time = HAL_GetTick();
			}
//			if (Wind_Measure(&windData) == HAL_OK) {
//					DEBUG("Wind: %.2f m/s @ %.1f deg\n", windData.Speed, windData.Direction);
//			}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: DEBUG("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
