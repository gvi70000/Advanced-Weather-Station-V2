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
#define SENSOR_ENV        (1 << 0)
#define SENSOR_AS7331     (1 << 1)
#define SENSOR_ENS160     (1 << 2)
#define SENSOR_TCS34003   (1 << 3)
#define SENSOR_TSL25911   (1 << 4)
#define SENSOR_WIND       (1 << 5)
#define SENSOR_AS3935     (1 << 6)
#define SENSOR_FRAME_MASK 0x3F   // bits 0–5
#define UART_FRAME_START  0xAA55
#define UART_PAYLOAD_SIZE 78

// These are usd for Temperature and RH compensation
#define DT_SEC             1.0f

#define TEMP_DEADBAND      0.2f
#define RH_DEADBAND        0.5f

#define MAX_TEMP_STEP      0.2f
#define MAX_RH_STEP        0.5f

#define TEMP_OFFSET_MIN   -1.0f
#define TEMP_OFFSET_MAX    1.0f

#define RH_OFFSET_MIN    -3.0f
#define RH_OFFSET_MAX     3.0f

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

volatile uint8_t sensorReadyFlags = 0;

// Data to store the Height, Temperature, RH, Pressure - sound speed is calculated in Wind.c
//extern Wind_EnvData_t externalData;

// BMP581 pressure and temperature result
BMP581_sensor_data_t BMP581_Data;
// HDC302x sensor handles Address set before HDC302x_Init()
HDC302x_t Sensor1;
HDC302x_t Sensor2;

//AS3935_LightningData_t AS3935_Data;
//AS7331_DataOut_t AS7331_Data;
//ENS160_Data_t ENS160_Data;
ENS160_Resistance_t ENS160_Resistance;
//TCS34003_LightData_t TCS34003_Data;
//TSL25911_LightData_t TSL25911_Data;

//Wind_t windData;
extern Wind_Input_t CalibData; // Height, PathLenD01, PathLenD12 and PathLenD20


uint32_t time = 0;

typedef struct __attribute__((packed)) {
    /* --- Environmental data --- */
    extern Wind_EnvData_t externalData;
    // Temperature (4f) – °C
    // RH          (4f) – %
    // Pressure    (4f) – hPa
	// DewPoint	   (4f) – °C
    // SoundSpeed  (4f) – m/s

    /* --- Lightning sensor (AS3935) --- */
    AS3935_LightningData_t Lightning;
    // LightningEnergy     (4u) – 20-bit energy value
    // DistanceEstimation  (1u) – distance estimation code

    /* --- UV sensor (AS7331) --- */
    AS7331_DataOut_t UV;
    // TEMP_C100 (2u) – Temperature in °C * 100
    // UVA       (2u) – UVA counts
    // UVB       (2u) – UVB counts
    // UVC       (2u) – UVC counts

    /* --- Air quality (ENS160) --- */
    ENS160_Data_t AirQuality;
    // AQI   (1u) – Air Quality Index [1..5]
    // TVOC  (2u) – Total Volatile Organic Compounds (ppb)
    // eCO2  (2u) – Equivalent CO2 (ppm)
    // Raw buffer also available (5 × 1u)

    /* --- Color sensor (TCS34003) --- */
    TCS34003_LightData_t LightRGB;
    // ClearChannel (2u)
    // RedChannel   (2u)
    // GreenChannel (2u)
    // BlueChannel  (2u)
    // FullArray    (8 × 1u)

    /* --- Light sensor (TSL25911) --- */
    TSL25911_LightData_t Light;
    // FullSpectrum (2u)
    // Infrared     (2u)
    // Visible      (2u)
    // Lux          (4f)

    /* --- Wind data --- */
    Wind_t Wind;
    // Speed     (4f) – m/s
    // Direction (4f) – degrees

} UART_Payload_t;

typedef struct __attribute__((packed)) {
    uint16_t Start;        // 0xAA55
    uint16_t Length;       // 78
    UART_Payload_t Payload;
    uint16_t CR;          // optional but recommended
} UART_Frame_t;

/* Global or local instance */
UART_Frame_t txFrameBuf;
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t CRC16(uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

void SendDataToESP(void) {
		txFrameBuf.CR = CRC16((uint8_t*)&txFrameBuf.Payload, sizeof(UART_Payload_t));
    // huart2 is the instance identified in your main.c
    HAL_UART_Transmit(&huart1, (uint8_t*)&txFrameBuf, sizeof(UART_Frame_t), HAL_MAX_DELAY);
}
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
	txFrameBuf.Start = UART_FRAME_START;
	txFrameBuf.Length = UART_PAYLOAD_SIZE;
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
	HDC302x_StartAutoMeasurement(&Sensor1, HDC302X_CMD_AUTO_MEASUREMENT_1_EVERY_1S_LPM0);
	HDC302x_StartAutoMeasurement(&Sensor2, HDC302X_CMD_AUTO_MEASUREMENT_1_EVERY_1S_LPM0);
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
			// Once every 1003ms
			if (BMP_Ready) {
				BMP_Ready = 0;
				float temp1 = 0.0f, temp2 = 0.0f;
				float rh1 = 0.0f, rh2 = 0.0f;
				static float prevOutT1 = 0.0f, prevOutT2 = 0.0f;
				static float prevOutRH1 = 0.0f, prevOutRH2 = 0.0f;
				// Read sensors
				BMP581_Get_TempPressData(&BMP581_Data);
				if (HDC302x_ReadData(&Sensor1) == HAL_OK) {
					temp1 = Sensor1.Data.Temperature;
					rh1   = Sensor1.Data.Humidity;
				}
				if (HDC302x_ReadData(&Sensor2) == HAL_OK) {
					temp2 = Sensor2.Data.Temperature;
					rh2   = Sensor2.Data.Humidity;
				}
				//  TEMPERATURE CONTROL (Adaptive PID)
				float errT1 = BMP581_Data.temperature - temp1;
				float errT2 = BMP581_Data.temperature - temp2;
				// Deadband
				if (fabsf(errT1) < TEMP_DEADBAND) errT1 = 0.0f;
				if (fabsf(errT2) < TEMP_DEADBAND) errT2 = 0.0f;
				float outT1 = PID_Update_Adaptive(&pidTemp1, errT1, DT_SEC);
				float outT2 = PID_Update_Adaptive(&pidTemp2, errT2, DT_SEC);
				// Rate limiting
				float deltaT1 = outT1 - prevOutT1;
				if (deltaT1 > MAX_TEMP_STEP) outT1 = prevOutT1 + MAX_TEMP_STEP;
				if (deltaT1 < -MAX_TEMP_STEP) outT1 = prevOutT1 - MAX_TEMP_STEP;
				float deltaT2 = outT2 - prevOutT2;
				if (deltaT2 > MAX_TEMP_STEP) outT2 = prevOutT2 + MAX_TEMP_STEP;
				if (deltaT2 < -MAX_TEMP_STEP) outT2 = prevOutT2 - MAX_TEMP_STEP;
				prevOutT1 = outT1;
				prevOutT2 = outT2;
				// Clamp offsets
				if (outT1 > TEMP_OFFSET_MAX) outT1 = TEMP_OFFSET_MAX;
				if (outT1 < TEMP_OFFSET_MIN) outT1 = TEMP_OFFSET_MIN;
				if (outT2 > TEMP_OFFSET_MAX) outT2 = TEMP_OFFSET_MAX;
				if (outT2 < TEMP_OFFSET_MIN) outT2 = TEMP_OFFSET_MIN;
				// HUMIDITY CONTROL (Adaptive PID, symmetric)
				float avgRH = (rh1 + rh2) * 0.5f;
				float errRH1 = avgRH - rh1;
				float errRH2 = avgRH - rh2;
				// Deadband
				if (fabsf(errRH1) < RH_DEADBAND) errRH1 = 0.0f;
				if (fabsf(errRH2) < RH_DEADBAND) errRH2 = 0.0f;
				float outRH1 = PID_Update_Adaptive(&pidRH, errRH1, DT_SEC);
				float outRH2 = PID_Update_Adaptive(&pidRH, errRH2, DT_SEC);
				// Rate limiting
				float deltaRH1 = outRH1 - prevOutRH1;
				if (deltaRH1 > MAX_RH_STEP) outRH1 = prevOutRH1 + MAX_RH_STEP;
				if (deltaRH1 < -MAX_RH_STEP) outRH1 = prevOutRH1 - MAX_RH_STEP;
				float deltaRH2 = outRH2 - prevOutRH2;
				if (deltaRH2 > MAX_RH_STEP) outRH2 = prevOutRH2 + MAX_RH_STEP;
				if (deltaRH2 < -MAX_RH_STEP) outRH2 = prevOutRH2 - MAX_RH_STEP;
				prevOutRH1 = outRH1;
				prevOutRH2 = outRH2;
				// Clamp offsets
				if (outRH1 > RH_OFFSET_MAX) outRH1 = RH_OFFSET_MAX;
				if (outRH1 < RH_OFFSET_MIN) outRH1 = RH_OFFSET_MIN;
				if (outRH2 > RH_OFFSET_MAX) outRH2 = RH_OFFSET_MAX;
				if (outRH2 < RH_OFFSET_MIN) outRH2 = RH_OFFSET_MIN;
				HDC302x_SetOffset(&Sensor1, outT1, outRH1);
				HDC302x_SetOffset(&Sensor2, outT2, outRH2);
				// ENVIRONMENT OUTPUT - Apply corrections
				temp1 = temp1 + outT1;
				temp2 = temp2 + outT2;
				rh1 = rh1 + outRH1;
				rh2 = rh2 + outRH2;
				txFrameBuf.Payload.externalData.Temperature = (temp1 + temp2 + BMP581_Data.temperature) / 3.0f;
				txFrameBuf.Payload.externalData.RH = (rh1 + rh2) * 0.5f;
				txFrameBuf.Payload.externalData.Pressure = BMP581_Data.pressure;
				ENS160_SetEnvCompensation(txFrameBuf.Payload.externalData.Temperature, txFrameBuf.Payload.externalData.RH);
				Wind_ComputeSoundSpeed();
				sensorReadyFlags |= SENSOR_ENV;
			}
			
			if (AS3935_Ready) {
					AS3935_Ready = 0;
					AS3935_LightningData_t lightningTmp;
					AS3935_ReadLightningData(&lightningTmp);
					txFrameBuf.Payload.Lightning = lightningTmp;
					sensorReadyFlags |= SENSOR_AS3935;
			}
			
			// Once every 794ms
			if (AS7331_Ready) {
				AS7331_Ready = 0;
				AS7331_ReadUVData(&txFrameBuf.Payload.UV);
				sensorReadyFlags |= SENSOR_AS7331;
			}
			
			// Once every 1054ms
			if (ENS160_Ready) {
				ENS160_Ready = 0;
				//ENS160_ReadDeviceStatus();
				ENS160_ReadAllData(&txFrameBuf.Payload.AirQuality, &ENS160_Resistance);
				sensorReadyFlags |= SENSOR_ENS160;
			}
			
			// Once every 1003ms
			if (TCS34003_Ready) {
				TCS34003_Ready = 0;
        if (TCS34003_GetLightData(&txFrameBuf.Payload.LightRGB) != HAL_OK) {
					DEBUG("TCS34003 read failed!\n");
        }
				sensorReadyFlags |= SENSOR_TCS34003;
			}
			
			// Once every 584ms
			if (TSL25911_Ready) {
				TSL25911_Ready = 0;
				HAL_Delay(1);
        if (TSL25911_ReadLightData(&txFrameBuf.Payload.Light) != HAL_OK) {
					DEBUG("TSL25911 read failed!\n");
        }
				sensorReadyFlags |= SENSOR_TSL25911;
				DEBUG("Period: %d ms \n", HAL_GetTick() - time);
				time = HAL_GetTick();
			}
			
//			if (Wind_Measure(&txFrameBuf.Payload.Wind) == HAL_OK) {
//					DEBUG("Wind: %.2f m/s @ %.1f deg\n", txFrameBuf.Payload.Wind.Speed, txFrameBuf.Payload.Wind.Direction);
//					sensorReadyFlags |= SENSOR_WIND;
//			}
			if ((sensorReadyFlags & SENSOR_FRAME_MASK) == SENSOR_FRAME_MASK) {
					SendDataToESP();
					sensorReadyFlags = 0; // keep lightning if present
			}
			
			// ToDo Implement the data received from ESP
			// extern Wind_Input_t CalibData; from Wind library
			// The Reset functionality
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
