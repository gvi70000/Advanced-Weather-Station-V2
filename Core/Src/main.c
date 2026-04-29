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
#include <math.h>
/* I2C3 sensors */
#include "BMP581.h"
#include "HDC302x.h"
#include "TCS34003.h"
#include "ENS160.h"
/* I2C1 sensors */
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

/* =========================================================================
 * Sensor ready bitmasks
 * ======================================================================= */
#define SENSOR_ENV        (1u << 0)  /* BMP581 + HDC302x environmental data  */
#define SENSOR_AS7331     (1u << 1)  /* UV sensor                             */
#define SENSOR_ENS160     (1u << 2)  /* Air quality                           */
#define SENSOR_TCS34003   (1u << 3)  /* RGB colour                            */
#define SENSOR_TSL25911   (1u << 4)  /* Ambient light                         */
#define SENSOR_WIND       (1u << 5)  /* Wind (disabled — commented out below) */
#define SENSOR_AS3935     (1u << 6)  /* Lightning (preserved across frames)   */

/* Bits that must ALL be set before a sensor data frame is sent to the ESP.
 * SENSOR_WIND (bit 5) is excluded while wind measurement is disabled.
 * SENSOR_AS3935 (bit 6) is excluded — lightning is asynchronous and is
 * preserved in the payload until a new event overwrites it.               */
#define SENSOR_FRAME_MASK  0x1Fu     /* bits 0–4: ENV|AS7331|ENS160|TCS|TSL  */

/* =========================================================================
 * STM32 -> ESP  UART frame protocol
 *
 * Frame A — Sensor data (68 bytes):
 *   [0x23][0x40][Payload(64)][CRC16(2)]
 *   SOF = { 0x23, 0x40 }  (Start + Length byte, 0x40 = 64)
 *   CRC covers Payload only (64 bytes).
 *
 * Frame B — Wind calibration readback (17 bytes):
 *   [0x23][0x24][0x0C][D01(4)][D12(4)][D20(4)][CRC16(2)]
 *   SOF = { 0x23, 0x24 }  (Start + FrameType CAL)
 *   CRC covers Length byte + 3 x uint32 = 13 bytes.
 * ======================================================================= */
#define UART_FRAME_START   0x23u   /* '#'  first SOF byte for both TX frames  */
#define UART_FRAME_CAL     0x24u   /* '$'  second SOF byte for calibration TX */

/* =========================================================================
 * ESP -> STM  UART frame protocol
 *
 * Single frame type — 20 bytes:
 *   [0x23][FrameType(1)][Wind_Input_t(16)][CRC16(2)]
 *   CRC covers FrameType + Wind_Input_t = 17 bytes.
 *
 * FrameType values:
 *   0x24  UART_FRAME_CAL      Boot-time: ESP sends Height + D01/D12/D20 from NVS
 *   0x25  UART_FRAME_TMP      Height field carries float temperature reference (°C)
 *   0x26  UART_FRAME_RH       Height field carries float RH reference (%)
 *   0x27  UART_FRAME_RST      System reset request (Wind_Input_t payload ignored)
 *   0x28  UART_FRAME_CAL_REQ  Run Wind_CalibrateReflector() request
 * ======================================================================= */
#define UART_FRAME_TMP      0x25u   /* '%' temperature reference              */
#define UART_FRAME_RH       0x26u   /* '&' RH reference                       */
#define UART_FRAME_RST      0x27u   /* ''' reset                              */
#define UART_FRAME_CAL_REQ  0x28u   /* '(' calibration run request            */

/* ESP->STM frame size = Start(1) + FrameType(1) + Wind_Input_t(16) + CRC(2) */
#define UART_ESP_RX_SIZE   20u

/* =========================================================================
 * AS3935 interrupt type encoding in DistanceEstimation byte
 *
 * The AS3935_LightningData_t.DistanceEstimation byte carries two fields:
 *   Bits [5:0]  = distance code  (AS3935_Distance_t, 0x3F = out of range)
 *   Bits [7:4]  = interrupt type (AS3935_INT_t: 0x01=NH, 0x04=D, 0x08=L)
 *
 * Bits [7:6] of the raw register are reserved/always-zero per datasheet,
 * so packing the INT type there is non-destructive and backward-compatible
 * with any ESP code that masks off the upper nibble before reading distance.
 *
 * Helper macros:
 *   AS3935_ENCODE_DIST(irq, dist)  — pack for the wire
 *   AS3935_DECODE_INT(byte)        — extract INT type
 *   AS3935_DECODE_DIST(byte)       — extract distance code
 * ======================================================================= */
#define AS3935_ENCODE_DIST(irq, dist)  ((uint8_t)(((uint8_t)(irq) << 4) | ((dist) & 0x3Fu)))
#define AS3935_DECODE_INT(byte)        (((byte) >> 4) & 0x0Fu)
#define AS3935_DECODE_DIST(byte)       ((byte) & 0x3Fu)

/* =========================================================================
 * PID / sensor filtering constants
 * ======================================================================= */
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
/* Data-ready flags set by HAL_GPIO_EXTI_Callback (gpio.c) */
extern volatile uint8_t AS3935_Ready;
extern volatile uint8_t AS7331_Ready;
extern volatile uint8_t ENS160_Ready;
extern volatile uint8_t BMP_Ready;
extern volatile uint8_t TCS34003_Ready;
extern volatile uint8_t TSL25911_Ready;
extern volatile uint8_t ESP_Transmitting;

volatile uint8_t sensorReadyFlags = 0;

/* Shared environmental data — written by main loop, read by Wind.c */
extern Wind_EnvData_t externalData;

/* Sensor data structs */
BMP581_sensor_data_t BMP581_Data;
HDC302x_t            Sensor1;
HDC302x_t            Sensor2;
ENS160_Resistance_t  ENS160_Resistance;

/* Wind calibration data: Height, PathLenD01, PathLenD12, PathLenD20 */
extern Wind_Input_t calibData;

/* =========================================================================
 * UART_Payload_t — 64 bytes, must match ESP UART_Payload_t exactly.
 *
 * TCS34003: full TCS34003_LightData_t (22 B) is sent — RGBC(8) + IR(2)
 * + CCT(4) + Lux(4) + Irradiance(4).  All fields computed on STM and
 * published by ESP to individual MQTT topics.
 *
 * AS3935 note: LightningData_t.DistanceEstimation encodes both the INT type
 * (bits [7:4]) and the distance code (bits [5:0]).  See AS3935_ENCODE_DIST.
 * ======================================================================= */
typedef struct __attribute__((packed)) {
    Wind_EnvData_t       Env;          /* 20 B: Temp, RH, Pressure, DewPt, SndSpd */
    Wind_t               Wind;         /*  8 B: Speed (f32), Direction (f32)        */
    AS3935_LightningData_t Lightning;  /*  5 B: Energy (u32), DistanceEstimation(u8)*/
    AS7331_DataOut_t     UV;           /*  8 B: TEMP_C100, UVA, UVB, UVC (u16 x4)  */
    ENS160_Data_t        AirQuality;   /*  5 B: AQI (u8), TVOC (u16), eCO2 (u16)   */
    TCS34003_LightData_t LightRGB;     /* 22 B: RGBC(8)+IR(2)+CCT(4)+Lux(4)+Irr(4) */
    TSL25911_LightData_t Light;        /* 10 B: FS, IR, Vis (u16 x3), Lux (f32)    */
} UART_Payload_t;                      /* = 64 bytes total                          */

_Static_assert(sizeof(UART_Payload_t) == 78u,
    "UART_Payload_t size mismatch with ESP — check struct members");

/* =========================================================================
 * STM -> ESP sensor data frame.
 * Wire:  [0x23][0x4E][Payload(78)][CRC16(2)]  = 82 bytes.
 * ======================================================================= */
typedef struct __attribute__((packed)) {
    uint8_t        Start;    /* 0x23                              */
    uint8_t        Length;   /* sizeof(UART_Payload_t) = 64=0x40 */
    UART_Payload_t Payload;
    uint16_t       CR;       /* CRC16 over Payload only           */
} UART_TX_Frame_t;

_Static_assert(sizeof(UART_TX_Frame_t) == 82u,
    "UART_TX_Frame_t size mismatch with ESP");

/* =========================================================================
 * STM -> ESP calibration frame (after Wind_CalibrateReflector).
 * Wire:  [0x23][0x24][0x0C][D01(4)][D12(4)][D20(4)][CRC16(2)]  = 17 bytes.
 * CRC covers: Length(1) + D01(4) + D12(4) + D20(4) = 13 bytes.
 * ======================================================================= */
typedef struct __attribute__((packed)) {
    uint8_t  Start;       /* UART_FRAME_START 0x23          */
    uint8_t  FrameType;   /* UART_FRAME_CAL   0x24          */
    uint8_t  Length;      /* 3 * sizeof(uint32_t) = 12      */
    uint32_t PathLenD01;
    uint32_t PathLenD12;
    uint32_t PathLenD20;
    uint16_t CR;
} UART_TX_Cal_Frame_t;

_Static_assert(sizeof(UART_TX_Cal_Frame_t) == 17u,
    "UART_TX_Cal_Frame_t size mismatch");

/* =========================================================================
 * ESP -> STM command frame (all types share this layout).
 * Wire:  [0x23][FrameType(1)][Wind_Input_t(16)][CRC16(2)]  = 20 bytes.
 * CRC covers: FrameType(1) + Wind_Input_t(16) = 17 bytes.
 * ======================================================================= */
typedef struct __attribute__((packed)) {
    uint8_t      Start;      /* UART_FRAME_START 0x23                      */
    uint8_t      FrameType;  /* 0x24..0x28 — selects action                */
    Wind_Input_t Payload;    /* Wind_Input_t: Height(4), D01(4),D12(4),D20(4) */
    uint16_t     CR;         /* CRC16 over FrameType + Wind_Input_t        */
} UART_RX_Frame_t;

_Static_assert(sizeof(UART_RX_Frame_t) == 20u,
    "UART_RX_Frame_t size mismatch");

/* DMA buffers */
static UART_TX_Frame_t  txFrameBuf;
static uint8_t          espRxBuf[UART_ESP_RX_SIZE];

/* Runtime state */
static float            temperatureCalibrated = 0.0f;
static float            humidityCalibrated    = 0.0f;
static uint8_t          isReset               = 0;
static volatile uint8_t isCalibrate           = 0;
volatile uint8_t        dmaTxBusy             = 0;  /* extern in usart.c */
uint32_t                time                  = 0;

typedef struct {
    float kp, ki, kd;
    float integral;
    float prevError;
    float outMin, outMax;
    float errAvg;
    float errVar;
} PID_t;

PID_t pidTemp1 = { .kp = 0.3f,  .ki = 0.05f, .kd = 0.1f,  .outMin = -5.0f,  .outMax = 5.0f  };
PID_t pidTemp2 = { .kp = 0.3f,  .ki = 0.05f, .kd = 0.1f,  .outMin = -5.0f,  .outMax = 5.0f  };
PID_t pidRH    = { .kp = 0.2f,  .ki = 0.02f, .kd = 0.05f, .outMin = -10.0f, .outMax = 10.0f };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* -------------------------------------------------------------------------
 * CRC16 (Modbus) — matches the ESP implementation exactly.
 * ---------------------------------------------------------------------- */
static uint16_t CRC16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8u; j++) {
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xA001u) : (crc >> 1);
        }
    }
    return crc;
}

/* -------------------------------------------------------------------------
 * SendDataToESP — non-blocking DMA TX.
 *
 * Wire frame: [0x23][0x4E][Payload(78)][CRC16(2)] = 82 bytes.
 * The ESP processSTMbyte() syncs on the two-byte sequence {0x23, 0x4E}.
 * STM_START_TX asserts SET_ESP_MSG so the ESP backs off (FIX-3 on ESP side).
 * dmaTxBusy is cleared in HAL_UART_TxCpltCallback (usart.c) on DMA complete.
 * ---------------------------------------------------------------------- */
void SendDataToESP(void) {
    if (dmaTxBusy) return;
    txFrameBuf.CR = CRC16((const uint8_t *)&txFrameBuf.Payload,
                          sizeof(UART_Payload_t));
    dmaTxBusy = 1;
    STM_START_TX;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&txFrameBuf,
                          sizeof(UART_TX_Frame_t));
}

/* -------------------------------------------------------------------------
 * SendCalibrationToESP — DMA TX of calibrated path lengths.
 *
 * Called after a successful Wind_CalibrateReflector().
 * Wire frame: [0x23][0x24][0x0C][D01][D12][D20][CRC16] = 17 bytes.
 * The ESP processSTMbyte() syncs on {0x23, 0x24} (CAL_SOF_B0/B1).
 * CRC covers: Length(1) + D01(4) + D12(4) + D20(4) = 13 bytes.
 * ---------------------------------------------------------------------- */
void SendCalibrationToESP(void) {
    if (dmaTxBusy) return;
    static UART_TX_Cal_Frame_t calFrame;   /* static: DMA needs it alive after return */
    calFrame.Start      = UART_FRAME_START;
    calFrame.FrameType  = UART_FRAME_CAL;
    calFrame.Length     = (uint8_t)(3u * sizeof(uint32_t));  /* 12 = 0x0C */
    calFrame.PathLenD01 = calibData.PathLenD01;
    calFrame.PathLenD12 = calibData.PathLenD12;
    calFrame.PathLenD20 = calibData.PathLenD20;
    calFrame.CR = CRC16((const uint8_t *)&calFrame.Length,
                        1u + 3u * sizeof(uint32_t));
    dmaTxBusy = 1;
    STM_START_TX;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&calFrame,
                          sizeof(UART_TX_Cal_Frame_t));
    DEBUG("Cal sent: D01=%lu D12=%lu D20=%lu um\n",
          (unsigned long)calFrame.PathLenD01,
          (unsigned long)calFrame.PathLenD12,
          (unsigned long)calFrame.PathLenD20);
}

/* HAL_UART_TxCpltCallback is defined in usart.c — handles both
 * USART3 (PGA460, sets ust_tx_done) and USART1 (ESP, clears
 * dmaTxBusy and deasserts STM_STOP_TX).                       */

/* -------------------------------------------------------------------------
 * HAL_UART_RxCpltCallback — process one 20-byte ESP->STM frame.
 *
 * All command types share the same 20-byte UART_RX_Frame_t layout:
 *   [0x23][FrameType][Wind_Input_t(16)][CRC16(2)]
 * CRC is verified over FrameType + Wind_Input_t (17 bytes).
 *
 * Wind_Input_t field usage by FrameType:
 *   0x24 (CAL)      : full struct  — Height (float bits), D01, D12, D20
 *   0x25 (TMP)      : Height field carries IEEE-754 float temperature (°C)
 *   0x26 (RH)       : Height field carries IEEE-754 float RH (%)
 *   0x27 (RST)      : payload ignored — trigger NVIC reset
 *   0x28 (CAL_REQ)  : payload ignored — trigger Wind_CalibrateReflector()
 * ---------------------------------------------------------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance != USART1) return;

    const UART_RX_Frame_t *f = (const UART_RX_Frame_t *)espRxBuf;

    /* Validate start byte.
     * DMA is fixed 20-byte circular — TC fires every 20 bytes regardless of
     * content.  If Start is wrong the DMA counter is misaligned (e.g. due to
     * a partial send or UART glitch).  Abort and restart the DMA to reset the
     * 20-byte window; the next complete ESP frame will re-align correctly.
     * The ESP always sends exactly CMD_FRAME_SIZE = 20 bytes per frame and
     * calls Serial1.flush(), so a single restart is sufficient to recover.   */
    if (f->Start != UART_FRAME_START) {
        HAL_UART_AbortReceive(&huart1);
        HAL_UART_Receive_DMA(&huart1, espRxBuf, UART_ESP_RX_SIZE);
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
        DEBUG("ESP RX sync lost — DMA restarted");
        return;
    }

    /* Verify CRC over FrameType (1) + Wind_Input_t (16) = 17 bytes */
    uint16_t calcCRC = CRC16((const uint8_t *)&f->FrameType,
                             1u + sizeof(Wind_Input_t));
    if (calcCRC != f->CR) {
        DEBUG("ESP RX CRC error (type=0x%02X)\n", f->FrameType);
        return;
    }

    switch (f->FrameType) {

        case UART_FRAME_CAL:
            /* Boot-time: ESP sends Height + acoustic path lengths from NVS.
             * Copy the whole Wind_Input_t directly into calibData.          */
            calibData = f->Payload;
            DEBUG("Cal RX: H=%lu D01=%lu D12=%lu D20=%lu\n",
                  (unsigned long)f->Payload.Height,
                  (unsigned long)f->Payload.PathLenD01,
                  (unsigned long)f->Payload.PathLenD12,
                  (unsigned long)f->Payload.PathLenD20);
            break;

        case UART_FRAME_TMP: {
            /* Height field carries the float temperature reference (°C).   */
            float tmp;
            memcpy(&tmp, &f->Payload.Height, sizeof(float));
            temperatureCalibrated = tmp;
            DEBUG("Temp ref: %.2f C\n", tmp);
            break;
        }

        case UART_FRAME_RH: {
            /* Height field carries the float RH reference (%).             */
            float tmp;
            memcpy(&tmp, &f->Payload.Height, sizeof(float));
            humidityCalibrated = tmp;
            DEBUG("RH ref: %.2f %%\n", tmp);
            break;
        }

        case UART_FRAME_RST:
            isReset = 1;
            break;

        case UART_FRAME_CAL_REQ:
            isCalibrate = 1;
            break;

        default:
            DEBUG("ESP RX unknown FrameType 0x%02X\n", f->FrameType);
            break;
    }
}

/* -------------------------------------------------------------------------
 * Adaptive PID controller
 * ---------------------------------------------------------------------- */
static void PID_Adapt(PID_t *pid, float error) {
    const float alpha = 0.05f;
    pid->errAvg = (1.0f - alpha) * pid->errAvg + alpha * error;
    float diff  = error - pid->errAvg;
    pid->errVar = (1.0f - alpha) * pid->errVar + alpha * (diff * diff);
    float absErr = fabsf(error);
    if (absErr > 1.0f  && pid->errVar < 0.1f) pid->kp += 0.005f;
    if (fabsf(pid->errAvg) > 0.5f)            pid->ki += 0.001f;
    if (pid->errVar > 0.5f) { pid->kd += 0.005f; pid->kp *= 0.995f; }
    if (pid->kp > 1.0f)  pid->kp = 1.0f;   if (pid->kp < 0.05f) pid->kp = 0.05f;
    if (pid->ki > 0.2f)  pid->ki = 0.2f;   if (pid->ki < 0.0f)  pid->ki = 0.0f;
    if (pid->kd > 0.5f)  pid->kd = 0.5f;   if (pid->kd < 0.0f)  pid->kd = 0.0f;
}

static float PID_Update_Adaptive(PID_t *pid, float error, float dt) {
    PID_Adapt(pid, error);
    float P = pid->kp * error;
    pid->integral += error * dt;
    if (pid->integral > pid->outMax) pid->integral = pid->outMax;
    if (pid->integral < pid->outMin) pid->integral = pid->outMin;
    float I   = pid->ki * pid->integral;
    float D   = pid->kd * (error - pid->prevError) / dt;
    pid->prevError = error;
    float out = P + I + D;
    if (out > pid->outMax) out = pid->outMax;
    if (out < pid->outMin) out = pid->outMin;
    return out;
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
#ifdef AS3935_MODE_IRQ
    Enable_EXTI_AS3935();
#endif /* AS3935_MODE_IRQ */

    /* Initialise TX frame fixed header fields */
    txFrameBuf.Start  = UART_FRAME_START;
    txFrameBuf.Length = (uint8_t)sizeof(UART_Payload_t);   /* 78 = 0x4E */

    /* Start DMA RX from ESP in circular mode — TC interrupt only */
    HAL_UART_Receive_DMA(&huart1, espRxBuf, UART_ESP_RX_SIZE);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

    /* HDC302x temperature / humidity sensors */
    Sensor1.Address = HDC302X_SENSOR_1_ADDR;
    if (HDC302x_Init(&Sensor1) != HAL_OK) {
        DEBUG("Sensor 1 init failed!\n");
    } else {
        DEBUG("Sensor 1 init OK.\n");
    }
    Sensor2.Address = HDC302X_SENSOR_2_ADDR;
    if (HDC302x_Init(&Sensor2) != HAL_OK) {
        DEBUG("Sensor 2 init failed!\n");
    } else {
        DEBUG("Sensor 2 init OK.\n");
    }
    HDC302x_StartAutoMeasurement(&Sensor1, HDC302X_CMD_AUTO_MEASUREMENT_1_PER_SECOND_LPM0);
    HDC302x_StartAutoMeasurement(&Sensor2, HDC302X_CMD_AUTO_MEASUREMENT_1_PER_SECOND_LPM0);

    /* BMP581 pressure / temperature (interrupt-driven) */
    if (BMP581_Init() == HAL_OK) {
        DEBUG("BMP581 init OK!\n");
    } else {
        DEBUG("BMP581 init FAIL!\n");
    }

    /* ENS160 air quality */
    if (ENS160_Init() != HAL_OK) {
        DEBUG("ENS160 init failed!\n");
    }

    /* AS3935 lightning sensor */
    if (AS3935_Init() != HAL_OK) {
        DEBUG("AS3935 init failed!\n");
    }
#ifdef AS3935_MODE_CALIB
    else {
        /* Antenna tuning: TIM2 CH4 is configured for input capture in
         * CALIB mode.  Start the timer base then run the sweep.        */
        HAL_TIM_Base_Start(&htim2);
        DEBUG("_______________________\n");
        AS3935_TuneCap_t bestCap = AS3935_TuneAntenna();
        AS3935_SetTuningCapacitor(bestCap);
        HAL_TIM_Base_Stop(&htim2);
        DEBUG("AS3935 tuning done, cap=%d\n", (int)bestCap);
    }
#endif /* AS3935_MODE_CALIB */

    /* UV, colour and ambient light sensors */
    if (AS7331_Init()   != HAL_OK) { DEBUG("AS7331 init failed!\n");   }
    if (TCS34003_Init() != HAL_OK) { DEBUG("TCS34003 init failed!\n"); }
    if (TSL25911_Init() != HAL_OK) { DEBUG("TSL25911 init failed!\n"); }

    /* Wind measurement — uncomment when PGA460 hardware is fitted */
    /* PGA460_Init(0); */
    /* Wind_Init();    */

    time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
        /* --- System reset requested by ESP --- */
        if (isReset) {
            HAL_Delay(10);
            __NVIC_SystemReset();
        }

        /* --- Wind calibration requested by ESP (MQTT) --- */
        if (isCalibrate) {
            isCalibrate = 0;
            DEBUG("Calibration requested by ESP\n");
            if (Wind_CalibrateReflector() == HAL_OK) {
                SendCalibrationToESP();
            } else {
                DEBUG("Calibration FAILED\n");
            }
        }

        /* --- BMP581 + HDC302x (~1 Hz, BMP interrupt-driven) --- */
        if (BMP_Ready) {
            BMP_Ready = 0;
            float temp1 = 0.0f, temp2 = 0.0f, rh1 = 0.0f, rh2 = 0.0f;
            static float prevOutT1  = 0.0f, prevOutT2  = 0.0f;
            static float prevOutRH1 = 0.0f, prevOutRH2 = 0.0f;

            BMP581_Get_TempPressData(&BMP581_Data);
            if (HDC302x_ReadData(&Sensor1) == HAL_OK) { temp1 = Sensor1.Data.Temperature; rh1 = Sensor1.Data.Humidity; }
            if (HDC302x_ReadData(&Sensor2) == HAL_OK) { temp2 = Sensor2.Data.Temperature; rh2 = Sensor2.Data.Humidity; }

            /* Temperature PID */
            float tempRef = (temperatureCalibrated != 0.0f) ? temperatureCalibrated : BMP581_Data.temperature;
            float errT1 = tempRef - temp1; if (fabsf(errT1) < TEMP_DEADBAND) errT1 = 0.0f;
            float errT2 = tempRef - temp2; if (fabsf(errT2) < TEMP_DEADBAND) errT2 = 0.0f;
            float outT1 = PID_Update_Adaptive(&pidTemp1, errT1, DT_SEC);
            float outT2 = PID_Update_Adaptive(&pidTemp2, errT2, DT_SEC);
            float dT1 = outT1 - prevOutT1;
            if (dT1 >  MAX_TEMP_STEP) outT1 = prevOutT1 + MAX_TEMP_STEP;
            if (dT1 < -MAX_TEMP_STEP) outT1 = prevOutT1 - MAX_TEMP_STEP;
            float dT2 = outT2 - prevOutT2;
            if (dT2 >  MAX_TEMP_STEP) outT2 = prevOutT2 + MAX_TEMP_STEP;
            if (dT2 < -MAX_TEMP_STEP) outT2 = prevOutT2 - MAX_TEMP_STEP;
            prevOutT1 = outT1; prevOutT2 = outT2;
            if (outT1 > TEMP_OFFSET_MAX) outT1 = TEMP_OFFSET_MAX;
            if (outT1 < TEMP_OFFSET_MIN) outT1 = TEMP_OFFSET_MIN;
            if (outT2 > TEMP_OFFSET_MAX) outT2 = TEMP_OFFSET_MAX;
            if (outT2 < TEMP_OFFSET_MIN) outT2 = TEMP_OFFSET_MIN;

            /* Humidity PID */
            float rhRef = (humidityCalibrated != 0.0f) ? humidityCalibrated : (rh1 + rh2) * 0.5f;
            float errRH1 = rhRef - rh1; if (fabsf(errRH1) < RH_DEADBAND) errRH1 = 0.0f;
            float errRH2 = rhRef - rh2; if (fabsf(errRH2) < RH_DEADBAND) errRH2 = 0.0f;
            float outRH1 = PID_Update_Adaptive(&pidRH, errRH1, DT_SEC);
            float outRH2 = PID_Update_Adaptive(&pidRH, errRH2, DT_SEC);
            float dRH1 = outRH1 - prevOutRH1;
            if (dRH1 >  MAX_RH_STEP) outRH1 = prevOutRH1 + MAX_RH_STEP;
            if (dRH1 < -MAX_RH_STEP) outRH1 = prevOutRH1 - MAX_RH_STEP;
            float dRH2 = outRH2 - prevOutRH2;
            if (dRH2 >  MAX_RH_STEP) outRH2 = prevOutRH2 + MAX_RH_STEP;
            if (dRH2 < -MAX_RH_STEP) outRH2 = prevOutRH2 - MAX_RH_STEP;
            prevOutRH1 = outRH1; prevOutRH2 = outRH2;
            if (outRH1 > RH_OFFSET_MAX) outRH1 = RH_OFFSET_MAX;
            if (outRH1 < RH_OFFSET_MIN) outRH1 = RH_OFFSET_MIN;
            if (outRH2 > RH_OFFSET_MAX) outRH2 = RH_OFFSET_MAX;
            if (outRH2 < RH_OFFSET_MIN) outRH2 = RH_OFFSET_MIN;

            HDC302x_SetOffset(&Sensor1, outT1, outRH1);
            HDC302x_SetOffset(&Sensor2, outT2, outRH2);

            txFrameBuf.Payload.Env.Temperature = (temp1 + outT1 + temp2 + outT2) * 0.5f;
            txFrameBuf.Payload.Env.RH          = (rh1 + outRH1 + rh2 + outRH2) * 0.5f;
            txFrameBuf.Payload.Env.Pressure    = BMP581_Data.pressure;

            ENS160_SetEnvCompensation(txFrameBuf.Payload.Env.Temperature,
                                      txFrameBuf.Payload.Env.RH);

            externalData.Temperature = txFrameBuf.Payload.Env.Temperature;
            externalData.RH          = txFrameBuf.Payload.Env.RH;
            externalData.Pressure    = txFrameBuf.Payload.Env.Pressure;
            Wind_ComputeSoundSpeed();
            txFrameBuf.Payload.Env.DewPoint   = externalData.DewPoint;
            txFrameBuf.Payload.Env.SoundSpeed = externalData.SoundSpeed;

            sensorReadyFlags |= SENSOR_ENV;
        }

        /* --- AS3935 lightning sensor (EXTI interrupt-driven) ---
         *
         * AS3935_GetInterruptType() reads the INT register and handles the
         * mandatory >= 2 ms post-IRQ delay internally.
         *
         * All three interrupt types are forwarded to the ESP so it can publish
         * each event to its own MQTT topic.  The interrupt type is packed into
         * bits [7:4] of DistanceEstimation (bits [5:0] carry the distance code;
         * bits [7:6] are reserved-zero in the raw register).
         *
         * INT_L  (0x08) — real lightning strike: read energy + distance.
         * INT_D  (0x04) — disturber detected: no valid distance, send 0x3F.
         * INT_NH (0x01) — noise floor too high: no valid distance, send 0x3F.
         *
         * SENSOR_AS3935 is set on every interrupt so the ESP receives an
         * up-to-date event in the next regular sensor frame.
         * The flag is preserved (not cleared) after each frame send so the
         * last lightning reading persists in the payload until overwritten.  */
        if (AS3935_Ready) {
            AS3935_Ready = 0;
            AS3935_INT_t irqType = AS3935_GetInterruptType();

            if (irqType == AS3935_INT_L) {
                AS3935_LightningData_t tmp;
                if (AS3935_ReadLightningData(&tmp) == HAL_OK) {
                    txFrameBuf.Payload.Lightning.LightningEnergy    = tmp.LightningEnergy;
                    txFrameBuf.Payload.Lightning.DistanceEstimation =
                        AS3935_ENCODE_DIST(AS3935_INT_L, tmp.DistanceEstimation);
                    DEBUG("AS3935: LIGHTNING  E=%lu  D=0x%02X\n",
                          (unsigned long)tmp.LightningEnergy,
                          tmp.DistanceEstimation);
                }
            } else if (irqType == AS3935_INT_D) {
                txFrameBuf.Payload.Lightning.LightningEnergy    = 0u;
                txFrameBuf.Payload.Lightning.DistanceEstimation =
                    AS3935_ENCODE_DIST(AS3935_INT_D, AS3935_DIST_OUT_OF_RANGE);
                DEBUG("AS3935: DISTURBER\n");
            } else if (irqType == AS3935_INT_NH) {
                txFrameBuf.Payload.Lightning.LightningEnergy    = 0u;
                txFrameBuf.Payload.Lightning.DistanceEstimation =
                    AS3935_ENCODE_DIST(AS3935_INT_NH, AS3935_DIST_OUT_OF_RANGE);
                DEBUG("AS3935: NOISE HIGH\n");
            }
            /* Set flag for all valid interrupt types */
            if (irqType != AS3935_INT_NONE) {
                sensorReadyFlags |= SENSOR_AS3935;
            }
        }

        /* --- AS7331 UV sensor (~794 ms, interrupt-driven) --- */
        if (AS7331_Ready) {
            AS7331_Ready = 0;
            AS7331_ReadUVData(&txFrameBuf.Payload.UV);
            sensorReadyFlags |= SENSOR_AS7331;
        }

        /* --- ENS160 air quality (~1054 ms, interrupt-driven) --- */
        if (ENS160_Ready) {
            ENS160_Ready = 0;
            ENS160_ReadAllData(&txFrameBuf.Payload.AirQuality, &ENS160_Resistance);
            sensorReadyFlags |= SENSOR_ENS160;
        }

        /* --- TCS34003 colour sensor (~1003 ms, interrupt-driven) --- */
        if (TCS34003_Ready) {
            TCS34003_Ready = 0;
            if (TCS34003_GetLightData(&txFrameBuf.Payload.LightRGB) == HAL_OK) {
                /* Full TCS34003_LightData_t: RGBC + IR + CCT + Lux + Irradiance */
            } else {
                DEBUG("TCS34003 read failed!\n");
            }
            sensorReadyFlags |= SENSOR_TCS34003;
        }

        /* --- TSL25911 ambient light (~584 ms, interrupt-driven) --- */
        if (TSL25911_Ready) {
            TSL25911_Ready = 0;
            HAL_Delay(1);
            if (TSL25911_ReadLightData(&txFrameBuf.Payload.Light) != HAL_OK) {
                DEBUG("TSL25911 read failed!\n");
            }
            sensorReadyFlags |= SENSOR_TSL25911;
            DEBUG("Period: %lu ms\n", (unsigned long)(HAL_GetTick() - time));
            time = HAL_GetTick();
        }

        /* --- Wind measurement (disabled until PGA460 hardware is fitted) ---
        if (Wind_Measure(&txFrameBuf.Payload.Wind) == HAL_OK) {
            DEBUG("Wind: %.2f m/s @ %.1f deg\n",
                  txFrameBuf.Payload.Wind.Speed,
                  txFrameBuf.Payload.Wind.Direction);
            sensorReadyFlags |= SENSOR_WIND;
        }
        */

        /* --- Transmit sensor frame when all mandatory sensors are ready ---
         *
         * SENSOR_FRAME_MASK (0x1F) covers bits 0-4 only.
         * SENSOR_AS3935 (bit 6) is excluded: lightning is asynchronous and
         * must not block the regular ~1 Hz data cycle.
         *
         * After sending, SENSOR_AS3935 is preserved so the last lightning
         * event stays in the payload for the next frame.  All other bits
         * are cleared.                                                     */
        if ((sensorReadyFlags & SENSOR_FRAME_MASK) == SENSOR_FRAME_MASK) {
            SendDataToESP();
            sensorReadyFlags &= SENSOR_AS3935;
        }

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

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM       = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN       = 85;
  RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ       = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR       = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) { Error_Handler(); }
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
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */