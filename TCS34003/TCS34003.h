/***************************************************************************
 * @file [TCS34003].h/.c
 * This file contains definitions, data structures, and functions
 * for interfacing with the TCS34007 Color Light-to-Digital Converter.
 *
 * Copyright (c) [2024] Grozea Ion gvi70000
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 ***************************************************************************/

#ifndef TCS34003_H
#define TCS34003_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* ============================================================================
 * I2C Address
 * ========================================================================= */

/** @brief 8-bit I²C address (7-bit 0x29 shifted left, confirmed by I²C scan). */
#define TCS34003_I2C_ADDR               (0x29U << 1U)

/* ============================================================================
 * Register Addresses (Table 19, p.13)
 * ========================================================================= */

#define TCS34003_REG_ENABLE             0x80U  ///< Enables states and interrupts         (R/W, reset 0x00)
#define TCS34003_REG_ATIME              0x81U  ///< RGBC integration time                 (R/W, reset 0xFF)
#define TCS34003_REG_WTIME              0x83U  ///< Wait time                             (R/W, reset 0xFF)

/* Clear Channel Interrupt Threshold Registers (0x84–0x87) */
#define TCS34003_REG_AILTL              0x84U  ///< Clear interrupt low  threshold low  byte (R/W, reset 0x00)
#define TCS34003_REG_AILTH              0x85U  ///< Clear interrupt low  threshold high byte (R/W, reset 0x00)
#define TCS34003_REG_AIHTL              0x86U  ///< Clear interrupt high threshold low  byte (R/W, reset 0x00)
#define TCS34003_REG_AIHTH              0x87U  ///< Clear interrupt high threshold high byte (R/W, reset 0x00)

/* Interrupt and Configuration Registers */
#define TCS34003_REG_PERS               0x8CU  ///< Interrupt persistence filter          (R/W, reset 0x00)
#define TCS34003_REG_CONFIG             0x8DU  ///< Configuration register                (R/W, reset 0x40)
#define TCS34003_REG_CONTROL            0x8FU  ///< Gain control register                 (R/W, reset 0x00)
#define TCS34003_REG_AUX                0x90U  ///< Auxiliary control register            (R/W, reset 0x00)

/* Device Identification Registers (read-only) */
#define TCS34003_REG_REVID              0x91U  ///< Revision ID register                  (R)
#define TCS34003_REG_ID                 0x92U  ///< Device ID register                    (R)

/* Status and Data Registers */
#define TCS34003_REG_STATUS             0x93U  ///< Device status                         (R,   reset 0x00)
#define TCS34003_REG_CDATAL             0x94U  ///< Clear / IR channel low  data byte     (R,   reset 0x00)
#define TCS34003_REG_CDATAH             0x95U  ///< Clear / IR channel high data byte     (R,   reset 0x00)
#define TCS34003_REG_RDATAL             0x96U  ///< Red   ADC low  data byte              (R,   reset 0x00)
#define TCS34003_REG_RDATAH             0x97U  ///< Red   ADC high data byte              (R,   reset 0x00)
#define TCS34003_REG_GDATAL             0x98U  ///< Green ADC low  data byte              (R,   reset 0x00)
#define TCS34003_REG_GDATAH             0x99U  ///< Green ADC high data byte              (R,   reset 0x00)
#define TCS34003_REG_BDATAL             0x9AU  ///< Blue  ADC low  data byte              (R,   reset 0x00)
#define TCS34003_REG_BDATAH             0x9BU  ///< Blue  ADC high data byte              (R,   reset 0x00)

/* IR Channel and Interrupt Control */
#define TCS34003_REG_IR                 0xC0U  ///< IR channel access                     (R/W, reset 0x00)
#define TCS34003_REG_IFORCE             0xE4U  ///< Force interrupt (write any value)      (W)
#define TCS34003_REG_CICLEAR            0xE6U  ///< Clear channel interrupt clear          (W)
#define TCS34003_REG_AICLEAR            0xE7U  ///< Clear all interrupts                   (W)

/* ============================================================================
 * Calculation Coefficients
 * ========================================================================= */

/** @defgroup CCT_Coefficients Correlated Colour Temperature (CCT) coefficients
 *  Used to convert raw RGBC counts to CIE XYZ and then to CCT (Kelvin).
 *  @{
 */
#define TCS34003_CCT_COEFF_X_RED        (-0.14282f) ///< Red   weight for CIE X
#define TCS34003_CCT_COEFF_X_GREEN        1.54924f  ///< Green weight for CIE X
#define TCS34003_CCT_COEFF_X_BLUE       (-0.95641f) ///< Blue  weight for CIE X

#define TCS34003_CCT_COEFF_Y_RED        (-0.32466f) ///< Red   weight for CIE Y (luminance)
#define TCS34003_CCT_COEFF_Y_GREEN        1.57837f  ///< Green weight for CIE Y
#define TCS34003_CCT_COEFF_Y_BLUE       (-0.73191f) ///< Blue  weight for CIE Y

#define TCS34003_CCT_COEFF_Z_RED        (-0.68202f) ///< Red   weight for CIE Z
#define TCS34003_CCT_COEFF_Z_GREEN        0.77073f  ///< Green weight for CIE Z
#define TCS34003_CCT_COEFF_Z_BLUE         0.56332f  ///< Blue  weight for CIE Z

/** CCT cubic polynomial: CCT = C1·n³ + C2·n² + C3·n + C4
 *  where n = (x - X0) / (Y0 - y),  x and y are CIE chromaticity coordinates. */
#define TCS34003_CCT_COEFF_C1             449.0f    ///< Cubic   term coefficient
#define TCS34003_CCT_COEFF_C2            3525.0f    ///< Quadratic term coefficient
#define TCS34003_CCT_COEFF_C3            6823.3f    ///< Linear   term coefficient
#define TCS34003_CCT_COEFF_C4            5520.33f   ///< Constant term
/** @} */

/** @defgroup Lux_Coefficients Luminance (lux) coefficients
 *  @{
 */
#define TCS34003_LUX_COEFF_CLEAR          0.136f    ///< Clear channel weight in lux formula
#define TCS34003_LUX_COEFF_GREEN          0.119f    ///< Green channel weight in lux formula
/** @} */

/** @defgroup Irradiance_Constants Irradiance calculation constants
 *  @{
 */

/** @brief Typical clear-channel irradiance responsivity at reference conditions
 *         (datasheet Fig. 8: AGAIN=16x, ATIME=0xF6 / 27.8 ms, CCT=2700K).
 *         Units: counts/(µW/cm²). */
#define TCS34003_RESPONSIVITY_CLEAR_REF   14.0f

/** @brief Reference gain at which Re was characterised (datasheet Fig. 8). */
#define TCS34003_CALIB_AGAIN_REF          16.0f

/** @brief Reference integration time in ms at which Re was characterised (datasheet Fig. 8). */
#define TCS34003_CALIB_ATIME_MS_REF       27.8f

/** @brief Integration time step size in ms per cycle (datasheet Fig. 11, typ.). */
#define TCS34003_ATIME_STEP_MS            2.78f

/** @brief Conversion factor: µW/cm² → W/m²  (1 µW/cm² = 0.01 W/m²). */
#define TCS34003_MICRO_TO_W_CONV          0.01f

/** @} */

/** @brief Mid-point of the 16-bit ADC range, used as a default interrupt threshold. */
#define TCS34003_THRESHOLD_MID            0x7FFFU

/** @brief CIE x chromaticity offset for McCamy CCT approximation. */
#define TCS34003_CCT_COEFF_X0             0.3320f

/** @brief CIE y chromaticity offset for McCamy CCT approximation. */
#define TCS34003_CCT_COEFF_Y0             0.1858f

/* ============================================================================
 * Register Structures and Enumerations
 * ========================================================================= */

/**
 * @brief ENABLE Register (0x80, R/W, reset 0x00).
 *
 * Controls power, ADC, wait timer, and interrupt enable.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Raw byte value — use for read/write operations.
        struct __attribute__((packed)) {
            uint8_t PON  : 1; ///< [0]   Power ON. 1 = activate internal oscillator.
            uint8_t AEN  : 1; ///< [1]   ADC Enable. 1 = enable RGBC ADC.
            uint8_t RES2 : 1; ///< [2]   Reserved. Write as 0.
            uint8_t WEN  : 1; ///< [3]   Wait Enable. 1 = activate wait timer.
            uint8_t AIEN : 1; ///< [4]   ALS Interrupt Enable. 1 = allow ALS interrupts.
            uint8_t RES5 : 1; ///< [5]   Reserved. Write as 0.
            uint8_t SAI  : 1; ///< [6]   Sleep After Interrupt. 1 = power down after RGBC if interrupt asserted.
            uint8_t RES7 : 1; ///< [7]   Reserved. Write as 0.
        } Bits;               ///< Bit-field access.
    } Val;
} TCS34003_REG_ENABLE_t;

/**
 * @brief RGBC Integration Time (ATIME register 0x81, R/W, reset 0xFF).
 *
 * Controls internal ADC integration time.
 * Saturation count = min(CYCLES × 1024, 65535).
 */
typedef enum {
    TCS34003_ATIME_1_CYCLE    = 0xFF, ///< 1 cycle   (2.78 ms),  max count  1024
    TCS34003_ATIME_10_CYCLES  = 0xF6, ///< 10 cycles (27.8 ms),  max count 10240
    TCS34003_ATIME_37_CYCLES  = 0xDB, ///< 37 cycles (103 ms),   max count 37888
    TCS34003_ATIME_64_CYCLES  = 0xC0, ///< 64 cycles (178 ms),   max count 65535
    TCS34003_ATIME_256_CYCLES = 0x00  ///< 256 cycles (712 ms),  max count 65535
} TCS34003_ATIME_t;

/**
 * @brief Wait Time (WTIME register 0x83, R/W, reset 0xFF).
 *
 * Low-power inter-measurement delay. Steps of 2.78 ms; ×12 when WLONG=1.
 */
typedef enum {
    TCS34003_WTIME_1_CYCLE    = 0xFF, ///< 2.78 ms  (WLONG=0) /  0.03 s (WLONG=1)
    TCS34003_WTIME_30_CYCLES  = 0xE2, ///< 83.4 ms  (WLONG=0) /  1.00 s (WLONG=1) — use with WLONG=1 for ~1 s refresh
    TCS34003_WTIME_85_CYCLES  = 0xAB, ///< 236 ms   (WLONG=0) /  2.84 s (WLONG=1)
    TCS34003_WTIME_256_CYCLES = 0x00  ///< 712 ms   (WLONG=0) /  8.54 s (WLONG=1)
} TCS34003_WTIME_t;

/**
 * @brief Clear Channel Interrupt Threshold Registers (0x84–0x87, R/W).
 *
 * 16-bit low and high thresholds compared against CDATA.
 * Interrupt fires when CDATA is outside [LowThreshold, HighThreshold] for
 * the number of consecutive cycles configured in APERS.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t FullArray[4]; ///< Byte array for direct sequential I²C write (AILTL…AIHTH).
        struct {
            union {
                struct { uint8_t LSB; uint8_t MSB; } Bytes; ///< Individual bytes.
                uint16_t Value;                              ///< 16-bit low threshold.
            } LowThreshold;

            union {
                struct { uint8_t LSB; uint8_t MSB; } Bytes; ///< Individual bytes.
                uint16_t Value;                              ///< 16-bit high threshold.
            } HighThreshold;
        };
    };
} TCS34003_CC_Threshold_t;

/**
 * @brief Interrupt Persistence Filter (PERS register 0x8C, bits [3:0], R/W).
 *
 * Selects how many consecutive out-of-range RGBC cycles are needed before
 * an interrupt is asserted.
 */
typedef enum {
    TCS34003_APERS_EVERY_CYCLE     = 0x00, ///< Interrupt every RGBC cycle (regardless of thresholds).
    TCS34003_APERS_OUT_OF_RANGE_1  = 0x01, ///< Any single value outside threshold range.
    TCS34003_APERS_OUT_OF_RANGE_2  = 0x02, ///<  2 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_3  = 0x03, ///<  3 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_5  = 0x04, ///<  5 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_10 = 0x05, ///< 10 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_15 = 0x06, ///< 15 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_20 = 0x07, ///< 20 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_25 = 0x08, ///< 25 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_30 = 0x09, ///< 30 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_35 = 0x0A, ///< 35 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_40 = 0x0B, ///< 40 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_45 = 0x0C, ///< 45 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_50 = 0x0D, ///< 50 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_55 = 0x0E, ///< 55 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_60 = 0x0F  ///< 60 consecutive values out of range.
} TCS34003_APERS_t;

/**
 * @brief Configuration Register (CONFIG 0x8D, R/W, reset 0x40).
 *
 * @note  Bit 6 is reserved but must always be written as 1 (hence 0x40 base).
 */
typedef enum {
    TCS34003_CONFIG_WLONG_DISABLE = 0x40, ///< WLONG = 0: wait time as programmed in WTIME.
    TCS34003_CONFIG_WLONG_ENABLE  = 0x42  ///< WLONG = 1: wait time is 12× the WTIME value.
} TCS34003_CONFIG_t;

/**
 * @brief RGBC Gain Control (CONTROL register 0x8F, bits [1:0], R/W).
 */
typedef enum {
    TCS34003_AGAIN_1X  = 0x00, ///<  1× gain.
    TCS34003_AGAIN_4X  = 0x01, ///<  4× gain.
    TCS34003_AGAIN_16X = 0x02, ///< 16× gain.
    TCS34003_AGAIN_64X = 0x03  ///< 64× gain.
} TCS34003_AGAIN_t;

/**
 * @brief Auxiliary Register (AUX 0x90, bit 5, R/W).
 *
 * Enables the ALS saturation detection interrupt (ASIEN).
 */
typedef enum {
    TCS34003_AUX_ASIEN_DISABLE = 0x00, ///< ALS saturation interrupt disabled.
    TCS34003_AUX_ASIEN_ENABLE  = 0x20  ///< ALS saturation interrupt enabled.
} TCS34003_AUX_t;

/**
 * @brief STATUS Register (0x93, read-only, reset 0x00).
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Raw byte value.
        struct __attribute__((packed)) {
            uint8_t AVALID : 1; ///< [0]     RGBC Valid: RGBC cycle completed since AEN asserted.
            uint8_t RES13  : 3; ///< [3:1]   Reserved.
            uint8_t AINT   : 1; ///< [4]     ALS Interrupt: threshold + persistence conditions met.
            uint8_t RES56  : 2; ///< [6:5]   Reserved.
            uint8_t ASAT   : 1; ///< [7]     ALS Saturation: ADC reached upper dynamic range limit.
        } Bits;
    } Val;
} TCS34003_STATUS_t;

/**
 * @brief RGBC Data Registers (0x94–0x9B, read-only).
 *
 * Holds 16-bit ADC counts for Clear, Red, Green, and Blue channels.
 * Read via a single 8-byte burst starting at CDATAL (0x94).
 */
typedef struct __attribute__((packed)) {
    union {
        struct {
            uint16_t ClearChannel; ///< Clear / IR channel (CDATAL/CDATAH, 0x94–0x95).
            uint16_t RedChannel;   ///< Red   channel      (RDATAL/RDATAH, 0x96–0x97).
            uint16_t GreenChannel; ///< Green channel      (GDATAL/GDATAH, 0x98–0x99).
            uint16_t BlueChannel;  ///< Blue  channel      (BDATAL/BDATAH, 0x9A–0x9B).
        };
        uint8_t FullArray[8]; ///< Raw byte array for direct I²C burst read.
    };
} TCS34003_RGBC_Data_t;

/**
 * @brief IR Data (read via IR Register 0xC0 remapping onto Clear channel).
 *
 * When IR sensor access is enabled the Clear channel ADC reports IR counts.
 * Read as a 2-byte burst starting at CDATAL (0x94).
 */
typedef struct __attribute__((packed)) {
    union {
        uint16_t Value;       ///< 16-bit IR count.
        uint8_t  FullArray[2]; ///< Raw bytes for I²C burst read.
    };
} TCS34003_IR_Data_t;

/**
 * @brief IR Sensor Access control (IR register 0xC0, bit 7).
 *
 * When enabled the Clear channel ADC is remapped to the IR (centre) photodiode.
 */
typedef enum {
    TCS34003_IR_ACCESS_DISABLE = 0x00, ///< Clear channel reports visible light.
    TCS34003_IR_ACCESS_ENABLE  = 0x80  ///< Clear channel reports IR sensor data.
} TCS34003_IR_Access_t;

/* ============================================================================
 * Composite Structures
 * ========================================================================= */

/**
 * @brief Processed light measurement output.
 *
 * Populated by TCS34003_GetLightData(). Contains raw RGBC + IR counts and
 * the derived photometric / radiometric values.
 */
typedef struct __attribute__((packed)) {
    TCS34003_RGBC_Data_t RGBC;      ///< Raw RGBC ADC counts.
    TCS34003_IR_Data_t   IR;        ///< Raw IR count.
    float                CCT;       ///< Correlated Colour Temperature (K).
    float                Lux;       ///< Illuminance (lux).
    float                Irradiance;///< Irradiance (W/m²).
} TCS34003_LightData_t;

/**
 * @brief Mirror of all writable / readable TCS3400 registers.
 *
 * Maintained in RAM by the driver; provides a shadow copy of the hardware state.
 */
typedef struct __attribute__((packed)) {
    TCS34003_REG_ENABLE_t   ENABLE;   ///< 0x80 R/W — Enable register.
    TCS34003_ATIME_t        ATIME;    ///< 0x81 R/W — Integration time.
    TCS34003_WTIME_t        WTIME;    ///< 0x83 R/W — Wait time.
    TCS34003_CC_Threshold_t CC_TH;    ///< 0x84–0x87 R/W — Clear channel thresholds.
    TCS34003_APERS_t        APERS;    ///< 0x8C R/W — Interrupt persistence.
    TCS34003_CONFIG_t       CONFIG;   ///< 0x8D R/W — Configuration (WLONG).
    TCS34003_AGAIN_t        AGAIN;    ///< 0x8F R/W — Gain control.
    TCS34003_AUX_t          AUX;      ///< 0x90 R/W — ALS saturation interrupt.
    /* 0x91 REVID — read-only, not cached */
    /* 0x92 ID    — read-only, not cached */
    TCS34003_STATUS_t       STATUS;   ///< 0x93 R   — Device status.
    TCS34003_IR_Access_t    IR_Access;///< 0xC0 R/W — IR sensor access.
} TCS34003_REGISTERS_t;

/* ============================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief  Initialise the TCS3400 sensor.
 * @details Configures: PON + AEN, 712 ms integration, 236 ms wait (WLONG=0),
 *          1× gain, interrupt every RGBC cycle, mid-scale thresholds, then
 *          clears any pending interrupts and enables AIEN.
 * @retval HAL_OK on success, HAL error code on failure.
 */
HAL_StatusTypeDef TCS34003_Init(void);

/**
 * @brief  Write the ENABLE register.
 * @param  value  Raw byte to write (see TCS34003_REG_ENABLE_t).
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef TCS34003_SetEnable(uint8_t value);

/**
 * @brief  Set RGBC integration time (ATIME register).
 * @param  value  Integration time selection (TCS34003_ATIME_t).
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef TCS34003_SetIntegrationTime(TCS34003_ATIME_t value);

/**
 * @brief  Set inter-measurement wait time (WTIME register).
 * @param  value  Wait time selection (TCS34003_WTIME_t).
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef TCS34003_SetWaitTime(TCS34003_WTIME_t value);

/**
 * @brief  Set Clear channel interrupt thresholds.
 * @param  low   16-bit lower bound; interrupt fires when CDATA < low.
 * @param  high  16-bit upper bound; interrupt fires when CDATA > high.
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef TCS34003_SetClearChannelThreshold(uint16_t low, uint16_t high);

/**
 * @brief  Enable ALS interrupts (set AIEN bit in ENABLE register).
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef TCS34003_EnableInterrupts(void);

/**
 * @brief  Set interrupt persistence filter (PERS register).
 * @param  value  Persistence selection (TCS34003_APERS_t).
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef TCS34003_SetInterruptPersistence(TCS34003_APERS_t value);

/**
 * @brief  Write the CONFIG register (controls WLONG).
 * @param  value  Configuration value (TCS34003_CONFIG_t).
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef TCS34003_SetConfig(TCS34003_CONFIG_t value);

/**
 * @brief  Set RGBC analogue gain (CONTROL register).
 * @param  value  Gain selection (TCS34003_AGAIN_t).
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef TCS34003_SetGain(TCS34003_AGAIN_t value);

/**
 * @brief  Write the AUX register (ALS saturation interrupt enable).
 * @param  value  AUX value (TCS34003_AUX_t).
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef TCS34003_SetAux(TCS34003_AUX_t value);

/**
 * @brief  Control IR sensor mapping onto the Clear channel.
 * @param  value  TCS34003_IR_ACCESS_ENABLE or TCS34003_IR_ACCESS_DISABLE.
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef TCS34003_SetIRAccess(TCS34003_IR_Access_t value);

/**
 * @brief  Clear all active interrupts (write to AICLEAR 0xE7).
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef TCS34003_ClearInterrupts(void);

/**
 * @brief  Force an interrupt (write to IFORCE 0xE4).
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef TCS34003_ForceInterrupt(void);

/**
 * @brief  Read the STATUS register into the internal register shadow.
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef TCS34003_GetStatus(void);

/**
 * @brief  Read RGBC + IR data and compute CCT, lux, and irradiance.
 *
 * @details Sequence:
 *  1. Read STATUS — returns HAL_ERROR if AVALID is not set.
 *  2. Burst-read 8 bytes of RGBC from CDATAL (0x94).
 *  3. Enable IR access, burst-read 2 bytes from CDATAL, disable IR access.
 *  4. Compute CCT (McCamy approximation).
 *  5. Compute lux (weighted Clear + Green).
 *  6. Compute irradiance (Clear / responsivity × unit conversion).
 *  7. Clear all interrupts.
 *
 * @param[out] data  Pointer to caller-allocated TCS34003_LightData_t.
 * @retval HAL_OK on success, HAL_ERROR if ALS data not yet valid.
 */
HAL_StatusTypeDef TCS34003_GetLightData(TCS34003_LightData_t *data);

#endif /* TCS34003_H */
