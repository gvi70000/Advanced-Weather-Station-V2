/***************************************************************************
 * @file [AS7331].h/.c
 * This file contains definitions, data structures, and functions
 * for interfacing with the AS7331 spectral UV (UVA/B/C) sensor.
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

#ifndef __AS7331_H
#define __AS7331_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "i2c.h"
#include <stdint.h>

// I2C address
#define AS7331_I2C_ADDR         (0x74 << 1)  // A1=0, A0=0 → 7-bit address 0x74

// Expected DEVID value (bits [7:4] of AGEN register)
#define AS7331_DEVID            0x02         // DEVID field = 0b0010

// Temperature conversion coefficients (datasheet section 7.7)
// T[°C] = raw_12bit * 0.05 - 66.9
// T[°C * 100] = raw_12bit * 5 - 6690
#define AS7331_TEMP_SCALE       5            // Multiply raw 12-bit value by this
#define AS7331_TEMP_OFFSET      6690         // Subtract this to get °C * 100

// Register addresses (configuration state registers — accessible via I2C in Configuration state)
#define AS7331_REG_OSR          0x00  // Operational State Register
#define AS7331_REG_TEMP         0x01  // Temperature result register (16-bit, measurement state)
#define AS7331_REG_AGEN         0x02  // Device ID register (read-only)
#define AS7331_REG_MRES1        0x02  // UVA result register (measurement state) — same address as AGEN; accesses the output bank
#define AS7331_REG_MRES2        0x03  // UVB result register (measurement state)
#define AS7331_REG_MRES3        0x04  // UVC result register (measurement state)
#define AS7331_REG_OUTCONVL     0x05  // Conversion time LSW register (24-bit, bits [15:0])
#define AS7331_REG_OUTCONVH     0x06  // Conversion time MSB register (24-bit, bits [23:16])
#define AS7331_REG_CREG1        0x06  // Configuration register 1 (configuration state) — same address as OUTCONVH
#define AS7331_REG_CREG2        0x07  // Configuration register 2 (configuration state)
#define AS7331_REG_CREG3        0x08  // Configuration register 3 (configuration state)
#define AS7331_REG_BREAK        0x09  // Break time register (steps of 8 µs, max 255 * 8 µs = 2040 µs)
#define AS7331_REG_EDGES        0x0A  // SYN falling edge count register (SYND mode)
#define AS7331_REG_OPTREG       0x0B  // Optimization register (write 0b01110010 | INIT_IDX)

// OSR register burst-read size when reading OSR + STATUS together
#define AS7331_OSR_STATUS_SIZE  2     // 2 bytes: OSR (byte 0) + STATUS (byte 1)
// UV data burst-read size: TEMP(2) + MRES1(2) + MRES2(2) + MRES3(2) = 8 bytes
#define AS7331_UV_DATA_SIZE     8

// ─── Enumerations ─────────────────────────────────────────────────────────────

/**
 * @brief OSR:SS — Start/Stop measurement control.
 */
typedef enum {
    AS7331_SS_STOP  = 0,  // Stop measurement
    AS7331_SS_START = 1   // Start measurement
} AS7331_SS_t;

/**
 * @brief OSR:PD — Power-down control.
 * @note  PD=1 means power-down is ON (device is powered down).
 *        PD=0 means power-down is OFF (device is powered on).
 */
typedef enum {
    AS7331_PD_OFF = 0,  // Power-down disabled (device active)
    AS7331_PD_ON  = 1   // Power-down enabled (device in power-down state)
} AS7331_PWR_t;

/**
 * @brief OSR:SW_RES — Software reset control.
 */
typedef enum {
    AS7331_SW_RST_OFF = 0,  // Normal operation
    AS7331_SW_RST_ON  = 1   // Software reset active
} AS7331_SW_RST_t;

/**
 * @brief OSR:DOS — Device operational state.
 */
typedef enum {
    AS7331_DOS_NOP           = 0b000,  // No operation (used for SS-only writes)
    AS7331_DOS_CONFIGURATION = 0b010,  // Configuration state
    AS7331_DOS_MEASUREMENT   = 0b011   // Measurement state
} AS7331_DOS_t;

/**
 * @brief CREG1:GAIN — ADC gain selection.
 */
typedef enum {
    AS7331_GAIN_2048X = 0b0000,  // Gain = 2048x (highest sensitivity)
    AS7331_GAIN_1024X = 0b0001,
    AS7331_GAIN_512X  = 0b0010,
    AS7331_GAIN_256X  = 0b0011,
    AS7331_GAIN_128X  = 0b0100,
    AS7331_GAIN_64X   = 0b0101,
    AS7331_GAIN_32X   = 0b0110,
    AS7331_GAIN_16X   = 0b0111,
    AS7331_GAIN_8X    = 0b1000,
    AS7331_GAIN_4X    = 0b1001,
    AS7331_GAIN_2X    = 0b1010,
    AS7331_GAIN_1X    = 0b1011   // Gain = 1x (widest range)
} AS7331_GAIN_t;

/**
 * @brief CREG1:TIME — Integration time selection.
 */
typedef enum {
    AS7331_TIME_1MS    = 0b0000,
    AS7331_TIME_2MS    = 0b0001,
    AS7331_TIME_4MS    = 0b0010,
    AS7331_TIME_8MS    = 0b0011,
    AS7331_TIME_16MS   = 0b0100,
    AS7331_TIME_32MS   = 0b0101,
    AS7331_TIME_64MS   = 0b0110,
    AS7331_TIME_128MS  = 0b0111,
    AS7331_TIME_256MS  = 0b1000,
    AS7331_TIME_512MS  = 0b1001,
    AS7331_TIME_1024MS = 0b1010,
    AS7331_TIME_2048MS = 0b1011,
    AS7331_TIME_4096MS = 0b1100,
    AS7331_TIME_8192MS = 0b1101,
    AS7331_TIME_16384MS = 0b1110,
    AS7331_TIME_1MS_ALT = 0b1111  // Alternate 1 ms selection
} AS7331_TIME_t;

/**
 * @brief CREG2:EN_TM — Conversion time measurement enable (SYND mode only).
 */
typedef enum {
    AS7331_EN_TM_DISABLED = 0,  // Conversion time measurement disabled
    AS7331_EN_TM_ENABLED  = 1   // Conversion time measurement enabled (result in OUTCONV)
} AS7331_EN_TM_t;

/**
 * @brief CREG2:EN_DIV — Digital divider enable.
 */
typedef enum {
    AS7331_EN_DIV_DISABLED = 0,  // Digital divider disabled
    AS7331_EN_DIV_ENABLED  = 1   // Digital divider enabled
} AS7331_EN_DIV_t;

/**
 * @brief CREG2:DIV — Digital divider ratio selection.
 */
typedef enum {
    AS7331_DIV_2   = 0b000,
    AS7331_DIV_4   = 0b001,
    AS7331_DIV_8   = 0b010,
    AS7331_DIV_16  = 0b011,
    AS7331_DIV_32  = 0b100,
    AS7331_DIV_64  = 0b101,
    AS7331_DIV_128 = 0b110,
    AS7331_DIV_256 = 0b111
} AS7331_DIV_t;

/**
 * @brief CREG3:MMODE — Measurement mode selection.
 */
typedef enum {
    AS7331_MMODE_CONT = 0b00,  // Continuous mode
    AS7331_MMODE_CMD  = 0b01,  // Command (single-shot) mode
    AS7331_MMODE_SYNS = 0b10,  // SYN start mode (falling edge at SYN starts conversion)
    AS7331_MMODE_SYND = 0b11   // SYN start+stop mode (SYN edges control start and stop)
} AS7331_MMODE_t;

/**
 * @brief CREG3:SB — Standby mode control.
 */
typedef enum {
    AS7331_SB_OFF = 0,  // Standby disabled
    AS7331_SB_ON  = 1   // Standby enabled (lower power between measurements)
} AS7331_SB_t;

/**
 * @brief CREG3:RDYOD — READY pin output type.
 */
typedef enum {
    AS7331_RDYOD_PUSH_PULL  = 0,  // Push-pull output (default)
    AS7331_RDYOD_OPEN_DRAIN = 1   // Open-drain output
} AS7331_RDYOD_t;

/**
 * @brief CREG3:CCLK — Internal clock frequency selection.
 */
typedef enum {
    AS7331_CLK_1MHZ = 0b00,  // ~1 MHz (typ. 0.975 MHz)
    AS7331_CLK_2MHZ = 0b01,  // ~2 MHz
    AS7331_CLK_4MHZ = 0b10,  // ~4 MHz
    AS7331_CLK_8MHZ = 0b11   // ~8 MHz (typ. 7.8 MHz)
} AS7331_CLK_t;

// ─── Register bitfield structures ─────────────────────────────────────────────

/**
 * @brief OSR register (0x00) bitfield — configuration and measurement state control.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                      // Complete 8-bit register value
        struct __attribute__((packed)) {
            uint8_t DOS      : 3;           // Bits 0-2: Device operational state
            uint8_t SW_RES   : 1;           // Bit 3: Software reset
            uint8_t RESERVED : 2;           // Bits 4-5: Reserved
            uint8_t PD       : 1;           // Bit 6: Power-down (1 = power-down ON)
            uint8_t SS       : 1;           // Bit 7: Start/Stop measurement
        } BitField;
    } Val;
} AS7331_OSR_t;

/**
 * @brief STATUS register (byte 1 of the OSR+STATUS 2-byte read at 0x00) bitfield.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                      // Complete 8-bit register value
        struct __attribute__((packed)) {
            uint8_t POWERSTATE  : 1;        // Bit 0: Device is in power-down state
            uint8_t STANDBYSTATE: 1;        // Bit 1: Device is in standby state
            uint8_t NOTREADY    : 1;        // Bit 2: Measurement not yet complete
            uint8_t NDATA       : 1;        // Bit 3: New data available in result registers
            uint8_t LDATA       : 1;        // Bit 4: Result registers overrun (previous data lost)
            uint8_t ADCOF       : 1;        // Bit 5: ADC overflow occurred
            uint8_t MRESOF      : 1;        // Bit 6: Measurement result overflow
            uint8_t OUTCONVOF   : 1;        // Bit 7: Conversion time counter overflow
        } BitField;
    } Val;
} AS7331_STATUS_t;

/**
 * @brief AGEN register (0x02) bitfield — device identity (configuration state only).
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                      // Complete 8-bit register value
        struct __attribute__((packed)) {
            uint8_t MUT   : 4;              // Bits 0-3: Mutation number of control register bank
            uint8_t DEVID : 4;              // Bits 4-7: Device ID (should read 0x2 = AS7331)
        } BitField;
    } Val;
} AS7331_AGEN_t;

/**
 * @brief CREG1 register (0x06 in configuration state) bitfield.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                      // Complete 8-bit register value
        struct __attribute__((packed)) {
            uint8_t TIME : 4;               // Bits 0-3: Integration time selection
            uint8_t GAIN : 4;               // Bits 4-7: Gain selection
        } BitField;
    } Val;
} AS7331_CREG1_t;

/**
 * @brief CREG2 register (0x07) bitfield.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                      // Complete 8-bit register value
        struct __attribute__((packed)) {
            uint8_t DIV       : 3;          // Bits 0-2: Digital divider value
            uint8_t EN_DIV    : 1;          // Bit 3: Enable digital divider
            uint8_t RESERVED  : 2;          // Bits 4-5: Reserved
            uint8_t EN_TM     : 1;          // Bit 6: Enable conversion time measurement (SYND only)
            uint8_t RESERVED1 : 1;          // Bit 7: Reserved
        } BitField;
    } Val;
} AS7331_CREG2_t;

/**
 * @brief CREG3 register (0x08) bitfield.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                      // Complete 8-bit register value
        struct __attribute__((packed)) {
            uint8_t CCLK      : 2;          // Bits 0-1: Internal clock frequency
            uint8_t RESERVED  : 1;          // Bit 2: Reserved
            uint8_t RDYOD     : 1;          // Bit 3: READY pin output mode
            uint8_t SB        : 1;          // Bit 4: Standby enable
            uint8_t RESERVED1 : 1;          // Bit 5: Reserved
            uint8_t MMODE     : 2;          // Bits 6-7: Measurement mode
        } BitField;
    } Val;
} AS7331_CREG3_t;

/**
 * @brief Shadow register map for AS7331 configuration-state registers.
 * @note  Result registers (TEMP, MRES1-3, OUTCONV) are not included here as they
 *        are read-only output values populated directly into AS7331_DataOut_t.
 */
typedef struct __attribute__((packed)) {
    AS7331_OSR_t    OSR;     // 0x00 — Operational state register
    AS7331_AGEN_t   AGEN;    // 0x02 — Device identity register (read-only)
    AS7331_CREG1_t  CREG1;   // 0x06 — Configuration register 1
    AS7331_CREG2_t  CREG2;   // 0x07 — Configuration register 2
    AS7331_CREG3_t  CREG3;   // 0x08 — Configuration register 3
    uint8_t         BREAK;   // 0x09 — Break time (value * 8 µs)
    uint8_t         EDGES;   // 0x0A — SYN edge count (SYND mode)
    uint8_t         OPTREG;  // 0x0B — Optimization register
} AS7331_Config_t;

/**
 * @brief Measurement result data structure.
 * @note  TEMP_C100 stores temperature in °C * 100 for integer precision.
 *        UV channels store raw ADC counts; convert to irradiance using datasheet Eq. 2.
 */
typedef struct __attribute__((packed)) {
    int16_t  TEMP_C100;  // Temperature in °C * 100 (e.g. 2500 = 25.00 °C)
    uint16_t UVA;        // UVA channel (MRES1) — counts, 1 ms integration baseline
    uint16_t UVB;        // UVB channel (MRES2) — counts
    uint16_t UVC;        // UVC channel (MRES3) — counts
} AS7331_DataOut_t;

// ─── Function prototypes ──────────────────────────────────────────────────────

/**
 * @brief Initialize the AS7331 sensor (reset, verify device ID, configure, start measurement).
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_Init(void);

/**
 * @brief Read and verify the device ID from the AGEN register (0x02).
 * @return HAL status. Returns HAL_ERROR if DEVID does not match AS7331_DEVID.
 */
HAL_StatusTypeDef AS7331_VerifyDeviceID(void);

/**
 * @brief Perform a software reset via the OSR:SW_RES bit.
 * @param swReset AS7331_SW_RST_ON to assert reset, AS7331_SW_RST_OFF to release.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SoftReset(AS7331_SW_RST_t swReset);

/**
 * @brief Set the device power-down state via OSR:PD.
 * @param setPower AS7331_PD_OFF to power on, AS7331_PD_ON to power down.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetPower(AS7331_PWR_t setPower);

/**
 * @brief Set the device operational state (DOS field of OSR register).
 * @param dos Desired state from AS7331_DOS_t.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetOperationalState(AS7331_DOS_t dos);

/**
 * @brief Start or stop measurement via OSR:SS.
 * @param ss AS7331_SS_START or AS7331_SS_STOP.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_StartStopMeasurement(AS7331_SS_t ss);

/**
 * @brief Set the integration time in CREG1:TIME.
 * @param time Integration time selection from AS7331_TIME_t.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetIntegrationTime(AS7331_TIME_t time);

/**
 * @brief Set the gain in CREG1:GAIN.
 * @param gain Gain selection from AS7331_GAIN_t.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetGain(AS7331_GAIN_t gain);

/**
 * @brief Set the digital divider ratio in CREG2:DIV.
 * @param divider Divider ratio from AS7331_DIV_t.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetDivider(AS7331_DIV_t divider);

/**
 * @brief Enable or disable the digital divider via CREG2:EN_DIV.
 * @param enableDivider AS7331_EN_DIV_ENABLED or AS7331_EN_DIV_DISABLED.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_EnableDivider(AS7331_EN_DIV_t enableDivider);

/**
 * @brief Enable or disable conversion time measurement via CREG2:EN_TM (SYND mode only).
 * @param enableTimer AS7331_EN_TM_ENABLED or AS7331_EN_TM_DISABLED.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_EnableInternalMeasurement(AS7331_EN_TM_t enableTimer);

/**
 * @brief Set the measurement mode via CREG3:MMODE.
 * @param mode Measurement mode from AS7331_MMODE_t.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetMeasurementMode(AS7331_MMODE_t mode);

/**
 * @brief Set standby mode via CREG3:SB.
 * @param standby AS7331_SB_ON or AS7331_SB_OFF.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetStandbyMode(AS7331_SB_t standby);

/**
 * @brief Set the READY pin output mode via CREG3:RDYOD.
 * @param rdyod AS7331_RDYOD_PUSH_PULL or AS7331_RDYOD_OPEN_DRAIN.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetReadyOutputMode(AS7331_RDYOD_t rdyod);

/**
 * @brief Set the internal clock frequency via CREG3:CCLK.
 * @param cclk Clock frequency from AS7331_CLK_t.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetClockFrequency(AS7331_CLK_t cclk);

/**
 * @brief Set the break time between consecutive measurements (BREAK register, 0x09).
 * @param breakTime Break time in units of 8 µs (0–255; max 2040 µs).
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetBreakTime(uint8_t breakTime);

/**
 * @brief Set the number of SYN falling edges to end a conversion (EDGES register, SYND mode).
 * @param edges Edge count (0–255).
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetEdges(uint8_t edges);

/**
 * @brief Write the OPTREG register (0x0B) with the INIT_IDX field.
 * @details Recommended write value is 0b01110010 | (initIdx & 0x01).
 * @param initIdx INIT_IDX bit value (0 or 1).
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetOptReg(uint8_t initIdx);

/**
 * @brief Read the OSR and STATUS registers (2-byte burst starting at 0x00).
 * @details Populates both the OSR and STATUS fields of the internal shadow register.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_GetOSR_Status(void);

/**
 * @brief Read UV and temperature measurement results from the device.
 * @details Performs a 8-byte burst read starting at AS7331_REG_TEMP (0x01) in
 *          measurement state, covering TEMP, MRES1 (UVA), MRES2 (UVB), MRES3 (UVC).
 *          Checks STATUS:NDATA before reading.
 * @param uvData Pointer to AS7331_DataOut_t to receive converted results.
 * @return HAL_OK    — new data successfully read and parsed.
 *         HAL_BUSY  — no new data available (NDATA = 0); caller should retry.
 *         HAL_ERROR — I2C communication failure.
 */
HAL_StatusTypeDef AS7331_ReadUVData(AS7331_DataOut_t *uvData);

/**
 * @brief Read the 24-bit conversion time from OUTCONVL (0x05) and OUTCONVH (0x06).
 * @details Only valid in SYND mode with CREG2:EN_TM = 1.
 * @param conversionTime Pointer to store the 24-bit conversion time (clock counts).
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_ReadConversionTime(uint32_t *conversionTime);

#ifdef __cplusplus
}
#endif

#endif /* __AS7331_H */
