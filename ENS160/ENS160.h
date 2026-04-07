/***************************************************************************
 * @file [ENS160].h/.c
 * This file contains definitions, data structures, and functions
 * for interfacing with the ENS160 air quality sensor.
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

#ifndef __ENS160_H
#define __ENS160_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "i2c.h"
#include <stdint.h>

// I2C address
#define ENS160_I2C_ADDRESS      (0x52 << 1)  // ADDR pin low → 7-bit address 0x52

// Part ID
#define ENS160_PART_ID          0x0160       // Expected value of PART_ID register

// Register addresses
#define ENS160_REG_PART_ID      0x00  // Device identity register (2 bytes)
#define ENS160_REG_OPMODE       0x10  // Operating mode register
#define ENS160_REG_CONFIG       0x11  // Interrupt pin configuration register
#define ENS160_REG_COMMAND      0x12  // Additional system commands register
#define ENS160_REG_TEMP_IN      0x13  // Host ambient temperature input register (2 bytes)
#define ENS160_REG_RH_IN        0x15  // Host relative humidity input register (2 bytes)
#define ENS160_REG_STATUS       0x20  // Device status register
#define ENS160_REG_DATA_AQI     0x21  // Air quality index data register
#define ENS160_REG_DATA_TVOC    0x22  // TVOC concentration register (2 bytes, ppb)
#define ENS160_REG_DATA_ECO2    0x24  // Equivalent CO2 concentration register (2 bytes, ppm)
#define ENS160_REG_DATA_T       0x30  // Calculated temperature register (2 bytes)
#define ENS160_REG_DATA_RH      0x32  // Calculated relative humidity register (2 bytes)
#define ENS160_REG_DATA_MISR    0x38  // Data integrity field register
#define ENS160_REG_GPR_WRITE    0x40  // General-purpose write registers (8 bytes)
#define ENS160_REG_GPR_READ     0x48  // General-purpose read registers (8 bytes)

// Register sizes
#define ENS160_SIZE1            1     // 1-byte transfer
#define ENS160_SIZE2            2     // 2-byte transfer
#define ENS160_SIZE5            5     // 5-byte transfer (AQI + TVOC + eCO2 burst)
#define ENS160_SIZE8            8     // 8-byte transfer (GPR block)

// Timing constants
#define ENS160_RESET_DELAY_MS   10    // Delay after reset before first access
#define ENS160_CLRGPR_DELAY_MS  2     // Delay after CLRGPR command
#define ENS160_CMD_DELAY_MS     50    // Delay after issuing a command

/**
 * @brief Operating mode values for OPMODE register (0x10).
 */
typedef enum {
    ENS160_OPMODE_DEEP_SLEEP = 0x00,  // Deep sleep — low-power standby
    ENS160_OPMODE_IDLE       = 0x01,  // Idle — low power, accepts commands
    ENS160_OPMODE_STANDARD   = 0x02,  // Standard gas sensing mode
    ENS160_OPMODE_RESET      = 0xF0   // Reset mode
} ENS160_OPMODE_t;

/**
 * @brief Command values for COMMAND register (0x12).
 */
typedef enum {
    ENS160_COMMAND_NOP         = 0x00,  // No operation
    ENS160_COMMAND_GET_APPVER  = 0x0E,  // Get firmware version → result in GPR_READ[4:6]
    ENS160_COMMAND_CLRGPR      = 0xCC   // Clear all GPR read registers
} ENS160_COMMAND_t;

/**
 * @brief CONFIG register (0x11) bitfield structure.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                    // Complete 8-bit register value
        struct __attribute__((packed)) {
            uint8_t INTEN    : 1;         // Bit 0: INTn pin enable
            uint8_t INTDAT   : 1;         // Bit 1: Assert INTn when DATA_XXX updated
            uint8_t RESERVED1: 1;         // Bit 2: Reserved
            uint8_t INTGPR   : 1;         // Bit 3: Assert INTn when GPR_READ updated
            uint8_t RESERVED2: 1;         // Bit 4: Reserved
            uint8_t INT_CFG  : 1;         // Bit 5: INTn drive (0 = open-drain, 1 = push-pull)
            uint8_t INTPOL   : 1;         // Bit 6: INTn polarity (0 = active low, 1 = active high)
            uint8_t RESERVED3: 1;         // Bit 7: Reserved
        } BitField;
    } Val;
} ENS160_CONFIG_t;

/**
 * @brief DEVICE_STATUS register (0x20) bitfield structure.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                    // Complete 8-bit register value
        struct __attribute__((packed)) {
            uint8_t NEWGPR   : 1;         // Bit 0: New data available in GPR_READ registers
            uint8_t NEWDAT   : 1;         // Bit 1: New data available in DATA_XXX registers
            uint8_t VALIDITY : 2;         // Bits 2-3: Validity flag (0=OK,1=Warm-up,2=Start-up,3=Invalid)
            uint8_t RESERVED : 2;         // Bits 4-5: Reserved
            uint8_t STATER   : 1;         // Bit 6: Error detected
            uint8_t STATAS   : 1;         // Bit 7: OPMODE is running
        } BitField;
    } Val;
} ENS160_STATUS_t;

/**
 * @brief Air quality data structure — maps directly onto a 5-byte burst read
 *        starting at DATA_AQI (0x21) through DATA_ECO2 MSB (0x25).
 * @note  Relies on little-endian layout and __attribute__((packed)); valid on STM32.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Buffer[ENS160_SIZE5];     // Raw 5-byte I2C read buffer
        struct __attribute__((packed)) {
            uint8_t  AQI;                 // 0x21 — Air quality index [1..5] (UBA)
            uint16_t TVOC;                // 0x22-0x23 — TVOC concentration (ppb)
            uint16_t eCO2;                // 0x24-0x25 — Equivalent CO2 (ppm)
        } Fields;
    };
} ENS160_Data_t;

/**
 * @brief Firmware version structure — populated from GPR_READ[4:6] after GET_APPVER command.
 */
typedef struct __attribute__((packed)) {
    uint8_t Major;    // Major firmware version (GPR_READ[4])
    uint8_t Minor;    // Minor firmware version (GPR_READ[5])
    uint8_t Release;  // Release firmware version (GPR_READ[6])
} ENS160_FW_t;

/**
 * @brief Raw sensor resistance structure.
 * @note  Only R1 (GPR_READ[0:1]) and R4 (GPR_READ[6:7]) are defined by the datasheet.
 *        Resistance in Ohms: Rires = 2^(Riraw / 2048).
 */
typedef struct __attribute__((packed)) {
    uint32_t R1;  // Sensor 1 resistance (Ohms) — GPR_READ[0:1]
    uint32_t R4;  // Sensor 4 resistance (Ohms) — GPR_READ[6:7]
} ENS160_Resistance_t;

/**
 * @brief Complete shadow register map for the ENS160.
 */
typedef struct __attribute__((packed)) {
    ENS160_OPMODE_t  OPMODE;                    // 0x10 — Operating mode
    ENS160_CONFIG_t  CONFIG;                    // 0x11 — Interrupt configuration
    ENS160_COMMAND_t COMMAND;                   // 0x12 — Pending command
    uint16_t         TEMP_IN;                   // 0x13-0x14 — Temperature compensation input
    uint16_t         RH_IN;                     // 0x15-0x16 — Humidity compensation input
    ENS160_STATUS_t  STATUS;                    // 0x20 — Device status
    uint8_t          GPR_READ[ENS160_SIZE8];    // 0x48-0x4F — General-purpose read registers
} ENS160_Registers_t;

// ─── Function prototypes ──────────────────────────────────────────────────────

/**
 * @brief Initialize the ENS160 sensor (reset, verify part ID, configure interrupts,
 *        set environmental compensation defaults, start standard sensing).
 * @return HAL status.
 */
HAL_StatusTypeDef ENS160_Init(void);

/**
 * @brief Read and verify the PART_ID register (0x00).
 * @return HAL status. Returns HAL_ERROR if the ID does not match ENS160_PART_ID.
 */
HAL_StatusTypeDef ENS160_ReadPartID(void);

/**
 * @brief Read the firmware version into the internal FW structure via GET_APPVER command.
 * @return HAL status.
 */
HAL_StatusTypeDef ENS160_GetFirmwareVersion(void);

/**
 * @brief Write ambient temperature and humidity to the ENS160 compensation registers.
 * @param temperatureC    Ambient temperature in degrees Celsius.
 * @param humidityPercent Relative humidity in percent.
 * @return HAL status.
 */
HAL_StatusTypeDef ENS160_SetEnvCompensation(float temperatureC, float humidityPercent);

/**
 * @brief Write the operating mode register (0x10).
 * @param mode Desired operating mode from ENS160_OPMODE_t.
 * @return HAL status.
 */
HAL_StatusTypeDef ENS160_SetOperatingMode(ENS160_OPMODE_t mode);

/**
 * @brief Read the DEVICE_STATUS register (0x20) into the internal shadow register.
 * @return HAL status.
 */
HAL_StatusTypeDef ENS160_ReadDeviceStatus(void);

/**
 * @brief Read air quality data and raw sensor resistances from the ENS160.
 * @param data       Pointer to ENS160_Data_t to receive AQI, TVOC and eCO2 values.
 * @param resistance Pointer to ENS160_Resistance_t to receive R1 and R4 resistance values.
 * @return HAL_OK if new data was read, HAL_BUSY if no new data was available,
 *         HAL_ERROR on I2C failure.
 */
HAL_StatusTypeDef ENS160_ReadAllData(ENS160_Data_t *data, ENS160_Resistance_t *resistance);

#ifdef __cplusplus
}
#endif

#endif /* __ENS160_H */
