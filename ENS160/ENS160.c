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

#include "ENS160.h"
#include "i2c.h"
#include <math.h>

// Global register shadow and data stores.
// 'volatile' is required because fields may be written in main context and read in ISR context.
static volatile ENS160_Registers_t  ens160_regs;
static          ENS160_FW_t         ens160_fw;

// ─── Static helpers ───────────────────────────────────────────────────────────

/**
 * @brief Execute the command currently stored in ens160_regs.COMMAND.
 * @details Writes COMMAND to register 0x12. The ENS160 only acts on commands
 *          while in IDLE mode. The caller is responsible for ensuring the correct
 *          operating mode before calling this function.
 * @return HAL status.
 */
static HAL_StatusTypeDef ENS160_ExecuteCommand(void) {
    uint8_t cmd = (uint8_t)ens160_regs.COMMAND;
    return HAL_I2C_Mem_Write(&hi2c3, ENS160_I2C_ADDRESS, ENS160_REG_COMMAND,
                             I2C_MEMADD_SIZE_8BIT, &cmd, ENS160_SIZE1, TIMEOUT_COMM);
}

// ─── Public functions ─────────────────────────────────────────────────────────

// REG 0x00-0x01 - Part ID
/**
 * @brief ENS160_ReadPartID function implementation.
 * @details Reads 2 bytes from register 0x00 and verifies the value equals
 *          ENS160_PART_ID (0x0160, little-endian). Returns HAL_ERROR if the
 *          ID does not match, indicating a wrong device or I2C fault.
 * @return HAL status.
 */
HAL_StatusTypeDef ENS160_ReadPartID(void) {
    uint16_t part_id = 0;
    if (HAL_I2C_Mem_Read(&hi2c3, ENS160_I2C_ADDRESS, ENS160_REG_PART_ID,
                         I2C_MEMADD_SIZE_8BIT, (uint8_t *)&part_id,
                         ENS160_SIZE2, TIMEOUT_COMM) != HAL_OK) {
        return HAL_ERROR;
    }
    return (part_id == ENS160_PART_ID) ? HAL_OK : HAL_ERROR;
}

// REG 0x10 - Operating Mode
/**
 * @brief ENS160_SetOperatingMode function implementation.
 * @details Writes the requested operating mode to OPMODE register (0x10) and
 *          updates the shadow register.
 * @param mode Desired operating mode.
 * @return HAL status.
 */
HAL_StatusTypeDef ENS160_SetOperatingMode(ENS160_OPMODE_t mode) {
    uint8_t val = (uint8_t)mode;
    if (HAL_I2C_Mem_Write(&hi2c3, ENS160_I2C_ADDRESS, ENS160_REG_OPMODE,
                          I2C_MEMADD_SIZE_8BIT, &val,
                          ENS160_SIZE1, TIMEOUT_COMM) != HAL_OK) {
        return HAL_ERROR;
    }
    ens160_regs.OPMODE = mode;
    return HAL_OK;
}

// REG 0x11 - CONFIG
/**
 * @brief Write the interrupt configuration register (0x11) from the shadow register.
 * @return HAL status.
 */
static HAL_StatusTypeDef ENS160_SetConfig(void) {
    return HAL_I2C_Mem_Write(&hi2c3, ENS160_I2C_ADDRESS, ENS160_REG_CONFIG,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t *)(void *)&ens160_regs.CONFIG.Val,
                             ENS160_SIZE1, TIMEOUT_COMM);
}

// REG 0x12 - COMMAND
/**
 * @brief ENS160_GetFirmwareVersion function implementation.
 * @details Issues GET_APPVER command (0x0E) in IDLE mode, then reads the
 *          firmware version from GPR_READ[4:6]:
 *            GPR_READ[4] = Major version
 *            GPR_READ[5] = Minor version
 *            GPR_READ[6] = Release version
 *          The sensor must already be in IDLE mode when this is called.
 * @return HAL status.
 */
HAL_StatusTypeDef ENS160_GetFirmwareVersion(void) {
    ens160_regs.COMMAND = ENS160_COMMAND_GET_APPVER;
    if (ENS160_ExecuteCommand() != HAL_OK) return HAL_ERROR;
    HAL_Delay(ENS160_CMD_DELAY_MS);

    uint8_t gpr[ENS160_SIZE8] = {0};
    if (HAL_I2C_Mem_Read(&hi2c3, ENS160_I2C_ADDRESS, ENS160_REG_GPR_READ,
                         I2C_MEMADD_SIZE_8BIT, gpr, ENS160_SIZE8,
                         TIMEOUT_COMM) != HAL_OK) {
        return HAL_ERROR;
    }

    ens160_fw.Major   = gpr[4];
    ens160_fw.Minor   = gpr[5];
    ens160_fw.Release = gpr[6];
    return HAL_OK;
}

// REG 0x13-0x14 (TEMP_IN) + 0x15-0x16 (RH_IN)
/**
 * @brief ENS160_SetEnvCompensation function implementation.
 * @details Converts temperature and humidity to the ENS160 fixed-point formats
 *          and writes them to the compensation input registers:
 *            TEMP_IN = (temperatureC + 273.15) * 64           [0x13-0x14]
 *            RH_IN   = humidityPercent * 512                   [0x15-0x16]
 *          Both registers are written individually as 2-byte little-endian values.
 * @param temperatureC    Ambient temperature in degrees Celsius.
 * @param humidityPercent Relative humidity in percent.
 * @return HAL status.
 */
HAL_StatusTypeDef ENS160_SetEnvCompensation(float temperatureC, float humidityPercent) {
    uint16_t temp_raw = (uint16_t)((temperatureC + 273.15f) * 64.0f);
    uint16_t rh_raw   = (uint16_t)(humidityPercent * 512.0f);

    ens160_regs.TEMP_IN = temp_raw;
    ens160_regs.RH_IN   = rh_raw;

    if (HAL_I2C_Mem_Write(&hi2c3, ENS160_I2C_ADDRESS, ENS160_REG_TEMP_IN,
                          I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ens160_regs.TEMP_IN,
                          ENS160_SIZE2, TIMEOUT_COMM) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_I2C_Mem_Write(&hi2c3, ENS160_I2C_ADDRESS, ENS160_REG_RH_IN,
                          I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ens160_regs.RH_IN,
                          ENS160_SIZE2, TIMEOUT_COMM) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

// REG 0x20 - DEVICE_STATUS
/**
 * @brief ENS160_ReadDeviceStatus function implementation.
 * @details Reads 1 byte from the DEVICE_STATUS register (0x20) into
 *          ens160_regs.STATUS.
 * @return HAL status.
 */
HAL_StatusTypeDef ENS160_ReadDeviceStatus(void) {
    return HAL_I2C_Mem_Read(&hi2c3, ENS160_I2C_ADDRESS, ENS160_REG_STATUS,
                            I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ens160_regs.STATUS.Val,
                            ENS160_SIZE1, TIMEOUT_COMM);
}

// REG 0x21 (DATA_AQI) + 0x22-0x23 (DATA_TVOC) + 0x24-0x25 (DATA_ECO2)
// REG 0x48-0x4F (GPR_READ) — R1 = GPR_READ[0:1], R4 = GPR_READ[6:7]
/**
 * @brief ENS160_ReadAllData function implementation.
 * @details Polls DEVICE_STATUS for new data, then if available:
 *          1. Reads 5 bytes starting at DATA_AQI (0x21) in a single burst
 *             to populate AQI, TVOC and eCO2 fields in *data.
 *          2. Reads 8 bytes of GPR_READ (0x48) to extract raw sensor resistances.
 *             Only R1 (GPR_READ[0:1]) and R4 (GPR_READ[6:7]) are documented
 *             by the datasheet; both are converted using: R = 2^(Rraw / 2048).
 * @param data       Pointer to ENS160_Data_t to receive AQI, TVOC, eCO2.
 * @param resistance Pointer to ENS160_Resistance_t to receive R1 and R4 in Ohms.
 * @return HAL_OK    — new data successfully read.
 *         HAL_BUSY  — no new data available (NEWDAT and NEWGPR both clear); caller should retry.
 *         HAL_ERROR — I2C communication failure.
 */
HAL_StatusTypeDef ENS160_ReadAllData(ENS160_Data_t *data, ENS160_Resistance_t *resistance) {
    if (ENS160_ReadDeviceStatus() != HAL_OK) return HAL_ERROR;

    if (!ens160_regs.STATUS.Val.BitField.NEWDAT && !ens160_regs.STATUS.Val.BitField.NEWGPR) {
        return HAL_BUSY;  // No new data — not an error; caller should poll again later
    }

    // Burst read DATA_AQI (0x21) through DATA_ECO2 MSB (0x25) — 5 bytes
    if (HAL_I2C_Mem_Read(&hi2c3, ENS160_I2C_ADDRESS, ENS160_REG_DATA_AQI,
                         I2C_MEMADD_SIZE_8BIT, data->Buffer,
                         ENS160_SIZE5, TIMEOUT_COMM) != HAL_OK) {
        return HAL_ERROR;
    }

    // Read GPR_READ registers (0x48-0x4F) — 8 bytes
    uint8_t gpr[ENS160_SIZE8] = {0};
    if (HAL_I2C_Mem_Read(&hi2c3, ENS160_I2C_ADDRESS, ENS160_REG_GPR_READ,
                         I2C_MEMADD_SIZE_8BIT, gpr,
                         ENS160_SIZE8, TIMEOUT_COMM) != HAL_OK) {
        return HAL_ERROR;
    }

    // R1 = GPR_READ[0:1], R4 = GPR_READ[6:7] per datasheet Table 8.
    // Raw value is a 16-bit little-endian unsigned integer.
    // Physical resistance (Ohms): R = 2^(Rraw / 2048)
    uint16_t r1raw = (uint16_t)(gpr[0] | ((uint16_t)gpr[1] << 8));
    uint16_t r4raw = (uint16_t)(gpr[6] | ((uint16_t)gpr[7] << 8));
    resistance->R1 = (uint32_t)powf(2.0f, (float)r1raw / 2048.0f);
    resistance->R4 = (uint32_t)powf(2.0f, (float)r4raw / 2048.0f);

    return HAL_OK;
}

// ─── Initialization ───────────────────────────────────────────────────────────

/**
 * @brief ENS160_Init function implementation.
 * @details Performs a full sensor startup sequence:
 *          1.  Reset the sensor by writing OPMODE = RESET, then wait for power-up.
 *          2.  Verify PART_ID equals 0x0160.
 *          3.  Enter IDLE mode (required for commands and configuration writes).
 *          4.  Clear GPR registers via CLRGPR command.
 *          5.  Read firmware version via GET_APPVER command.
 *          6.  Configure interrupt pin — polling only (no INTn assertion).
 *          7.  Set default environmental compensation (25 °C, 50 %RH).
 *          8.  Start standard gas sensing by writing OPMODE = STANDARD.
 * @return HAL status.
 */
HAL_StatusTypeDef ENS160_Init(void) {
    // Step 1: Reset
    if (ENS160_SetOperatingMode(ENS160_OPMODE_RESET) != HAL_OK) return HAL_ERROR;
    HAL_Delay(ENS160_RESET_DELAY_MS);

    // Step 2: Verify part ID
    if (ENS160_ReadPartID() != HAL_OK) return HAL_ERROR;

    // Step 3: Enter IDLE mode
    if (ENS160_SetOperatingMode(ENS160_OPMODE_IDLE) != HAL_OK) return HAL_ERROR;

    // Step 4: Clear GPR registers
    ens160_regs.COMMAND = ENS160_COMMAND_CLRGPR;
    if (ENS160_ExecuteCommand() != HAL_OK) return HAL_ERROR;
    HAL_Delay(ENS160_CLRGPR_DELAY_MS);

    // Step 5: Read firmware version
    if (ENS160_GetFirmwareVersion() != HAL_OK) return HAL_ERROR;

    // Step 6: Configure interrupt pin — all interrupts disabled, pure polling mode.
    // Remove or update these lines if interrupt-driven data ready is implemented.
    ens160_regs.CONFIG.Val.BitField.INTEN     = 0;  // INTn pin disabled
    ens160_regs.CONFIG.Val.BitField.INTDAT    = 0;  // No assertion on DATA update
    ens160_regs.CONFIG.Val.BitField.INTGPR    = 0;  // No assertion on GPR update
    ens160_regs.CONFIG.Val.BitField.INT_CFG   = 0;  // Open-drain (irrelevant when disabled)
    ens160_regs.CONFIG.Val.BitField.INTPOL    = 0;  // Active low (irrelevant when disabled)
    if (ENS160_SetConfig() != HAL_OK) return HAL_ERROR;

    // Step 7: Set default environmental compensation
    if (ENS160_SetEnvCompensation(25.0f, 50.0f) != HAL_OK) return HAL_ERROR;

    // Step 8: Start standard gas sensing
    if (ENS160_SetOperatingMode(ENS160_OPMODE_STANDARD) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}
