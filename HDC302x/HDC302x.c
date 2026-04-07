/***************************************************************************
 * @file [HDC302X].h/.c
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

#include "HDC302x.h"
#include "i2c.h"
#include <math.h>

/**
 * @brief Calculates CRC-8 (polynomial 0x31, init 0xFF) over 2 bytes.
 * @param data Pointer to 2-byte buffer.
 * @return Computed CRC byte.
 */
static uint8_t CalculateCRC(const uint8_t* data) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < 2; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }
    return crc;
}

/**
 * @brief Verifies the CRC of a [MSB, LSB, CRC] triplet received from the sensor.
 * @param data Pointer to 3-byte buffer [MSB, LSB, CRC].
 * @return HAL_OK if CRC matches, HAL_ERROR otherwise.
 */
static HAL_StatusTypeDef VerifyCRC(const uint8_t* data) {
    return (CalculateCRC(data) == data[2]) ? HAL_OK : HAL_ERROR;
}

/**
 * @brief Sends a 2-byte heater command followed by the 3-byte heater power payload.
 * @details Always transmits all 5 bytes — the heater config command always requires the power payload.
 *          Caller must set sensorObj->Cmd and sensorObj->Heater before calling.
 * @param sensorObj Pointer to sensor handle.
 * @return HAL status.
 */
static HAL_StatusTypeDef SendHeaterCommand(HDC302x_t* sensorObj) {
    uint8_t buffer[5];
    buffer[0] = (uint8_t)(sensorObj->Cmd.Command >> 8);
    buffer[1] = (uint8_t)(sensorObj->Cmd.Command & 0xFF);
    buffer[2] = sensorObj->Heater.MSB;
    buffer[3] = sensorObj->Heater.LSB;
    buffer[4] = sensorObj->Heater.HCRC;
    return HAL_I2C_Master_Transmit(&hi2c3, sensorObj->Address, buffer, 5, I2C_TIMEOUT);
}

/**
 * @brief Sends a 2-byte command with an optional 3-byte config payload to the sensor.
 * @details Command bytes are sent MSB first (big-endian, as required by HDC302x).
 *          The config payload is appended only when CFG_CRC is non-zero.
 *          Caller must set sensorObj->Cmd and sensorObj->Config before calling.
 * @param sensorObj Pointer to sensor handle.
 * @return HAL status.
 */
static HAL_StatusTypeDef SendDeviceCommand(HDC302x_t* sensorObj) {
    uint8_t buffer[5];  // 2 bytes command + up to 3 bytes config
    uint8_t len = 0;

    buffer[0] = (uint8_t)(sensorObj->Cmd.Command >> 8);
    buffer[1] = (uint8_t)(sensorObj->Cmd.Command & 0xFF);
    len = 2;

    // Append config payload only when CRC is non-zero (indicates a valid config is set)
    if (sensorObj->Config.CFG_CRC != 0x00) {
        buffer[2] = sensorObj->Config.CFG_MSB;
        buffer[3] = sensorObj->Config.CFG_LSB;
        buffer[4] = sensorObj->Config.CFG_CRC;
        len = 5;
    }

    return HAL_I2C_Master_Transmit(&hi2c3, sensorObj->Address, buffer, len, I2C_TIMEOUT);
}

/**
 * @brief Performs a hardware reset of the HDC302x sensor via the RST GPIO pin.
 * @details Drives RST low for 50 ms then high for 50 ms. Call before HDC302x_Init()
 *          when a full power-on reset sequence is required.
 */
void HDC302x_Reset() {
	HDC_RST_Off;
	HAL_Delay(50);
	HDC_RST_On;
	HAL_Delay(50);
}

/**
 * @brief Initializes the HDC302x sensor and starts continuous auto measurement at 4 Hz.
 * @details Performs the following sequence:
 *            1. Soft reset (HDC302X_CMD_SOFT_RESET).
 *            2. Reads the status register to confirm reset completed.
 *            3. Clears the status register (device_reset flag and any stale alerts).
 *            4. Programs the default power-on state to 4 Hz / lowest-noise
 *               (HDC302X_CONFIG_4HZ_LOWEST_NOISE via HDC302X_CMD_PROGRAM_READ_DEFAULT_STATE),
 *               stored in NVM so the sensor resumes auto measurement after a power cycle.
 *            5. Starts continuous auto measurement at 4 Hz, lowest noise
 *               (HDC302X_CMD_AUTO_MEASUREMENT_4_PER_SECOND_LPM0).
 * @param sensorObj Pointer to sensor handle. Caller must set Address before calling.
 * @return HAL status.
 */
HAL_StatusTypeDef HDC302x_Init(HDC302x_t* sensorObj) {
    HAL_StatusTypeDef status;

    // Step 1: Send soft reset
    sensorObj->Config.CFG_CRC = 0x00;  // No payload with the reset command
    sensorObj->Cmd = HDC302X_CMD_SOFT_RESET;
    status = SendDeviceCommand(sensorObj);
    if (status != HAL_OK) {
        return status;
    }
    HAL_Delay(2);  // 2 ms delay after reset (datasheet minimum is 1 ms)

    // Step 2: Read status register to confirm reset completed
    status = HDC302x_ReadStatus(sensorObj);
    if (status != HAL_OK) {
        return status;
    }

    // Step 3: Clear status register (clears device_reset flag and any stale alerts)
    status = HDC302x_ClearStatus(sensorObj);
    if (status != HAL_OK) {
        return status;
    }

    // Step 4: Program the default power-on measurement state to 4 Hz, lowest noise.
    // Stored in NVM so the sensor resumes auto measurement after a power cycle.
    sensorObj->Config = HDC302X_CONFIG_4HZ_LOWEST_NOISE;
    sensorObj->Cmd    = HDC302X_CMD_PROGRAM_READ_DEFAULT_STATE;
    status = SendDeviceCommand(sensorObj);
    if (status != HAL_OK) {
        return status;
    }

    // Step 5: Start continuous auto measurement at 4 Hz, lowest noise mode
    sensorObj->Config.CFG_CRC = 0x00;  // No config payload for the auto-measurement command
    return HDC302x_StartAutoMeasurement(sensorObj, HDC302X_CMD_AUTO_MEASUREMENT_4_PER_SECOND_LPM0);
}

/**
 * @brief Starts continuous auto measurement mode at the specified rate and noise level.
 * @details HDC302x_Init() already starts 4 Hz, lowest noise by default. Call this function
 *          only if a different rate or noise mode is needed at runtime. Use the predefined
 *          command macros, e.g.:
 *            - HDC302X_CMD_AUTO_MEASUREMENT_4_PER_SECOND_LPM0 — 4 Hz, lowest noise (default)
 *            - HDC302X_CMD_AUTO_MEASUREMENT_2_PER_SECOND_LPM2 — 2 Hz, low power mode 2
 *            - HDC302X_CMD_AUTO_MEASUREMENT_1_EVERY_2S_LPM3   — 0.5 Hz, lowest power
 * @param sensorObj Pointer to sensor handle.
 * @param cmd       Auto measurement command macro selecting rate and noise mode.
 * @return HAL status.
 */
HAL_StatusTypeDef HDC302x_StartAutoMeasurement(HDC302x_t* sensorObj, HDC302x_Command_t cmd) {
	sensorObj->Cmd = cmd;
	sensorObj->Config.CFG_CRC = 0x00;
	return SendDeviceCommand(sensorObj);
}

/**
 * @brief Reads temperature and humidity from the sensor output register.
 * @details Sends HDC302X_CMD_MEASURE_READ and receives 6 bytes:
 *          [T_MSB, T_LSB, T_CRC, RH_MSB, RH_LSB, RH_CRC].
 *          Both CRCs are verified before converting raw values.
 *          Conversion formulas (datasheet):
 *            Temperature (°C) = raw_T  * (165.0 / 65535.0) - 40.0
 *            Humidity    (%)  = raw_RH * (100.0 / 65535.0)
 *          Results are stored in sensorObj->Data.
 * @param sensorObj Pointer to sensor handle.
 * @return HAL status.
 */
HAL_StatusTypeDef HDC302x_ReadData(HDC302x_t* sensorObj) {
    HAL_StatusTypeDef status;
    uint8_t rx[6];  // [T_MSB, T_LSB, T_CRC, RH_MSB, RH_LSB, RH_CRC]

    // Send measurement read command
    sensorObj->Config.CFG_CRC = 0x00;
    sensorObj->Cmd = HDC302X_CMD_MEASURE_READ;
    status = SendDeviceCommand(sensorObj);
    if (status != HAL_OK) {
        return status;
    }

    // Receive 6 bytes
    status = HAL_I2C_Master_Receive(&hi2c3, sensorObj->Address, rx, sizeof(rx), I2C_TIMEOUT);
    if (status != HAL_OK) {
        return status;
    }

    // Verify CRC for temperature word and humidity word
    if (VerifyCRC(&rx[0]) != HAL_OK) {
        return HAL_ERROR;  // Temperature CRC mismatch
    }
    if (VerifyCRC(&rx[3]) != HAL_OK) {
        return HAL_ERROR;  // Humidity CRC mismatch
    }

    // Convert raw values to physical units
    uint16_t raw_t  = ((uint16_t)rx[0] << 8) | rx[1];
    uint16_t raw_rh = ((uint16_t)rx[3] << 8) | rx[4];

    sensorObj->Data.Temperature = ((float)raw_t  * HDC302X_TEMP_COEFF1) - HDC302X_TEMP_COEFF2;
    sensorObj->Data.Humidity    =  (float)raw_rh * HDC302X_RH_COEFF;

    // Clamp humidity to valid physical range [0, 100]
    if (sensorObj->Data.Humidity < 0.0f)    sensorObj->Data.Humidity = 0.0f;
    if (sensorObj->Data.Humidity > 100.0f)  sensorObj->Data.Humidity = 100.0f;

    return HAL_OK;
}

/**
 * @brief Reads the sensor status register.
 * @details Sends HDC302X_READ_STATUS and receives 3 bytes [MSB, LSB, CRC].
 *          CRC is verified before storing the result. Value is stored as
 *          (MSB << 8) | LSB so bitfield positions match the datasheet.
 *          Result is stored in sensorObj->Status.
 * @param sensorObj Pointer to sensor handle.
 * @return HAL status.
 */
HAL_StatusTypeDef HDC302x_ReadStatus(HDC302x_t* sensorObj) {
    HAL_StatusTypeDef status;
    uint8_t rx[3];  // [MSB, LSB, CRC]

    sensorObj->Config.CFG_CRC = 0x00;
    sensorObj->Cmd = HDC302X_READ_STATUS;
    status = SendDeviceCommand(sensorObj);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_I2C_Master_Receive(&hi2c3, sensorObj->Address, rx, sizeof(rx), I2C_TIMEOUT);
    if (status != HAL_OK) {
        return status;
    }

    if (VerifyCRC(rx) != HAL_OK) {
        return HAL_ERROR;  // Status CRC mismatch
    }

    sensorObj->Status.Val.Value = ((uint16_t)rx[0] << 8) | rx[1];
    return HAL_OK;
}

/**
 * @brief Clears the sensor status register.
 * @details Sends HDC302X_CLEAR_STATUS, which resets the device_reset flag
 *          and any latched alert flags in the status register.
 * @param sensorObj Pointer to sensor handle.
 * @return HAL status.
 */
HAL_StatusTypeDef HDC302x_ClearStatus(HDC302x_t* sensorObj) {
    sensorObj->Config.CFG_CRC = 0x00;
    sensorObj->Cmd = HDC302X_CLEAR_STATUS;
    return SendDeviceCommand(sensorObj);
}

/**
 * @brief Reads the min/max temperature and humidity history from the sensor.
 * @details Issues four sequential read commands to retrieve minimum and maximum
 *          recorded temperature and humidity values. Results are stored in
 *          sensorObj->History.MIN and sensorObj->History.MAX.
 * @param sensorObj Pointer to sensor handle.
 * @return HAL status.
 */
HAL_StatusTypeDef HDC302x_ReadHistory(HDC302x_t* sensorObj) {
    HAL_StatusTypeDef status;
    uint8_t rx[3];

    typedef struct {
        HDC302x_Command_t	cmd;
        float*				dest;
        float				coeff;
        float				offset;
    } HistoryRead_t;

    HistoryRead_t reads[4] = {
        { HDC302X_CMD_READ_HISTORY_MIN_TEMPERATURE, &sensorObj->History.MIN.Temperature, HDC302X_TEMP_COEFF1, -HDC302X_TEMP_COEFF2 },
        { HDC302X_CMD_READ_HISTORY_MAX_TEMPERATURE, &sensorObj->History.MAX.Temperature, HDC302X_TEMP_COEFF1, -HDC302X_TEMP_COEFF2 },
        { HDC302X_CMD_READ_HISTORY_MIN_RH,          &sensorObj->History.MIN.Humidity,    HDC302X_RH_COEFF,     0.0f               },
        { HDC302X_CMD_READ_HISTORY_MAX_RH,          &sensorObj->History.MAX.Humidity,    HDC302X_RH_COEFF,     0.0f               },
    };

    for (uint8_t i = 0; i < 4; i++) {
        sensorObj->Config.CFG_CRC = 0x00;
        sensorObj->Cmd = reads[i].cmd;

        status = SendDeviceCommand(sensorObj);
        if (status != HAL_OK) {
            return status;
        }

        status = HAL_I2C_Master_Receive(&hi2c3, sensorObj->Address, rx, sizeof(rx), I2C_TIMEOUT);
        if (status != HAL_OK) {
            return status;
        }

        if (VerifyCRC(rx) != HAL_OK) {
            return HAL_ERROR;  // CRC mismatch on history read
        }

        uint16_t raw = ((uint16_t)rx[0] << 8) | rx[1];
        *reads[i].dest = ((float)raw * reads[i].coeff) + reads[i].offset;
    }

    return HAL_OK;
}

/**
 * @brief Calculates the dew point temperature from the latest sensor reading.
 * @details Uses the Magnus-Tetens approximation:
 *            gamma = (A * T / (B + T)) + ln(RH / 100)
 *            Td    = B * gamma / (A - gamma)
 *          where A = DEW_POINT_CONST_A (17.27), B = DEW_POINT_CONST_B (237.7 °C).
 *          Requires HDC302x_ReadData() to have been called first.
 * @param sensorObj Pointer to sensor handle.
 * @return Dew point in degrees Celsius.
 */
float HDC302x_GetDewPoint(HDC302x_t* sensorObj) {
    float t     = sensorObj->Data.Temperature;
    float rh    = sensorObj->Data.Humidity;
    float gamma = ((DEW_POINT_CONST_A * t) / (DEW_POINT_CONST_B + t)) + logf(rh / 100.0f);
    return (DEW_POINT_CONST_B * gamma) / (DEW_POINT_CONST_A - gamma);
}

/**
 * @brief Configures the heater power level and enables the integrated heater.
 * @details Sends HDC302X_CMD_HEATER_CONFIG with the power payload via SendHeaterCommand,
 *          then sends HDC302X_CMD_HEATER_ENABLE via SendDeviceCommand.
 *          Use a predefined power level macro such as HDC302X_POW_HALF for safe
 *          condensation removal (gentle ramp per datasheet sec 7.3.7;
 *          full power risks bursting condensed droplets).
 * @param sensorObj Pointer to sensor handle.
 * @param power     Heater power level. Use a predefined HDC302X_POW_* macro.
 * @return HAL status.
 */
HAL_StatusTypeDef HDC302x_HeaterEnable(HDC302x_t* sensorObj, HDC302x_HeaterConfig_t power) {
    HAL_StatusTypeDef status;

    // Step 1: Send heater config command with power payload
    sensorObj->Cmd    = HDC302X_CMD_HEATER_CONFIG;
    sensorObj->Heater = power;
    status = SendHeaterCommand(sensorObj);
    if (status != HAL_OK) return status;

    // Step 2: Enable heater (no payload)
    sensorObj->Config.CFG_CRC = 0x00;
    sensorObj->Cmd = HDC302X_CMD_HEATER_ENABLE;
    return SendDeviceCommand(sensorObj);
}

/**
 * @brief Disables the integrated heater.
 * @details Sends HDC302X_CMD_HEATER_DISABLE (0x3066) with no payload.
 * @param sensorObj Pointer to sensor handle.
 * @return HAL status.
 */
HAL_StatusTypeDef HDC302x_HeaterDisable(HDC302x_t* sensorObj) {
	sensorObj->Config.CFG_CRC = 0x00;
	sensorObj->Cmd = HDC302X_CMD_HEATER_DISABLE;
	return SendDeviceCommand(sensorObj);
}

/**
 * @brief Advances the condensation removal state machine.
 * @details Call on every successful HDC302x_ReadData(). Monitors the dew point
 *          margin and drives the heater through the OFF → ACTIVE → COOLING → OFF cycle.
 *          Per datasheet sec 7.3.7:
 *            - Ramp to 100 °C must take 5–10 s → use HDC302X_POW_HALF (safe ramp rate).
 *            - Monitor RH; stop when RH is near 0 % (condensate evaporated).
 *            - Let the device cool before returning to normal service.
 *          Do not use T/RH for atmospheric calculations while HeaterState != HDC_HEATER_OFF,
 *          as readings are elevated during heating and cooling.
 * @param sensorObj Pointer to sensor handle.
 * @return Current HDC302x_HeaterState_t so the caller can suppress atmospheric data if needed.
 */
HDC302x_HeaterState_t HDC302x_UpdateHeater(HDC302x_t* sensorObj) {
	float t		= sensorObj->Data.Temperature;
	float rh	= sensorObj->Data.Humidity;
	float td	= HDC302x_GetDewPoint(sensorObj);
	uint32_t now	= HAL_GetTick();

	switch (sensorObj->HeaterState) {

	case HDC_HEATER_OFF:
		// Activate heater when temperature approaches dew point
		if ((t - td) < DEW_MARGIN_C) {
			sensorObj->AmbientTemp  = t;
			sensorObj->HeaterStartMs = now;
			HDC302x_HeaterEnable(sensorObj, HEATER_POWER);
			sensorObj->HeaterState = HDC_HEATER_ACTIVE;
		}
		break;

	case HDC_HEATER_ACTIVE:
		// Stop heater when condensate is gone (RH near 0%) or timeout reached
		if ((rh < HEATER_RH_CLEAR) || ((now - sensorObj->HeaterStartMs) >= HEATER_TIMEOUT_MS)) {
			HDC302x_HeaterDisable(sensorObj);
			sensorObj->HeaterStartMs = now;  // Reuse timer for cooling phase
			sensorObj->HeaterState = HDC_HEATER_COOLING;
		}
		break;

	case HDC_HEATER_COOLING:
		// Return to normal when sensor has cooled back to near ambient or timeout
		if ((t <= (sensorObj->AmbientTemp + 5.0f)) ||
			((now - sensorObj->HeaterStartMs) >= HEATER_COOLING_MS)) {
			sensorObj->HeaterState = HDC_HEATER_OFF;
		}
		break;

	default:
		sensorObj->HeaterState = HDC_HEATER_OFF;
		break;
	}

	return sensorObj->HeaterState;
}

/**
 * @brief Encodes a temperature and relative-humidity offset into the HDC302x packed register format.
 * @details The device stores both offsets as a single 16-bit word plus CRC (command 0xA004).
 *          Bit layout (MSB to LSB):
 *            Bit 15    : RH sign  (1 = add, 0 = subtract)
 *            Bits 14-8 : RH_OS[6:0], LSB = 0.1953125 %RH,  max ±24.8046875 %RH
 *            Bit  7    : T  sign  (1 = add, 0 = subtract)
 *            Bits  6-0 : T_OS[6:0],  LSB = 0.1708984375 °C, max ±21.7041015625 °C
 *          Values outside the representable range are clamped silently.
 * @param temp_offset_c   Desired temperature correction in °C (negative to subtract).
 * @param rh_offset_pct   Desired RH correction in %RH        (negative to subtract).
 * @param out             Output structure to receive MSB, LSB, and CRC bytes.
 * @return HAL_OK always (clamping handles out-of-range inputs).
 */
HAL_StatusTypeDef HDC302x_EncodeOffset(float temp_offset_c, float rh_offset_pct, HDC302x_Offset_t* out) {
    // --- Temperature offset ---
    uint8_t t_sign = (temp_offset_c >= 0.0f) ? 1u : 0u;
    float   t_abs  = (temp_offset_c >= 0.0f) ? temp_offset_c : -temp_offset_c;
    // LSB resolution = 175/65535 * 2^6 ≈ 0.1708984375 °C, 7 bits max
    uint8_t t_bits = (uint8_t)(t_abs / 0.1708984375f + 0.5f);
    if (t_bits > 0x7F) t_bits = 0x7F;  // Clamp to 7 bits (max ≈ 21.7 °C)

    // --- Relative humidity offset ---
    uint8_t rh_sign = (rh_offset_pct >= 0.0f) ? 1u : 0u;
    float   rh_abs  = (rh_offset_pct >= 0.0f) ? rh_offset_pct : -rh_offset_pct;
    // LSB resolution = 100/65535 * 2^9 ≈ 0.1953125 %RH, 7 bits max
    uint8_t rh_bits = (uint8_t)(rh_abs / 0.1953125f + 0.5f);
    if (rh_bits > 0x7F) rh_bits = 0x7F;  // Clamp to 7 bits (max ≈ 24.8 %RH)

    // Pack: MSB = [RH+/-, RH6..RH0], LSB = [T+/-, T6..T0]
    out->MSB = (uint8_t)((rh_sign << 7) | (rh_bits & 0x7F));
    out->LSB = (uint8_t)((t_sign  << 7) | (t_bits  & 0x7F));

    uint8_t tmp[2] = { out->MSB, out->LSB };
    out->C_RC = CalculateCRC(tmp);

    return HAL_OK;
}

/**
 * @brief Programs a temperature and RH measurement offset into the sensor's non-volatile memory.
 * @details The device must be in sleep mode while the offset is written (the function temporarily
 *          stops auto-measurement, writes the offset, waits 50 ms for EEPROM programming, then
 *          restarts auto-measurement at 4 Hz / lowest noise).  If a different measurement rate is
 *          needed after the call, follow up with HDC302x_StartAutoMeasurement().
 *
 *          Example — correct a +3 °C self-heating error:
 *            HDC302x_SetOffset(&sensor, -3.0f, 0.0f);
 *
 * @param sensorObj       Pointer to sensor handle.
 * @param temp_offset_c   Temperature correction in °C (negative subtracts).
 * @param rh_offset_pct   RH correction in %RH          (negative subtracts, 0.0 for none).
 * @return HAL status.
 */
HAL_StatusTypeDef HDC302x_SetOffset(HDC302x_t* sensorObj, float temp_offset_c, float rh_offset_pct) {
    HAL_StatusTypeDef status;

    // Step 1: Return to sleep (Trigger-On-Demand base state) so EEPROM can be written
    sensorObj->Config.CFG_CRC = 0x00;
    sensorObj->Cmd = HDC302X_CMD_RETURN_TO_TRIGGER;
    status = SendDeviceCommand(sensorObj);
    if (status != HAL_OK) return status;
    HAL_Delay(1);  // Settle before EEPROM access

    // Step 2: Encode the offset
    HDC302x_Offset_t offset;
    HDC302x_EncodeOffset(temp_offset_c, rh_offset_pct, &offset);

    // Step 3: Send offset command (0xA0 0x04) followed by [MSB, LSB, CRC]
    uint8_t buffer[5];
    buffer[0] = 0xA0;       // Command MSB
    buffer[1] = 0x04;       // Command LSB
    buffer[2] = offset.MSB;
    buffer[3] = offset.LSB;
    buffer[4] = offset.C_RC;
    status = HAL_I2C_Master_Transmit(&hi2c3, sensorObj->Address, buffer, 5, I2C_TIMEOUT);
    if (status != HAL_OK) return status;

    // Step 4: Wait for EEPROM programming to complete (datasheet: t_PROG, I2C blocked during this)
    HAL_Delay(50);

    // Step 5: Resume 4 Hz / lowest-noise auto measurement
    return HDC302x_StartAutoMeasurement(sensorObj, HDC302X_CMD_AUTO_MEASUREMENT_4_PER_SECOND_LPM0);
}

/**
 * @brief Encodes a (RH, temperature) pair into the HDC302x packed ALERT threshold format.
 * @details Threshold format (datasheet sec 7.5.7.4.2):
 *            Bits 15-9 : 7 MSBs of the 16-bit RH raw value  (≈1 %RH  resolution)
 *            Bits  8-0 : 9 MSBs of the 16-bit T  raw value  (≈0.5 °C resolution)
 *          Temperature range: −45 °C to +125 °C.  RH range: 0 % to 100 %.
 * @param rh_pct  Relative humidity threshold in %RH.
 * @param temp_c  Temperature threshold in °C.
 * @param out     Output structure to receive MSB, LSB, and CRC bytes.
 * @return HAL_OK always (values are clamped to valid sensor range).
 */
HAL_StatusTypeDef HDC302x_EncodeAlertThreshold(float rh_pct, float temp_c, HDC302x_AlertThreshold_t* out) {
    // Clamp to sensor range
    if (rh_pct  <   0.0f) rh_pct  =   0.0f;
    if (rh_pct  > 100.0f) rh_pct  = 100.0f;
    if (temp_c  < -45.0f) temp_c  = -45.0f;
    if (temp_c  > 125.0f) temp_c  = 125.0f;

    // Convert to 16-bit raw values (inverse of datasheet formulas)
    uint16_t raw_rh = (uint16_t)(rh_pct  / 100.0f  * 65535.0f + 0.5f);
    uint16_t raw_t  = (uint16_t)((temp_c + 45.0f) / 175.0f * 65535.0f + 0.5f);

    // Keep only 7 MSBs of RH and 9 MSBs of T
    uint16_t rh7 = (raw_rh >> 9) & 0x7Fu;   // bits [15:9] → bits [6:0]
    uint16_t t9  = (raw_t  >> 7) & 0x1FFu;  // bits [15:7] → bits [8:0]

    // Pack into 16-bit threshold word: [RH6..RH0, T8..T0]
    uint16_t word = (uint16_t)((rh7 << 9) | t9);

    out->MSB = (uint8_t)(word >> 8);
    out->LSB = (uint8_t)(word & 0xFF);
    uint8_t tmp[2] = { out->MSB, out->LSB };
    out->CR_C = CalculateCRC(tmp);

    return HAL_OK;
}

/**
 * @brief Configures all four ALERT thresholds on the sensor.
 * @details Programs Set High, Clear High, Set Low, and Clear Low thresholds in sequence.
 *          Pre-built thresholds from HDC302x_EncodeAlertThreshold() or the convenience
 *          HDC302X_ALERT_DATA_READY_* macros can be passed directly.
 *
 *          Meteo station "data-ready interrupt" usage:
 *            HDC302x_AlertConfig_t alertCfg = {
 *                .SetHigh   = HDC302X_ALERT_DATA_READY_SET_HIGH,
 *                .ClearHigh = HDC302X_ALERT_DATA_READY_CLR_HIGH,
 *                .SetLow    = HDC302X_ALERT_DATA_READY_SET_LOW,
 *                .ClearLow  = HDC302X_ALERT_DATA_READY_CLR_LOW,
 *            };
 *            HDC302x_ConfigureAlert(&sensor, &alertCfg);
 *          The ALERT GPIO EXTI ISR should then call HDC302x_AlertCallback().
 *
 * @param sensorObj Pointer to sensor handle.
 * @param cfg       Pointer to structure holding all four pre-encoded thresholds.
 * @return HAL status.
 */
HAL_StatusTypeDef HDC302x_ConfigureAlert(HDC302x_t* sensorObj, const HDC302x_AlertConfig_t* cfg) {
    typedef struct {
        HDC302x_Command_t          cmd;
        const HDC302x_AlertThreshold_t* thr;
    } AlertWrite_t;

    AlertWrite_t writes[4] = {
        { HDC302X_CMD_CONFIGURE_ALERT_THRESHOLD_SET_HIGH,   &cfg->SetHigh   },
        { HDC302X_CMD_CONFIGURE_ALERT_THRESHOLD_CLEAR_HIGH, &cfg->ClearHigh },
        { HDC302X_CMD_CONFIGURE_ALERT_THRESHOLD_SET_LOW,    &cfg->SetLow    },
        { HDC302X_CMD_CONFIGURE_ALERT_THRESHOLD_CLEAR_LOW,  &cfg->ClearLow  },
    };

    for (uint8_t i = 0; i < 4; i++) {
        uint8_t buffer[5];
        buffer[0] = (uint8_t)(writes[i].cmd.Command >> 8);
        buffer[1] = (uint8_t)(writes[i].cmd.Command & 0xFF);
        buffer[2] = writes[i].thr->MSB;
        buffer[3] = writes[i].thr->LSB;
        buffer[4] = writes[i].thr->CR_C;

        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c3, sensorObj->Address, buffer, 5, I2C_TIMEOUT);
        if (status != HAL_OK) return status;
    }

    return HAL_OK;
}

/**
 * @brief Weak callback invoked from the ALERT GPIO EXTI interrupt handler.
 * @details Override this function in application code to react to ALERT events
 *          (data-ready or out-of-range measurements).  The default implementation
 *          is empty so linking succeeds without an override.
 *
 *          Typical override in main.c or sensor task:
 *            void HDC302x_AlertCallback(HDC302x_t* sensorObj) {
 *                HDC302x_ReadData(sensorObj);
 *                // process sensorObj->Data ...
 *                HDC302x_ClearStatus(sensorObj); // deassert ALERT
 *            }
 *
 *          Wire to EXTI in stm32g4xx_it.c (ALERT pin example: PA5):
 *            void EXTI9_5_IRQHandler(void) {
 *                HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
 *            }
 *            void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
 *                if (GPIO_Pin == GPIO_PIN_5) {
 *                    HDC302x_AlertCallback(&hdc_sensor1);
 *                }
 *            }
 *
 * @param sensorObj Pointer to the sensor handle associated with the ALERT pin.
 */
__attribute__((weak)) void HDC302x_AlertCallback(HDC302x_t* sensorObj) {
    (void)sensorObj;  // Default: do nothing. Override in application.
}
