/**
 * @file HDC302x.c
 * @brief Implementation file for HDC302x temperature and humidity sensor library.
 *
 * Contains functions for initializing, reading data, and configuring the HDC302x sensor.
 * Target: STM32F302, Keil MDK, STM32 HAL.
 */

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
