#include "AS3935.h"
#include "i2c.h"
#include "gpio.h"
#include "tim.h"
#include "debug.h"
#include <stdio.h>
#include <stdlib.h>
static AS3935_REGS_t AS3935_Sensor;
extern TIM_HandleTypeDef htim2;

static inline uint32_t GetTimerClock(TIM_HandleTypeDef* htim) {
    // TIM2 is on APB1 170MHz
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();

    // If APB1 prescaler is not DIV1, timer clock is doubled
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
        return pclk1 * 2;
    else
        return pclk1;
}

/**
 * @brief Write data to a register of the AS3935 sensor.
 *
 * @param reg Register address to write to.
 * @param data Pointer to the data buffer to be written.
 * @param len Length of data to write.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
static HAL_StatusTypeDef AS3935_WriteRegister(uint8_t reg, uint8_t* data, uint8_t len) {
    return WriteRegister(AS3935_I2C_ADDRESS, reg, data, len, &hi2c1);
}

/**
 * @brief Read data from a register of the AS3935 sensor.
 *
 * @param reg Register address to read from.
 * @param data Pointer to the data buffer to store the read data.
 * @param len Length of data to read.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
static HAL_StatusTypeDef AS3935_ReadRegister(uint8_t reg, uint8_t* data, uint8_t len) {
    return ReadRegister(AS3935_I2C_ADDRESS, reg, data, len, &hi2c1);
}

/**
 * @brief Sends a direct command to the AS3935 lightning sensor.
 * 
 * Direct commands are used for specific operations, such as oscillator calibration or reset.
 * The command is written to the `AS3935_DIRECT_COMMAND` address to trigger the desired action.
 * 
 * @param commandReg The direct command to send (refer to AS3935 command definitions).
 * @retval HAL_OK     Command sent successfully.
 * @retval HAL_ERROR  Communication failure.
 */
static inline HAL_StatusTypeDef AS3935_SendDirectCommand(uint8_t commandReg) {
    // Write the direct command (0x96) to the specified register
		return AS3935_WriteRegister(commandReg, (uint8_t*)AS3935_DIRECT_COMMAND, 1);
}

/**
 * @brief Initializes the AS3935 lightning sensor with default settings.
 * 
 * This function configures the AS3935 sensor using predefined defaults:
 * - Wakes up the sensor.
 * - Configures noise floor level and detection efficiency.
 * - Configures power state and AFE gain.
 * - Sets lightning statistics.
 * - Configures interrupts and frequency.
 * - Calibrates oscillators.
 * 
 * @retval HAL_OK     Initialization successful.
 * @retval HAL_ERROR  Initialization failed at any step.
 */
HAL_StatusTypeDef AS3935_Init(void) {
    uint8_t verify;

    // Step 1: Reset sensor to default values
    if (AS3935_SendDirectCommand(PRESET_DEFAULT) != HAL_OK) return HAL_ERROR;
    HAL_Delay(AS3935_CMD_DELAY);

    // Step 2: Wake up sensor and set OUTDOOR gain
    AS3935_Sensor.POWER.Val.BitField.PWD = AS3935_POWER_UP;
    AS3935_Sensor.POWER.Val.BitField.GAIN = AFE_GAIN_OUTDOOR; // 0x09
    if (AS3935_WriteRegister(AFE_GAIN, &AS3935_Sensor.POWER.Val.Value, 1) != HAL_OK) return HAL_ERROR;
    if (AS3935_ReadRegister(AFE_GAIN, &verify, 1) != HAL_OK) return HAL_ERROR;
    DEBUG("REG 0x00 [GAIN] written: 0x%02X, readback: 0x%02X\n", AS3935_Sensor.POWER.Val.Value, verify);
    HAL_Delay(AS3935_CMD_DELAY);

    // Step 3: Tune Antenna (e.g., to 88 pF)
    if (AS3935_setTuningCapacitor(TUN_CAP_96pF) != HAL_OK) return HAL_ERROR;//TUN_CAP_88pF //Tune_Antenna()

    // Step 4: Configure for high sensitivity (low NF, high WDTH)
    AS3935_Sensor.NOISE.Val.BitField.NF_LEV = NOISE_FLOOR_390uVrms; // 0
    AS3935_Sensor.NOISE.Val.BitField.WDTH = EFFICIENCY_100;         // 15
    if (AS3935_WriteRegister(THRESHOLD, &AS3935_Sensor.NOISE.Val.Value, 1) != HAL_OK) return HAL_ERROR;
    if (AS3935_ReadRegister(THRESHOLD, &verify, 1) != HAL_OK) return HAL_ERROR;
    DEBUG("REG 0x01 [NOISE] written: 0x%02X, readback: 0x%02X\n", AS3935_Sensor.NOISE.Val.Value, verify);

    // Step 5: Configure lightning detection statistics
    AS3935_Sensor.STATISTICS.Val.BitField.SREJ = SREJ_LEVEL_2;               // moderate spike rejection
    AS3935_Sensor.STATISTICS.Val.BitField.MIN_NUM_LIGH = MIN_NUM_LIGHTNING_1; // fastest response
    if (AS3935_ClearStatistics(CLEAR_STAT_ENABLED) != HAL_OK) return HAL_ERROR;
    if (AS3935_ReadRegister(LIGHTNING_REG, &verify, 1) != HAL_OK) return HAL_ERROR;
    DEBUG("REG 0x02 [STATS] written: 0x%02X, readback: 0x%02X\n", AS3935_Sensor.STATISTICS.Val.Value, verify);

    // Step 6: Configure interrupt masking and frequency divider
    AS3935_Sensor.INT_FREQ.Val.Value = 0;
    AS3935_Sensor.INT_FREQ.Val.BitField.MASK_DIST = MASK_DIST_DISABLED; // allow disturbers
    AS3935_Sensor.INT_FREQ.Val.BitField.LCO_FDIV = LCO_FDIV_RATIO_16;
    uint8_t irqCfg = AS3935_Sensor.INT_FREQ.Val.Value & 0xF0;
    if (AS3935_WriteRegister(INT_MASK_ANT, &irqCfg, 1) != HAL_OK) return HAL_ERROR;
    if (AS3935_ReadRegister(INT_MASK_ANT, &verify, 1) != HAL_OK) return HAL_ERROR;
    DEBUG("REG 0x03 [INT_FREQ] written: 0x%02X, readback: 0x%02X\n", irqCfg, verify);

    // Step 7: Calibrate internal oscillators
    if (AS3935_SendDirectCommand(CALIB_RCO) != HAL_OK) return HAL_ERROR;
    if (AS3935_SetOscillatorDisplay(OSC_DISPLAY_TRCO_ONLY) != HAL_OK) return HAL_ERROR;
    HAL_Delay(20);
    if (AS3935_SetOscillatorDisplay(OSC_DISPLAY_DISABLED) != HAL_OK) return HAL_ERROR;

    // Step 8: Verify calibration success
//    AS3935_ReadRegister(CALIB_SRCO, &AS3935_Sensor.SRCO.Val.Value, 1);
//		AS3935_ReadRegister(CALIB_TRCO, &AS3935_Sensor.TRCO.Val.Value, 1);

//    // Step 9: Enable external interrupt if calibration passed
//		if ((AS3935_Sensor.SRCO.Val.Value & 0xC0) == 0x80 && (AS3935_Sensor.TRCO.Val.Value & 0xC0) == 0x80) {
//				AS3935_ReadAllRegisters();
//				return HAL_OK;
//		}

    return HAL_OK; // Calibration failed
}

/**
 * @brief Reads all the AS3935 sensor registers into the AS3935_Sensor structure.
 *
 * @retval HAL_OK     Operation was successful, and all registers were read.
 * @retval HAL_ERROR  Reading one or more registers failed.
 */
HAL_StatusTypeDef AS3935_ReadAllRegisters(void) {
    HAL_StatusTypeDef status;

    // Read register 0x00 (POWER)
    status = AS3935_ReadRegister(AFE_GAIN, &AS3935_Sensor.POWER.Val.Value, 1);
    if (status != HAL_OK) return HAL_ERROR;

    // Read register 0x01 (NOISE)
    status = AS3935_ReadRegister(THRESHOLD, &AS3935_Sensor.NOISE.Val.Value, 1);
    if (status != HAL_OK) return HAL_ERROR;

    // Read register 0x02 (STATISTICS)
    status = AS3935_ReadRegister(LIGHTNING_REG, &AS3935_Sensor.STATISTICS.Val.Value, 1);
    if (status != HAL_OK) return HAL_ERROR;

    // Read register 0x03 (INT_FREQ)
    status = AS3935_ReadRegister(INT_MASK_ANT, &AS3935_Sensor.INT_FREQ.Val.Value, 1);
    if (status != HAL_OK) return HAL_ERROR;

    // Read registers 0x04 to 0x07 (ENERGY)
    status = AS3935_ReadRegister(ENERGY_LIGHT_LSB, AS3935_Sensor.ENERGY.ByteArray, 4);
    if (status != HAL_OK) return HAL_ERROR;

    // Read register 0x08 (IRQ)
    status = AS3935_ReadRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
    if (status != HAL_OK) return HAL_ERROR;

    // Read register 0x3A (TRCO)
    status = AS3935_ReadRegister(CALIB_TRCO, &AS3935_Sensor.TRCO.Val.Value, 1);
    if (status != HAL_OK) return HAL_ERROR;

    // Read register 0x3B (SRCO)
    status = AS3935_ReadRegister(CALIB_SRCO, &AS3935_Sensor.SRCO.Val.Value, 1);
    if (status != HAL_OK) return HAL_ERROR;

    DEBUG("AS3935 Register Dump:\n");
    DEBUG("REG 0x00 [POWER]:     0x%02X\n", AS3935_Sensor.POWER.Val.Value);
    DEBUG("REG 0x01 [NOISE]:     0x%02X\n", AS3935_Sensor.NOISE.Val.Value);
    DEBUG("REG 0x02 [STATS]:     0x%02X\n", AS3935_Sensor.STATISTICS.Val.Value);
    DEBUG("REG 0x03 [INT_FREQ]:  0x%02X\n", AS3935_Sensor.INT_FREQ.Val.Value);
    DEBUG("REG 0x04 06 [ENERGY]: %02X %02X %02X\n", AS3935_Sensor.ENERGY.ByteArray[0], AS3935_Sensor.ENERGY.ByteArray[1], AS3935_Sensor.ENERGY.ByteArray[2]);
    DEBUG("REG 0x07 [DISTANCE]:  0x%02X\n", AS3935_Sensor.ENERGY.ByteArray[3]);
    DEBUG("REG 0x08 [IRQ+CAP]:   0x%02X\n", AS3935_Sensor.IRQ.Val.Value);
    DEBUG("REG 0x3A [TRCO]:      0x%02X\n", AS3935_Sensor.TRCO.Val.Value);
    DEBUG("REG 0x3B [SRCO]:      0x%02X\n", AS3935_Sensor.SRCO.Val.Value);
    return HAL_OK;
}

/**
 * @brief Configures the power state of the AS3935 sensor.
 * 
 * @param pwr The desired power state (e.g., power-down or normal operation).
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_SetPower(AS3935_PowerState_t powerState) {
    AS3935_Sensor.POWER.Val.BitField.PWD = powerState;
    return AS3935_WriteRegister(AFE_GAIN, &AS3935_Sensor.POWER.Val.Value, 1);
}

/**
 * @brief Sets the AFE gain of the AS3935 sensor.
 * 
 * @param gain The desired AFE gain level.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_SetGain(AS3935_AFE_Gain_t gain) {
    AS3935_Sensor.POWER.Val.BitField.GAIN = gain;
    return AS3935_WriteRegister(AFE_GAIN, &AS3935_Sensor.POWER.Val.Value, 1);
}

/**
 * @brief Configures the watchdog threshold of the AS3935 sensor.
 * 
 * @param wdth The desired watchdog threshold.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_SetWatchdog(AS3935_WDTH_t watchdogThreshold) {
    AS3935_Sensor.NOISE.Val.BitField.WDTH = watchdogThreshold;
    return AS3935_WriteRegister(THRESHOLD, &AS3935_Sensor.NOISE.Val.Value, 1);
}

/**
 * @brief Sets the noise floor level of the AS3935 sensor.
 * 
 * @param level The desired noise floor level.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_SetNoise(AS3935_NoiseFloorLevel_t noiseLevel) {
    AS3935_Sensor.NOISE.Val.BitField.NF_LEV = noiseLevel;
    return AS3935_WriteRegister(THRESHOLD, &AS3935_Sensor.NOISE.Val.Value, 1);
}

/**
 * @brief Configures the spike rejection level of the AS3935 sensor.
 * 
 * @param srej The desired spike rejection level.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_SetSpikeRejectionLevel(AS3935_SREJ_t spikeRejectionLevel) {
    AS3935_Sensor.STATISTICS.Val.BitField.SREJ = spikeRejectionLevel;
    return AS3935_WriteRegister(LIGHTNING_REG, &AS3935_Sensor.STATISTICS.Val.Value, 1);
}

/**
 * @brief Sets the minimum number of lightning events required for validation.
 * 
 * @param minNum The minimum number of lightning events.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_SetLightningNo(AS3935_MinLightning_t noLightningEvents) {
    AS3935_Sensor.STATISTICS.Val.BitField.MIN_NUM_LIGH = noLightningEvents;
    return AS3935_WriteRegister(LIGHTNING_REG, &AS3935_Sensor.STATISTICS.Val.Value, 1);
}

/**
 * @brief Clears the lightning statistics in the AS3935 sensor.
 * 
 * @param clearStat The clear statistics command.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_ClearStatistics(AS3935_ClearStat_t clearStat) {
    AS3935_Sensor.STATISTICS.Val.BitField.CL_STAT = clearStat;
    if (AS3935_WriteRegister(LIGHTNING_REG, &AS3935_Sensor.STATISTICS.Val.Value, 1) != HAL_OK) {
        return HAL_ERROR;
    }
    if (clearStat == CLEAR_STAT_ENABLED) {
        AS3935_Sensor.STATISTICS.Val.BitField.CL_STAT = CLEAR_STAT_DISABLED;
        if (AS3935_WriteRegister(LIGHTNING_REG, &AS3935_Sensor.STATISTICS.Val.Value, 1) != HAL_OK) {
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

/**
 * @brief Reads the interrupt register (0x03) and updates the AS3935_Sensor structure.
 *
 * This function reads `REG0x03`, updates `AS3935_Sensor.INT_FREQ`, and logs the result.
 *
 * @retval INT_L  Lightning detected.
 * @retval INT_D  Disturber detected.
 * @retval INT_NH Noise level too high.
 * @retval INT_NI No interrupt or communication error.
 */
AS3935_INT_t AS3935_GetInterruptType(void) {
    // Optional: wait 2 ms after IRQ goes high (per datasheet recommendation)
    HAL_Delay(2);

    // Read interrupt register
    if (AS3935_ReadRegister(INT_MASK_ANT, &AS3935_Sensor.INT_FREQ.Val.Value, 1) != HAL_OK) {
        DEBUG("AS3935: Failed to read interrupt register (0x03)\n");
        return INT_NI; // Communication error or invalid state
    }

    AS3935_INT_t irqType = (AS3935_INT_t)(AS3935_Sensor.INT_FREQ.Val.BitField.INT);

    // Log decoded interrupt type
    switch (irqType) {
        case INT_L:
            DEBUG("AS3935 IRQ: Lightning detected (INT_L)\n");
            break;
        case INT_D:
            DEBUG("AS3935 IRQ: Disturber detected (INT_D)\n");
            break;
        case INT_NH:
            DEBUG("AS3935 IRQ: Noise level too high (INT_NH)\n");
            break;
        case INT_NI:
            DEBUG("AS3935 IRQ: No interrupt or register cleared (INT_NI)\n");
            break;
        default:
            DEBUG("AS3935 IRQ: Unknown interrupt code 0x%02X\n", irqType);
            break;
    }
    return irqType;
}

/**
 * @brief Configures the disturber mask in the AS3935 sensor.
 * 
 * @param maskDist The desired disturber mask state.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_SetdDisturberMask(AS3935_MaskDist_t maskDist) {
    AS3935_Sensor.INT_FREQ.Val.BitField.MASK_DIST = maskDist;
    return AS3935_WriteRegister(INT_MASK_ANT, &AS3935_Sensor.INT_FREQ.Val.Value, 1);
}

/**
 * @brief Sets the frequency division ratio for the LCO signal.
 * 
 * @param fdivRatio The desired frequency division ratio.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_setFrequencyDivisionRatio(AS3935_LCO_FDiv_t fdivRatio) {
    AS3935_Sensor.INT_FREQ.Val.BitField.LCO_FDIV = fdivRatio;
    return AS3935_WriteRegister(INT_MASK_ANT, &AS3935_Sensor.INT_FREQ.Val.Value, 1);
}

/**
 * @brief Reads lightning energy and distance estimation from registers 0x04 to 0x07.
 * 
 * This function reads the lightning energy (20 bits) and the distance estimation (6 bits) 
 * from the AS3935 sensor. The values are parsed into the `AS3935_Energy_t` structure, 
 * which combines the two data points into an easy-to-use representation. It also reads 
 * the interrupt register to ensure a new interrupt is triggered for the next event.
 * 
 * @param energy Pointer to an `AS3935_Energy_t` structure to store the read values.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_ReadLightningEnergyAndDistance(AS3935_Energy_t *energy) {
    // Read registers 0x04 to 0x07 into the byte array of the structure
    if (AS3935_ReadRegister(ENERGY_LIGHT_LSB, energy->ByteArray, 4) != HAL_OK) {
        return HAL_ERROR; // Communication error
    }
    // Extract the lightning energy (20 bits) and distance estimation (6 bits)
    energy->LightningEnergy = (energy->LSB) | ((uint32_t)(energy->MSB) << 8) | ((uint32_t)(energy->MMSB) << 16);
    energy->DistanceEstimation = energy->Distance;
    return HAL_OK;
}

/**
 * @brief Configures the tuning capacitors for the AS3935 sensor.
 * 
 * @param tuningCap The desired tuning capacitor value.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_setTuningCapacitor(AS3935_TUNE_CAP_t tuningCap) {
    AS3935_Sensor.IRQ.Val.BitField.TUN_CAP = tuningCap;
    return AS3935_WriteRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
}

/**
 * @brief Configures the oscillator display settings.
 * 
 * @param oscDisplay The desired oscillator display setting.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_SetOscillatorDisplay(AS3935_OSCDisplay_t oscDisplay) {
    AS3935_Sensor.IRQ.Val.BitField.OSC_DISP = oscDisplay;
    return AS3935_WriteRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
}

/**
 * @brief Reads the TRCO calibration status.
 * 
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_ReadTRCOCalibrationStatus(void) {
    return AS3935_ReadRegister(CALIB_TRCO, &AS3935_Sensor.TRCO.Val.Value, 1);
}

/**
 * @brief Reads the SRCO calibration status.
 * 
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_ReadSRCOCalibrationStatus(void) {
    return AS3935_ReadRegister(CALIB_SRCO, &AS3935_Sensor.SRCO.Val.Value, 1);
}

HAL_StatusTypeDef AS3935_ApplySensitivityProfile(AS3935_Profile_t profile) {
    switch (profile) {
        case AS3935_PROFILE_INDOOR_VERY_SENSITIVE:
            AS3935_SetGain(AFE_GAIN_INDOOR);
            AS3935_SetNoise(NOISE_FLOOR_860uVrms);
            AS3935_SetWatchdog(EFFICIENCY_80);
            AS3935_SetSpikeRejectionLevel(SREJ_LEVEL_2);
            break;
        case AS3935_PROFILE_INDOOR_BALANCED_1:
            AS3935_SetGain(AFE_GAIN_INDOOR);
            AS3935_SetNoise(NOISE_FLOOR_1100uVrms);
            AS3935_SetWatchdog(EFFICIENCY_80);
            AS3935_SetSpikeRejectionLevel(SREJ_LEVEL_2);
            break;
        case AS3935_PROFILE_INDOOR_BALANCED_2:
            AS3935_SetGain(AFE_GAIN_INDOOR);
            AS3935_SetNoise(NOISE_FLOOR_1800uVrms);
            AS3935_SetWatchdog(EFFICIENCY_80);
            AS3935_SetSpikeRejectionLevel(SREJ_LEVEL_3);
            break;
        case AS3935_PROFILE_INDOOR_DISTURBER_SENSITIVE:
            AS3935_SetGain(AFE_GAIN_INDOOR);
            AS3935_SetNoise(NOISE_FLOOR_2000uVrms);
            AS3935_SetWatchdog(EFFICIENCY_40);
            AS3935_SetSpikeRejectionLevel(SREJ_LEVEL_3);
            break;
        case AS3935_PROFILE_INDOOR_DISTURBER_STRICT:
            AS3935_SetGain(AFE_GAIN_INDOOR);
            AS3935_SetNoise(NOISE_FLOOR_2000uVrms);
            AS3935_SetWatchdog(EFFICIENCY_10);
            AS3935_SetSpikeRejectionLevel(SREJ_LEVEL_3);
            break;
        case AS3935_PROFILE_OUTDOOR_VERY_SENSITIVE:
            AS3935_SetGain(AFE_GAIN_OUTDOOR);
            AS3935_SetNoise(NOISE_FLOOR_860uVrms);
            AS3935_SetWatchdog(EFFICIENCY_80);
            AS3935_SetSpikeRejectionLevel(SREJ_LEVEL_2);
            break;
        case AS3935_PROFILE_OUTDOOR_SENSITIVE:
            AS3935_SetGain(AFE_GAIN_OUTDOOR);
            AS3935_SetNoise(NOISE_FLOOR_1100uVrms);
            AS3935_SetWatchdog(EFFICIENCY_80);
            AS3935_SetSpikeRejectionLevel(SREJ_LEVEL_2);
            break;
        case AS3935_PROFILE_OUTDOOR_BALANCED:
            AS3935_SetGain(AFE_GAIN_OUTDOOR);
            AS3935_SetNoise(NOISE_FLOOR_1800uVrms);
            AS3935_SetWatchdog(EFFICIENCY_70);
            AS3935_SetSpikeRejectionLevel(SREJ_LEVEL_3);
            break;
        case AS3935_PROFILE_OUTDOOR_DISTURBER_STRICT:
            AS3935_SetGain(AFE_GAIN_OUTDOOR);
            AS3935_SetNoise(NOISE_FLOOR_2000uVrms);
            AS3935_SetWatchdog(EFFICIENCY_40);
            AS3935_SetSpikeRejectionLevel(SREJ_LEVEL_3);
            break;
        case AS3935_PROFILE_OUTDOOR_MINIMAL_SENSITIVITY:
            AS3935_SetGain(AFE_GAIN_OUTDOOR);
            AS3935_SetNoise(NOISE_FLOOR_2000uVrms);
            AS3935_SetWatchdog(EFFICIENCY_30);
            AS3935_SetSpikeRejectionLevel(SREJ_LEVEL_7);
            break;
        default:
            return HAL_ERROR;
    }

    return HAL_OK;
}

static uint32_t MeasureFrequencyWithTimer(void) {
    const uint32_t timerClock = GetTimerClock(&htim2) / (htim2.Init.Prescaler + 1);
    const uint16_t sampleCnt = 201;
    const uint16_t periods = 200;
    uint32_t captureValue[201] = {0};
    uint32_t totalPeriod = 0;
		//DEBUG("APB1: %u, TIM2 Clock: %u\n", HAL_RCC_GetPCLK1Freq(), timerClock);
    HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_4);

    for (uint16_t i = 0; i < sampleCnt; i++) {
        while (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC4) == RESET);
        captureValue[i] = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_4);
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC4);
    }

    HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_4);

    for (uint16_t i = 0; i < periods; i++) {
        if (captureValue[i + 1] >= captureValue[i]) {
            totalPeriod += (captureValue[i + 1] - captureValue[i]);
        } else {
            totalPeriod += ((htim2.Instance->ARR + 1) - captureValue[i] + captureValue[i + 1]);
        }
    }

    uint32_t averagePeriod = totalPeriod / periods;
    return (averagePeriod > 0) ? (timerClock / averagePeriod) : 0;
}

AS3935_TUNE_CAP_t Tune_Antenna(void) {
    AS3935_TUNE_CAP_t cap_value;
    AS3935_TUNE_CAP_t best_cap_value = TUN_CAP_0pF;
    uint32_t min_combined_error = UINT32_MAX;

    const uint32_t target_LCO  = 500000;
    const uint32_t target_TRCO = 32768;
    const uint32_t target_SRCO = 1100000;

    Disable_EXTI_AS3935();
    //MX_TIM3_Init();

    // Set LCO Frequency Division Ratio = 16
    AS3935_Sensor.INT_FREQ.Val.BitField.LCO_FDIV = LCO_FDIV_RATIO_16;
    AS3935_WriteRegister(INT_MASK_ANT, &AS3935_Sensor.INT_FREQ.Val.Value, 1);

    for (cap_value = TUN_CAP_0pF; cap_value <= TUN_CAP_120pF; cap_value++) {
        AS3935_Sensor.IRQ.Val.BitField.TUN_CAP = cap_value;

        // --- Measure LCO ---
        AS3935_Sensor.IRQ.Val.BitField.OSC_DISP = OSC_DISPLAY_LCO_ONLY;
        AS3935_WriteRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
        HAL_Delay(20);
        uint32_t lco_freq = 16 * MeasureFrequencyWithTimer();
        uint32_t lco_diff = ABS_DIFF(lco_freq, target_LCO);

        // --- Measure TRCO ---
        AS3935_Sensor.IRQ.Val.BitField.OSC_DISP = OSC_DISPLAY_TRCO_ONLY;
        AS3935_WriteRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
        HAL_Delay(20);
        uint32_t trco_freq = MeasureFrequencyWithTimer();
        uint32_t trco_diff = ABS_DIFF(trco_freq, target_TRCO);

        // --- Measure SRCO ---
        AS3935_Sensor.IRQ.Val.BitField.OSC_DISP = OSC_DISPLAY_SRCO_ONLY;
        AS3935_WriteRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
        HAL_Delay(20);
        uint32_t srco_freq = MeasureFrequencyWithTimer();
        uint32_t srco_diff = ABS_DIFF(srco_freq, target_SRCO);

        // Disable oscillator output
        AS3935_Sensor.IRQ.Val.BitField.OSC_DISP = OSC_DISPLAY_DISABLED;
        AS3935_WriteRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);

        DEBUG("Cap: %d | LCO: %u Hz | TRCO: %u Hz | SRCO: %u Hz\n", cap_value, lco_freq, trco_freq, srco_freq);

        uint32_t combined_diff = lco_diff + trco_diff + srco_diff;

        if (combined_diff < min_combined_error) {
            min_combined_error = combined_diff;
            best_cap_value = cap_value;
        }
    }

    //HAL_TIM_Base_MspDeInit(&htim3);

    AS3935_Sensor.IRQ.Val.BitField.OSC_DISP = OSC_DISPLAY_DISABLED;
    AS3935_WriteRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);

    DEBUG("Best Cap: %d with Min Combined ?f: %u\n", best_cap_value, min_combined_error);
    return best_cap_value;
}