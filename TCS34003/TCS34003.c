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

#include "main.h"
#include "TCS34003.h"

/** @brief I²C handle wired to the bus the TCS3400 is connected to. */
extern I2C_HandleTypeDef hi2c3;

/* ============================================================================
 * Private Data
 * ========================================================================= */

/** @brief Shadow copy of all writable TCS3400 registers. */
static TCS34003_REGISTERS_t TCS34003_Sensor;

/* ============================================================================
 * Initialisation
 * ========================================================================= */

/**
 * @brief  Initialise the TCS3400 sensor.
 * @details Configured for minimum sensitivity and ~1 second refresh rate:
 *          - PON only first, AEN added after all registers are configured.
 *          - Integration time: 1 cycle / 2.78 ms (minimum sensitivity).
 *          - Wait time: 30 cycles x 33.36 ms (WLONG=1) = 1000.8 ms.
 *          - Total cycle: 2.78 ms + 1000.8 ms ~1003 ms (~1 Hz).
 *          - WLONG enabled (x12 multiplier on WTIME).
 *          - WEN enabled to activate the wait timer.
 *          - Gain: 1x (minimum).
 *          - Interrupt persistence: every RGBC cycle.
 *          - Clear channel thresholds: mid-scale (both = 0x7FFF).
 *          - Any pending interrupts cleared, then AIEN enabled.
 * @retval HAL_OK on success, first HAL error encountered otherwise.
 */
HAL_StatusTypeDef TCS34003_Init(void) {
    HAL_StatusTypeDef status;

    /* Power ON only - configure all registers before enabling ADC. */
    status = TCS34003_SetEnable(0x01); /* PON=1, AEN=0, WEN=0 */
    if (status != HAL_OK) return status;

    /* 1 cycle / 2.78 ms - shortest integration = lowest sensitivity. */
    status = TCS34003_SetIntegrationTime(TCS34003_ATIME_1_CYCLE);
    if (status != HAL_OK) return status;

    /* 30 cycles with WLONG=1: 30 x 33.36 ms = 1000.8 ms wait.
     * Combined with ATIME: ~1003 ms total cycle (~1 Hz). */
    status = TCS34003_SetWaitTime(TCS34003_WTIME_30_CYCLES);
    if (status != HAL_OK) return status;

    /* Enable WLONG (x12 multiplier). Bit 6 must always be written as 1. */
    status = TCS34003_SetConfig(TCS34003_CONFIG_WLONG_ENABLE);
    if (status != HAL_OK) return status;

    /* 1x gain - minimum analogue amplification. */
    status = TCS34003_SetGain(TCS34003_AGAIN_1X);
    if (status != HAL_OK) return status;

    /* Interrupt on every completed RGBC cycle. */
    status = TCS34003_SetInterruptPersistence(TCS34003_APERS_EVERY_CYCLE);
    if (status != HAL_OK) return status;

    /* Mid-scale thresholds guarantee an interrupt after the first cycle. */
    status = TCS34003_SetClearChannelThreshold(TCS34003_THRESHOLD_MID, TCS34003_THRESHOLD_MID);
    if (status != HAL_OK) return status;

    /* Clear any stale interrupt flags before enabling. */
    status = TCS34003_ClearInterrupts();
    if (status != HAL_OK) return status;

    /* All registers configured - enable ADC and wait timer (PON=1, WEN=1, AEN=1). */
    status = TCS34003_SetEnable(0x0B);
    if (status != HAL_OK) return status;

    /* Enable ALS interrupts (AIEN bit). */
    status = TCS34003_EnableInterrupts();
    if (status != HAL_OK) return status;

    return HAL_OK;
}

/* ============================================================================
 * Register Write Functions
 * ========================================================================= */

/**
 * @brief  Write the ENABLE register.
 * @param  value  Raw byte to write.
 * @retval HAL status.
 */
HAL_StatusTypeDef TCS34003_SetEnable(uint8_t value) {
    TCS34003_Sensor.ENABLE.Val.Value = value;
    return HAL_I2C_Mem_Write(&hi2c3, TCS34003_I2C_ADDR, TCS34003_REG_ENABLE, I2C_MEMADD_SIZE_8BIT, &TCS34003_Sensor.ENABLE.Val.Value, 1, TIMEOUT_COMM);
}

/**
 * @brief  Set RGBC integration time (ATIME register).
 * @param  value  TCS34003_ATIME_t selection.
 * @retval HAL status.
 */
HAL_StatusTypeDef TCS34003_SetIntegrationTime(TCS34003_ATIME_t value) {
    TCS34003_Sensor.ATIME = value;
    return HAL_I2C_Mem_Write(&hi2c3, TCS34003_I2C_ADDR, TCS34003_REG_ATIME, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&TCS34003_Sensor.ATIME, 1, TIMEOUT_COMM);
}

/**
 * @brief  Set inter-measurement wait time (WTIME register).
 * @param  value  TCS34003_WTIME_t selection.
 * @retval HAL status.
 */
HAL_StatusTypeDef TCS34003_SetWaitTime(TCS34003_WTIME_t value) {
    TCS34003_Sensor.WTIME = value;
    return HAL_I2C_Mem_Write(&hi2c3, TCS34003_I2C_ADDR, TCS34003_REG_WTIME, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&TCS34003_Sensor.WTIME, 1, TIMEOUT_COMM);
}

/**
 * @brief  Set Clear channel interrupt thresholds.
 * @param  low   16-bit lower bound.
 * @param  high  16-bit upper bound.
 * @retval HAL status.
 */
HAL_StatusTypeDef TCS34003_SetClearChannelThreshold(uint16_t low, uint16_t high) {
    HAL_StatusTypeDef status;

    TCS34003_Sensor.CC_TH.LowThreshold.Value  = low;
    TCS34003_Sensor.CC_TH.HighThreshold.Value = high;

    status = HAL_I2C_Mem_Write(&hi2c3, TCS34003_I2C_ADDR, TCS34003_REG_AILTL, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&TCS34003_Sensor.CC_TH.LowThreshold.Bytes, 2, TIMEOUT_COMM);
    if (status != HAL_OK) return status;

    return HAL_I2C_Mem_Write(&hi2c3, TCS34003_I2C_ADDR, TCS34003_REG_AIHTL, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&TCS34003_Sensor.CC_TH.HighThreshold.Bytes, 2, TIMEOUT_COMM);
}

/**
 * @brief  Enable ALS interrupts by setting the AIEN bit in the ENABLE register.
 * @details Performs a read-modify-write to preserve all other ENABLE bits.
 * @retval HAL status.
 */
HAL_StatusTypeDef TCS34003_EnableInterrupts(void) {
    HAL_StatusTypeDef status;
    uint8_t enable_reg;

    status = HAL_I2C_Mem_Read(&hi2c3, TCS34003_I2C_ADDR, TCS34003_REG_ENABLE, I2C_MEMADD_SIZE_8BIT, &enable_reg, 1, TIMEOUT_COMM);
    if (status != HAL_OK) return status;

    enable_reg |= (1U << 4U); /* AIEN = bit 4 */

		TCS34003_Sensor.ENABLE.Val.Value = enable_reg;  // sync shadow
		return HAL_I2C_Mem_Write(&hi2c3, TCS34003_I2C_ADDR, TCS34003_REG_ENABLE, I2C_MEMADD_SIZE_8BIT, &TCS34003_Sensor.ENABLE.Val.Value, 1, TIMEOUT_COMM);
}

/**
 * @brief  Set interrupt persistence filter (PERS register).
 * @param  value  TCS34003_APERS_t selection.
 * @retval HAL status.
 */
HAL_StatusTypeDef TCS34003_SetInterruptPersistence(TCS34003_APERS_t value) {
    TCS34003_Sensor.APERS = value;
    return HAL_I2C_Mem_Write(&hi2c3, TCS34003_I2C_ADDR, TCS34003_REG_PERS, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&TCS34003_Sensor.APERS, 1, TIMEOUT_COMM);
}

/**
 * @brief  Write the CONFIG register.
 * @param  value  TCS34003_CONFIG_t selection.
 * @retval HAL status.
 */
HAL_StatusTypeDef TCS34003_SetConfig(TCS34003_CONFIG_t value) {
    TCS34003_Sensor.CONFIG = value;
    return HAL_I2C_Mem_Write(&hi2c3, TCS34003_I2C_ADDR, TCS34003_REG_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&TCS34003_Sensor.CONFIG, 1, TIMEOUT_COMM);
}

/**
 * @brief  Set RGBC analogue gain (CONTROL register).
 * @param  value  TCS34003_AGAIN_t selection.
 * @retval HAL status.
 */
HAL_StatusTypeDef TCS34003_SetGain(TCS34003_AGAIN_t value)
{
    TCS34003_Sensor.AGAIN = value;
    return HAL_I2C_Mem_Write(&hi2c3, TCS34003_I2C_ADDR, TCS34003_REG_CONTROL, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&TCS34003_Sensor.AGAIN, 1, TIMEOUT_COMM);
}

/**
 * @brief  Write the AUX register.
 * @param  value  TCS34003_AUX_t selection.
 * @retval HAL status.
 */
HAL_StatusTypeDef TCS34003_SetAux(TCS34003_AUX_t value) {
    TCS34003_Sensor.AUX = value;
    return HAL_I2C_Mem_Write(&hi2c3, TCS34003_I2C_ADDR, TCS34003_REG_AUX, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&TCS34003_Sensor.AUX, 1, TIMEOUT_COMM);
}

/**
 * @brief  Control IR sensor mapping onto the Clear channel (IR register 0xC0).
 * @param  value  TCS34003_IR_ACCESS_ENABLE or TCS34003_IR_ACCESS_DISABLE.
 * @retval HAL status.
 */
HAL_StatusTypeDef TCS34003_SetIRAccess(TCS34003_IR_Access_t value) {
    TCS34003_Sensor.IR_Access = value;
    return HAL_I2C_Mem_Write(&hi2c3, TCS34003_I2C_ADDR, TCS34003_REG_IR, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&TCS34003_Sensor.IR_Access, 1, TIMEOUT_COMM);
}

/**
 * @brief  Clear all active interrupts (AICLEAR register 0xE7).
 * @retval HAL status.
 */
HAL_StatusTypeDef TCS34003_ClearInterrupts(void) {
    uint8_t dummy = 0x00U;
    return HAL_I2C_Mem_Write(&hi2c3, TCS34003_I2C_ADDR, TCS34003_REG_AICLEAR, I2C_MEMADD_SIZE_8BIT, &dummy, 1, TIMEOUT_COMM);
}

/**
 * @brief  Force an interrupt (IFORCE register 0xE4).
 * @retval HAL status.
 */
HAL_StatusTypeDef TCS34003_ForceInterrupt(void) {
    uint8_t dummy = 0x00U;
    return HAL_I2C_Mem_Write(&hi2c3, TCS34003_I2C_ADDR, TCS34003_REG_IFORCE, I2C_MEMADD_SIZE_8BIT, &dummy, 1, TIMEOUT_COMM);
}

/* ============================================================================
 * Status and Data Read Functions
 * ========================================================================= */

/**
 * @brief  Convert TCS34003_AGAIN_t enum to a numeric gain multiplier.
 * @param  again  Gain setting from the shadow register.
 * @retval Numeric gain value (1.0, 4.0, 16.0, or 64.0).
 */
static float TCS34003_AgainToFloat(TCS34003_AGAIN_t again) {
    switch (again) {
        case TCS34003_AGAIN_4X:  return 4.0f;
        case TCS34003_AGAIN_16X: return 16.0f;
        case TCS34003_AGAIN_64X: return 64.0f;
        case TCS34003_AGAIN_1X:  /* fall-through */
        default:                 return 1.0f;
    }
}

/**
 * @brief  Convert TCS34003_ATIME_t enum to integration time in milliseconds.
 * @details Cycles = 256 - register_value; each cycle = TCS34003_ATIME_STEP_MS.
 * @param  atime  Integration time setting from the shadow register.
 * @retval Integration time in ms.
 */
static float TCS34003_AtimeToMs(TCS34003_ATIME_t atime) {
    float cycles = (float)(256U - (uint8_t)atime);
    return cycles * TCS34003_ATIME_STEP_MS;
}

/**
 * @brief  Read the STATUS register into the internal shadow.
 * @retval HAL status.
 */
HAL_StatusTypeDef TCS34003_GetStatus(void) {
    return HAL_I2C_Mem_Read(&hi2c3, TCS34003_I2C_ADDR, TCS34003_REG_STATUS, I2C_MEMADD_SIZE_8BIT, &TCS34003_Sensor.STATUS.Val.Value, 1, TIMEOUT_COMM);
}

/**
 * @brief  Read RGBC + IR data and compute CCT, lux, and irradiance.
 *
 * @details
 *  Step 1: Read STATUS; return HAL_ERROR if AVALID (bit 0) is not set.
 *  Step 2: Burst-read 8 bytes of RGBC data starting at CDATAL (0x94).
 *  Step 3: Enable IR sensor access, burst-read 2 bytes from CDATAL,
 *          then disable IR sensor access.
 *  Step 4: Compute CIE XYZ tristimulus values from RGB counts.
 *  Step 5: Compute CCT using the McCamy approximation:
 *              x   = X / (X+Y+Z),  y = Y / (X+Y+Z)
 *              n   = (x - 0.3320) / (0.1858 - y)
 *              CCT = 449*n^3 + 3525*n^2 + 6823.3*n + 5520.33
 *  Step 6: Compute illuminance: Lux = 0.136*Clear + 0.119*Green
 *  Step 7: Compute irradiance:  E   = (Clear / Re_actual) * 0.01  [W/m2]
 *              Re_actual = Re_ref * (AGAIN_actual / AGAIN_ref) * (ATIME_actual / ATIME_ref)
 *              Re_ref = 14.0 counts/(uW/cm2) at AGAIN=16x, ATIME=27.8 ms (datasheet Fig. 8).
 *  Step 8: Clear all interrupts.
 *
 * @param[out] data  Caller-allocated output structure.
 * @retval HAL_OK on success.
 * @retval HAL_ERROR if AVALID is not set (data not yet ready).
 */
HAL_StatusTypeDef TCS34003_GetLightData(TCS34003_LightData_t *data) {
    HAL_StatusTypeDef status;

    /* --- Step 1: Validate data-ready flag --------------------------------- */
    status = TCS34003_GetStatus();
    if (status != HAL_OK) return status;

    if (!TCS34003_Sensor.STATUS.Val.Bits.AVALID) { /* AVALID = bit 0 */
        return HAL_ERROR;
    }

    /* --- Step 2: Burst-read RGBC (8 bytes: Clear / Red / Green / Blue) ---- */
    status = HAL_I2C_Mem_Read(&hi2c3, TCS34003_I2C_ADDR, TCS34003_REG_CDATAL, I2C_MEMADD_SIZE_8BIT, data->RGBC.FullArray, sizeof(data->RGBC.FullArray), TIMEOUT_COMM);
    if (status != HAL_OK) return status;

    /* --- Step 3: Read IR data (remap Clear channel to IR photodiode) ------ */
    status = TCS34003_SetIRAccess(TCS34003_IR_ACCESS_ENABLE);
    if (status != HAL_OK) return status;

    status = HAL_I2C_Mem_Read(&hi2c3, TCS34003_I2C_ADDR, TCS34003_REG_CDATAL, I2C_MEMADD_SIZE_8BIT, data->IR.FullArray, sizeof(data->IR.FullArray), TIMEOUT_COMM);

    /* Disable IR access unconditionally — must not leave IR bit set regardless
     * of whether the read above succeeded. If it failed, still restore the
     * Clear channel mapping before returning the error. */
    TCS34003_SetIRAccess(TCS34003_IR_ACCESS_DISABLE);

    if (status != HAL_OK) return status;

    /* --- Steps 4-5: CCT via McCamy approximation -------------------------- */
    float X = (TCS34003_CCT_COEFF_X_RED   * (float)data->RGBC.RedChannel) + (TCS34003_CCT_COEFF_X_GREEN * (float)data->RGBC.GreenChannel) + (TCS34003_CCT_COEFF_X_BLUE  * (float)data->RGBC.BlueChannel);
    float Y = (TCS34003_CCT_COEFF_Y_RED   * (float)data->RGBC.RedChannel) + (TCS34003_CCT_COEFF_Y_GREEN * (float)data->RGBC.GreenChannel) + (TCS34003_CCT_COEFF_Y_BLUE  * (float)data->RGBC.BlueChannel);
    float Z = (TCS34003_CCT_COEFF_Z_RED   * (float)data->RGBC.RedChannel) + (TCS34003_CCT_COEFF_Z_GREEN * (float)data->RGBC.GreenChannel) + (TCS34003_CCT_COEFF_Z_BLUE  * (float)data->RGBC.BlueChannel);
    float XYZ = X + Y + Z;
    if (XYZ < 1.0f) {
        /* All channels near zero — insufficient light for CCT calculation. */
        data->CCT = 0.0f;
    } else {
        float x_c = X / XYZ;                             														/* CIE x chromaticity */
        float y_c = Y / XYZ;                             														/* CIE y chromaticity */
        float n   = (x_c - TCS34003_CCT_COEFF_X0) / (TCS34003_CCT_COEFF_Y0 - y_c);	/* McCamy n           */
        float n2  = n * n;
        data->CCT = (TCS34003_CCT_COEFF_C1 * n2 * n) + (TCS34003_CCT_COEFF_C2 * n2) + (TCS34003_CCT_COEFF_C3 * n) +  TCS34003_CCT_COEFF_C4;
    }

    /* --- Step 6: Illuminance (lux) ---------------------------------------- */
    data->Lux = (TCS34003_LUX_COEFF_CLEAR * (float)data->RGBC.ClearChannel) + (TCS34003_LUX_COEFF_GREEN * (float)data->RGBC.GreenChannel);

    /* --- Step 7: Irradiance (W/m2) ----------------------------------------
     * Re_ref is specified at AGAIN=16x, ATIME=27.8 ms (datasheet Fig. 8).
     * Scale it to the actual operating gain and integration time so the
     * result remains valid regardless of which settings are active.
     * Re_actual = Re_ref * (AGAIN_actual / AGAIN_ref) * (ATIME_actual / ATIME_ref) */
    float again_actual = TCS34003_AgainToFloat(TCS34003_Sensor.AGAIN);
    float atime_actual = TCS34003_AtimeToMs(TCS34003_Sensor.ATIME);
    float re_actual    = TCS34003_RESPONSIVITY_CLEAR_REF * (again_actual / TCS34003_CALIB_AGAIN_REF) * (atime_actual / TCS34003_CALIB_ATIME_MS_REF);
    data->Irradiance = ((float)data->RGBC.ClearChannel / re_actual) * TCS34003_MICRO_TO_W_CONV;

    /* --- Step 8: Clear interrupt flags ------------------------------------ */
    return TCS34003_ClearInterrupts();
}
