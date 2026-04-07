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

#include "AS7331.h"

// Global shadow register store for configuration registers.
// 'volatile' is required because fields may be written in main context and read in ISR context.
static volatile AS7331_Config_t  as7331_cfg;

// Combined OSR + STATUS shadow, populated by AS7331_GetOSR_Status().
static volatile AS7331_STATUS_t  as7331_status;

// ─── Static helpers ───────────────────────────────────────────────────────────

/**
 * @brief Write 1 byte to a configuration register.
 * @param reg  Register address.
 * @param data Byte value to write.
 * @return HAL status.
 */
static HAL_StatusTypeDef AS7331_WriteReg(uint8_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(&hi2c1, AS7331_I2C_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &data, 1, TIMEOUT_COMM);
}

/**
 * @brief Read one or more bytes from a register.
 * @param reg  Starting register address.
 * @param buf  Pointer to receive buffer.
 * @param len  Number of bytes to read.
 * @return HAL status.
 */
static HAL_StatusTypeDef AS7331_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len) {
    return HAL_I2C_Mem_Read(&hi2c1, AS7331_I2C_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len, TIMEOUT_COMM);
}

// ─── OSR write helper (used by all OSR-modifying functions) ───────────────────

/**
 * @brief Write the OSR shadow value to the OSR register (0x00).
 * @return HAL status.
 */
static HAL_StatusTypeDef AS7331_WriteOSR(void) {
    return AS7331_WriteReg(AS7331_REG_OSR, as7331_cfg.OSR.Val.Value);
}

// ─── REG 0x00 - OSR ───────────────────────────────────────────────────────────

/**
 * @brief AS7331_SoftReset function implementation.
 * @details Sets or clears the SW_RES bit in the OSR register shadow and writes it.
 *          Assert SW_RES_ON, wait ≥ 100 ms, then assert SW_RES_OFF to complete the reset.
 * @param swReset AS7331_SW_RST_ON to assert reset, AS7331_SW_RST_OFF to release.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SoftReset(AS7331_SW_RST_t swReset) {
    as7331_cfg.OSR.Val.BitField.SW_RES = swReset;
    return AS7331_WriteOSR();
}

/**
 * @brief AS7331_SetPower function implementation.
 * @details Controls OSR:PD. PD=0 powers the device on; PD=1 enters power-down.
 *          After powering on (PD=0) allow ≥ 2 ms startup time (TSTARTPD) before use.
 * @param setPower AS7331_PD_OFF or AS7331_PD_ON.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetPower(AS7331_PWR_t setPower) {
    as7331_cfg.OSR.Val.BitField.PD = setPower;
    return AS7331_WriteOSR();
}

/**
 * @brief AS7331_SetOperationalState function implementation.
 * @details Sets the DOS field in the OSR register. Switching to MEASUREMENT state
 *          resets all output result registers and the STATUS register immediately.
 *          CREG registers are not accessible in MEASUREMENT state.
 * @param dos Desired state from AS7331_DOS_t.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetOperationalState(AS7331_DOS_t dos) {
    as7331_cfg.OSR.Val.BitField.DOS = dos;
    return AS7331_WriteOSR();
}

/**
 * @brief AS7331_StartStopMeasurement function implementation.
 * @details Sets or clears OSR:SS. In CONT mode SS=1 starts the measurement loop;
 *          SS=0 aborts after the current conversion. In CMD mode SS auto-clears
 *          at end of conversion.
 * @param ss AS7331_SS_START or AS7331_SS_STOP.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_StartStopMeasurement(AS7331_SS_t ss) {
    as7331_cfg.OSR.Val.BitField.SS = ss;
    return AS7331_WriteOSR();
}

// ─── REG 0x02 - AGEN ──────────────────────────────────────────────────────────

/**
 * @brief AS7331_VerifyDeviceID function implementation.
 * @details Reads the AGEN register (0x02) in Configuration state and checks that
 *          the DEVID field (bits [7:4]) equals AS7331_DEVID (0x2).
 *          Must be called while the device is in Configuration state.
 * @return HAL_OK if DEVID matches, HAL_ERROR on I2C failure or ID mismatch.
 */
HAL_StatusTypeDef AS7331_VerifyDeviceID(void) {
    if (AS7331_ReadRegs(AS7331_REG_AGEN, &as7331_cfg.AGEN.Val.Value, 1) != HAL_OK) {
        return HAL_ERROR;
    }
    return (as7331_cfg.AGEN.Val.BitField.DEVID == AS7331_DEVID) ? HAL_OK : HAL_ERROR;
}

// ─── REG 0x06 - CREG1 ────────────────────────────────────────────────────────

/**
 * @brief AS7331_SetIntegrationTime function implementation.
 * @details Updates CREG1:TIME and writes the CREG1 register (0x06).
 *          Must be called in Configuration state.
 * @param time Integration time from AS7331_TIME_t.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetIntegrationTime(AS7331_TIME_t time) {
    as7331_cfg.CREG1.Val.BitField.TIME = time;
    return AS7331_WriteReg(AS7331_REG_CREG1, as7331_cfg.CREG1.Val.Value);
}

/**
 * @brief AS7331_SetGain function implementation.
 * @details Updates CREG1:GAIN and writes the CREG1 register (0x06).
 *          Must be called in Configuration state.
 * @param gain Gain from AS7331_GAIN_t.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetGain(AS7331_GAIN_t gain) {
    as7331_cfg.CREG1.Val.BitField.GAIN = gain;
    return AS7331_WriteReg(AS7331_REG_CREG1, as7331_cfg.CREG1.Val.Value);
}

// ─── REG 0x07 - CREG2 ────────────────────────────────────────────────────────

/**
 * @brief AS7331_SetDivider function implementation.
 * @details Updates CREG2:DIV and writes the CREG2 register (0x07).
 * @param divider Divider ratio from AS7331_DIV_t.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetDivider(AS7331_DIV_t divider) {
    as7331_cfg.CREG2.Val.BitField.DIV = divider;
    return AS7331_WriteReg(AS7331_REG_CREG2, as7331_cfg.CREG2.Val.Value);
}

/**
 * @brief AS7331_EnableDivider function implementation.
 * @details Updates CREG2:EN_DIV and writes the CREG2 register (0x07).
 * @param enableDivider AS7331_EN_DIV_ENABLED or AS7331_EN_DIV_DISABLED.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_EnableDivider(AS7331_EN_DIV_t enableDivider) {
    as7331_cfg.CREG2.Val.BitField.EN_DIV = enableDivider;
    return AS7331_WriteReg(AS7331_REG_CREG2, as7331_cfg.CREG2.Val.Value);
}

/**
 * @brief AS7331_EnableInternalMeasurement function implementation.
 * @details Updates CREG2:EN_TM and writes the CREG2 register (0x07).
 *          When EN_TM = 1 in SYND mode, the conversion time (as clock counts)
 *          is stored in OUTCONVL/OUTCONVH after each measurement.
 * @param enableTimer AS7331_EN_TM_ENABLED or AS7331_EN_TM_DISABLED.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_EnableInternalMeasurement(AS7331_EN_TM_t enableTimer) {
    as7331_cfg.CREG2.Val.BitField.EN_TM = enableTimer;
    return AS7331_WriteReg(AS7331_REG_CREG2, as7331_cfg.CREG2.Val.Value);
}

// ─── REG 0x08 - CREG3 ────────────────────────────────────────────────────────

/**
 * @brief AS7331_SetMeasurementMode function implementation.
 * @details Updates CREG3:MMODE and writes the CREG3 register (0x08).
 *          Must be called in Configuration state.
 * @param mode Measurement mode from AS7331_MMODE_t.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetMeasurementMode(AS7331_MMODE_t mode) {
    as7331_cfg.CREG3.Val.BitField.MMODE = mode;
    return AS7331_WriteReg(AS7331_REG_CREG3, as7331_cfg.CREG3.Val.Value);
}

/**
 * @brief AS7331_SetStandbyMode function implementation.
 * @details Updates CREG3:SB and writes the CREG3 register (0x08).
 *          SB can only be changed in Configuration state.
 * @param standby AS7331_SB_ON or AS7331_SB_OFF.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetStandbyMode(AS7331_SB_t standby) {
    as7331_cfg.CREG3.Val.BitField.SB = standby;
    return AS7331_WriteReg(AS7331_REG_CREG3, as7331_cfg.CREG3.Val.Value);
}

/**
 * @brief AS7331_SetReadyOutputMode function implementation.
 * @details Updates CREG3:RDYOD and writes the CREG3 register (0x08).
 * @param rdyod AS7331_RDYOD_PUSH_PULL or AS7331_RDYOD_OPEN_DRAIN.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetReadyOutputMode(AS7331_RDYOD_t rdyod) {
    as7331_cfg.CREG3.Val.BitField.RDYOD = rdyod;
    return AS7331_WriteReg(AS7331_REG_CREG3, as7331_cfg.CREG3.Val.Value);
}

/**
 * @brief AS7331_SetClockFrequency function implementation.
 * @details Updates CREG3:CCLK and writes the CREG3 register (0x08).
 * @param cclk Clock frequency from AS7331_CLK_t.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetClockFrequency(AS7331_CLK_t cclk) {
    as7331_cfg.CREG3.Val.BitField.CCLK = cclk;
    return AS7331_WriteReg(AS7331_REG_CREG3, as7331_cfg.CREG3.Val.Value);
}

// ─── REG 0x09 - BREAK ────────────────────────────────────────────────────────

/**
 * @brief AS7331_SetBreakTime function implementation.
 * @details Writes the BREAK register (0x09). Each count = 8 µs; max = 255 * 8 = 2040 µs.
 *          Set a value long enough for the I2C result fetch to complete before the next
 *          conversion starts (CONT / SYNS / SYND modes).
 * @param breakTime Break time in units of 8 µs.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetBreakTime(uint8_t breakTime) {
    as7331_cfg.BREAK = breakTime;
    return AS7331_WriteReg(AS7331_REG_BREAK, as7331_cfg.BREAK);
}

// ─── REG 0x0A - EDGES ────────────────────────────────────────────────────────

/**
 * @brief AS7331_SetEdges function implementation.
 * @details Writes the EDGES register (0x0A). Only used in SYND mode to specify
 *          how many SYN falling edges must occur before the conversion ends.
 * @param edges Number of edges (0–255).
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetEdges(uint8_t edges) {
    as7331_cfg.EDGES = edges;
    return AS7331_WriteReg(AS7331_REG_EDGES, as7331_cfg.EDGES);
}

// ─── REG 0x0B - OPTREG ───────────────────────────────────────────────────────

/**
 * @brief AS7331_SetOptReg function implementation.
 * @details Writes the OPTREG register (0x0B). The fixed upper bits are 0b0111001x;
 *          only bit 0 (INIT_IDX) is configurable. Default after reset is 0b01110010.
 * @param initIdx INIT_IDX bit value (0 or 1).
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_SetOptReg(uint8_t initIdx) {
    as7331_cfg.OPTREG = (uint8_t)(0b01110010 | (initIdx & 0x01));
    return AS7331_WriteReg(AS7331_REG_OPTREG, as7331_cfg.OPTREG);
}

// ─── REG 0x00 (read 2 bytes) - OSR + STATUS ──────────────────────────────────

/**
 * @brief AS7331_GetOSR_Status function implementation.
 * @details Reads 2 bytes starting at 0x00. Byte 0 = OSR (write shadow), byte 1 = STATUS.
 *          The STATUS register reflects the device's internal state and is read-only.
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_GetOSR_Status(void) {
    uint8_t buf[AS7331_OSR_STATUS_SIZE];
    if (AS7331_ReadRegs(AS7331_REG_OSR, buf, AS7331_OSR_STATUS_SIZE) != HAL_OK) {
        return HAL_ERROR;
    }
    as7331_cfg.OSR.Val.Value = buf[0];  // Update OSR shadow with readback
    as7331_status.Val.Value  = buf[1];  // Populate STATUS shadow
    return HAL_OK;
}

// ─── REG 0x05-0x06 - OUTCONVL + OUTCONVH ────────────────────────────────────

/**
 * @brief AS7331_ReadConversionTime function implementation.
 * @details Reads OUTCONVL (0x05, 2 bytes: bits [15:0]) and OUTCONVH (0x06, 2 bytes:
 *          bits [23:16] in the LSB, upper byte is 0x00) and combines them into a 24-bit value.
 *          Only valid in SYND mode when CREG2:EN_TM = 1.
 *          Note: OUTCONVH and CREG1 share address 0x06 — the device returns the output bank
 *          register (OUTCONVH) when read from Measurement state.
 * @param conversionTime Pointer to store the 24-bit conversion time (internal clock counts).
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_ReadConversionTime(uint32_t *conversionTime) {
    uint8_t outconvl[2];  // OUTCONVL: bits [15:0] of the 24-bit counter
    uint8_t outconvh[2];  // OUTCONVH: bits [23:16] in byte 0; byte 1 = 0x00

    if (AS7331_ReadRegs(AS7331_REG_OUTCONVL, outconvl, 2) != HAL_OK) return HAL_ERROR;
    if (AS7331_ReadRegs(AS7331_REG_OUTCONVH, outconvh, 2) != HAL_OK) return HAL_ERROR;

    *conversionTime = ((uint32_t)outconvh[0] << 16)
                    | ((uint32_t)outconvl[1] << 8)
                    |  (uint32_t)outconvl[0];
    return HAL_OK;
}

// ─── REG 0x01 (TEMP) + 0x02 (MRES1/UVA) + 0x03 (MRES2/UVB) + 0x04 (MRES3/UVC) ───────

/**
 * @brief AS7331_ReadUVData function implementation.
 * @details Reads STATUS:NDATA first to confirm new data is available, then performs
 *          an 8-byte burst read starting at AS7331_REG_TEMP (0x01):
 *            Bytes 0-1: TEMP  (12-bit raw in bits [11:0])
 *            Bytes 2-3: MRES1 (UVA, 16-bit unsigned count)
 *            Bytes 4-5: MRES2 (UVB, 16-bit unsigned count)
 *            Bytes 6-7: MRES3 (UVC, 16-bit unsigned count)
 *          Temperature conversion: T[°C*100] = (raw_12bit * AS7331_TEMP_SCALE) - AS7331_TEMP_OFFSET
 *          i.e. T[°C] = raw * 0.05 - 66.9 (datasheet section 7.7).
 * @param uvData Pointer to AS7331_DataOut_t to receive all converted measurement results.
 * @return HAL_OK    — new data read and parsed.
 *         HAL_BUSY  — no new data yet (NDATA = 0); caller should retry during BREAK period.
 *         HAL_ERROR — I2C failure.
 */
HAL_StatusTypeDef AS7331_ReadUVData(AS7331_DataOut_t *uvData) {
    // Update STATUS to check if new data is available
    if (AS7331_GetOSR_Status() != HAL_OK) return HAL_ERROR;

    if (as7331_status.Val.BitField.NDATA == 0) {
        return HAL_BUSY;  // No new data — not an error; poll again during BREAK period
    }

    // Burst read 8 bytes: TEMP(2) + MRES1(2) + MRES2(2) + MRES3(2)
    uint8_t buf[AS7331_UV_DATA_SIZE];
    if (AS7331_ReadRegs(AS7331_REG_TEMP, buf, AS7331_UV_DATA_SIZE) != HAL_OK) {
        return HAL_ERROR;
    }

    // Parse temperature — 12-bit value in bits [11:0], upper nibble reserved
    uint16_t temp_raw = (uint16_t)(((uint16_t)buf[1] << 8) | buf[0]) & 0x0FFFu;
    uvData->TEMP_C100 = (int16_t)((int32_t)temp_raw * AS7331_TEMP_SCALE - AS7331_TEMP_OFFSET);

    // Parse UVA (MRES1), UVB (MRES2), UVC (MRES3) — all 16-bit little-endian
    uvData->UVA = (uint16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    uvData->UVB = (uint16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    uvData->UVC = (uint16_t)(((uint16_t)buf[7] << 8) | buf[6]);

    return HAL_OK;
}

// ─── Initialization ───────────────────────────────────────────────────────────

/**
 * @brief AS7331_Init function implementation.
 * @details Performs the full sensor startup sequence for CONT mode operation:
 *          1.  Software reset — assert SW_RES, wait 100 ms, release SW_RES.
 *          2.  Power on — clear OSR:PD, wait 2 ms (TSTARTPD).
 *          3.  Enter Configuration state (OSR:DOS = 010b).
 *          4.  Verify device ID from AGEN register (DEVID must equal AS7331_DEVID).
 *          5.  Set integration time (1024 ms) and gain (1x).
 *          6.  Set measurement mode to continuous (CONT).
 *          7.  Set READY pin to push-pull output.
 *          8.  Set internal clock to 1 MHz.
 *          9.  Set break time (127 * 8 µs = 1016 µs) — allows I2C data read between conversions.
 *          10. Enter Measurement state (OSR:DOS = 011b).
 *          11. Start measurement (OSR:SS = 1).
 * @return HAL status.
 */
HAL_StatusTypeDef AS7331_Init(void) {
    // Step 1: Software reset
    if (AS7331_SoftReset(AS7331_SW_RST_ON)  != HAL_OK) return HAL_ERROR;
    HAL_Delay(100);  // ≥ 100 ms for reset to complete
    if (AS7331_SoftReset(AS7331_SW_RST_OFF) != HAL_OK) return HAL_ERROR;

    // Step 2: Power on and wait for analog startup
    if (AS7331_SetPower(AS7331_PD_OFF) != HAL_OK) return HAL_ERROR;
    HAL_Delay(2);  // TSTARTPD = 1.2 ms typ; 2 ms provides margin

    // Step 3: Enter Configuration state
    if (AS7331_SetOperationalState(AS7331_DOS_CONFIGURATION) != HAL_OK) return HAL_ERROR;

    // Step 4: Verify device ID
    if (AS7331_VerifyDeviceID() != HAL_OK) return HAL_ERROR;

    // Step 5: Configure integration time and gain
    if (AS7331_SetIntegrationTime(AS7331_TIME_1024MS) != HAL_OK) return HAL_ERROR;
    if (AS7331_SetGain(AS7331_GAIN_1X)                != HAL_OK) return HAL_ERROR;

    // Step 6: Set continuous measurement mode
    if (AS7331_SetMeasurementMode(AS7331_MMODE_CONT) != HAL_OK) return HAL_ERROR;

    // Step 7: READY pin as push-pull output
    if (AS7331_SetReadyOutputMode(AS7331_RDYOD_PUSH_PULL) != HAL_OK) return HAL_ERROR;

    // Step 8: Internal clock at 1 MHz (lowest frequency = lowest power)
    if (AS7331_SetClockFrequency(AS7331_CLK_1MHZ) != HAL_OK) return HAL_ERROR;

    // Step 9: Break time — 127 * 8 µs = 1016 µs between conversions for I2C data fetch
    if (AS7331_SetBreakTime(127) != HAL_OK) return HAL_ERROR;

    // Step 10: Switch to Measurement state
    if (AS7331_SetOperationalState(AS7331_DOS_MEASUREMENT) != HAL_OK) return HAL_ERROR;

    // Step 11: Start continuous measurement
    if (AS7331_StartStopMeasurement(AS7331_SS_START) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}
