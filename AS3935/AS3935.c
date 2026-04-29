/***************************************************************************
 * @file    AS3935.c
 * @brief   STM32G4 HAL driver for the AS3935 Franklin Lightning Sensor.
 *
 * MIT License
 * Copyright (c) 2026 Grozea Ion gvi70000
 ***************************************************************************/

#include "AS3935.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include "debug.h"
#include <stdio.h>   /* printf — required when DEBUG_ENABLED=1 */
#include <stdint.h>

/* -------------------------------------------------------------------------
 * Register addresses
 * ---------------------------------------------------------------------- */
#define AS3935_REG_AFE_GAIN         0x00
#define AS3935_REG_THRESHOLD        0x01
#define AS3935_REG_LIGHTNING        0x02
#define AS3935_REG_INT_MASK_ANT     0x03
#define AS3935_REG_ENERGY_LSB       0x04  /* S_LIG_L  — burst start */
#define AS3935_REG_ENERGY_MSB       0x05  /* S_LIG_M  */
#define AS3935_REG_ENERGY_MMSB      0x06  /* S_LIG_MM [4:0] */
#define AS3935_REG_DISTANCE         0x07  /* DISTANCE [5:0] */
#define AS3935_REG_FREQ_DISP_IRQ    0x08
#define AS3935_REG_CALIB_TRCO       0x3A
#define AS3935_REG_CALIB_SRCO       0x3B
#define AS3935_REG_PRESET_DEFAULT   0x3C
#define AS3935_REG_CALIB_RCO        0x3D

/* -------------------------------------------------------------------------
 * Static register shadow (single instance)
 * ---------------------------------------------------------------------- */
static volatile AS3935_REGS_t AS3935_Sensor;

/* -------------------------------------------------------------------------
 * Timer handle used by AS3935_TuneAntenna()
 * (configured externally as input-capture on TIM2 CH4)
 * ---------------------------------------------------------------------- */
extern TIM_HandleTypeDef htim2;

/* =========================================================================
 * Private helpers
 * ======================================================================= */

/**
 * @brief  Write one or more bytes to an AS3935 register.
 * @param  reg   Register address.
 * @param  pData Pointer to data buffer.
 * @param  len   Number of bytes to write.
 * @return HAL status.
 */
static HAL_StatusTypeDef AS3935_WriteReg(uint8_t reg, volatile uint8_t *pData, uint8_t len) {
    return HAL_I2C_Mem_Write(&hi2c1, AS3935_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)pData, len, TIMEOUT_COMM);
}

/**
 * @brief  Read one or more bytes from an AS3935 register.
 * @param  reg   Register address.
 * @param  pData Pointer to receive buffer.
 * @param  len   Number of bytes to read.
 * @return HAL status.
 */
static HAL_StatusTypeDef AS3935_ReadRegs(uint8_t reg, volatile uint8_t *pData, uint8_t len) {
    return HAL_I2C_Mem_Read(&hi2c1, AS3935_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)pData, len, TIMEOUT_COMM);
}

/**
 * @brief  Send a direct command (0x96) to a command register.
 * @details Per datasheet §"Send Direct Command Byte": write the value 0x96
 *          to REG0x3C (PRESET_DEFAULT) or REG0x3D (CALIB_RCO).
 * @param  cmdReg AS3935_REG_PRESET_DEFAULT or AS3935_REG_CALIB_RCO.
 * @return HAL status.
 */
static HAL_StatusTypeDef AS3935_SendDirectCommand(uint8_t cmdReg) {
    uint8_t cmd = AS3935_DIRECT_COMMAND; /* 0x96 — must be a real variable, not a cast-from-integer pointer */
    return AS3935_WriteReg(cmdReg, &cmd, 1);
}

/**
 * @brief  Return the TIM2 tick frequency in Hz (accounts for APB prescaler).
 */
static uint32_t AS3935_GetTimerClockHz(void) {
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    /* If APB1 prescaler ≠ /1 the timer clock is doubled (RM0440 §6.2). */
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
        return pclk1 * 2U;
    return pclk1;
}

/**
 * @brief  Measure the frequency on TIM2 CH4 using first-and-last capture.
 *
 * @details Instead of summing 200 individual periods (which accumulates
 *          rounding error and is sensitive to single noisy edges), this
 *          method captures N+1 edges and computes:
 *
 *              f = (N * timerClock) / (t_last - t_first)
 *
 *          Using a large N (500 periods) over a long gate time gives a
 *          stable, alias-free result for all three AS3935 oscillators:
 *            - LCO/16 ~ 31 kHz  -> 500 periods = ~16 ms gate
 *            - TRCO   ~ 31 kHz  -> 500 periods = ~16 ms gate
 *            - SRCO   ~ 1.1 MHz -> 500 periods = ~0.5 ms gate
 *
 * @return Measured frequency in Hz, or 0 if capture timed out.
 */
static uint32_t AS3935_MeasureFrequencyHz(void) {
    /* timerClock is already divided by prescaler in GetTimerClockHz(),
     * but that function returns the pre-prescaler value — divide here. */
    const uint32_t timerClock = AS3935_GetTimerClockHz() / (htim2.Init.Prescaler + 1);
    const uint32_t N = 500;   /* number of complete periods to gate       */
    const uint32_t TIMEOUT_TICKS = timerClock / 2; /* 500 ms max wait     */
    uint32_t t0 = 0, tN = 0;
    uint32_t elapsed = 0;
    uint32_t edgeCount = 0;
    /* Reset counter so wrap-around arithmetic is simple. */
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC4);
    HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_4);
    /* Capture first edge. */
    elapsed = 0;
    while (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC4) == RESET) {
        if (++elapsed > TIMEOUT_TICKS) {
            HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_4);
            return 0;
        }
    }
    t0 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_4);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC4);
    /* Capture N more edges (= N complete periods). */
    for (edgeCount = 0; edgeCount < N; edgeCount++) {
        elapsed = 0;
        while (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC4) == RESET) {
            if (++elapsed > TIMEOUT_TICKS) {
                HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_4);
                return 0;
            }
        }
        tN = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_4);
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC4);
    }
    HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_4);
    /* Total span in timer ticks, handling 32-bit wrap. */
    uint32_t span = (tN >= t0) ? (tN - t0) : (0xFFFFFFFF - t0 + tN + 1);
    if (span == 0) return 0;
    /* f = N * timerClock / span */
    return (uint32_t)(((uint64_t)N * timerClock) / span);
}

/* =========================================================================
 * Public API
 * ======================================================================= */

/**
 * @brief  Initialise the AS3935 with default operating parameters.
 * @details Sequence:
 *          1.  PRESET_DEFAULT — reset all registers.
 *          2.  Power up, set outdoor AFE gain.
 *          3.  Set tuning capacitor (96 pF — adjust per antenna measurement).
 *          4.  Configure noise floor and watchdog threshold.
 *          5.  Configure spike rejection and minimum lightning count.
 *          6.  Configure interrupt mask and LCO divider.
 *          7.  Calibrate internal RC oscillators (CALIB_RCO procedure).
 *          8.  Verify TRCO and SRCO calibration status registers.
 * @return HAL_OK on success, HAL_ERROR on any I2C failure or calib failure.
 */
HAL_StatusTypeDef AS3935_Init(void) {
    /* Step 1 — Reset all registers to datasheet defaults. */
    if (AS3935_SendDirectCommand(AS3935_REG_PRESET_DEFAULT) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(AS3935_CMD_DELAY_MS);
    /* Step 2 — Power up and set outdoor AFE gain.
     * Datasheet default for AFE_GB is Indoor (10010b); override to Outdoor. */
    AS3935_Sensor.POWER.Val.BitField.PWD    = AS3935_POWER_UP;
    AS3935_Sensor.POWER.Val.BitField.AFE_GB = AS3935_AFE_GAIN_OUTDOOR;
    if (AS3935_WriteReg(AS3935_REG_AFE_GAIN, (uint8_t *)&AS3935_Sensor.POWER.Val.Value, 1) != HAL_OK)
        return HAL_ERROR;
    //DEBUG("REG 0x00 [PWR/GAIN] = 0x%02X\n", AS3935_Sensor.POWER.Val.Value);
    /* Step 3 — Start with minimum capacitance; call AS3935_TuneAntenna()
     * after Init to find and apply the optimal value. */
    if (AS3935_SetTuningCapacitor(AS3935_TUN_CAP_0pF) != HAL_OK)
        return HAL_ERROR;

    /* Step 4 — Noise floor and watchdog threshold.
     * Default (860 µVrms, WDTH=2) is a reasonable starting point outdoors. */
    AS3935_Sensor.NOISE.Val.BitField.NF_LEV = AS3935_NF_860uVrms;
    AS3935_Sensor.NOISE.Val.BitField.WDTH   = AS3935_WDTH_2;
    if (AS3935_WriteReg(AS3935_REG_THRESHOLD, (uint8_t *)&AS3935_Sensor.NOISE.Val.Value, 1) != HAL_OK)
        return HAL_ERROR;
    //DEBUG("REG 0x01 [NOISE] = 0x%02X\n", AS3935_Sensor.NOISE.Val.Value);

    /* Step 5 — Spike rejection and minimum lightning count.
     * REG0x02[7] is a reserved bit whose datasheet default is 1 — preserve it. */
    AS3935_Sensor.STATISTICS.Val.Value = 0x80; /* reserved bit pre-set */
    AS3935_Sensor.STATISTICS.Val.BitField.SREJ = AS3935_SREJ_2;
    AS3935_Sensor.STATISTICS.Val.BitField.MIN_NUM_LIGH = AS3935_MIN_LIGHTNING_1;
    /* Clear existing statistics once at startup (toggle CL_STAT 0→1→0). */
    AS3935_Sensor.STATISTICS.Val.BitField.CL_STAT = 1;
    if (AS3935_WriteReg(AS3935_REG_LIGHTNING, (uint8_t *)&AS3935_Sensor.STATISTICS.Val.Value, 1) != HAL_OK)
        return HAL_ERROR;
    AS3935_Sensor.STATISTICS.Val.BitField.CL_STAT = 0;
    if (AS3935_WriteReg(AS3935_REG_LIGHTNING, (uint8_t *)&AS3935_Sensor.STATISTICS.Val.Value, 1) != HAL_OK)
        return HAL_ERROR;
    //DEBUG("REG 0x02 [STATS] = 0x%02X\n", AS3935_Sensor.STATISTICS.Val.Value);
    /* Step 6 — Interrupt mask and LCO frequency divider.
     * Write only the writable upper bits; keep INT field [3:0] as zero. */
    AS3935_Sensor.INT_FREQ.Val.Value = 0;
    AS3935_Sensor.INT_FREQ.Val.BitField.MASK_DIST = AS3935_MASK_DIST_OFF;
    AS3935_Sensor.INT_FREQ.Val.BitField.LCO_FDIV  = AS3935_LCO_FDIV_16;
    uint8_t irqCfg = AS3935_Sensor.INT_FREQ.Val.Value & 0xF0; /* mask read-only INT bits */
    if (AS3935_WriteReg(AS3935_REG_INT_MASK_ANT, &irqCfg, 1) != HAL_OK)
        return HAL_ERROR;
    //DEBUG("REG 0x03 [INT_FREQ] = 0x%02X\n", irqCfg);
    /* Step 7 — Calibrate internal RC oscillators.
     * Datasheet wakeup procedure (§"Clock Generation"):
     *   a. Send CALIB_RCO direct command.
     *   b. Set REG0x08 DISP_SRCO = 1 (bit6).
     *   c. Wait ≥ 2 ms for calibration to complete.
     *   d. Clear DISP_SRCO = 0. */
    if (AS3935_SendDirectCommand(AS3935_REG_CALIB_RCO) != HAL_OK)
        return HAL_ERROR;
    if (AS3935_SetOscillatorDisplay(AS3935_OSC_SRCO) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(AS3935_CALIB_DELAY_MS);
    if (AS3935_SetOscillatorDisplay(AS3935_OSC_NONE) != HAL_OK)
        return HAL_ERROR;
    /* Step 8 — Verify calibration results. */
    if (AS3935_ReadTRCOCalibStatus() != HAL_OK)
			return HAL_ERROR;
    if (AS3935_ReadSRCOCalibStatus() != HAL_OK)
			return HAL_ERROR;
    if (AS3935_Sensor.TRCO.Val.BitField.TRCO_CALIB_NOK || AS3935_Sensor.SRCO.Val.BitField.SRCO_CALIB_NOK) {
        DEBUG("AS3935: Oscillator calibration FAILED (TRCO=0x%02X SRCO=0x%02X)\n", AS3935_Sensor.TRCO.Val.Value, AS3935_Sensor.SRCO.Val.Value);
        return HAL_ERROR;
    }
    //DEBUG("AS3935: Init OK (TRCO=0x%02X SRCO=0x%02X)\n", AS3935_Sensor.TRCO.Val.Value, AS3935_Sensor.SRCO.Val.Value);
    return HAL_OK;
}

/**
 * @brief  Read all AS3935 registers into the shadow structure and log them.
 * @return HAL_OK on success, HAL_ERROR on any I2C failure.
 */
HAL_StatusTypeDef AS3935_ReadAllRegisters(void) {
    /* Read contiguous block 0x00–0x08 (9 bytes) individually to avoid
     * mapping gaps (energy/distance are read-only and not in the shadow). */
    uint8_t raw[9];
    if (AS3935_ReadRegs(AS3935_REG_AFE_GAIN, raw, 9) != HAL_OK)
        return HAL_ERROR;
    AS3935_Sensor.POWER.Val.Value      = raw[0];
    AS3935_Sensor.NOISE.Val.Value      = raw[1];
    AS3935_Sensor.STATISTICS.Val.Value = raw[2];
    AS3935_Sensor.INT_FREQ.Val.Value   = raw[3];
    /* raw[4..7] = energy/distance — read on demand via AS3935_ReadLightningData */
    AS3935_Sensor.IRQ.Val.Value        = raw[8];
    if (AS3935_ReadRegs(AS3935_REG_CALIB_TRCO, &AS3935_Sensor.TRCO.Val.Value, 1) != HAL_OK)
        return HAL_ERROR;
    if (AS3935_ReadRegs(AS3935_REG_CALIB_SRCO, &AS3935_Sensor.SRCO.Val.Value, 1) != HAL_OK)
        return HAL_ERROR;
    DEBUG("AS3935 Register Dump:\n");
    DEBUG("  REG 0x00 [POWER]   = 0x%02X\n", AS3935_Sensor.POWER.Val.Value);
    DEBUG("  REG 0x01 [NOISE]   = 0x%02X\n", AS3935_Sensor.NOISE.Val.Value);
    DEBUG("  REG 0x02 [STATS]   = 0x%02X\n", AS3935_Sensor.STATISTICS.Val.Value);
    DEBUG("  REG 0x03 [INT_ANT] = 0x%02X\n", AS3935_Sensor.INT_FREQ.Val.Value);
    DEBUG("  REG 0x08 [IRQ_CAP] = 0x%02X\n", AS3935_Sensor.IRQ.Val.Value);
    DEBUG("  REG 0x3A [TRCO]    = 0x%02X\n", AS3935_Sensor.TRCO.Val.Value);
    DEBUG("  REG 0x3B [SRCO]    = 0x%02X\n", AS3935_Sensor.SRCO.Val.Value);
    return HAL_OK;
}

/* -------------------------------------------------------------------------
 * Configuration setters — each updates shadow and writes only the affected
 * register.
 * ---------------------------------------------------------------------- */

/** @brief Set sensor power state (AS3935_POWER_UP / AS3935_POWER_DOWN). */
HAL_StatusTypeDef AS3935_SetPower(AS3935_PowerState_t powerState) {
    AS3935_Sensor.POWER.Val.BitField.PWD = powerState;
    return AS3935_WriteReg(AS3935_REG_AFE_GAIN, (uint8_t *)&AS3935_Sensor.POWER.Val.Value, 1);
}

/** @brief Set AFE gain (AS3935_AFE_GAIN_INDOOR / AS3935_AFE_GAIN_OUTDOOR). */
HAL_StatusTypeDef AS3935_SetGain(AS3935_AFE_Gain_t gain) {
    AS3935_Sensor.POWER.Val.BitField.AFE_GB = gain;
    return AS3935_WriteReg(AS3935_REG_AFE_GAIN, (uint8_t *)&AS3935_Sensor.POWER.Val.Value, 1);
}

/** @brief Set watchdog threshold (AS3935_WDTH_0 … AS3935_WDTH_10). */
HAL_StatusTypeDef AS3935_SetWatchdog(AS3935_WDTH_t watchdogThreshold) {
    AS3935_Sensor.NOISE.Val.BitField.WDTH = watchdogThreshold;
    return AS3935_WriteReg(AS3935_REG_THRESHOLD, (uint8_t *)&AS3935_Sensor.NOISE.Val.Value, 1);
}

/** @brief Set noise floor level (AS3935_NF_390uVrms … AS3935_NF_2000uVrms). */
HAL_StatusTypeDef AS3935_SetNoiseFloor(AS3935_NoiseFloorLevel_t noiseLevel) {
    AS3935_Sensor.NOISE.Val.BitField.NF_LEV = noiseLevel;
    return AS3935_WriteReg(AS3935_REG_THRESHOLD, (uint8_t *)&AS3935_Sensor.NOISE.Val.Value, 1);
}

/** @brief Set spike rejection level (AS3935_SREJ_0 … AS3935_SREJ_11). */
HAL_StatusTypeDef AS3935_SetSpikeRejection(AS3935_SREJ_t spikeRejectionLevel) {
    AS3935_Sensor.STATISTICS.Val.BitField.SREJ = spikeRejectionLevel;
    return AS3935_WriteReg(AS3935_REG_LIGHTNING, (uint8_t *)&AS3935_Sensor.STATISTICS.Val.Value, 1);
}

/** @brief Set minimum lightning events before INT_L fires. */
HAL_StatusTypeDef AS3935_SetMinLightning(AS3935_MinLightning_t minEvents) {
    AS3935_Sensor.STATISTICS.Val.BitField.MIN_NUM_LIGH = minEvents;
    return AS3935_WriteReg(AS3935_REG_LIGHTNING, (uint8_t *)&AS3935_Sensor.STATISTICS.Val.Value, 1);
}

/**
 * @brief  Clear the internal lightning statistics.
 * @details Datasheet: toggle CL_STAT bit high→low (write 1 then 0).
 *          The shadow reserved bit [7] is always kept at 1.
 */
HAL_StatusTypeDef AS3935_ClearStatistics(void) {
    AS3935_Sensor.STATISTICS.Val.BitField.CL_STAT = 1;
    if (AS3935_WriteReg(AS3935_REG_LIGHTNING, (uint8_t *)&AS3935_Sensor.STATISTICS.Val.Value, 1) != HAL_OK)
        return HAL_ERROR;
    AS3935_Sensor.STATISTICS.Val.BitField.CL_STAT = 0;
    return AS3935_WriteReg(AS3935_REG_LIGHTNING, (uint8_t *)&AS3935_Sensor.STATISTICS.Val.Value, 1);
}

/** @brief Enable or disable disturber masking on IRQ. */
HAL_StatusTypeDef AS3935_SetDisturberMask(AS3935_MaskDist_t maskDist) {
    AS3935_Sensor.INT_FREQ.Val.BitField.MASK_DIST = maskDist;
    /* Write only the upper byte — keep INT [3:0] zero. */
    uint8_t val = AS3935_Sensor.INT_FREQ.Val.Value & 0xF0;
    return AS3935_WriteReg(AS3935_REG_INT_MASK_ANT, &val, 1);
}

/** @brief Set LCO frequency division ratio for antenna tuning output. */
HAL_StatusTypeDef AS3935_SetFrequencyDivisionRatio(AS3935_LCO_FDiv_t fdivRatio) {
    AS3935_Sensor.INT_FREQ.Val.BitField.LCO_FDIV = fdivRatio;
    uint8_t val = AS3935_Sensor.INT_FREQ.Val.Value & 0xF0;
    return AS3935_WriteReg(AS3935_REG_INT_MASK_ANT, &val, 1);
}

/** @brief Set internal tuning capacitor value. */
HAL_StatusTypeDef AS3935_SetTuningCapacitor(AS3935_TuneCap_t tuningCap) {
    AS3935_Sensor.IRQ.Val.BitField.TUN_CAP = tuningCap;
    return AS3935_WriteReg(AS3935_REG_FREQ_DISP_IRQ, (uint8_t *)&AS3935_Sensor.IRQ.Val.Value, 1);
}

/**
 * @brief  Select which oscillator (if any) is output on the IRQ pin.
 * @param  oscDisplay  AS3935_OSC_NONE / AS3935_OSC_TRCO / AS3935_OSC_SRCO / AS3935_OSC_LCO.
 */
HAL_StatusTypeDef AS3935_SetOscillatorDisplay(AS3935_OSCDisplay_t oscDisplay) {
    AS3935_Sensor.IRQ.Val.BitField.OSC_DISP = oscDisplay;
    return AS3935_WriteReg(AS3935_REG_FREQ_DISP_IRQ, (uint8_t *)&AS3935_Sensor.IRQ.Val.Value, 1);
}

/* -------------------------------------------------------------------------
 * Interrupt and data readback
 * ---------------------------------------------------------------------- */

/**
 * @brief  Read and decode the interrupt register.
 * @details Must be called at least 2 ms after the IRQ pin goes high.
 *          Reading REG0x03 clears the IRQ pin.
 * @return AS3935_INT_L / AS3935_INT_D / AS3935_INT_NH / AS3935_INT_NONE.
 */
AS3935_INT_t AS3935_GetInterruptType(void) {
    HAL_Delay(AS3935_IRQ_DELAY_MS); /* wait ≥ 2 ms per datasheet */
    if (AS3935_ReadRegs(AS3935_REG_INT_MASK_ANT, &AS3935_Sensor.INT_FREQ.Val.Value, 1) != HAL_OK) {
        DEBUG("AS3935: Failed to read INT register\n");
        return AS3935_INT_NONE;
    }
    AS3935_INT_t irqType = (AS3935_INT_t)AS3935_Sensor.INT_FREQ.Val.BitField.INT;
    switch (irqType) {
        case AS3935_INT_L:    DEBUG("AS3935: INT_L — lightning\n");       break;
        case AS3935_INT_D:    DEBUG("AS3935: INT_D — disturber\n");       break;
        case AS3935_INT_NH:   DEBUG("AS3935: INT_NH — noise too high\n"); break;
        default: DEBUG("AS3935: INT unknown 0x%02X\n", irqType);          break; //AS3935_INT_NONE
    }
    return irqType;
}

/**
 * @brief  Read lightning energy and distance from registers 0x04–0x07.
 * @details Energy is a dimensionless 20-bit value stored across three bytes:
 *            REG0x04[7:0] = S_LIG_L  (bits [7:0])
 *            REG0x05[7:0] = S_LIG_M  (bits [15:8])
 *            REG0x06[4:0] = S_LIG_MM (bits [19:16])
 *          Distance is REG0x07[5:0] — decoded with AS3935_Distance_t enum.
 * @param  data  Pointer to caller-allocated AS3935_LightningData_t.
 * @return HAL_OK, or HAL_ERROR on I2C failure.
 */
HAL_StatusTypeDef AS3935_ReadLightningData(AS3935_LightningData_t *data) {
    uint8_t raw[4];
    if (AS3935_ReadRegs(AS3935_REG_ENERGY_LSB, raw, 4) != HAL_OK)
        return HAL_ERROR;
    /* REG0x04 = LSB, REG0x05 = MSB, REG0x06[4:0] = MMSB */
    data->LightningEnergy = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) | ((uint32_t)(raw[2] & 0x1FU) << 16);
    /* REG0x07[5:0] = distance estimation */
    data->DistanceEstimation = raw[3] & 0x3FU;
    DEBUG("AS3935: Energy=%lu  Distance=0x%02X\n", (unsigned long)data->LightningEnergy, data->DistanceEstimation);
    return HAL_OK;
}

/* -------------------------------------------------------------------------
 * Calibration status
 * ---------------------------------------------------------------------- */

/** @brief Refresh TRCO calibration status in shadow. */
HAL_StatusTypeDef AS3935_ReadTRCOCalibStatus(void) {
    return AS3935_ReadRegs(AS3935_REG_CALIB_TRCO, &AS3935_Sensor.TRCO.Val.Value, 1);
}

/** @brief Refresh SRCO calibration status in shadow. */
HAL_StatusTypeDef AS3935_ReadSRCOCalibStatus(void) {
    return AS3935_ReadRegs(AS3935_REG_CALIB_SRCO, &AS3935_Sensor.SRCO.Val.Value, 1);
}

/* -------------------------------------------------------------------------
 * Sensitivity profiles
 * ---------------------------------------------------------------------- */

/**
 * @brief  Apply a pre-defined sensitivity profile.
 * @param  profile  One of the AS3935_Profile_t values.
 * @return HAL_OK, or HAL_ERROR if the profile index is unknown.
 */
HAL_StatusTypeDef AS3935_ApplySensitivityProfile(AS3935_Profile_t profile) {
    typedef struct {
        AS3935_AFE_Gain_t        gain;
        AS3935_NoiseFloorLevel_t nf;
        AS3935_WDTH_t            wdth;
        AS3935_SREJ_t            srej;
    } ProfileEntry_t;
    static const ProfileEntry_t profiles[] = {
			{ AS3935_AFE_GAIN_INDOOR,  AS3935_NF_860uVrms,  AS3935_WDTH_2, AS3935_SREJ_2  },	/* INDOOR_VERY_SENSITIVE     */
      { AS3935_AFE_GAIN_INDOOR,  AS3935_NF_1100uVrms, AS3935_WDTH_2, AS3935_SREJ_2  },	/* INDOOR_BALANCED_1         */
      { AS3935_AFE_GAIN_INDOOR,  AS3935_NF_1800uVrms, AS3935_WDTH_2, AS3935_SREJ_3  },	/* INDOOR_BALANCED_2         */
      { AS3935_AFE_GAIN_INDOOR,  AS3935_NF_2000uVrms, AS3935_WDTH_6, AS3935_SREJ_3  },	/* INDOOR_DISTURBER_RESIST   */
      { AS3935_AFE_GAIN_INDOOR,  AS3935_NF_2000uVrms, AS3935_WDTH_9, AS3935_SREJ_3  },	/* INDOOR_DISTURBER_STRICT   */
      { AS3935_AFE_GAIN_OUTDOOR, AS3935_NF_860uVrms,  AS3935_WDTH_2, AS3935_SREJ_2  },	/* OUTDOOR_VERY_SENSITIVE    */
      { AS3935_AFE_GAIN_OUTDOOR, AS3935_NF_1100uVrms, AS3935_WDTH_2, AS3935_SREJ_2  },	/* OUTDOOR_SENSITIVE         */
      { AS3935_AFE_GAIN_OUTDOOR, AS3935_NF_1800uVrms, AS3935_WDTH_3, AS3935_SREJ_3  },	/* OUTDOOR_BALANCED          */
      { AS3935_AFE_GAIN_OUTDOOR, AS3935_NF_2000uVrms, AS3935_WDTH_6, AS3935_SREJ_3  },	/* OUTDOOR_DISTURBER_RESIST  */
      { AS3935_AFE_GAIN_OUTDOOR, AS3935_NF_2000uVrms, AS3935_WDTH_7, AS3935_SREJ_7  },	/* OUTDOOR_MINIMAL_SENS      */
    };
    uint8_t idx = (uint8_t)profile - 1U;
    if (idx >= (uint8_t)(sizeof(profiles) / sizeof(profiles[0])))
        return HAL_ERROR;
    const ProfileEntry_t *p = &profiles[idx];
    if (AS3935_SetGain(p->gain) != HAL_OK)
			return HAL_ERROR;
    if (AS3935_SetNoiseFloor(p->nf) != HAL_OK)
			return HAL_ERROR;
    if (AS3935_SetWatchdog(p->wdth) != HAL_OK)
			return HAL_ERROR;
    if (AS3935_SetSpikeRejection(p->srej) != HAL_OK)
			return HAL_ERROR;
    return HAL_OK;
}

/* -------------------------------------------------------------------------
 * Antenna auto-tuning
 * ---------------------------------------------------------------------- */

/**
 * @brief  Find the tuning capacitor value that minimises combined frequency
 *         error across LCO (500 kHz), TRCO (32.768 kHz) and SRCO (1.1 MHz).
 * @details Iterates all 16 capacitor steps, measures each oscillator via
 *          TIM2 CH4 (input capture), and returns the best-fit cap value.
 *          The caller must pass the result to AS3935_SetTuningCapacitor().
 * @return  Best AS3935_TuneCap_t value found.
 */
AS3935_TuneCap_t AS3935_TuneAntenna(void) {
    /* Target frequencies per datasheet. */
    const uint32_t TARGET_LCO  = 500000;   /* Antenna resonance          */
    const uint32_t TARGET_TRCO = 32768;    /* Timer RC oscillator        */
    const uint32_t TARGET_SRCO = 1100000;  /* System RC oscillator       */
    AS3935_TuneCap_t bestCap  = AS3935_TUN_CAP_0pF;
    uint32_t minError = UINT32_MAX;
    /* Disable external interrupt to avoid spurious IRQ during tuning. */
    Disable_EXTI_AS3935(); // — call externally if needed

    /* LCO divider must be /16 before routing LCO to the IRQ pin. */
    AS3935_SetFrequencyDivisionRatio(AS3935_LCO_FDIV_16);
    for (AS3935_TuneCap_t cap = AS3935_TUN_CAP_0pF; cap <= AS3935_TUN_CAP_120pF; cap++) {
        AS3935_Sensor.IRQ.Val.BitField.TUN_CAP = cap;
        /* --- Measure LCO (antenna resonance, affected by tuning cap) --- */
        AS3935_Sensor.IRQ.Val.BitField.OSC_DISP = AS3935_OSC_LCO;
        AS3935_WriteReg(AS3935_REG_FREQ_DISP_IRQ, (uint8_t *)&AS3935_Sensor.IRQ.Val.Value, 1);
        HAL_Delay(50);
        uint32_t lcoFreq  = 16 * AS3935_MeasureFrequencyHz();
        /* --- Measure TRCO (~32.768 kHz, calibration quality check) --- */
        AS3935_Sensor.IRQ.Val.BitField.OSC_DISP = AS3935_OSC_TRCO;
        AS3935_WriteReg(AS3935_REG_FREQ_DISP_IRQ, (uint8_t *)&AS3935_Sensor.IRQ.Val.Value, 1);
        HAL_Delay(50);
        uint32_t trcoFreq = AS3935_MeasureFrequencyHz();
        /* --- Measure SRCO (~1.1 MHz, calibration quality check) --- */
        AS3935_Sensor.IRQ.Val.BitField.OSC_DISP = AS3935_OSC_SRCO;
        AS3935_WriteReg(AS3935_REG_FREQ_DISP_IRQ, (uint8_t *)&AS3935_Sensor.IRQ.Val.Value, 1);
        HAL_Delay(50);
        uint32_t srcoFreq = AS3935_MeasureFrequencyHz();
        /* Disable oscillator output before next iteration. */
        AS3935_Sensor.IRQ.Val.BitField.OSC_DISP = AS3935_OSC_NONE;
        AS3935_WriteReg(AS3935_REG_FREQ_DISP_IRQ, (uint8_t *)&AS3935_Sensor.IRQ.Val.Value, 1);
        /* Compute relative error in ppm for each oscillator so that all
         * three contribute equally regardless of their absolute frequency.
         *   ppm_error = (|f_meas - f_target| * 1 000 000) / f_target
         *
         * LCO is the only signal affected by the tuning capacitor, but
         * logging TRCO/SRCO lets you verify oscillator calibration health
         * across the full sweep without a separate measurement pass.       */
        uint32_t lcoPpm  = (AS3935_ABS_DIFF(lcoFreq,  TARGET_LCO)  * 1000000) / TARGET_LCO;
        uint32_t trcoPpm = (AS3935_ABS_DIFF(trcoFreq, TARGET_TRCO) * 1000000) / TARGET_TRCO;
        uint32_t srcoPpm = (AS3935_ABS_DIFF(srcoFreq, TARGET_SRCO) * 1000000) / TARGET_SRCO;
        /* Weight LCO 4x — it is the only signal the tuning cap affects.
         * TRCO and SRCO act as tie-breakers between caps with similar LCO
         * error, rewarding the cap value that keeps all oscillators healthy. */
        uint32_t combined = (4 * lcoPpm) + trcoPpm + srcoPpm;

        DEBUG("Cap %2d | LCO %6lu Hz (%5lu ppm) | TRCO %5lu Hz (%5lu ppm) | SRCO %7lu Hz (%5lu ppm) | score %lu\n", (int)cap,
              (unsigned long)lcoFreq,  (unsigned long)lcoPpm, (unsigned long)trcoFreq, (unsigned long)trcoPpm,
              (unsigned long)srcoFreq, (unsigned long)srcoPpm, (unsigned long)combined);
        if (combined < minError) {
            minError = combined;
            bestCap  = cap;
        }
    }
    /* Leave oscillator output disabled. */
    AS3935_Sensor.IRQ.Val.BitField.OSC_DISP = AS3935_OSC_NONE;
    AS3935_WriteReg(AS3935_REG_FREQ_DISP_IRQ, (uint8_t *)&AS3935_Sensor.IRQ.Val.Value, 1);
    DEBUG("AS3935: Best cap = %d  score = %lu ppm\n", (int)bestCap, (unsigned long)minError);
		Enable_EXTI_AS3935();
    return bestCap;
}