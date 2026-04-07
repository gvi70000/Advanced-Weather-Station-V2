/***************************************************************************
 * @file    AS3935.h
 * @brief   STM32G4 HAL driver for the AS3935 Franklin Lightning Sensor.
 *
 * MIT License
 * Copyright (c) 2024 Grozea Ion gvi70000
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ***************************************************************************/

#ifndef AS3935_H
#define AS3935_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* -------------------------------------------------------------------------
 * Bus & timing constants
 * ---------------------------------------------------------------------- */

/// @brief 8-bit I2C write address (7-bit addr 0x03, ADD1=1, ADD0=1).
#define AS3935_I2C_ADDRESS      0x06U   /* (0x03 << 1) */

/// @brief Value written to REG0x3C / REG0x3D to execute a direct command.
#define AS3935_DIRECT_COMMAND   0x96U

/// @brief Minimum delay (ms) after issuing a direct command (datasheet: 2 ms).
#define AS3935_CMD_DELAY_MS     4U

/// @brief Delay (ms) after CALIB_RCO before disabling oscillator display.
#define AS3935_CALIB_DELAY_MS   20U

/// @brief Delay (ms) after IRQ goes high before reading INT register.
#define AS3935_IRQ_DELAY_MS     2U

/// @brief Helper: absolute difference of two unsigned values.
#define AS3935_ABS_DIFF(a, b)   ((a) > (b) ? ((a) - (b)) : ((b) - (a)))

/// @brief Target LCO antenna frequency (Hz).
#define AS3935_TARGET_FREQUENCY     500000U

/// @brief Acceptable LCO frequency tolerance (Hz) — 3.5 % of 500 kHz.
#define AS3935_FREQUENCY_TOLERANCE  17500U

/* =========================================================================
 * REG 0x00 — Power management and AFE gain boost
 * ======================================================================= */

/**
 * @brief Sensor power state (REG0x00[0], PWD).
 */
typedef enum {
    AS3935_POWER_UP   = 0x00,  ///< Active (listening) mode.
    AS3935_POWER_DOWN = 0x01   ///< Power-down mode (I2C still active).
} AS3935_PowerState_t;

/**
 * @brief AFE gain setting (REG0x00[5:1], AFE_GB).
 *
 * Must match the deployment environment; incorrect setting yields poor results.
 */
typedef enum {
    AS3935_AFE_GAIN_OUTDOOR = 0x0E, ///< Outdoor mode  (REG value 01110b).
    AS3935_AFE_GAIN_INDOOR  = 0x12  ///< Indoor mode   (REG value 10010b).
} AS3935_AFE_Gain_t;

/**
 * @brief REG0x00 — Power / AFE gain register.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t PWD      : 1; ///< [0]   Power-down (1=off, 0=on).
            uint8_t AFE_GB   : 5; ///< [5:1] AFE gain boost (0–31).
            uint8_t Reserved : 2; ///< [7:6] Reserved, keep 0.
        } BitField;
    } Val;
} AS3935_PWR_t;

/* =========================================================================
 * REG 0x01 — Noise floor level and watchdog threshold
 * ======================================================================= */

/**
 * @brief Watchdog threshold (REG0x01[3:0], WDTH).
 *
 * WDTH=0 → widest window, highest detection efficiency (~100 %).
 * WDTH=10 → narrowest window, lowest detection efficiency (~0 %).
 * See datasheet Figure 40 for detection-efficiency curves.
 */
typedef enum {
    AS3935_WDTH_0  = 0x00, ///< Highest sensitivity (~100 % efficiency).
    AS3935_WDTH_1  = 0x01,
    AS3935_WDTH_2  = 0x02,
    AS3935_WDTH_3  = 0x03,
    AS3935_WDTH_4  = 0x04,
    AS3935_WDTH_5  = 0x05,
    AS3935_WDTH_6  = 0x06,
    AS3935_WDTH_7  = 0x07,
    AS3935_WDTH_8  = 0x08,
    AS3935_WDTH_9  = 0x09,
    AS3935_WDTH_10 = 0x0A  ///< Lowest sensitivity (~0 % efficiency).
} AS3935_WDTH_t;

/**
 * @brief Continuous noise floor threshold (REG0x01[6:4], NF_LEV).
 *
 * Outdoor values are for the standard antenna; indoor values are lower.
 * Default after PRESET_DEFAULT: NF_LEV = 010 (860 µVrms outdoor).
 */
typedef enum {
    AS3935_NF_390uVrms  = 0x00, ///< Outdoor 390 µVrms  / Indoor  28 µVrms.
    AS3935_NF_630uVrms  = 0x01, ///< Outdoor 630 µVrms  / Indoor  45 µVrms.
    AS3935_NF_860uVrms  = 0x02, ///< Outdoor 860 µVrms  / Indoor  62 µVrms (default).
    AS3935_NF_1100uVrms = 0x03, ///< Outdoor 1100 µVrms / Indoor  78 µVrms.
    AS3935_NF_1140uVrms = 0x04, ///< Outdoor 1140 µVrms / Indoor  95 µVrms.
    AS3935_NF_1570uVrms = 0x05, ///< Outdoor 1570 µVrms / Indoor 112 µVrms.
    AS3935_NF_1800uVrms = 0x06, ///< Outdoor 1800 µVrms / Indoor 130 µVrms.
    AS3935_NF_2000uVrms = 0x07  ///< Outdoor 2000 µVrms / Indoor 146 µVrms.
} AS3935_NoiseFloorLevel_t;

/**
 * @brief REG0x01 — Noise floor / watchdog threshold register.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t WDTH     : 4; ///< [3:0] Watchdog threshold (0–10).
            uint8_t NF_LEV   : 3; ///< [6:4] Noise floor level (0–7).
            uint8_t Reserved : 1; ///< [7]   Reserved, keep 0.
        } BitField;
    } Val;
} AS3935_NOISE_t;

/* =========================================================================
 * REG 0x02 — Spike rejection and lightning statistics
 * ======================================================================= */

/**
 * @brief Spike rejection level (REG0x02[3:0], SREJ).
 *
 * Higher values improve disturber rejection at the cost of detection
 * efficiency. Default after PRESET_DEFAULT: SREJ = 0010.
 */
typedef enum {
    AS3935_SREJ_0  = 0x00, ///< Highest efficiency (most sensitive).
    AS3935_SREJ_1  = 0x01,
    AS3935_SREJ_2  = 0x02, ///< Default.
    AS3935_SREJ_3  = 0x03,
    AS3935_SREJ_4  = 0x04,
    AS3935_SREJ_5  = 0x05,
    AS3935_SREJ_6  = 0x06,
    AS3935_SREJ_7  = 0x07,
    AS3935_SREJ_8  = 0x08,
    AS3935_SREJ_9  = 0x09,
    AS3935_SREJ_10 = 0x0A,
    AS3935_SREJ_11 = 0x0B  ///< Lowest efficiency (most robust).
} AS3935_SREJ_t;

/**
 * @brief Minimum number of lightning events required before INT_L fires
 *        (REG0x02[5:4], MIN_NUM_LIGH).
 */
typedef enum {
    AS3935_MIN_LIGHTNING_1  = 0x00, ///< 1 event  (fastest response).
    AS3935_MIN_LIGHTNING_5  = 0x01, ///< 5 events.
    AS3935_MIN_LIGHTNING_9  = 0x02, ///< 9 events.
    AS3935_MIN_LIGHTNING_16 = 0x03  ///< 16 events (most conservative).
} AS3935_MinLightning_t;

/**
 * @brief REG0x02 — Spike rejection / lightning statistics register.
 *
 * @note REG0x02[7] is a reserved bit whose default value is 1 (per datasheet
 *       Figure 23). It must be preserved in every write to this register.
 *       The shadow byte is initialised with 0x80 in AS3935_Init().
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t SREJ         : 4; ///< [3:0] Spike rejection (0–11).
            uint8_t MIN_NUM_LIGH : 2; ///< [5:4] Min lightning events.
            uint8_t CL_STAT      : 1; ///< [6]   Clear statistics (toggle 0→1→0).
            uint8_t Reserved     : 1; ///< [7]   Reserved — default 1, keep 1.
        } BitField;
    } Val;
} AS3935_STATISTICS_t;

/* =========================================================================
 * REG 0x03 — Interrupt type and antenna frequency tuning
 * ======================================================================= */

/**
 * @brief Interrupt type reported in REG0x03[3:0] (read-only).
 *
 * Read this register at least 2 ms after the IRQ pin goes high.
 */
typedef enum {
    AS3935_INT_NONE = 0x00, ///< No interrupt (or register already cleared).
    AS3935_INT_NH   = 0x01, ///< Noise level too high.
    AS3935_INT_D    = 0x04, ///< Disturber detected.
    AS3935_INT_L    = 0x08  ///< Lightning strike detected.
} AS3935_INT_t;

/**
 * @brief Disturber interrupt mask (REG0x03[5], MASK_DIST).
 */
typedef enum {
    AS3935_MASK_DIST_OFF = 0, ///< Disturber events raise INT_D on IRQ pin.
    AS3935_MASK_DIST_ON  = 1  ///< Disturber events silenced on IRQ pin.
} AS3935_MaskDist_t;

/**
 * @brief LCO frequency division ratio for antenna tuning (REG0x03[7:6]).
 *
 * The antenna's resonance frequency is divided before output on IRQ.
 */
typedef enum {
    AS3935_LCO_FDIV_16  = 0x00, ///< Divide by 16.
    AS3935_LCO_FDIV_32  = 0x01, ///< Divide by 32.
    AS3935_LCO_FDIV_64  = 0x02, ///< Divide by 64.
    AS3935_LCO_FDIV_128 = 0x03  ///< Divide by 128.
} AS3935_LCO_FDiv_t;

/**
 * @brief REG0x03 — Interrupt / frequency-divider register.
 *
 * @note The INT field [3:0] is read-only; writes to those bits are ignored.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t INT       : 4; ///< [3:0] Interrupt type (read-only).
            uint8_t Reserved  : 1; ///< [4]   Reserved, keep 0.
            uint8_t MASK_DIST : 1; ///< [5]   Disturber mask.
            uint8_t LCO_FDIV  : 2; ///< [7:6] LCO frequency division ratio.
        } BitField;
    } Val;
} AS3935_INT_FREQ_t;

/* =========================================================================
 * REG 0x04–0x07 — Lightning energy and distance estimation
 * ======================================================================= */

/**
 * @brief Distance estimation values decoded from REG0x07[5:0].
 *
 * Values are binary-encoded kilometres (see datasheet Figure 43).
 * Only the listed values are valid; intermediate codes are not assigned.
 */
typedef enum {
    AS3935_DIST_OUT_OF_RANGE   = 0x3F, ///< 63 — storm out of 40 km range.
    AS3935_DIST_OVERHEAD       = 0x01, ///< Storm is directly overhead.
    AS3935_DIST_5_KM           = 0x05, ///< 5 km.
    AS3935_DIST_6_KM           = 0x06, ///< 6 km.
    AS3935_DIST_8_KM           = 0x08, ///< 8 km.
    AS3935_DIST_10_KM          = 0x0A, ///< 10 km.
    AS3935_DIST_12_KM          = 0x0C, ///< 12 km.
    AS3935_DIST_14_KM          = 0x0E, ///< 14 km.
    AS3935_DIST_17_KM          = 0x11, ///< 17 km.
    AS3935_DIST_20_KM          = 0x14, ///< 20 km.
    AS3935_DIST_24_KM          = 0x18, ///< 24 km.
    AS3935_DIST_27_KM          = 0x1B, ///< 27 km.
    AS3935_DIST_31_KM          = 0x1F, ///< 31 km.
    AS3935_DIST_34_KM          = 0x22, ///< 34 km.
    AS3935_DIST_37_KM          = 0x25, ///< 37 km.
    AS3935_DIST_40_KM          = 0x28  ///< 40 km.
} AS3935_Distance_t;

/**
 * @brief Combined lightning energy and distance result.
 *
 * Populated by AS3935_ReadLightningData().
 * Energy is a dimensionless 20-bit value (no physical unit).
 * Distance is decoded from REG0x07[5:0] — see AS3935_Distance_t.
 */
typedef struct {
    uint32_t LightningEnergy;   ///< 20-bit energy (REG0x04–0x06[4:0]).
    uint8_t  DistanceEstimation;///< 6-bit distance code (REG0x07[5:0]).
} AS3935_LightningData_t;

/* =========================================================================
 * REG 0x08 — Oscillator display and internal tuning capacitors
 * ======================================================================= */

/**
 * @brief Internal tuning capacitor setting (REG0x08[3:0], TUN_CAP).
 *
 * Each step adds 8 pF in parallel with the antenna LC circuit.
 * Range: 0 pF (0x00) to 120 pF (0x0F).
 */
typedef enum {
    AS3935_TUN_CAP_0pF   = 0x00,
    AS3935_TUN_CAP_8pF   = 0x01,
    AS3935_TUN_CAP_16pF  = 0x02,
    AS3935_TUN_CAP_24pF  = 0x03,
    AS3935_TUN_CAP_32pF  = 0x04,
    AS3935_TUN_CAP_40pF  = 0x05,
    AS3935_TUN_CAP_48pF  = 0x06,
    AS3935_TUN_CAP_56pF  = 0x07,
    AS3935_TUN_CAP_64pF  = 0x08,
    AS3935_TUN_CAP_72pF  = 0x09,
    AS3935_TUN_CAP_80pF  = 0x0A,
    AS3935_TUN_CAP_88pF  = 0x0B,
    AS3935_TUN_CAP_96pF  = 0x0C,
    AS3935_TUN_CAP_104pF = 0x0D,
    AS3935_TUN_CAP_112pF = 0x0E,
    AS3935_TUN_CAP_120pF = 0x0F
} AS3935_TuneCap_t;

/**
 * @brief Oscillator output selection on the IRQ pin (REG0x08[7:5]).
 *
 * Three independent bits: DISP_LCO[7], DISP_SRCO[6], DISP_TRCO[5].
 * Only one oscillator should be enabled at a time for measurement.
 * Datasheet CALIB_RCO wakeup procedure requires DISP_SRCO (bit 6) = 1.
 */
typedef enum {
    AS3935_OSC_NONE  = 0x00, ///< All oscillator outputs disabled (normal mode).
    AS3935_OSC_TRCO  = 0x01, ///< TRCO (~32.768 kHz) on IRQ — bit5 set.
    AS3935_OSC_SRCO  = 0x02, ///< SRCO (~1.1 MHz)    on IRQ — bit6 set.
    AS3935_OSC_LCO   = 0x04  ///< LCO  (500 kHz/div) on IRQ — bit7 set.
} AS3935_OSCDisplay_t;

/**
 * @brief REG0x08 — IRQ oscillator display / tuning capacitor register.
 *
 * OSC_DISP occupies three separate bits [7:5]:
 *   bit7 = DISP_LCO, bit6 = DISP_SRCO, bit5 = DISP_TRCO.
 * They are grouped here as a 3-bit field for compact access; the
 * AS3935_OSCDisplay_t enum values (1/2/4) map directly onto those bits.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t TUN_CAP  : 4; ///< [3:0] Tuning capacitors (0–15 × 8 pF).
            uint8_t Reserved : 1; ///< [4]   Reserved, keep 0.
            uint8_t OSC_DISP : 3; ///< [7:5] Oscillator display (TRCO/SRCO/LCO).
        } BitField;
    } Val;
} AS3935_IRQ_t;

/* =========================================================================
 * REG 0x3A — TRCO calibration status (read-only)
 * ======================================================================= */

/**
 * @brief REG0x3A — Timer RC Oscillator calibration status.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t Reserved        : 6; ///< [5:0] Reserved.
            uint8_t TRCO_CALIB_NOK  : 1; ///< [6]   1 = calibration failed.
            uint8_t TRCO_CALIB_DONE : 1; ///< [7]   1 = calibration successful.
        } BitField;
    } Val;
} AS3935_TRCO_t;

/* =========================================================================
 * REG 0x3B — SRCO calibration status (read-only)
 * ======================================================================= */

/**
 * @brief REG0x3B — System RC Oscillator calibration status.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t Reserved        : 6; ///< [5:0] Reserved.
            uint8_t SRCO_CALIB_NOK  : 1; ///< [6]   1 = calibration failed.
            uint8_t SRCO_CALIB_DONE : 1; ///< [7]   1 = calibration successful.
        } BitField;
    } Val;
} AS3935_SRCO_t;

/* =========================================================================
 * Complete register shadow
 * ======================================================================= */

/**
 * @brief Shadow copy of all writable AS3935 registers.
 *
 * Maintained by the driver; read-only fields (energy, distance, INT,
 * calibration status) are refreshed on demand.
 */
typedef struct __attribute__((packed)) {
    AS3935_PWR_t        POWER;      ///< REG0x00 — Power / AFE gain.
    AS3935_NOISE_t      NOISE;      ///< REG0x01 — Noise floor / watchdog.
    AS3935_STATISTICS_t STATISTICS; ///< REG0x02 — Spike rejection / stats.
    AS3935_INT_FREQ_t   INT_FREQ;   ///< REG0x03 — Interrupt / LCO divider.
    AS3935_IRQ_t        IRQ;        ///< REG0x08 — Oscillator display / cap.
    AS3935_TRCO_t       TRCO;       ///< REG0x3A — TRCO calibration status.
    AS3935_SRCO_t       SRCO;       ///< REG0x3B — SRCO calibration status.
} AS3935_REGS_t;

/* =========================================================================
 * Sensitivity profiles
 * ======================================================================= */

/**
 * @brief Pre-defined sensitivity profiles combining gain, noise floor,
 *        watchdog threshold and spike rejection.
 */
typedef enum {
    AS3935_PROFILE_INDOOR_VERY_SENSITIVE = 1,
    AS3935_PROFILE_INDOOR_BALANCED_1,
    AS3935_PROFILE_INDOOR_BALANCED_2,
    AS3935_PROFILE_INDOOR_DISTURBER_RESISTANT,
    AS3935_PROFILE_INDOOR_DISTURBER_STRICT,
    AS3935_PROFILE_OUTDOOR_VERY_SENSITIVE,
    AS3935_PROFILE_OUTDOOR_SENSITIVE,
    AS3935_PROFILE_OUTDOOR_BALANCED,
    AS3935_PROFILE_OUTDOOR_DISTURBER_RESISTANT,
    AS3935_PROFILE_OUTDOOR_MINIMAL_SENSITIVITY
} AS3935_Profile_t;

/* =========================================================================
 * Public API
 * ======================================================================= */

HAL_StatusTypeDef AS3935_Init(void);
HAL_StatusTypeDef AS3935_ReadAllRegisters(void);

HAL_StatusTypeDef AS3935_SetPower(AS3935_PowerState_t powerState);
HAL_StatusTypeDef AS3935_SetGain(AS3935_AFE_Gain_t gain);
HAL_StatusTypeDef AS3935_SetWatchdog(AS3935_WDTH_t watchdogThreshold);
HAL_StatusTypeDef AS3935_SetNoiseFloor(AS3935_NoiseFloorLevel_t noiseLevel);
HAL_StatusTypeDef AS3935_SetSpikeRejection(AS3935_SREJ_t spikeRejectionLevel);
HAL_StatusTypeDef AS3935_SetMinLightning(AS3935_MinLightning_t minEvents);
HAL_StatusTypeDef AS3935_ClearStatistics(void);

AS3935_INT_t      AS3935_GetInterruptType(void);
HAL_StatusTypeDef AS3935_ReadLightningData(AS3935_LightningData_t *data);

HAL_StatusTypeDef AS3935_SetDisturberMask(AS3935_MaskDist_t maskDist);
HAL_StatusTypeDef AS3935_SetFrequencyDivisionRatio(AS3935_LCO_FDiv_t fdivRatio);
HAL_StatusTypeDef AS3935_SetTuningCapacitor(AS3935_TuneCap_t tuningCap);
HAL_StatusTypeDef AS3935_SetOscillatorDisplay(AS3935_OSCDisplay_t oscDisplay);

HAL_StatusTypeDef AS3935_ReadTRCOCalibStatus(void);
HAL_StatusTypeDef AS3935_ReadSRCOCalibStatus(void);

HAL_StatusTypeDef AS3935_ApplySensitivityProfile(AS3935_Profile_t profile);
AS3935_TuneCap_t  AS3935_TuneAntenna(void);

#endif /* AS3935_H */
