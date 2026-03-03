/**
 * @file HDC302x.h
 * @brief Header file for the HDC302x temperature and humidity sensor library.
 *
 * This library contains all the definitions, macros, and function prototypes
 * for initializing, configuring, and reading data from the HDC302x sensor.
 */

#ifndef __HDC302X_H
#define __HDC302X_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "i2c.h"
#include <stdint.h>

// Timing
#define HDC_READ_TIME						250	// Read HDC sensors every second (ms)
#define I2C_TIMEOUT							100	// I2C transaction timeout (ms)

// I2C addresses
#define HDC302X_SENSOR_1_ADDR		(0x44 << 1)	// I2C address for Sensor 1 (ADDR pin = GND)
#define HDC302X_SENSOR_2_ADDR		(0x45 << 1)	// I2C address for Sensor 2 (ADDR pin = VDD)

// Default thresholds
#define HDC302X_TEMP_THRESHOLD	((20.0f + 40.0f) / 165.0f) * 65536.0f	// Default temperature threshold (20 deg C)
#define HDC302X_HUM_THRESHOLD		(50.0f / 100.0f) * 65536.0f	// Default humidity threshold (50%)

// Conversion constants
#define HDC302X_RH_COEFF				0.00152587890625f	// Humidity conversion coefficient    (100 / 65535)
#define HDC302X_TEMP_COEFF1			0.0025177001953125f	// Temperature conversion coefficient (165 / 65535)
#define HDC302X_TEMP_COEFF2			40.0f	// Temperature offset (subtract 40 deg C)

// Dew point calculation constants (Magnus-Tetens approximation)
#define DEW_POINT_CONST_A				17.27f	// Magnus-Tetens constant A
#define DEW_POINT_CONST_B				237.7f	// Magnus-Tetens constant B

// Condensation detection and heater control thresholds
// Heater activates when (Temperature - DewPoint) < DEW_MARGIN_C
#define DEW_MARGIN_C						3.0f	// Start heater when within 3 deg C of dew point
#define HEATER_RH_CLEAR					5.0f	// Stop heater when RH drops below 5% (condensate gone)
#define HEATER_TIMEOUT_MS				20000U	// Max heater-on time 20s (safety cutoff)
#define HEATER_COOLING_MS				120000U	// Max cooling time 120s before resuming normal operation
// Per datasheet sec 7.3.7: ramp from ambient to 100 deg C must take 5-10s.
// HDC302X_POW_HALF produces a gentle ramp; full power risks bursting condensed droplets.
#define HEATER_POWER					HDC302X_POW_HALF	// Half power for safe condensation removal

/**
 * @brief Command structure.
 */
typedef struct __attribute__((packed)) {
    union {
        uint16_t	Command;			// Full 16-bit command
        uint8_t		CommandBytes[2];	// Byte-wise access to the command
    };
} HDC302x_Command_t;

// Trigger-On Demand Mode Commands
#define HDC302X_CMD_TRIGGER_ON_DEMAND_LOW_NOISE					((HDC302x_Command_t){{ .Command = 0x2400 }})	// On-demand measurement (low noise)
#define HDC302X_CMD_TRIGGER_ON_DEMAND_LOW_POWER_1				((HDC302x_Command_t){{ .Command = 0x240B }})	// On-demand measurement (low power 1)
#define HDC302X_CMD_TRIGGER_ON_DEMAND_LOW_POWER_2				((HDC302x_Command_t){{ .Command = 0x2416 }})	// On-demand measurement (low power 2)
#define HDC302X_CMD_TRIGGER_ON_DEMAND_LOW_POWER_3				((HDC302x_Command_t){{ .Command = 0x24FF }})	// On-demand measurement (low power 3)

// Auto Measurement Mode 0.5 Hz
#define HDC302X_CMD_AUTO_MEASUREMENT_1_EVERY_2S_LPM0			((HDC302x_Command_t){{ .Command = 0x2032 }})	// Auto measurement: 1 every 2 seconds (LPM0)
#define HDC302X_CMD_AUTO_MEASUREMENT_1_EVERY_2S_LPM1			((HDC302x_Command_t){{ .Command = 0x2024 }})	// Auto measurement: 1 every 2 seconds (LPM1)
#define HDC302X_CMD_AUTO_MEASUREMENT_1_EVERY_2S_LPM2			((HDC302x_Command_t){{ .Command = 0x202F }})	// Auto measurement: 1 every 2 seconds (LPM2)
#define HDC302X_CMD_AUTO_MEASUREMENT_1_EVERY_2S_LPM3			((HDC302x_Command_t){{ .Command = 0x20FF }})	// Auto measurement: 1 every 2 seconds (LPM3)

// Auto Measurement Mode 1 Hz
#define HDC302X_CMD_AUTO_MEASUREMENT_1_PER_SECOND_LPM0		((HDC302x_Command_t){{ .Command = 0x2130 }})	// Auto measurement: 1 per second (LPM0)
#define HDC302X_CMD_AUTO_MEASUREMENT_1_PER_SECOND_LPM1		((HDC302x_Command_t){{ .Command = 0x2126 }})	// Auto measurement: 1 per second (LPM1)
#define HDC302X_CMD_AUTO_MEASUREMENT_1_PER_SECOND_LPM2		((HDC302x_Command_t){{ .Command = 0x212D }})	// Auto measurement: 1 per second (LPM2)
#define HDC302X_CMD_AUTO_MEASUREMENT_1_PER_SECOND_LPM3		((HDC302x_Command_t){{ .Command = 0x21FF }})	// Auto measurement: 1 per second (LPM3)

// Auto Measurement Mode 2 Hz
#define HDC302X_CMD_AUTO_MEASUREMENT_2_PER_SECOND_LPM0		((HDC302x_Command_t){{ .Command = 0x2236 }})	// Auto measurement: 2 per second (LPM0)
#define HDC302X_CMD_AUTO_MEASUREMENT_2_PER_SECOND_LPM1		((HDC302x_Command_t){{ .Command = 0x2220 }})	// Auto measurement: 2 per second (LPM1)
#define HDC302X_CMD_AUTO_MEASUREMENT_2_PER_SECOND_LPM2		((HDC302x_Command_t){{ .Command = 0x222B }})	// Auto measurement: 2 per second (LPM2)
#define HDC302X_CMD_AUTO_MEASUREMENT_2_PER_SECOND_LPM3		((HDC302x_Command_t){{ .Command = 0x22FF }})	// Auto measurement: 2 per second (LPM3)

// Auto Measurement Mode 4 Hz
#define HDC302X_CMD_AUTO_MEASUREMENT_4_PER_SECOND_LPM0		((HDC302x_Command_t){{ .Command = 0x2334 }})	// Auto measurement: 4 per second (LPM0)
#define HDC302X_CMD_AUTO_MEASUREMENT_4_PER_SECOND_LPM1		((HDC302x_Command_t){{ .Command = 0x2322 }})	// Auto measurement: 4 per second (LPM1)
#define HDC302X_CMD_AUTO_MEASUREMENT_4_PER_SECOND_LPM2		((HDC302x_Command_t){{ .Command = 0x2329 }})	// Auto measurement: 4 per second (LPM2)
#define HDC302X_CMD_AUTO_MEASUREMENT_4_PER_SECOND_LPM3		((HDC302x_Command_t){{ .Command = 0x23FF }})	// Auto measurement: 4 per second (LPM3)

// Auto Measurement Mode 10 Hz
#define HDC302X_CMD_AUTO_MEASUREMENT_10_PER_SECOND_LPM0 	((HDC302x_Command_t){{ .Command = 0x2737 }})	// Auto measurement: 10 per second (LPM0)
#define HDC302X_CMD_AUTO_MEASUREMENT_10_PER_SECOND_LPM1 	((HDC302x_Command_t){{ .Command = 0x2721 }})	// Auto measurement: 10 per second (LPM1)
#define HDC302X_CMD_AUTO_MEASUREMENT_10_PER_SECOND_LPM2 	((HDC302x_Command_t){{ .Command = 0x272A }})	// Auto measurement: 10 per second (LPM2)
#define HDC302X_CMD_AUTO_MEASUREMENT_10_PER_SECOND_LPM3 	((HDC302x_Command_t){{ .Command = 0x27FF }})	// Auto measurement: 10 per second (LPM3)

// Trigger-On Demand Mode (with clock stretching)
#define HDC302X_CMD_TRIGGER_ON_DEMAND_LPM0								((HDC302x_Command_t){{ .Command = 0x2C06 }})	// Trigger-On Demand Mode (LPM0)
#define HDC302X_CMD_TRIGGER_ON_DEMAND_LPM1								((HDC302x_Command_t){{ .Command = 0x2C0D }})	// Trigger-On Demand Mode (LPM1)
#define HDC302X_CMD_TRIGGER_ON_DEMAND_LPM2								((HDC302x_Command_t){{ .Command = 0x2C10 }})	// Trigger-On Demand Mode (LPM2)

// Readout Commands
#define HDC302X_CMD_RETURN_TO_TRIGGER											((HDC302x_Command_t){{ .Command = 0x3093 }})	// Exit auto mode, return to Trigger-on Demand
#define HDC302X_CMD_MEASURE_READ													((HDC302x_Command_t){{ .Command = 0xE000 }})	// Measurement readout of T and RH
#define HDC302X_CMD_READ_RH_ONLY													((HDC302x_Command_t){{ .Command = 0xE001 }})	// Measurement readout of RH only

// Measurement History Commands
#define HDC302X_CMD_READ_HISTORY_MIN_TEMPERATURE					((HDC302x_Command_t){{ .Command = 0xE002 }})	// History readout: minimum temperature
#define HDC302X_CMD_READ_HISTORY_MAX_TEMPERATURE					((HDC302x_Command_t){{ .Command = 0xE003 }})	// History readout: maximum temperature
#define HDC302X_CMD_READ_HISTORY_MIN_RH										((HDC302x_Command_t){{ .Command = 0xE004 }})	// History readout: minimum RH
#define HDC302X_CMD_READ_HISTORY_MAX_RH										((HDC302x_Command_t){{ .Command = 0xE005 }})	// History readout: maximum RH

// ALERT Threshold Commands
#define HDC302X_CMD_CONFIGURE_ALERT_THRESHOLD_SET_LOW			((HDC302x_Command_t){{ .Command = 0x6100 }})	// Configure threshold: Set Low Alert
#define HDC302X_CMD_CONFIGURE_ALERT_THRESHOLD_SET_HIGH		((HDC302x_Command_t){{ .Command = 0x611D }})	// Configure threshold: Set High Alert
#define HDC302X_CMD_CONFIGURE_ALERT_THRESHOLD_CLEAR_LOW		((HDC302x_Command_t){{ .Command = 0x610B }})	// Configure threshold: Clear Low Alert
#define HDC302X_CMD_CONFIGURE_ALERT_THRESHOLD_CLEAR_HIGH	((HDC302x_Command_t){{ .Command = 0x6116 }})	// Configure threshold: Clear High Alert

// Read ALERT Threshold Commands
#define HDC302X_CMD_READ_ALERT_THRESHOLD_SET_LOW					((HDC302x_Command_t){{ .Command = 0xE102 }})	// Read threshold: Set Low Alert
#define HDC302X_CMD_READ_ALERT_THRESHOLD_SET_HIGH					((HDC302x_Command_t){{ .Command = 0xE11F }})	// Read threshold: Set High Alert
#define HDC302X_CMD_READ_ALERT_THRESHOLD_CLEAR_LOW				((HDC302x_Command_t){{ .Command = 0xE109 }})	// Read threshold: Clear Low Alert
#define HDC302X_CMD_READ_ALERT_THRESHOLD_CLEAR_HIGH				((HDC302x_Command_t){{ .Command = 0xE114 }})	// Read threshold: Clear High Alert

// Heater Commands
#define HDC302X_CMD_HEATER_ENABLE													((HDC302x_Command_t){{ .Command = 0x306D }})	// Enable integrated heater
#define HDC302X_CMD_HEATER_DISABLE												((HDC302x_Command_t){{ .Command = 0x3066 }})	// Disable integrated heater
#define HDC302X_CMD_HEATER_CONFIG													((HDC302x_Command_t){{ .Command = 0x306E }})	// Configure and read back heater settings

// Status Register Commands
#define HDC302X_READ_STATUS																((HDC302x_Command_t){{ .Command = 0xF32D }})	// Read status register
#define HDC302X_CLEAR_STATUS															((HDC302x_Command_t){{ .Command = 0x3041 }})	// Clear status register

// Reset Commands
#define HDC302X_CMD_SOFT_RESET														((HDC302x_Command_t){{ .Command = 0x30A2 }})	// Soft reset

// NIST and Manufacturer ID Commands
#define HDC302X_CMD_READ_NIST_ID_BYTE_5_4									((HDC302x_Command_t){{ .Command = 0x3683 }})	// Read NIST ID bytes 5 and 4
#define HDC302X_CMD_READ_NIST_ID_BYTE_3_2									((HDC302x_Command_t){{ .Command = 0x3684 }})	// Read NIST ID bytes 3 and 2
#define HDC302X_CMD_READ_NIST_ID_BYTE_1_0									((HDC302x_Command_t){{ .Command = 0x3685 }})	// Read NIST ID bytes 1 and 0
#define HDC302X_CMD_READ_MANUFACTURER_ID									((HDC302x_Command_t){{ .Command = 0x3781 }})	// Read manufacturer ID

// NVM Programming Commands
#define HDC302X_CMD_PROGRAM_ALERT_THRESHOLDS_TO_NVM				((HDC302x_Command_t){{ .Command = 0x6155 }})	// Program ALERT thresholds to NVM
#define HDC302X_CMD_PROGRAM_READ_OFFSET_VALUES						((HDC302x_Command_t){{ .Command = 0xA004 }})	// Program/read offset values for RH and T
#define HDC302X_CMD_PROGRAM_READ_DEFAULT_STATE						((HDC302x_Command_t){{ .Command = 0x61BB }})	// Program/read default power-on measurement state

/**
 * @brief Configuration structure for default power-on/reset state.
 */
typedef struct __attribute__((packed)) {
    uint8_t CFG_MSB;	// Configuration MSB
    uint8_t CFG_LSB;	// Configuration LSB
    uint8_t CFG_CRC;	// CRC for the configuration bytes
} HDC302x_Config_t;

// Predefined configurations — 0.5 Hz
#define HDC302X_CONFIG_0_5HZ_LOWEST_NOISE	((HDC302x_Config_t){ .CFG_MSB = 0x00, .CFG_LSB = 0x03, .CFG_CRC = 0xD2 })	// 0.5 Hz, Lowest Noise
#define HDC302X_CONFIG_0_5HZ_LPM1					((HDC302x_Config_t){ .CFG_MSB = 0x00, .CFG_LSB = 0x13, .CFG_CRC = 0x91 })	// 0.5 Hz, Low Power Mode 1
#define HDC302X_CONFIG_0_5HZ_LPM2					((HDC302x_Config_t){ .CFG_MSB = 0x00, .CFG_LSB = 0x23, .CFG_CRC = 0x54 })	// 0.5 Hz, Low Power Mode 2
#define HDC302X_CONFIG_0_5HZ_LPM3					((HDC302x_Config_t){ .CFG_MSB = 0x00, .CFG_LSB = 0x33, .CFG_CRC = 0x17 })	// 0.5 Hz, Low Power Mode 3

// Predefined configurations — 1 Hz
#define HDC302X_CONFIG_1HZ_LOWEST_NOISE		((HDC302x_Config_t){ .CFG_MSB = 0x00, .CFG_LSB = 0x05, .CFG_CRC = 0x74 })	// 1 Hz, Lowest Noise
#define HDC302X_CONFIG_1HZ_LPM1						((HDC302x_Config_t){ .CFG_MSB = 0x00, .CFG_LSB = 0x15, .CFG_CRC = 0x37 })	// 1 Hz, Low Power Mode 1
#define HDC302X_CONFIG_1HZ_LPM2						((HDC302x_Config_t){ .CFG_MSB = 0x00, .CFG_LSB = 0x25, .CFG_CRC = 0xF2 })	// 1 Hz, Low Power Mode 2
#define HDC302X_CONFIG_1HZ_LPM3						((HDC302x_Config_t){ .CFG_MSB = 0x00, .CFG_LSB = 0x35, .CFG_CRC = 0xB1 })	// 1 Hz, Low Power Mode 3

// Predefined configurations — 2 Hz
#define HDC302X_CONFIG_2HZ_LOWEST_NOISE		((HDC302x_Config_t){ .CFG_MSB = 0x00, .CFG_LSB = 0x07, .CFG_CRC = 0x16 })	// 2 Hz, Lowest Noise
#define HDC302X_CONFIG_2HZ_LPM1						((HDC302x_Config_t){ .CFG_MSB = 0x00, .CFG_LSB = 0x17, .CFG_CRC = 0x55 })	// 2 Hz, Low Power Mode 1
#define HDC302X_CONFIG_2HZ_LPM2						((HDC302x_Config_t){ .CFG_MSB = 0x00, .CFG_LSB = 0x27, .CFG_CRC = 0x90 })	// 2 Hz, Low Power Mode 2
#define HDC302X_CONFIG_2HZ_LPM3						((HDC302x_Config_t){ .CFG_MSB = 0x00, .CFG_LSB = 0x37, .CFG_CRC = 0xD3 })	// 2 Hz, Low Power Mode 3

// Predefined configurations — 4 Hz
#define HDC302X_CONFIG_4HZ_LOWEST_NOISE		((HDC302x_Config_t){ .CFG_MSB = 0x00, .CFG_LSB = 0x09, .CFG_CRC = 0x09 })	// 4 Hz, Lowest Noise
#define HDC302X_CONFIG_4HZ_LPM1						((HDC302x_Config_t){ .CFG_MSB = 0x00, .CFG_LSB = 0x19, .CFG_CRC = 0x4A })	// 4 Hz, Low Power Mode 1
#define HDC302X_CONFIG_4HZ_LPM2						((HDC302x_Config_t){ .CFG_MSB = 0x00, .CFG_LSB = 0x29, .CFG_CRC = 0x8F })	// 4 Hz, Low Power Mode 2
#define HDC302X_CONFIG_4HZ_LPM3						((HDC302x_Config_t){ .CFG_MSB = 0x00, .CFG_LSB = 0x39, .CFG_CRC = 0xCC })	// 4 Hz, Low Power Mode 3

// Predefined configurations — 10 Hz
#define HDC302X_CONFIG_10HZ_LOWEST_NOISE	((HDC302x_Config_t){ .CFG_MSB = 0x00, .CFG_LSB = 0x0B, .CFG_CRC = 0x6B })	// 10 Hz, Lowest Noise
#define HDC302X_CONFIG_10HZ_LPM1					((HDC302x_Config_t){ .CFG_MSB = 0x00, .CFG_LSB = 0x1B, .CFG_CRC = 0x28 })	// 10 Hz, Low Power Mode 1
#define HDC302X_CONFIG_10HZ_LPM2					((HDC302x_Config_t){ .CFG_MSB = 0x00, .CFG_LSB = 0x2B, .CFG_CRC = 0xED })	// 10 Hz, Low Power Mode 2
#define HDC302X_CONFIG_10HZ_LPM3					((HDC302x_Config_t){ .CFG_MSB = 0x00, .CFG_LSB = 0x3B, .CFG_CRC = 0xAE })	// 10 Hz, Low Power Mode 3

// Factory default configuration
#define HDC302X_CONFIG_FACTORY_DEFAULT		((HDC302x_Config_t){ .CFG_MSB = 0x00, .CFG_LSB = 0x00, .CFG_CRC = 0x81 })	// Restore factory default (sleep mode)

/**
 * @brief Heater configuration structure.
 */
typedef struct __attribute__((packed)) {
    uint8_t MSB;	// Most Significant Byte of the heater power
    uint8_t LSB;	// Least Significant Byte of the heater power
    uint8_t HCRC;	// CRC for the heater configuration bytes
} HDC302x_HeaterConfig_t;

// Predefined heater power levels
#define HDC302X_POW_OFF			((HDC302x_HeaterConfig_t){ .MSB = 0x00, .LSB = 0x00, .HCRC = 0x81 })	// Heater off
#define HDC302X_POW_1				((HDC302x_HeaterConfig_t){ .MSB = 0x00, .LSB = 0x01, .HCRC = 0xB0 })	// Heater power level 1
#define HDC302X_POW_2				((HDC302x_HeaterConfig_t){ .MSB = 0x00, .LSB = 0x02, .HCRC = 0xE3 })	// Heater power level 2
#define HDC302X_POW_4				((HDC302x_HeaterConfig_t){ .MSB = 0x00, .LSB = 0x04, .HCRC = 0x45 })	// Heater power level 4
#define HDC302X_POW_8				((HDC302x_HeaterConfig_t){ .MSB = 0x00, .LSB = 0x08, .HCRC = 0x38 })	// Heater power level 8
#define HDC302X_POW_F				((HDC302x_HeaterConfig_t){ .MSB = 0x00, .LSB = 0x0F, .HCRC = 0xAF })	// Heater power level F
#define HDC302X_POW_QUARTER	((HDC302x_HeaterConfig_t){ .MSB = 0x00, .LSB = 0x9F, .HCRC = 0x96 })	// Heater quarter power (datasheet Table 7-15: 009F CRC 96)
#define HDC302X_POW_HALF		((HDC302x_HeaterConfig_t){ .MSB = 0x03, .LSB = 0xFF, .HCRC = 0x00 })	// Heater half power   (datasheet Table 7-15: 03FF CRC 00)
#define HDC302X_POW_10			((HDC302x_HeaterConfig_t){ .MSB = 0x00, .LSB = 0x10, .HCRC = 0xC2 })	// Heater power level 10
#define HDC302X_POW_20			((HDC302x_HeaterConfig_t){ .MSB = 0x00, .LSB = 0x20, .HCRC = 0x07 })	// Heater power level 20
#define HDC302X_POW_40			((HDC302x_HeaterConfig_t){ .MSB = 0x00, .LSB = 0x40, .HCRC = 0xBC })	// Heater power level 40
#define HDC302X_POW_80			((HDC302x_HeaterConfig_t){ .MSB = 0x00, .LSB = 0x80, .HCRC = 0xFB })	// Heater power level 80
#define HDC302X_POW_F0			((HDC302x_HeaterConfig_t){ .MSB = 0x00, .LSB = 0xF0, .HCRC = 0x03 })	// Heater power level F0
#define HDC302X_POW_100			((HDC302x_HeaterConfig_t){ .MSB = 0x01, .LSB = 0x00, .HCRC = 0x75 })	// Heater power level 100
#define HDC302X_POW_200			((HDC302x_HeaterConfig_t){ .MSB = 0x02, .LSB = 0x00, .HCRC = 0x58 })	// Heater power level 200
#define HDC302X_POW_400			((HDC302x_HeaterConfig_t){ .MSB = 0x04, .LSB = 0x00, .HCRC = 0x02 })	// Heater power level 400
#define HDC302X_POW_800			((HDC302x_HeaterConfig_t){ .MSB = 0x08, .LSB = 0x00, .HCRC = 0xB6 })	// Heater power level 800
#define HDC302X_POW_F00			((HDC302x_HeaterConfig_t){ .MSB = 0x0F, .LSB = 0x00, .HCRC = 0x18 })	// Heater power level F00
#define HDC302X_POW_1000		((HDC302x_HeaterConfig_t){ .MSB = 0x10, .LSB = 0x00, .HCRC = 0xEF })	// Heater power level 1000
#define HDC302X_POW_2000		((HDC302x_HeaterConfig_t){ .MSB = 0x20, .LSB = 0x00, .HCRC = 0x5D })	// Heater power level 2000
#define HDC302X_POW_3FFF		((HDC302x_HeaterConfig_t){ .MSB = 0x3F, .LSB = 0xFF, .HCRC = 0x06 })	// Heater full power (3FFF)

/**
 * @brief Heater state machine states for condensation removal (datasheet sec 7.3.7).
 * @note  Call HDC302x_UpdateHeater() on every HDC read interrupt to advance the state.
 *        While state != HDC_HEATER_OFF measurements are elevated — do not use T/RH for
 *        atmospheric calculations until state returns to HDC_HEATER_OFF.
 */
typedef enum {
	HDC_HEATER_OFF			= 0,	// Normal — heater inactive, monitoring dew point
	HDC_HEATER_ACTIVE		= 1,	// Heater running — evaporating condensate
	HDC_HEATER_COOLING	= 2		// Heater off — cooling down, readings unreliable
} HDC302x_HeaterState_t;

/**
 * @brief Sensor operating modes.
 */
typedef enum {
    HDC302X_MODE_NORMAL			= 0x0,	// Normal operation mode
    HDC302X_MODE_LOW_POWER	= 0x1		// Low power operation mode
} HDC302x_Mode_t;

/**
 * @brief Structure to hold temperature and humidity data.
 */
typedef struct {
    float Temperature;	// Temperature in degrees Celsius
    float Humidity;			// Relative humidity in percent
} HDC302x_Data_t;

/**
 * @brief Structure to hold history data (min/max temperature and humidity).
 */
typedef struct {
    HDC302x_Data_t MAX;	// Maximum recorded temperature (deg C) and RH (%)
    HDC302x_Data_t MIN;	// Minimum recorded temperature (deg C) and RH (%)
} HDC302x_History_t;

/**
 * @brief Status register structure.
 */
typedef struct __attribute__((packed)) {
    union {
        uint16_t Value;	// Complete 16-bit register value
        struct {
            uint16_t checksum_error	: 1;	// Bit  0: Checksum error on last write (0 = pass, 1 = fail)
            uint16_t reserved1			: 3;	// Bits 1-3: Reserved
            uint16_t device_reset		: 1;	// Bit  4: Device reset detected (0 = no reset, 1 = reset)
            uint16_t reserved2			: 1;	// Bit  5: Reserved
            uint16_t t_low_alert		: 1;	// Bit  6: T Low tracking alert
            uint16_t t_high_alert		: 1;	// Bit  7: T High tracking alert
            uint16_t rh_low_alert		: 1;	// Bit  8: RH Low tracking alert
            uint16_t rh_high_alert	: 1;	// Bit  9: RH High tracking alert
            uint16_t t_alert				: 1;	// Bit 10: T tracking alert
            uint16_t rh_alert				: 1;	// Bit 11: RH tracking alert
            uint16_t reserved3			: 1;	// Bit 12: Reserved
            uint16_t heater_status	: 1;	// Bit 13: Heater status (0 = disabled, 1 = enabled)
            uint16_t reserved4			: 1;	// Bit 14: Reserved
            uint16_t overall_alert	: 1;	// Bit 15: Overall alert (0 = none, 1 = at least one active)
        } BitField;
    } Val;
} HDC302x_Status_t;

/**
 * @brief Main sensor structure representing an HDC302x device.
 */
typedef struct {
    uint8_t									Address;				// I2C address of the sensor (pre-shifted)
    HDC302x_Command_t				Cmd;						// Current command issued to the sensor
    HDC302x_Config_t				Config;					// Current configuration settings
    HDC302x_HeaterConfig_t	Heater;					// Heater configuration settings
    HDC302x_Status_t				Status;					// Status register data
    HDC302x_Data_t					Data;						// Latest temperature and humidity readings
    HDC302x_History_t				History;				// Min/max history data
    HDC302x_HeaterState_t		HeaterState;		// Current condensation-removal state
    uint32_t								HeaterStartMs;	// HAL_GetTick() when heater was enabled
    float										AmbientTemp;		// Ambient T captured before heater starts (for cooling check)
} HDC302x_t;

// Function prototypes
void HDC302x_Reset();
/**
 * @brief HDC302x_Init function.
 * @param sensorObj Pointer to sensor handle. Caller must set Address and hi2c.
 * @return HAL status.
 */
HAL_StatusTypeDef HDC302x_Init(HDC302x_t* sensorObj);
HAL_StatusTypeDef HDC302x_StartAutoMeasurement(HDC302x_t* sensorObj, HDC302x_Command_t cmd);
/**
 * @brief HDC302x_ReadData function.
 * @param sensorObj Pointer to sensor handle.
 * @return HAL status.
 */
HAL_StatusTypeDef HDC302x_ReadData(HDC302x_t* sensorObj);
/**
 * @brief HDC302x_ReadStatus function.
 * @param sensorObj Pointer to sensor handle.
 * @return HAL status.
 */
HAL_StatusTypeDef HDC302x_ReadStatus(HDC302x_t* sensorObj);
/**
 * @brief HDC302x_ClearStatus function.
 * @param sensorObj Pointer to sensor handle.
 * @return HAL status.
 */
HAL_StatusTypeDef HDC302x_ClearStatus(HDC302x_t* sensorObj);
/**
 * @brief HDC302x_ReadHistory function.
 * @param sensorObj Pointer to sensor handle.
 * @return HAL status.
 */
HAL_StatusTypeDef HDC302x_ReadHistory(HDC302x_t* sensorObj);
/**
 * @brief HDC302x_GetDewPoint function.
 * @param sensorObj Pointer to sensor handle (requires valid Data.Temperature/Humidity).
 * @return Dew point in degrees Celsius.
 */
float HDC302x_GetDewPoint(HDC302x_t* sensorObj);
HAL_StatusTypeDef HDC302x_HeaterEnable(HDC302x_t* sensorObj, HDC302x_HeaterConfig_t power);
HAL_StatusTypeDef HDC302x_HeaterDisable(HDC302x_t* sensorObj);
HDC302x_HeaterState_t HDC302x_UpdateHeater(HDC302x_t* sensorObj);

#ifdef __cplusplus
}
#endif

#endif /* __HDC302X_H */
