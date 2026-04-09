/***************************************************************************
 * @file [PGA460].h/.c
 * This file contains definitions, data structures, and functions
 * for interfacing with the PGA460 ultrasonic transducer driver.
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

// TO BE USED WITH PGA460 Configuration Tool.html to generate registers values

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "usart.h"
#include "tim.h"
#include "PGA460.h"
#include "debug.h"
#include "cordic.h"         /* CORDIC peripheral for atan2 and sqrt */

extern UART_HandleTypeDef huart3;
// Timeout for blocking UART transfers (ms). Increase for low baud rates.
#define PGA460_UART_TIMEOUT_MS  50U
// Array to hold the moment in time when the DECPL pin of each PGA460 goes high filled by TIM2 CH1/2/3 DM
extern volatile DecplTimes_t  ToF_Result[3];  
extern volatile uint8_t Decpl_RDY;


#ifndef M_PI
	#define M_PI 3.14159265358979323846f
#endif

// Reflection Path Length
#define DISTANCE						0.15588f		// Total reflection path length in meters

#define R_D									287.05f			// Specific gas constant for dry air (J/kg.K)
#define R_V									461.495f		// Specific gas constant for water vapor (J/kg.K)
#define GAMMA								1.4f				// Adiabatic index for air
#define GAMMA_R_D							GAMMA * R_D				// GAMMA * R_D
#define L										0.0065f			// Temperature lapse rate (K/m)
#define T0									288.15f			// Standard temperature at sea level (K)
#define P0									101325.0f		// Standard pressure at sea level (Pa)
#define G										9.80665f		// Gravitational acceleration (m/s2)
#define M										0.0289644f	// Molar mass of air (kg/mol)
#define R										8.3144598f	// Universal gas constant (J/(mol.K))
#define KELVIN_OFFSET				273.15f			// Conversion from Celsius to Kelvin
#define RH_DIVISOR					100.0f			// Converts RH percentage to a fraction
#define WATER_VAPOR_EFFECT	0.6077f			// Effect of water vapor on speed of sound
#define SATURATION_CONSTANT	6.1078f			// Constant for saturation vapor pressure calculation
#define SOUND_SPEED			343.0f
#define INV_SQRT3						0.57735026919f			// 1/sqrt(3)
#define DEG_PER_RAD					57.29577951308232f	// 180/pi
// Conversion factor: half round-trip, microseconds ? seconds
#define TOF_US_TO_S					(1e-6f)   // = 1f * 1e-6
// Geometry 
// 3 transducers on a circle: cone base D = 100 mm, at 120deg.
// They form a cone with the reflector in the tip at 150mm from the base
// The cone semi angle is 17.745deg, the Tx-Reflector-Rx distance is 2x157.493=314.986mm
// if c=496um/us the burst will get to receiver after 635us
// if c=306um/us the burst will get to receiver after 1036us
// 6 pulses at 58kHz are 104us
// Time window of interst is from 600us to 1200us
	
#define PGA460_BASE_DIAMETER_UM				100000	// In micro meters
#define PGA460_DISTANCE_UM						316228	// In micro meters
#define PGA460_CAPTURE_DELAY_MS				70			// Delay for sensor measurement to complete

#define PGA_CMD_SIZE				3
#define PGA_READ_SIZE				4
#define PGA_WRITE_SIZE			5
#define PGA_CMD_COUNT				26
#define PGA_MAX_OBJECTS			8 // PGA460 supports up to 8 objects per measurement
#define PGA_OBJECTS_TRACKED	1	// Number of objects we want to track (Must be = PGA_MAX_OBJECTS)

#if PGA_OBJECTS_TRACKED > PGA_MAX_OBJECTS
	#error "PGA_OBJECTS_TRACKED cannot be greater than PGA_MAX_OBJECTS!"
#endif
#define PGA_OBJ_DATA_SIZE (2 + (PGA_OBJECTS_TRACKED * 4))

#define ULTRASONIC_SENSOR_COUNT	3	// Number of ultrasonic sensors in the array


// TIM2 is 170MHz on STM32G431 and is setup to have 1us period
#define TOF_MIN_US			400.0f
#define TOF_MAX_US			2000.0f
#define PGA460_TEMP_ERR	100.0f

const PGA460_Regs_t s0 = {
		.EEData.UserData = {0}, // 0x00..0x13
		.EEData.TVG = {
		.TVGAIN0 = 48, .TVGAIN1 = 80, .TVGAIN2 = 0, .TVGAIN3 = 255,
		.TVGAIN4 = 240, .TVGAIN5 = 0, .TVGAIN6 = 0
	},
	.EEData.Sett = {
		.INIT_GAIN		= 0,		// low initial gain (near target)
		.FREQUENCY		= 144,	// 58.0 kHz (0.2*140+30) CUSA center freq
		.DEADTIME			= 0,		// deglitch=32 us, pulse DT=0
		.PULSE_P1			= 102,	// IO_IF_SEL=0 (time-based), DIAG=1, P1=4 pulses
		.PULSE_P2			= 0,		// P2=0 pulses (addr=0)
		.CURR_LIM_P1	= 63,		// keep your hardware-safe limits
		.CURR_LIM_P2	= 0,
		.REC_LENGTH		= 0,		// 4.096 ms window per preset
		.FREQ_DIAG		= 16,		// standard diag enable
		.SAT_FDIAG_TH	= 0,		// TI typical
		.FVOLT_DEC		= 0,		// TI typical
		.DECPL_TEMP		= 0,		// TI typical
		.DSP_SCALE		= 0,
		.TEMP_TRIM		= 136,
		.P1_GAIN_CTRL = 0,
		.P2_GAIN_CTRL = 0
	},
		.eeCtrl = { .Val.Value = 0 },
		.Filters = {
		.BPF_A2_MSB=137, .BPF_A2_LSB=97, .BPF_A3_MSB=252, .BPF_A3_LSB=206, .BPF_B1_MSB=1,
		.BPF_B1_LSB=153, .LPF_A2_MSB=128, .LPF_A2_LSB=205, .LPF_B1_MSB=0, .LPF_B1_LSB=103
	},
		.TestMux = { .Val.Value = 0 },
		.Stat0 = { .Val.Value = 0 },
		.Stat1 = { .Val.Value = 0 },
		.THR = {
		.P1_THR = {49, 65, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 7},
		.P2_THR = {49, 65, 0, 0, 0, 0, 248, 1, 255, 255, 255, 255, 255, 255, 255, 7}
		}
	};

const PGA460_Regs_t s1 = {
		.EEData.UserData = {0}, // 0x00..0x13
		.EEData.TVG = {
		.TVGAIN0 = 48, .TVGAIN1 = 80, .TVGAIN2 = 0, .TVGAIN3 = 255,
		.TVGAIN4 = 240, .TVGAIN5 = 0, .TVGAIN6 = 0
	},
	.EEData.Sett = {
		.INIT_GAIN		= 0,		// low initial gain (near target)
		.FREQUENCY		= 144,	// 58.0 kHz (0.2*140+30) CUSA center freq
		.DEADTIME			= 0,		// deglitch=32 us, pulse DT=0
		.PULSE_P1			= 102,	// IO_IF_SEL=0 (time-based), DIAG=1, P1=4 pulses
		.PULSE_P2			= 32,		// P2=0 pulses (addr=0)
		.CURR_LIM_P1	= 63,		// keep your hardware-safe limits
		.CURR_LIM_P2	= 0,
		.REC_LENGTH		= 0,		// 4.096 ms window per preset
		.FREQ_DIAG		= 16,		// standard diag enable
		.SAT_FDIAG_TH	= 0,		// TI typical
		.FVOLT_DEC		= 0,		// TI typical
		.DECPL_TEMP		= 0,		// TI typical
		.DSP_SCALE		= 0,
		.TEMP_TRIM		= 136,
		.P1_GAIN_CTRL = 0,
		.P2_GAIN_CTRL = 0
	},
		.eeCtrl = { .Val.Value = 0 },
		.Filters = {
		.BPF_A2_MSB=137, .BPF_A2_LSB=97, .BPF_A3_MSB=252, .BPF_A3_LSB=206, .BPF_B1_MSB=1,
		.BPF_B1_LSB=153, .LPF_A2_MSB=128, .LPF_A2_LSB=205, .LPF_B1_MSB=0, .LPF_B1_LSB=103
	},
		.TestMux = { .Val.Value = 0 },
		.Stat0 = { .Val.Value = 0 },
		.Stat1 = { .Val.Value = 0 },
		.THR = {
		.P1_THR = {49, 65, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 7},
		.P2_THR = {49, 65, 0, 0, 0, 0, 248, 1, 255, 255, 255, 255, 255, 255, 255, 7}
		}
	};

const PGA460_Regs_t s2 = {
		.EEData.UserData = {0}, // 0x00..0x13
		.EEData.TVG = {
		.TVGAIN0 = 48, .TVGAIN1 = 80, .TVGAIN2 = 0, .TVGAIN3 = 255,
		.TVGAIN4 = 240, .TVGAIN5 = 0, .TVGAIN6 = 0
	},
	.EEData.Sett = {
		.INIT_GAIN		= 0,		// low initial gain (near target)
		.FREQUENCY		= 144,	// 58.0 kHz (0.2*140+30) CUSA center freq
		.DEADTIME			= 0,		// deglitch=32 us, pulse DT=0
		.PULSE_P1			= 102,	// IO_IF_SEL=0 (time-based), DIAG=1, P1=4 pulses
		.PULSE_P2			= 64,		// P2=0 pulses (addr=0)
		.CURR_LIM_P1	= 63,		// keep your hardware-safe limits
		.CURR_LIM_P2	= 0,
		.REC_LENGTH		= 0,		// 4.096 ms window per preset
		.FREQ_DIAG		= 16,		// standard diag enable
		.SAT_FDIAG_TH	= 0,		// TI typical
		.FVOLT_DEC		= 0,		// TI typical
		.DECPL_TEMP		= 0,		// TI typical
		.DSP_SCALE		= 0,
		.TEMP_TRIM		= 136,
		.P1_GAIN_CTRL = 0,
		.P2_GAIN_CTRL = 0
	},
		.eeCtrl = { .Val.Value = 0 },
		.Filters = {
		.BPF_A2_MSB=137, .BPF_A2_LSB=97, .BPF_A3_MSB=252, .BPF_A3_LSB=206, .BPF_B1_MSB=1,
		.BPF_B1_LSB=153, .LPF_A2_MSB=128, .LPF_A2_LSB=205, .LPF_B1_MSB=0, .LPF_B1_LSB=103
	},
		.TestMux = { .Val.Value = 0 },
		.Stat0 = { .Val.Value = 0 },
		.Stat1 = { .Val.Value = 0 },
		.THR = {
		.P1_THR = {49, 65, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 7},
		.P2_THR = {49, 65, 0, 0, 0, 0, 248, 1, 255, 255, 255, 255, 255, 255, 255, 7}
		}
	};

// --- Array with 3 elements(sensors): [0] TVG, [1] Settings, [2] Thresholds ---
PGA460_Sensor_t sensors[ULTRASONIC_SENSOR_COUNT] = {{ .Registers = s0 }, { .Registers = s1 }, { .Registers = s2 }};

/* PGA460_EnvData_t is declared in PGA460.h
   It is filled in the main.c where we read the
   sensors and we fill the data in structure*/

PGA460_EnvData_t externalData = {
    .Height      = 58.0f,     // meters
    .Temperature = 23.63f,    // degC
    .RH          = 62.22f,    // %
    .Pressure    = 1027.7f,   // hPa
    .SoundSpeed  = 0.0f       // will be computed
};

// Helper to convert float to Q31 for CORDIC
static int32_t float_to_q31(float val) {
    if (val >= 1.0f) return 0x7FFFFFFF;
    if (val <= -1.0f) return 0x80000000;
    return (int32_t)(val * 2147483648.0f);
}

// Helper to convert Q31 back to float
static float q31_to_float(int32_t qval) {
    return (float)qval / 2147483648.0f;
}

/**
 * @brief Compute speed of sound in moist air from environmental data.
 * @details Uses virtual temperature method accounting for temperature, atmospheric
 *          pressure, and relative humidity.  If Pressure <= 0, it is estimated from
 *          Height using the ISA troposphere barometric formula.  Result is stored in
 *          env->SoundSpeed and also returned.
 * @param env  Pointer to PGA460_EnvData_t (Temperature, RH, Pressure, Height).
 * @return Speed of sound in m/s(um/us), or 343.0f if env is NULL.
 */
static float PGA460_ComputeSoundSpeed(PGA460_EnvData_t *env) {
    if (!env) return SOUND_SPEED;
	const float T_C = env->Temperature;
    const float T_K = T_C + KELVIN_OFFSET;

    // --- Pressure (hPa). If not provided, estimate from height with std. atmosphere (0?11 km) ---
    float P_hPa = env->Pressure;
    if (P_hPa <= 0.0) {
        const float h = env->Height; // meters
        // Barometric formula (troposphere):
        // P = P0 * (1 - L*h/T0)^(g*M/(R*L))  [Pa]
        const float term = 1.0 - (L * h) / T0;
        const float expo = (G * M) / (R * L);
        float P_Pa = P0 * pow(term, expo);
        if (P_Pa < 1.0) P_Pa = 1.0; // guard
        P_hPa = P_Pa * 0.01;        // convert Pa to hPa
        env->Pressure = (float)P_hPa;
    }
	// --- Relative humidity as fraction ---
    const float RH_frac = env->RH / RH_DIVISOR;
	// --- Saturation vapor pressure over water (Tetens, hPa) ---
    // es = 6.1078 * exp(17.2693882 * T_C / (T_C + 237.3))
    const float es = SATURATION_CONSTANT * expf(17.269f * T_C / (T_C + 237.3f));
	// Actual vapor pressure (hPa)
    const float e = RH_frac * es;
	 // Mixing ratio (kg/kg); 0.62198  Rd/Rv ratio factor
    const float w = 0.62198f * e / fmaxf(1e-6f, (P_hPa - e));
	// Specific humidity
    const float q = w / (1.0f + w);
	// Virtual temperature Tv = T * (1 + (Rv/Rd - 1)*q); WATER_VAPOR_EFFECT 0.6077
    const float Tv = T_K * (1.0f + 0.6077f * q);
    // Speed of sound in moist air: c = sqrt(gamma * Rd * Tv)
    float c = sqrtf(GAMMA_R_D * Tv); 
    env->SoundSpeed = c;
    return c;
}

/**
 * @brief PGA460_UpdateEnvironment function implementation.
 * @details Writes tempC, rh, hPa, and height_m into externalData and calls
 *          PGA460_ComputeSoundSpeed() to refresh externalData.SoundSpeed.
 *          Call whenever BMP581, HDC302x, or GPS data is refreshed.
 * @param tempC  Ambient temperature in degrees Celsius.
 * @param rh  Relative humidity in %.
 * @param hPa  Atmospheric pressure in hPa.
 * @param height_m  Altitude in metres above sea level (0 = estimate from pressure).
 */
void PGA460_UpdateEnvironment(float tempC, float rh, float hPa, float height_m) {
    externalData.Temperature = tempC;
    externalData.RH          = rh;
    externalData.Pressure    = hPa;
    externalData.Height      = height_m;
    /* Recompute speed of sound and store in externalData.SoundSpeed */
    PGA460_ComputeSoundSpeed(&externalData);
}

/**
 * @brief Read all EEPROM registers individually from a sensor and print them via printf.
 * @details Iterates through addresses 0x00-0x2B, reads each register with
 *          PGA460_RegisterRead(), and logs the named value.  Used during development
 *          to verify EEPROM contents after a bulk write or burn.
 * @param sensorID  Sensor UART address (0-7).
 */
static void PGA460_ReadAndPrintEEPROM(uint8_t sensorID) {
	printf("Reading and printing EEPROM registers for Sensor %u individually...\n", sensorID);
	uint8_t regValue = 0;
	// EEPROM registers range from 0x00 to 0x2B
	for (uint8_t regAddr = 0x00; regAddr <= 0x2B; ++regAddr) {
		// Read the single register
		if (PGA460_RegisterRead(sensorID, regAddr, &regValue) == HAL_OK) {
			const char* regName;
			switch (regAddr) {
				case REG_USER_DATA1: regName = "USER_DATA1"; break;
				case REG_USER_DATA2: regName = "USER_DATA2"; break;
				case REG_USER_DATA3: regName = "USER_DATA3"; break;
				case REG_USER_DATA4: regName = "USER_DATA4"; break;
				case REG_USER_DATA5: regName = "USER_DATA5"; break;
				case REG_USER_DATA6: regName = "USER_DATA6"; break;
				case REG_USER_DATA7: regName = "USER_DATA7"; break;
				case REG_USER_DATA8: regName = "USER_DATA8"; break;
				case REG_USER_DATA9: regName = "USER_DATA9"; break;
				case REG_USER_DATA10: regName = "USER_DATA10"; break;
				case REG_USER_DATA11: regName = "USER_DATA11"; break;
				case REG_USER_DATA12: regName = "USER_DATA12"; break;
				case REG_USER_DATA13: regName = "USER_DATA13"; break;
				case REG_USER_DATA14: regName = "USER_DATA14"; break;
				case REG_USER_DATA15: regName = "USER_DATA15"; break;
				case REG_USER_DATA16: regName = "USER_DATA16"; break;
				case REG_USER_DATA17: regName = "USER_DATA17"; break;
				case REG_USER_DATA18: regName = "USER_DATA18"; break;
				case REG_USER_DATA19: regName = "USER_DATA19"; break;
				case REG_USER_DATA20: regName = "USER_DATA20"; break;
				case REG_TVGAIN0: regName = "REG_TVGAIN0"; break;
				case REG_TVGAIN1: regName = "REG_TVGAIN1"; break;
				case REG_TVGAIN2: regName = "REG_TVGAIN2"; break;
				case REG_TVGAIN3: regName = "REG_TVGAIN3"; break;
				case REG_TVGAIN4: regName = "REG_TVGAIN4"; break;
				case REG_TVGAIN5: regName = "REG_TVGAIN5"; break;
				case REG_TVGAIN6: regName = "REG_TVGAIN6"; break;
				case REG_INIT_GAIN: regName = "REG_INIT_GAIN"; break;
				case REG_FREQUENCY: regName = "REG_FREQUENCY"; break;
				case REG_DEADTIME: regName = "REG_DEADTIME"; break;
				case REG_PULSE_P1: regName = "REG_PULSE_P1"; break;
				case REG_PULSE_P2: regName = "REG_PULSE_P2"; break;
				case REG_CURR_LIM_P1: regName = "REG_CURR_LIM_P1"; break;
				case REG_CURR_LIM_P2: regName = "REG_CURR_LIM_P2"; break;
				case REG_REC_LENGTH: regName = "REG_REC_LENGTH"; break;
				case REG_FREQ_DIAG: regName = "REG_FREQ_DIAG"; break;
				case REG_SAT_FDIAG_TH: regName = "REG_SAT_FDIAG_TH"; break;
				case REG_FVOLT_DEC: regName = "REG_FVOLT_DEC"; break;
				case REG_DECPL_TEMP: regName = "REG_DECPL_TEMP"; break;
				case REG_DSP_SCALE: regName = "DSP_SCALE"; break;
				case REG_TEMP_TRIM: regName = "REG_TEMP_TRIM"; break;
				case REG_P1_GAIN_CTRL: regName = "REG_P1_GAIN_CTRL"; break;
				case REG_P2_GAIN_CTRL: regName = "REG_P2_GAIN_CTRL"; break;
				case REG_EE_CRC: regName = "EE_CRC"; break;
				default: regName = "UNKNOWN"; break;
			}
			printf("  %s = %d;\n", regName, regValue);
		} else {
			printf("  Address 0x%02X: FAILED to read\n", regAddr);
		}
		HAL_Delay(100);
	}
	printf("EEPROM individual read complete.\n");
}

/**
 * @brief Compute the PGA460 UART frame checksum.
 * @details Sums all bytes in data[], folds any carry exceeding 0xFF by subtracting 255,
 *          and returns the bitwise complement of the 8-bit result.  The PGA460 protocol
 *          requires this checksum as the final byte of every outgoing frame.
 * @param data  Pointer to the byte array to checksum (typically CMD + payload).
 * @param len  Number of bytes to include in the sum.
 * @return Ones-complement checksum byte.
 */
static uint8_t PGA460_CalculateChecksum(const uint8_t *data, uint8_t len) {
	uint16_t carry = 0;
	for (uint8_t i = 0; i < len; i++) {
		carry += data[i];
		if (carry > 0xFF) {
			carry -= 255;
		}
	}
	return (uint8_t)(~carry & 0xFF);
}

/**
 * @brief Unlock or lock the PGA460 EEPROM by writing REG_EE_CTRL (0x40).
 * @details Sends a single-register write with PGA460_UNLOCK_EEPROM (0x68) to enable
 *          programming, or PGA460_LOCK_EEPROM (0x69) to trigger the burn and lock.
 *          Updates sensors[sensorID].Registers.eepromUnlocked on success.
 * @param sensorID  Sensor UART address (0-7).
 * @param unlock  1 = unlock (write 0x68); 0 = lock/burn (write 0x69).
 * @return HAL status.
 */
static HAL_StatusTypeDef PGA460_SetEEPROMAccess(const uint8_t sensorID, const uint8_t unlock) {
	uint8_t accessByte = unlock ? PGA460_UNLOCK_EEPROM : PGA460_LOCK_EEPROM;
	uint8_t cmd[5] = {PGA460_SYNC, (uint8_t)((sensorID << 5) | PGA460_CMD_REGISTER_WRITE), REG_EE_CTRL, accessByte, 0};
	cmd[4] = PGA460_CalculateChecksum(&cmd[1], 3); // Checksum over CMD, ADDR, DATA
	if (HAL_UART_Transmit(&huart3, cmd, sizeof(cmd), PGA460_UART_TIMEOUT_MS) != HAL_OK) {
		DEBUG("Sensor %d: EEPROM %s failed\n", sensorID, unlock ? "unlock" : "lock");
		return HAL_ERROR;
	}
	// Update status flag on success
	sensors[sensorID].Registers.eepromUnlocked = unlock ? 1 : 0;
	return HAL_OK;
}

/**
 * @brief PGA460_ScanAndReport function implementation.
 * @details Probes UART addresses 0-7 by reading REG_PULSE_P2 from each.  For each
 *          responding sensor, prints the UART address, the internal address stored in
 *          PULSE_P2[7:5], and the P2 pulse count.  Flags any address mismatch.
 *          Useful for initial bus setup and after PGA460_WriteUARTAddr().
 */
void PGA460_ScanAndReport(void) {
    uint8_t regValue = 0;
    uint8_t foundCount = 0;
    printf("\n--- Starting PGA460 Bus Scan (Addresses 0-7) ---\n");
    for (uint8_t addr = 0; addr <= 7; addr++) {
        // We use a shorter timeout for scanning to keep it fast
        if (PGA460_RegisterRead(addr, REG_PULSE_P2, &regValue) == HAL_OK) {
            foundCount++;            
            // Extract the address bits [7:5] from the register value
            uint8_t internalAddr = (regValue >> 5);
            uint8_t pulseCount = (regValue & 0x1F);
            printf("  [+] SENSOR FOUND at UART Address: %u\n", addr);
            printf("      -> Internal Register Address: %u\n", internalAddr);
            printf("      -> Pulse Count (P2): %u\n", pulseCount);
            if (addr != internalAddr) {
                printf("      [!] WARNING: Address Mismatch! (Listening on %u, Stored as %u)\n", addr, internalAddr);
            }
        }
        // Small delay to let the UART bus settle between pings
        HAL_Delay(10); 
    }
    if (foundCount == 0) {
        printf("  [-] No sensors detected on the bus. Check power and wiring.\n");
    } else {
        printf("--- Scan Complete. Found %u sensor(s). ---\n\n", foundCount);
    }
}

/**
 * @brief PGA460_WriteUARTAddr function implementation.
 * @details Scans the bus for the first responding sensor, writes the new UART address
 *          to REG_PULSE_P2[7:5] in RAM, then burns it to EEPROM so it persists.
 *          Steps: scan -> wake -> RAM write -> unlock -> burn -> verify.
 * @param targetAddr  New UART address to assign (0-7).
 * @return HAL status.
 */
HAL_StatusTypeDef PGA460_WriteUARTAddr(uint8_t targetAddr) {
uint8_t foundAddr = 0xFF;
    uint8_t regP2 = 0;
    uint8_t eeStatus = 0;
    // 1. SCAN: Find which address (0-7) responds
    for (uint8_t a = 0; a <= 7; a++) {
        if (PGA460_RegisterRead(a, REG_PULSE_P2, &regP2) == HAL_OK) {
            foundAddr = a;
            DEBUG("Found sensor at %u (Register says %u)\n", a, regP2);
            break;
        }
    }
    if (foundAddr == 0xFF) {
        DEBUG("No sensor found on bus.\n");
        return HAL_ERROR;
    }
    // 2. WAKE UP: Move digital core to READY state
    // Mandatory before EEPROM operations.
    PGA460_SetThresholds(foundAddr);
    HAL_Delay(50);
    // 3. RAM UPDATE: Set new address and ensure P2_PULSE = 0
    // Based on your requirement: targetAddr << 5
    uint8_t writeVal = (targetAddr << 5); 
    PGA460_RegisterWrite(foundAddr, REG_PULSE_P2, writeVal);
    HAL_Delay(50);
    // 4. TI UNLOCK & BURN: Make it permanent
    // We now talk to targetAddr because RAM update is instant
    DEBUG("Burning Address %u to EEPROM...\n", targetAddr);  
    // Unlock Sequence (Register 0x40)
    PGA460_RegisterWrite(targetAddr, 0x40, 0x68);
    PGA460_RegisterWrite(targetAddr, 0x40, 0x69);
    // Command 22: EEPROM Burn
    if (PGA460_BurnEEPROM(targetAddr) != HAL_OK) {
        DEBUG("Burn Command failed to send.\n");
        return HAL_ERROR;
    }
    // 5. WAIT & VERIFY
    HAL_Delay(100); // Wait for physical EEPROM write
    PGA460_RegisterRead(targetAddr, 0x40, &eeStatus);
    if (eeStatus & 0x80) {
        DEBUG("EEPROM ERROR: 0x%02X. Burn rejected.\n", eeStatus);
        return HAL_ERROR;
    }
    DEBUG("SUCCESS: Sensor set to Address %u\n", targetAddr);
    return HAL_OK;
}

/**
 * @brief PGA460_Init function implementation.
 * @details Initialises all three sensors in sequence.  For each sensor:
 *            1. Resets the Measures struct (tof_us, width, amplitude = 0).
 *            2. Writes the threshold map (mandatory to transition out of sleep).
 *          The burnEEPROM path is compiled but commented out; enable it when
 *          the final configuration needs committing to non-volatile storage.
 * @param burnEEPROM  1 = burn configuration to EEPROM flash after writing; 0 = RAM only.
 * @return HAL status.
 */
HAL_StatusTypeDef PGA460_Init(uint8_t burnEEPROM) {
	for (uint8_t i = 0; i < ULTRASONIC_SENSOR_COUNT; ++i) {
		// Step 1: Assign transducer config and reset measures
		sensors[i].Measures.tof_us		= 0;
		sensors[i].Measures.width			= 0;
		sensors[i].Measures.amplitude	= 0;
		// Step 2: Threshold write (uses current sensors[i].Registers.THR) !!! Mandatory step to start the sensor
		if (PGA460_SetThresholds(i) != HAL_OK) {
			DEBUG("Sensor %u: Threshold write failed!\n", i);
			return HAL_ERROR;
		}
		PGA460_ReadAndPrintEEPROM(i);
//		// Step 2: EEPROM bulk write (shadow -> device)
//		if (PGA460_EEPROMBulkWrite(i) != HAL_OK) {
//			DEBUG("Sensor %u: EEPROM bulk write failed!\n", i);
//			return HAL_ERROR;
//		}
////		PGA460_ReadAndPrintEEPROM(i);
//		if(burnEEPROM) {
//			// Step 3: Optional EEPROM Burn
//			if (PGA460_BurnEEPROM(i) != HAL_OK) {
//				DEBUG("Sensor %u: EEPROM burn failed!\n", i);
//				return HAL_ERROR;
//			}
//		}
//		// Step 4: TVG write (push current TVG from mirror)
//		if (PGA460_SetTVG(i, PGA460_GAIN_58_90dB, &sensors[i].Registers.EEData.TVG) != HAL_OK) {
//				DEBUG("Sensor %u: TVG bulk write failed!\n", i);
//				return HAL_ERROR;
//		}
		// Step 5: Diagnostics/status
//		if (PGA460_CheckStatus(i) != HAL_OK) {
//			DEBUG("Sensor %u: Status check failed!\n", i);
//			//return HAL_ERROR;
//		}
//		// Frequency (Hz) via system diagnostics (diag=0)
//		float diagVal = 0.0f;
//		if (PGA460_GetSystemDiagnostics(i, 1, 0, &diagVal) == HAL_OK)
//				DEBUG("Sensor %u: Frequency Diagnostic = %.2f Hz\n", i, diagVal);
//		if (PGA460_GetSystemDiagnostics(i, 0, 1, &diagVal) == HAL_OK)
//				DEBUG("Sensor %u: Decay Period Diagnostic = %.2f us\n", i, diagVal);
//		if (PGA460_GetSystemDiagnostics(i, 0, 2, &diagVal) == HAL_OK)
//				DEBUG("Sensor %u: Die Temperature = %.2f C\n", i, diagVal);
//		if (PGA460_GetSystemDiagnostics(i, 0, 3, &diagVal) == HAL_OK)
//				DEBUG("Sensor %u: Noise Level = %.0f\n", i, diagVal);
//		// Step 7: Optional Echo Data Dump for verification
//		uint8_t echoBuf[128];
//		if (PGA460_GetEchoDataDump(i, PGA460_CMD_BURST_AND_LISTEN_PRESET1, echoBuf) != HAL_OK) {
//			DEBUG("Sensor %u: Echo Data Dump failed!\n", i);
//			return HAL_ERROR;
//		}
		
//		DEBUG("Sensor %u: Echo Data Dump:\n", i);
//		for (uint8_t j = 0; j < 128; ++j) {
//			DEBUG("%u%s", echoBuf[j], (j < 127) ? "," : "\n");
//		}
		DEBUG("-----\n");
		HAL_Delay(100);
	}
	return HAL_OK;
}

/**
 * @brief PGA460_RegisterRead function implementation.
 * @details Sends a 4-byte TX frame [SYNC][CMD][REG_ADDR][CHK] and reads the
 *          3-byte response [DIAG][DATA][CHK].  The DATA byte is returned in *regValue.
 *          The CMD byte encodes the sensor UART address in bits [7:5].
 * @param sensorID  Sensor UART address (0-7).
 * @param regAddr  Register address to read.
 * @param regValue  Output: register value.
 * @return HAL status.
 */
HAL_StatusTypeDef PGA460_RegisterRead(uint8_t sensorID, uint8_t regAddr, uint8_t *regValue) {
	// --- TX frame: [SYNC][CMD][REG_ADDR][CHK] ---
	uint8_t cmd[4] = {PGA460_SYNC, (uint8_t)((sensorID << 5) | PGA460_CMD_REGISTER_READ), regAddr, 0};
	cmd[3] = PGA460_CalculateChecksum(&cmd[1], 2); // Checksum over CMD + REG_ADDR
	if (HAL_UART_Transmit(&huart3, cmd, sizeof(cmd), PGA460_UART_TIMEOUT_MS) != HAL_OK) return HAL_ERROR;
	// --- RX frame: [DIAG][DATA][CHK] ---
	if (HAL_UART_Receive(&huart3, cmd, 3, PGA460_UART_TIMEOUT_MS) != HAL_OK) return HAL_ERROR;
	*regValue = cmd[1]; // DATA byte
	return HAL_OK;
}

/**
 * @brief PGA460_RegisterWrite function implementation.
 * @details Sends a 5-byte TX frame [SYNC][CMD][REG_ADDR][DATA][CHK].
 *          A 1 ms delay is applied after transmission; increase to 10 ms when writing
 *          TVG, threshold, or gain registers followed immediately by a read.
 * @param sensorID  Sensor UART address (0-7).
 * @param regAddr  Register address to write.
 * @param regValue  Value to write.
 * @return HAL status.
 */
HAL_StatusTypeDef PGA460_RegisterWrite(uint8_t sensorID, uint8_t regAddr, uint8_t regValue) {
	// TX frame: [SYNC][CMD][REG_ADDR][REG_VALUE][CHK]
	uint8_t cmd[5] = {PGA460_SYNC, (uint8_t)((sensorID << 5) | PGA460_CMD_REGISTER_WRITE), regAddr, regValue, 0};
	cmd[4] = PGA460_CalculateChecksum(&cmd[1], 3); // checksum over CMD+ADDR+DATA
	if (HAL_UART_Transmit(&huart3, cmd, sizeof(cmd), PGA460_UART_TIMEOUT_MS) != HAL_OK) return HAL_ERROR;
	// For commands 10 and 22: Wait 60 us if INIT_GAIN, TVG, THR or P1_GAIN_CTRL or P2_GAIN_CTRL
	// is written to before a read, otherwise wait 3.3 us for other functions.
	HAL_Delay(1); // Increase to 10 if needed
	return HAL_OK;
}

/**
 * @brief PGA460_CheckStatus function implementation.
 * @details Reads DEV_STAT0 (0x4C) and DEV_STAT1 (0x4D) and uses DEBUG() to report
 *          any active fault bits: EEPROM CRC errors, wakeup errors, thermal shutdown,
 *          and over/under-voltage conditions on VPWR, AVDD, and IOREG.
 * @param sensorID  Sensor UART address (0-7).
 * @return HAL status.
 */
HAL_StatusTypeDef PGA460_CheckStatus(uint8_t sensorID) {
	// Read both DEV_STAT registers
	if (PGA460_RegisterRead(sensorID, REG_DEV_STAT0, &sensors[sensorID].Registers.Stat0.Val.Value) != HAL_OK || PGA460_RegisterRead(sensorID, REG_DEV_STAT1, &sensors[sensorID].Registers.Stat1.Val.Value) != HAL_OK) {
		DEBUG("Sensor %d: Communication error! Could not read status registers.\n", (int)sensorID);
		return HAL_ERROR;
	}
	// --- DEV_STAT0 Flags ---
	if (sensors[sensorID].Registers.Stat0.Val.BitField.CMW_WU_ERR)
		DEBUG("Sensor %d: [CMW_WU_ERR] Wake-up error!\n", (int)sensorID);
	if (sensors[sensorID].Registers.Stat0.Val.BitField.THR_CRC_ERR)
		DEBUG("Sensor %d: [THR_CRC_ERR] Threshold CRC error!\n", (int)sensorID);
	if (sensors[sensorID].Registers.Stat0.Val.BitField.EE_CRC_ERR)
		DEBUG("Sensor %d: [EE_CRC_ERR] EEPROM CRC error!\n", (int)sensorID);
	if (sensors[sensorID].Registers.Stat0.Val.BitField.TRIM_CRC_ERR)
		DEBUG("Sensor %d: [TRIM_CRC_ERR] Trim CRC error!\n", (int)sensorID);
	// --- DEV_STAT1 Flags ---
	if (sensors[sensorID].Registers.Stat1.Val.BitField.TSD_PROT)
		DEBUG("Sensor %d: [TSD_PROT] Thermal shutdown occurred!\n", (int)sensorID);
	if (sensors[sensorID].Registers.Stat1.Val.BitField.IOREG_OV)
		DEBUG("Sensor %d: [IOREG_OV] IOREG Overvoltage!\n", (int)sensorID);
	if (sensors[sensorID].Registers.Stat1.Val.BitField.IOREG_UV)
		DEBUG("Sensor %d: [IOREG_UV] IOREG Undervoltage!\n", (int)sensorID);
	if (sensors[sensorID].Registers.Stat1.Val.BitField.AVDD_OV)
		DEBUG("Sensor %d: [AVDD_OV] AVDD Overvoltage!\n", (int)sensorID);
	if (sensors[sensorID].Registers.Stat1.Val.BitField.AVDD_UV)
		DEBUG("Sensor %d: [AVDD_UV] AVDD Undervoltage!\n", (int)sensorID);
	if (sensors[sensorID].Registers.Stat1.Val.BitField.VPWR_OV)
		DEBUG("Sensor %d: [VPWR_OV] VPWR Overvoltage!\n", (int)sensorID);
	if (sensors[sensorID].Registers.Stat1.Val.BitField.VPWR_UV)
		DEBUG("Sensor %d: [VPWR_UV] VPWR Undervoltage!\n", (int)sensorID);
	return HAL_OK;
}

/**
 * @brief PGA460_EEPROMBulkWrite function implementation.
 * @details Unlocks the EEPROM if needed, then sends the full 44-byte EEImage_t
 *          (UserData + TVG + Settings) from sensors[sensorID].Registers.EEData
 *          using CMD 0x0C.  Frame format: [SYNC][CMD][44 bytes][CHK].
 * @param sensorID  Sensor UART address (0-7).
 * @return HAL status.
 */
HAL_StatusTypeDef PGA460_EEPROMBulkWrite(uint8_t sensorID) {
	// Step 1: Unlock EEPROM if needed
	if (sensors[sensorID].Registers.eepromUnlocked == 0) {
		if (PGA460_SetEEPROMAccess(sensorID, 1) != HAL_OK) {
			DEBUG("Sensor %d: EEPROM unlock failed before bulk write\n", sensorID);
			return HAL_ERROR;
		}
	}
	// Step 2: Construct UART frame
	uint8_t frame[46] = {0};
	frame[0] = PGA460_SYNC;
	frame[1] = (uint8_t)((sensorID << 5) | (PGA460_CMD_EEPROM_BULK_WRITE));
	memcpy(&frame[2], &sensors[sensorID].Registers.EEData, 44); // 44 bytes
	frame[45] = PGA460_CalculateChecksum(&frame[1], 44); // Checksum over CMD + EEPROM bytes
	// Step 4: Transmit
	if (HAL_UART_Transmit(&huart3, frame, sizeof(frame), PGA460_UART_TIMEOUT_MS) != HAL_OK) {
		DEBUG("Sensor %d: EEPROM Bulk Write Failed!\n", sensorID);
		return HAL_ERROR;
	}
	HAL_Delay(1);  // Wait for EEPROM write
	if (PGA460_SetEEPROMAccess(sensorID, 0) != HAL_OK) {
		DEBUG("Sensor %d: EEPROM lock failed after bulk write\n", sensorID);
		return HAL_ERROR;
	}
	return HAL_OK;
}

/**
 * @brief PGA460_BurnEEPROM function implementation.
 * @details Executes the TI-specified two-step burn sequence on REG_EE_CTRL (0x40):
 *            1. Write 0x68 to begin the unlock sequence.
 *            2. Write 0x69 to start the internal charge pump and program the EEPROM.
 *            3. Wait 1000 ms (TI minimum; do not interrupt the sensor during this time).
 *            4. Read EE_CTRL back and verify bit 2 (EE_PRGM_OK) is set.
 *            5. Write 0x00 to leave EE_CTRL in a clean state.
 *          Use sparingly: the PGA460 EEPROM supports a limited number of burn cycles.
 * @param sensorID  Sensor UART address (0-7).
 * @return HAL status.
 */
HAL_StatusTypeDef PGA460_BurnEEPROM(uint8_t sensorID) {
    uint8_t burnStat = 0;
    // 1. Unlock EEPROM (0x68)
    if (PGA460_RegisterWrite(sensorID, 0x40, 0x68) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(1); // Short pause between unlock steps
    // 2. Trigger Burn (0x69)
    // Writing 0x69 sets the EEPRGM bit which initiates the internal charge pump
    if (PGA460_RegisterWrite(sensorID, 0x40, 0x69) != HAL_OK) {
        return HAL_ERROR;
    }
    // 3. WAIT (Crucial!) 
    // TI uses 1000ms. The process is slow. Do not interrupt the sensor here.
    HAL_Delay(1000);
    // 4. Read back status from EE_CNTRL
    if (PGA460_RegisterRead(sensorID, 0x40, &burnStat) != HAL_OK) {
        return HAL_ERROR;
    }
    // 5. Check Bit 2 (EE_PRGM_OK)
    if ((burnStat & 0x04) == 0x04) {
        DEBUG("Sensor %u: EEPROM Burn Successful (Status: 0x%02X)\n", sensorID, burnStat);
        // TI-style cleanup: Set back to 0 to lock
        PGA460_RegisterWrite(sensorID, 0x40, 0x00);
        return HAL_OK;
    } else {
        DEBUG("Sensor %u: Burn Failed (Status: 0x%02X, expected bit 2 set)\n", sensorID, burnStat);
        return HAL_ERROR;
    }
}

/**
 * @brief PGA460_GetTVG function implementation.
 * @details Sends CMD 0x0D (TVG Bulk Read) and receives [DIAG][TVGAIN0..TVGAIN6]
 *          (8 bytes total).  The 7 TVG bytes are copied into
 *          sensors[sensorID].Registers.EEData.TVG.  Returns HAL_ERROR if the
 *          response is all 0xFF (UART idle flood).
 * @param sensorID  Sensor UART address (0-7).
 * @return HAL status.
 */
HAL_StatusTypeDef PGA460_GetTVG(uint8_t sensorID) {
	// Send command [SYNC][CMD][CHK]
	uint8_t tx[3];
	tx[0] = PGA460_SYNC; // 0x55
	tx[1] = (uint8_t)((sensorID << 5) | (PGA460_CMD_TVG_BULK_READ)); // Command code
	tx[2] = PGA460_CalculateChecksum(&tx[1], 1);
	if (HAL_UART_Transmit(&huart3, tx, sizeof(tx), PGA460_UART_TIMEOUT_MS) != HAL_OK) {
		return HAL_ERROR;
	}
	// --- Step 2: Receive DIAG + 7 TVG bytes ---
	// RX: [DIAG][TVGAIN0..TVGAIN6] => 1 + 7 = 8 bytes
	uint8_t rx[8] = {0};
	if (HAL_UART_Receive(&huart3, rx, sizeof(rx), PGA460_UART_TIMEOUT_MS) != HAL_OK) {
		return HAL_ERROR;
	}
	// Optional: detect idle flood (all 0xFF)
	uint8_t idle = 1;
	for (uint32_t i = 0; i < sizeof(rx); i++) {
		if (rx[i] != 0xFF) {
			idle = 0;
			break;
		}
	}
	if (idle) return HAL_ERROR;
	// --- Step 3: Copy TVG bytes into mirror (skip rx[0] = DIAG) ---
	memcpy(&sensors[sensorID].Registers.EEData.TVG, &rx[1], sizeof(TVG_t));
	return HAL_OK;
}

/**
 * @brief PGA460_SetTVG function implementation.
 * @details Updates DECPL_TEMP[7:6] with the requested AFE gain range (preserving the
 *          lower 6 bits), then sends CMD 0x0E (TVG Bulk Write) with the 7-byte TVG
 *          payload.  Mirrors the written values into sensors[sensorID].Registers.
 * @param sensorID  Sensor UART address (0-7).
 * @param gain_range  AFE gain range (see PGA460_GainRange_t).
 * @param tvg  Pointer to the 7-byte TVG profile to write.
 * @return HAL status.
 */
HAL_StatusTypeDef PGA460_SetTVG(uint8_t sensorID, PGA460_GainRange_t gain_range, const TVG_t *tvg) {
	// 1) Update DECPL_TEMP[7:6] = AFE_GAIN_RNG, preserve other bits.
	sensors[sensorID].Registers.EEData.Sett.DECPL_TEMP = (sensors[sensorID].Registers.EEData.Sett.DECPL_TEMP & 0x3F) | gain_range;
	if (PGA460_RegisterWrite(sensorID, REG_DECPL_TEMP, sensors[sensorID].Registers.EEData.Sett.DECPL_TEMP) != HAL_OK) {
		return HAL_ERROR;
	}
	// 2) TVG bulk write: [SYNC][CMD=0x0E][7 TVG bytes][CHK]
	uint8_t frame[10];
	frame[0] = PGA460_SYNC;
	frame[1] = (uint8_t)((sensorID << 5) | (PGA460_CMD_TVG_BULK_WRITE));
	memcpy(&frame[2], tvg, sizeof(TVG_t)); // 7 bytes payload
	frame[9] = PGA460_CalculateChecksum(&frame[1], 1 + sizeof(TVG_t)); // checksum over CMD+DATA
	if (HAL_UART_Transmit(&huart3, frame, sizeof(frame), PGA460_UART_TIMEOUT_MS) != HAL_OK) {
		return HAL_ERROR;
	}
	// Mirror locally on success
	memcpy(&sensors[sensorID].Registers.EEData.TVG, tvg, sizeof(TVG_t));
	HAL_Delay(PGA460_CAPTURE_DELAY_MS);
	return HAL_OK;
}

/**
 * @brief PGA460_GetThresholds function implementation.
 * @details Sends CMD 0x0F (Threshold Bulk Read) and receives [DIAG][32 threshold bytes][CRC]
 *          (34 bytes total).  Copies the payload into sensors[sensorID].Registers.THR.
 * @param sensorID  Sensor UART address (0-7).
 * @return HAL status.
 */
HAL_StatusTypeDef PGA460_GetThresholds(uint8_t sensorID) {
	// 1) Send prebuilt command [SYNC][CMD][CHK]
	uint8_t tx[3];
	tx[0] = PGA460_SYNC; // 0x55
	tx[1] = (uint8_t)((sensorID << 5) | (PGA460_CMD_THRESHOLD_BULK_READ)); // Command code
	tx[2] = PGA460_CalculateChecksum(&tx[1], 1);
	if (HAL_UART_Transmit(&huart3, tx, sizeof(tx), PGA460_UART_TIMEOUT_MS) != HAL_OK) {
		return HAL_ERROR;
	}
	// 2) Receive DIAG (1) + THR block (33) = 34 bytes total
	uint8_t rx[34] = {0}; // rx[0]=DIAG, rx[1..32]=P1/P2 (32 bytes), rx[33]=THR_CRC
	if (HAL_UART_Receive(&huart3, rx, sizeof(rx), PGA460_UART_TIMEOUT_MS) != HAL_OK) {
		return HAL_ERROR;
	}
	// 3) Copy 33 bytes (P1_THR[16], P2_THR[16], THR_CRC) into mirror (skip DIAG)
	memcpy(&sensors[sensorID].Registers.THR, &rx[1], sizeof(Thresholds_t));
	return HAL_OK;
}

/**
 * @brief PGA460_SetThresholds function implementation.
 * @details Sends CMD 0x10 (Threshold Bulk Write) with the 33-byte threshold payload
 *          (P1_THR[16] + P2_THR[16] + THR_CRC) from sensors[sensorID].Registers.THR.
 *          This is a mandatory initialisation step to transition the sensor out of sleep.
 * @param sensorID  Sensor UART address (0-7).
 * @return HAL status.
 */
HAL_StatusTypeDef PGA460_SetThresholds(uint8_t sensorID) {
	// Build a 33-byte image (P1[16] + P2[16] + CRC) without mutating shadow yet
	uint8_t frame[35] = {0};
	frame[0] = PGA460_SYNC;
	frame[1] = (uint8_t)((sensorID << 5) | (PGA460_CMD_THRESHOLD_BULK_WRITE));
	memcpy(&frame[2], &sensors[sensorID].Registers.THR, 32);
	// Frame: [SYNC][CMD=0x10][33 bytes][CHK] => total 36 bytes
	frame[34] = PGA460_CalculateChecksum(&frame[1], 33); // checksum over CMD + 33 data bytes
	if (HAL_UART_Transmit(&huart3, frame, sizeof(frame), PGA460_UART_TIMEOUT_MS) != HAL_OK) {
		return HAL_ERROR;
	}
	HAL_Delay(PGA460_CAPTURE_DELAY_MS);
	return HAL_OK;
}

/**
 * @brief PGA460_UltrasonicCmd function implementation.
 * @details Builds and transmits the appropriate UART frame for the requested command:
 *            4-byte frame for burst/listen, temp/noise, and result-read commands.
 *            3-byte frame for echo dump and broadcast commands.
 *          For broadcast commands (address >= 0x11) the address field in CMD is 0.
 *          For burst/listen commands with noWait = 0, waits PGA460_CAPTURE_DELAY_MS
 *          (70 ms) after transmission to allow the measurement window to complete.
 * @param sensorID  Sensor UART address (0-7).
 * @param cmd  Command code (see PGA460_Command_t).
 * @param noWait  1 = return immediately; 0 = delay after burst commands.
 * @return HAL status.
 */
HAL_StatusTypeDef PGA460_UltrasonicCmd(const uint8_t sensorID, const PGA460_Command_t cmd, const uint8_t noWait) {
    uint8_t frame[4];
    uint16_t len = 0;
    frame[0] = PGA460_SYNC; // 0x55
    // Check if the command is a broadcast command (0x11 to 0x19).
    // If so, force the address bits to 0. Otherwise, use the sensorID.
    uint8_t addr = (cmd >= PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P1) ? 0 : sensorID;
    // Pack the address into the top 3 bits and the command into the bottom 5 bits.
    frame[1] = (uint8_t)((addr << 5) | (cmd)); 
    switch (cmd) {
        // 4-byte frames: [SYNC][CMD][OBJ][CHK]
        case PGA460_CMD_BURST_AND_LISTEN_PRESET1:
        case PGA460_CMD_BURST_AND_LISTEN_PRESET2:
        case PGA460_CMD_LISTEN_ONLY_PRESET1:
        case PGA460_CMD_LISTEN_ONLY_PRESET2:
        case PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P1:
        case PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P2:
        case PGA460_CMD_BROADCAST_LISTEN_ONLY_P1:
        case PGA460_CMD_BROADCAST_LISTEN_ONLY_P2:
            frame[2] = PGA_OBJECTS_TRACKED; // 0x01
            frame[3] = PGA460_CalculateChecksum(&frame[1], 2); // Covers CMD + OBJ
            len = 4;
            break;
        // 3-byte frames: [SYNC][CMD][CHK]
        case PGA460_CMD_TEMP_AND_NOISE_MEASUREMENT:
        case PGA460_CMD_ULTRASONIC_MEASUREMENT_RESULT:
        case PGA460_CMD_TEMP_AND_NOISE_RESULT:
        case PGA460_CMD_TRANSDUCER_ECHO_DATA_DUMP:
        case PGA460_CMD_SYSTEM_DIAGNOSTICS:
        case PGA460_CMD_BROADCAST_TEMP_NOISE_MEASURE:
            frame[2] = PGA460_CalculateChecksum(&frame[1], 1); // Covers only CMD
            len = 3;
            break;
        default:
            DEBUG("Sensor %d: Unsupported CMD 0x%02X\n", sensorID, (unsigned)cmd);
            return HAL_ERROR;
    }
    if (HAL_UART_Transmit(&huart3, frame, len, PGA460_UART_TIMEOUT_MS) != HAL_OK) {
        DEBUG("Sensor %d: Ultrasonic TX failed (CMD=0x%02X)\n", sensorID, frame[1]);
        return HAL_ERROR;
    }
    // Only burst/listen & listen-only start a capture; give them time to run.
    if (!noWait && (len == 4)) {
        HAL_Delay(PGA460_CAPTURE_DELAY_MS); // ~70 ms
    }
		return HAL_OK;
}

/**
 * @brief PGA460_GetUltrasonicMeasurement function implementation.
 * @details Sends CMD 0x05 (Ultrasonic Measurement Result) and receives the response.
 *          Iterates through up to PGA_OBJECTS_TRACKED objects; stores the first valid
 *          result (rawTOF > 0 and != 0xFFFF) into sensors[sensorID].Measures.
 *          Also updates externalData.SoundSpeed via PGA460_ComputeSoundSpeed().
 * @param sensorID  Sensor UART address (0-7).
 * @return HAL status.
 */
HAL_StatusTypeDef PGA460_GetUltrasonicMeasurement(const uint8_t sensorID) {
	uint8_t localBuffer[PGA_OBJ_DATA_SIZE] = {0};
	uint8_t tx[3];
	tx[0] = PGA460_SYNC; // 0x55
	tx[1] = (uint8_t)((sensorID << 5) | (PGA460_CMD_ULTRASONIC_MEASUREMENT_RESULT)); // Command code
	tx[2] = PGA460_CalculateChecksum(&tx[1], 1);
    // 1) Request UMR
    if (HAL_UART_Transmit(&huart3, tx, sizeof(tx), PGA460_UART_TIMEOUT_MS) != HAL_OK) {
        DEBUG("Sensor %d: Measurement Result Request Failed!\n", sensorID);
        return HAL_ERROR;
    }
    // 2) Read response
    if (HAL_UART_Receive(&huart3, localBuffer, sizeof(localBuffer), PGA460_UART_TIMEOUT_MS) != HAL_OK) {
        DEBUG("Sensor %d: Measurement Result Retrieval Failed!\n", sensorID);
        return HAL_ERROR;
    }
    // Update speed of sound from environment
    // PGA460_ComputeSoundSpeed(&externalData);
    HAL_StatusTypeDef status = HAL_ERROR;
    uint8_t stored = 0;
    // 3) Parse each object, store first valid into sensors[sensorID].Measures
    for (uint8_t i = 0; i < PGA_OBJECTS_TRACKED; i++) {
        const uint8_t base = 2 + (i * 4); // skip 2-byte diagnostic header
        const uint16_t rawTOF   = ((uint16_t)localBuffer[base] << 8) | localBuffer[base + 1];
        const uint8_t  widthRaw = localBuffer[base + 2];
        const uint8_t  amplitude = localBuffer[base + 3];
        //DEBUG("Sensor %d: Raw TOF = %04X\n", sensorID, rawTOF);
        if (rawTOF > 0 && rawTOF != 0xFFFF) {
            const uint16_t width_us = (uint16_t)widthRaw * 16;
            if (!stored) {
                sensors[sensorID].Measures.tof_us    = rawTOF;
                sensors[sensorID].Measures.width     = (uint8_t)((width_us > 255) ? 255 : width_us);
                sensors[sensorID].Measures.amplitude = amplitude;
                stored = 1;
            }
            status = HAL_OK;
        } else {
            DEBUG("Sensor %d: Obj %u -> Invalid (TOF = 0x%04X)\n", sensorID, (unsigned)(i + 1), rawTOF);
        }
    }
    return status;
}

/**
 * @brief PGA460_GetEchoDataDump function implementation.
 * @details Enables Echo Data Dump mode in REG_EE_CTRL (bit 7), triggers the requested
 *          preset, sends CMD 0x07, and receives 130 bytes (2-byte header + 128 bins).
 *          Disables the mode on exit.  Uses goto cleanup to ensure the mode is always
 *          disabled even on error.
 * @param sensorID  Sensor UART address (0-7).
 * @param preset  Preset command to trigger (CMD_BURST_AND_LISTEN_PRESET1 or PRESET2).
 * @param echoOut  Output buffer for 128 amplitude bytes (caller must provide 128 bytes).
 * @return HAL status.
 */
HAL_StatusTypeDef PGA460_GetEchoDataDump(uint8_t sensorID, uint8_t preset, uint8_t *echoOut) {
	HAL_StatusTypeDef ret = HAL_OK;
	uint8_t response[ECHO_DATA_TOTAL_BYTES] = {0}; // 2-byte header + 128 bins
	// 1) Enable Echo Data Dump Mode (EE_CTRL.DATADUMP_EN = 1)
	if (PGA460_RegisterWrite(sensorID, REG_EE_CTRL, ECHO_DATA_DUMP_ENABLE) != HAL_OK) {
		DEBUG("Sensor %d: Failed to enable Echo Data Dump mode!\n", sensorID);
		return HAL_ERROR;
	}
	HAL_Delay(10);
	// 2) Trigger the requested preset (burst+listen or listen-only)
	if (PGA460_UltrasonicCmd(sensorID, (PGA460_Command_t)preset, 0) != HAL_OK) {
		DEBUG("Sensor %d: Echo trigger preset failed!\n", sensorID);
		ret = HAL_ERROR;
		goto cleanup_disable_dump;
	}
	HAL_Delay(PGA460_CAPTURE_DELAY_MS); // allow full capture (~65 ms)
	// 3) Request Bulk Echo Data Dump (CMD = 0x07)
	uint8_t tx[3];
	tx[0] = PGA460_SYNC; // 0x55
	tx[1] = (uint8_t)((sensorID << 5) | (PGA460_CMD_TRANSDUCER_ECHO_DATA_DUMP)); // Command code
	tx[2] = PGA460_CalculateChecksum(&tx[1], 1);
	if (HAL_UART_Transmit(&huart3, tx, sizeof(tx), PGA460_UART_TIMEOUT_MS) != HAL_OK) {
		DEBUG("Sensor %d: Echo Data Dump request TX failed!\n", sensorID);
		ret = HAL_ERROR;
		goto cleanup_disable_dump;
	}
	// 4) Receive Echo Data Dump (2-byte header + 128 samples)
	if (HAL_UART_Receive(&huart3, response, sizeof(response), PGA460_UART_TIMEOUT_MS) != HAL_OK) {
		DEBUG("Sensor %d: Echo Data Dump RX failed!\n", sensorID);
		ret = HAL_ERROR;
		goto cleanup_disable_dump;
	}
	// 5) Copy only the 128 echo bins (skip the 2-byte header)
	memcpy(echoOut, &response[2], 128);
cleanup_disable_dump:
	// 6) Disable Echo Data Dump Mode (best-effort)
	// The previous status should be preserved. The return value should not be overwritten.
	if (PGA460_RegisterWrite(sensorID, REG_EE_CTRL, ECHO_DATA_DUMP_DISABLE) != HAL_OK) {
		DEBUG("Sensor %d: Failed to disable Echo Data Dump mode!\n", sensorID);
	}
	// Return the status from the main operation, not the cleanup operation.
	return ret;
}

/**
 * @brief PGA460_SetTemperatureOffset function implementation.
 * @details Reads the PGA460 internal temperature, computes (externalTempC - internal),
 *          clamps the signed offset to [-64, +63] degrees, encodes it as sign-magnitude
 *          in REG_TEMP_TRIM (bit 7 = sign, bits 6:0 = magnitude), and writes it to the
 *          device.  Mirrors the new value into sensors[sensorID].Registers.
 * @param sensorID  Sensor UART address (0-7).
 * @param externalTempC  Reference temperature in degrees Celsius (from BMP581 or HDC302x).
 * @return HAL status.
 */
HAL_StatusTypeDef PGA460_SetTemperatureOffset(uint8_t sensorID, float externalTempC) {
	// 1) Read internal temperature
	float internalTempC = PGA460_ReadTemperatureOrNoise(sensorID, PGA460_CMD_GET_TEMP);
	if (internalTempC == PGA460_TEMP_ERR) {
		DEBUG("Sensor %d: Failed to read internal temperature!\n", sensorID);
		return HAL_ERROR;
	}
	DEBUG("Sensor %d: Internal Temperature = %.2f C\n", sensorID, internalTempC);
	// 2) Compute signed offset (external - internal), clamp to [-64, +63]
	int8_t offset = (int8_t)roundf(externalTempC - internalTempC);
	if (offset >  63)
		offset =  63;
	if (offset < -64)
		offset = -64;
	// 3) Read current TEMP_TRIM (0x28)
	uint8_t tempTrim = 0;
	if (PGA460_RegisterRead(sensorID, REG_TEMP_TRIM, &tempTrim) != HAL_OK) {
		DEBUG("Sensor %d: Failed to read TEMP_TRIM register!\n", sensorID);
		return HAL_ERROR;
	}
	DEBUG("Sensor %d: Original TEMP_TRIM = 0x%02X\n", sensorID, tempTrim);
	// 4) Build sign-magnitude byte: bit7 = sign, bits6:0 = magnitude
	uint8_t magnitude = (uint8_t)((offset < 0) ? -offset : offset);
	uint8_t signedOffset = ((offset < 0) ? 0x80 : 0x00) | (magnitude & 0x7F);
	// If TEMP_TRIM has no reserved bits, write the whole byte:
	tempTrim = signedOffset;
	// 5) Write updated TEMP_TRIM
	if (PGA460_RegisterWrite(sensorID, REG_TEMP_TRIM, tempTrim) != HAL_OK) {
		DEBUG("Sensor %d: Failed to write TEMP_TRIM register!\n", sensorID);
		return HAL_ERROR;
	}
	// Mirror locally on success
	sensors[sensorID].Registers.EEData.Sett.TEMP_TRIM = tempTrim;
	DEBUG("Sensor %d: New TEMP_TRIM = 0x%02X (Offset: %+d C)\n", sensorID, tempTrim, offset);
	return HAL_OK;
}

/**
 * @brief PGA460_GetSystemDiagnostics function implementation.
 * @details Optionally triggers a Preset 1 burst (run = 1), then sends CMD 0x08
 *          (System Diagnostics) and receives the 4-byte response.
 *          The diag parameter selects the returned value:
 *            0 = transducer frequency in Hz (from FREQ_TICKS at 0.5 us per tick)
 *            1 = echo decay period in us (DECAY_TICKS * 16)
 *            2 = internal die temperature in deg C (via PGA460_ReadTemperatureOrNoise)
 *            3 = ambient noise level, raw 8-bit (via PGA460_ReadTemperatureOrNoise)
 * @param sensorID  Sensor UART address (0-7).
 * @param run  1 = trigger a burst before reading; 0 = use the last stored result.
 * @param diag  0 = frequency, 1 = decay period, 2 = temperature, 3 = noise.
 * @param diagResult  Output: diagnostic value in units described above.
 * @return HAL status.
 */
HAL_StatusTypeDef PGA460_GetSystemDiagnostics(const uint8_t sensorID, const uint8_t run, const uint8_t diag, float *diagResult) {
	// Optional: start a capture using P1 before running diagnostics
	if (run) {
		if (PGA460_UltrasonicCmd(sensorID, PGA460_CMD_BURST_AND_LISTEN_PRESET1, 0) != HAL_OK) {
			DEBUG("Sensor %d: Burst-and-Listen Command Failed!\n", sensorID);
			return HAL_ERROR;
		}
		HAL_Delay(5); // allow measurement to complete
	}
	// Send System Diagnostics (0x08) ? 3B command [SYNC][CMD][CHK]
	uint8_t cmd[4] = {PGA460_SYNC, (uint8_t)((sensorID << 5) | PGA460_CMD_SYSTEM_DIAGNOSTICS), 0, 0};
	cmd[2] = PGA460_CalculateChecksum(&cmd[1], 1);
	if (HAL_UART_Transmit(&huart3, cmd, 3, PGA460_UART_TIMEOUT_MS) != HAL_OK) {
		DEBUG("Sensor %d: System Diagnostics Request Failed!\n", sensorID);
		return HAL_ERROR;
	}
	// Receive 4B response: [DIAG][FREQ_TICKS][DECAY_TICKS][PCHK]
	if (HAL_UART_Receive(&huart3, cmd, sizeof(cmd), PGA460_UART_TIMEOUT_MS) != HAL_OK) {
		DEBUG("Sensor %d: System Diagnostics Retrieval Failed!\n", sensorID);
		return HAL_ERROR;
	}
	// Special cases handled via temp/noise helper
	if (diag == 2) {  // Temperature (degC)
		*diagResult = PGA460_ReadTemperatureOrNoise(sensorID, PGA460_CMD_GET_TEMP);
		if (*diagResult == PGA460_TEMP_ERR) {
			DEBUG("Sensor %d: Failed to Retrieve Temperature!\n", sensorID);
			return HAL_ERROR;
		}
		return HAL_OK;
	}
	if (diag == 3) {  // Noise (raw 8-bit)
		*diagResult = PGA460_ReadTemperatureOrNoise(sensorID, PGA460_CMD_GET_NOISE);
		if (*diagResult == PGA460_TEMP_ERR) {
			DEBUG("Sensor %d: Failed to Retrieve Noise!\n", sensorID);
			return HAL_ERROR;
		}
		return HAL_OK;
	}
	// Standard diagnostics
	switch (diag) {
		case 0: { // Frequency (Hz) from ticks of 0.5 us
			const uint8_t ticks = cmd[1];
			if (ticks == 0) {
				DEBUG("Sensor %d: Invalid Frequency Value (0 ticks)!\n", sensorID);
				return HAL_ERROR;
			}
			*diagResult = 1.0f / (ticks * 0.5e-6f); // Hz
			break;
		}
		case 1: { // Decay period (us), tick = 16 us
			*diagResult = (float)cmd[2] * 16.0f; // us
			break;
		}
		default:
			DEBUG("Sensor %d: Invalid Diagnostic Type (%u)!\n", sensorID, (unsigned)diag);
			return HAL_ERROR;
	}
	//DEBUG("Sensor %d: Diagnostic [%u] Result: %.2f\n", sensorID, (unsigned)diag, *diagResult);
	return HAL_OK;
}

/**
 * @brief PGA460_ReadTemperatureOrNoise function implementation.
 * @details Sends CMD 0x04 (Temp/Noise Measurement) with the mode selector byte,
 *          waits PGA460_CAPTURE_DELAY_MS, then sends CMD 0x06 (Temp/Noise Result)
 *          and receives the 4-byte response [DIAG][TEMP][NOISE][CHK].
 *          Temperature: T_C = (raw_byte - 64) / 1.5
 *          Noise: returned as the raw 8-bit amplitude byte.
 * @param sensorID  Sensor UART address (0-7).
 * @param mode  PGA460_CMD_GET_TEMP or PGA460_CMD_GET_NOISE.
 * @return Temperature in deg C, noise as raw byte, or PGA460_TEMP_ERR (999.0f) on failure.
 */
float PGA460_ReadTemperatureOrNoise(const uint8_t sensorID, const PGA460_CmdType_t mode) {
	const uint8_t isNoise = (mode == PGA460_CMD_GET_NOISE);
	float result = PGA460_TEMP_ERR;  // default invalid
	// Step 1: Request Temperature or Noise Measurement
	// [SYNC][CMD=0x04][SEL(0=temp,1=noise)][CHK(CMD+SEL)]
	uint8_t cmd[4] = {PGA460_SYNC, (uint8_t)((sensorID << 5) | PGA460_CMD_TEMP_AND_NOISE_MEASUREMENT), isNoise ? 1 : 0, 0};
	cmd[3] = PGA460_CalculateChecksum(&cmd[1], 2); // over CMD+SEL
	if (HAL_UART_Transmit(&huart3, cmd, sizeof(cmd), PGA460_UART_TIMEOUT_MS) != HAL_OK) {
		DEBUG("Sensor %d: Temp/Noise Measurement Command Failed!\n", sensorID);
		return result;
	}
	HAL_Delay(PGA460_CAPTURE_DELAY_MS);  // small settle for measurement to complete
	// Step 2: Send command to read the result (fixed 3-byte command)
	cmd[1] = (uint8_t)((sensorID << 5) | (PGA460_CMD_TEMP_AND_NOISE_RESULT)); // CMD
	cmd[2] = PGA460_CalculateChecksum(&cmd[1], 1); // over CMD+SEL
	if (HAL_UART_Transmit(&huart3, cmd, PGA_CMD_SIZE, PGA460_UART_TIMEOUT_MS) != HAL_OK) {
		DEBUG("Sensor %d: Result Request Command Failed!\n", sensorID);
		return result;
	}
	// Step 3: Read 4-byte result frame: [DIAG][TEMP][NOISE][PCHK]
	if (HAL_UART_Receive(&huart3, cmd, sizeof(cmd), PGA460_UART_TIMEOUT_MS) != HAL_OK) {
		DEBUG("Sensor %d: Failed to Receive Temp/Noise Data!\n", sensorID);
		return result;
	}
	// Step 4: Parse result
	if (!isNoise) {
		result = ((float)cmd[1] - 64.0f) / 1.5f;  // Temperature in degC
		//DEBUG("Sensor %d: Temperature (C) = %.2f\n", sensorID, result);
	} else {
		result = (float)cmd[2]; // Noise level (raw 8-bit)
		//DEBUG("Sensor %d: Noise Level = %.2f\n", sensorID, result);
	}
	return result;
}

/*===========================================================================
 * WIND MEASUREMENT & CALIBRATION
 *===========================================================================*/

/* Cone generator = v(R2 + h2) = v(502 + 1502) mm, converted to um */
#define CONE_R_MM           50.0f
#define CONE_H_MM           150.0f
#define CONE_L_MM           158.114f					/* v(502+1502)    */
#define CONE_L_UM           158114						/* L in um        */
#define NOMINAL_PATH_UM     2.0f * CONE_L_UM	/* 2 * CONE_L_UM */
/*
 * Effective horizontal path unit vectors  *_ij = (pos_j - pos_i) / L
 *
 * Transducer positions in (x=East, y=North) frame (mm):
 *   T0 = (0,           +50)          - North
 *   T1 = (+43.3013,    -25)          - 120deg East
 *   T2 = (-43.3013,    -25)          - 240deg East
 *
 * Distances |Ti-Tj| = Rv3 = 86.6025 mm for all pairs (equilateral triangle).
 */
#define U01_X    0.27386126465714610976890091958966f   /* (T1x - T0x) / L */
#define U01_Y   (-0.47434129805077349254335479464184f) /* (T1y - T0y) / L */

#define U12_X   (-0.54772252931429221953780183917933f) /* (T2x - T1x) / L */
#define U12_Y    0.00000000000000000000000000000000f   /* (T2y - T1y) / L */

#define U20_X    0.27386126465714610976890091958966f   /* (T0x - T2x) / L */
#define U20_Y    0.47434129805077349254335479464184f   /* (T0y - T2y) / L */

// Least-squares weight K = (A^T A)^{-1} scalar = 1/0.45 = 20/9 (A^T A is diagonal due to 120deg symmetry; both diagonal entries = 0.45)
#define LS_K    2.22222222222222f   /* 20/9 */

/*===========================================================================
 * CORDIC CONSTANTS
 *===========================================================================*/

/*
 * Maximum expected wind speed (m/s = um/us).  vx and vy are divided by this
 * before being written to the CORDIC (which requires inputs in [-1, 1]).
 * 100 m/s covers any meteorological condition with comfortable headroom.
 */
#define WIND_CORDIC_SCALE   100.0f						/* m/s (= um/us)						*/
#define WIND_CORDIC_INV     0.01f							/* 1 / WIND_CORDIC_SCALE		*/

/* Q1.31 scale factor: maps float [-1,1] to int32_t */
#define Q31_SCALE           2147483647.0f			/* 2^31 - 1                 */
#define Q31_INV             4.656612873e-10f	/* 1 / Q31_SCALE						*/

/* CORDIC output for PHASE: angle/p in Q1.31 multiply by p to get radians	*/
#define INV_Q31_TO_PI       1.46291807927e-9f	/* p / (2^31 - 1)           */

/* CORDIC PHASE output [-1,+1] maps to [-180°,+180°]	*/
#define CORDIC_PHASE_TO_DEG   180.0f   

#define CORDIC_Q31_INV_TO_DEG   (CORDIC_PHASE_TO_DEG * Q31_INV)
/*===========================================================================
 * CALIBRATION CONSTANTS
 *===========================================================================*/

#define N_CAL               16     /* number of sample pairs to average    */
#define CAL_PAIR_DELAY_MS   100    /* inter-pair delay during calibration   */

/*
 * USER_DATA byte offset within sensor EEPROM where D_ij is stored.
 * Bytes 0-3 of USER_DATA in the transmitting sensor = IEEE-754 float32.
 * Sensor 0 stores D01, Sensor 1 stores D12, Sensor 2 stores D20.
 */
#define CAL_USERDATA_OFFSET 0x00

#define TOF_MIN_TICKS	400
#define TOF_MAX_TICKS	2000
		
/* Calibrated one-way path lengths in um for each transmitting sensor.
 * Index i = transmitter index.  Populated by PGA460_CalibrateReflector(). */
static float g_pathLen_um[3] = {NOMINAL_PATH_UM, NOMINAL_PATH_UM, NOMINAL_PATH_UM};

/*===========================================================================
 * SLIDING-WINDOW MEDIAN FILTER
 *
 * Each of the 6 one-way ToF values has its own circular buffer of depth
 * N_FILT.  On every call to PGA460_MeasureWind() one raw hardware measurement
 * cycle is collected (3 transmitter firings -> 6 ToFs via TIM2 DECPL DMA),
 * inserted into the six buffers, and then the median of each buffer is used
 * for the wind calculation.
 *
 * Why filter at the ToF level:
 *   - Outliers (missed threshold crossing, acoustic ringing, UART glitch) are
 *     most easily identified as a single anomalous ToF on one path.
 *   - The b_ij = D/2 * (1/t_ij - 1/t_ji) formula is nonlinear: a 10 us
 *     outlier causes a ~1.9 m/s wind error.  Rejecting it before the
 *     reciprocal step is far more effective than filtering the output.
 *   - Working in (vx, vy) space means no circular wraparound problem;
 *     direction and speed are computed once from the already-filtered medians.
 *
 * Window size N_FILT = 5 (odd, for an exact median):
 *   - Each full cycle takes ~45 ms (3x broadcast + burst + settle).
 *   - At N=5 the window spans ~225 ms -- fast enough to track gusts while
 *     rejecting individual bad echoes.
 *   - Median of 5 tolerates up to 2 simultaneous outliers per window.
 *   - Storage: 6 x 5 x 4 = 120 bytes + 6 head bytes = 126 bytes total.
 *
 * Buffer layout:
 *   g_tof_buf[path][slot]  where path is one of the 6 one-way paths
 *   g_tof_head[path]       circular-buffer write index (0..N_FILT-1)
 *   g_tof_count[path]      number of valid samples so far (0..N_FILT)
 *
 * Until the buffer is full, the median is computed over g_tof_count samples
 * only (no zero-padding), so the very first call returns a single-sample
 * result and convergence improves over the next N_FILT-1 calls.
 *===========================================================================*/

#define N_FILT          5      /* sliding-window depth (must be odd)         */
#define N_FILT_HALF     2      /* N_FILT / 2, index of median in sorted copy */

/* Symbolic indices for the six one-way ToF paths */
typedef enum {
    TOF_01 = 0,   /* Tx0 -> Rx1 */
    TOF_10 = 1,   /* Tx1 -> Rx0 */
    TOF_12 = 2,   /* Tx1 -> Rx2 */
    TOF_21 = 3,   /* Tx2 -> Rx1 */
    TOF_20 = 4,   /* Tx2 -> Rx0 */
    TOF_02 = 5,   /* Tx0 -> Rx2 */
    TOF_PATH_COUNT = 6
} ToFPath_t;

static float   g_tof_buf  [TOF_PATH_COUNT][N_FILT];  /* circular buffers     */
static uint8_t g_tof_head [TOF_PATH_COUNT];           /* next write index     */
static uint8_t g_tof_count[TOF_PATH_COUNT];           /* valid sample count   */

/**
 * @brief Clear all six per-path sliding-window ToF buffers.
 * @details Resets head index and sample count to zero for every path.
 *          Call after PGA460_Init() and after any recalibration.
 */
void PGA460_ResetFilter(void) {
    for (uint8_t p = 0; p < (uint8_t)TOF_PATH_COUNT; p++) {
        g_tof_head [p] = 0;
        g_tof_count[p] = 0;
        for (uint8_t s = 0; s < N_FILT; s++) {
            g_tof_buf[p][s] = 0.0f;
        }
    }
}

/**
 * @brief Insert one new ToF sample into the circular buffer for the given path.
 * @details Overwrites the oldest sample when the buffer is full (circular behaviour).
 * @param path  Path index (see ToFPath_t).
 * @param tof_us  One-way ToF measurement in us.
 */
static void filter_insert(ToFPath_t path, float tof_us) {
    uint8_t h = g_tof_head[path];
    g_tof_buf[path][h] = tof_us;
    g_tof_head[path]   = (uint8_t)((h + 1) % N_FILT);
    if (g_tof_count[path] < N_FILT) {
        g_tof_count[path]++;
    }
}

/**
 * @brief Return the median of the current samples in the buffer for the given path.
 * @details Copies the live samples into a local array, sorts it by insertion sort
 *          (max N_FILT = 5 elements, max 10 FPU ops), and returns the middle element.
 *          Before the buffer is full returns the lower median of available samples.
 * @param path  Path index (see ToFPath_t).
 * @return Median ToF in us, or 0.0f if no samples are present.
 */
static float filter_median(ToFPath_t path){
    uint8_t n = g_tof_count[path];
    if (n == 0) { return 0.0f; }
    /* Local copy -- do not disturb the circular buffer */
    float tmp[N_FILT];
    for (uint8_t i = 0; i < n; i++) {
        tmp[i] = g_tof_buf[path][i];
    }

    /* Insertion sort (ascending) */
    for (uint8_t i = 1; i < n; i++) {
        float   key = tmp[i];
        uint8_t j   = i;
        while (j > 0 && tmp[j - 1] > key) {
            tmp[j] = tmp[j - 1];
            j--;
        }
        tmp[j] = key;
    }
    return tmp[n / 2];   /* lower-median for even n, exact median for odd n */
}

/*===========================================================================
 * PRIVATE HELPERS
 *===========================================================================*/

/**
 * @brief Compute atan2(y, x) in degrees using the STM32G431 CORDIC peripheral.
 * @details Configures CORDIC for the PHASE function (6 iterations, Q1.31, ~24-bit accuracy)
 *          and writes the normalised (x, y) vector directly to the CORDIC registers.
 *          Input is pre-normalised by the largest component to maximise CORDIC precision.
 *          CORDIC PHASE writes X first then Y; output = atan2(Y, X) / pi in Q1.31.
 *          For wind direction: call as cordic_atan2_deg(vx, vy) where x=East, y=North.
 *          Returns the TO direction; add 180 deg to get the FROM (meteorological) direction.
 * @param y  North wind component (m/s) -- first argument maps to CORDIC Y.
 * @param x  East wind component (m/s) -- second argument maps to CORDIC X.
 * @return Angle in degrees in [-180, +180); 0.0f for near-zero input vectors.
 */
static float cordic_atan2_deg(float y, float x) {
    /* --- Normalise to unit circle --- */
    float abs_x = (x < 0.0f) ? -x : x;
    float abs_y = (y < 0.0f) ? -y : y;
    float norm  = (abs_x > abs_y) ? abs_x : abs_y;
    if (norm < 1e-9f) { return 0.0f; } /* near-zero vector: undefined direction, treat as 0 */
    float xn = x / norm;
    float yn = y / norm;
    /* --- Configure CORDIC: PHASE, 6 iterations (24-bit accuracy), Q1.31 --- */
    CORDIC->CSR = CORDIC_FUNCTION_PHASE
                | CORDIC_PRECISION_6CYCLES   /* NB_ITER = 6 ~24 bit	*/
                | CORDIC_SCALE_0
                | CORDIC_NBWRITE_2           /* write X then Y			*/
                | CORDIC_NBREAD_1            /* read one result			*/
                | CORDIC_INSIZE_32BITS
                | CORDIC_OUTSIZE_32BITS;

    /* Write X first, then Y (CORDIC HW order for PHASE) */
    CORDIC->WDATA = (int32_t)(xn * Q31_SCALE);
    CORDIC->WDATA = (int32_t)(yn * Q31_SCALE);
    /* Poll RRDY flag */
    while (!(CORDIC->CSR & CORDIC_CSR_RRDY)) {}
    int32_t raw = (int32_t)CORDIC->RDATA;  /* angle / p in Q1.31 */

    /* Convert: raw * (p / (2^31-1)) * (180/p) = raw * (180 / (2^31-1)) */
    return (float)raw * CORDIC_Q31_INV_TO_DEG;
}

/**
 * @brief Compute sqrt(x^2 + y^2) using the STM32G431 CORDIC MODULUS function.
 * @details Configures CORDIC for the MODULUS function (6 iterations, Q1.31).
 *          Input is pre-normalised by the larger component so both fit [-1, 1].
 *          Result is de-normalised by multiplying back the normalisation factor.
 * @param x  East wind component (m/s).
 * @param y  North wind component (m/s).
 * @return Euclidean magnitude in the same units as the inputs; 0.0f for near-zero vectors.
 */
static float cordic_magnitude(float x, float y) {
    float abs_x = (x < 0.0f) ? -x : x;
    float abs_y = (y < 0.0f) ? -y : y;
    float norm  = (abs_x > abs_y) ? abs_x : abs_y;
    if (norm < 1e-9f) { return 0.0f; }
    float xn = x / norm;
    float yn = y / norm;
    /* --- Configure CORDIC: MODULUS, 6 iterations, Q1.31 --- */
    CORDIC->CSR = CORDIC_FUNCTION_MODULUS
                | CORDIC_PRECISION_6CYCLES
                | CORDIC_SCALE_0
                | CORDIC_NBWRITE_2
                | CORDIC_NBREAD_1
                | CORDIC_INSIZE_32BITS
                | CORDIC_OUTSIZE_32BITS;

    CORDIC->WDATA = (int32_t)(xn * Q31_SCALE);
    CORDIC->WDATA = (int32_t)(yn * Q31_SCALE);

    while (!(CORDIC->CSR & CORDIC_CSR_RRDY)) {}

    int32_t raw = (int32_t)CORDIC->RDATA;   /* magnitude in Q1.31 */

    /* Re-apply normalisation: result = (raw / Q31_SCALE) * norm */
    return (float)raw * Q31_INV * norm;
}

/**
 * @brief Block until the DECPL DMA bitmask callback sets Decpl_RDY, or until timeout.
 * @details Polls Decpl_RDY in a busy-wait loop.  Decpl_RDY is set by
 *          HAL_TIM_IC_CaptureCallback() in tim.c only when all three TIM2 channels
 *          (CH1, CH2, CH3) have each completed their two-edge DMA capture.
 * @param timeout_ms  Maximum wait time in milliseconds.
 * @return HAL_OK when Decpl_RDY goes high; HAL_TIMEOUT if the deadline expires.
 */
static HAL_StatusTypeDef wait_decpl_ready(uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();
    while (!Decpl_RDY) {
        if ((HAL_GetTick() - t0) >= timeout_ms) {
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

/**
 * @brief Fire one transmitter and capture echo timestamps on both receivers via TIM2 DMA.
 * @details Sequence:
 *            1. Reset Decpl_RDY and zero ToF_Result[].
 *            2. Arm TIM2 input-capture DMA on CH1/2/3 for 2 edges each.
 *            3. Send BROADCAST_LISTEN_ONLY_P2 -- all DECPL pins assert immediately(P2 = 0 pulses); DMA captures this as E0 (reference, discarded).
 *            4. Send BURST_AND_LISTEN_PRESET1 on the transmitter -- 6 pulses fired; Tx DECPL asserts at burst end -> E1[tx] = T_burst reference.
 *            5. Wait for all three DMA TCs via wait_decpl_ready() (10 ms timeout).
 *            6. Compute ToF = TICK_TO_US(E1[rx] - E1[tx]) for each receiver.
 *            7. Validate: delta ticks must be in [TOF_MIN_TICKS, TOF_MAX_TICKS].
 *          Unsigned tick subtraction is wrap-safe for deltas up to 12.6 s.
 * @param tx  Transmitter index (0-2).
 * @param tof_rx1_us  Output: one-way ToF to first receiver in us.
 * @param tof_rx2_us  Output: one-way ToF to second receiver in us.
 * @return HAL_OK, HAL_ERROR (invalid ToF), or HAL_TIMEOUT (no DECPL response).
 */
static HAL_StatusTypeDef run_one_measurement(uint8_t tx, float  *tof_rx1_us, float  *tof_rx2_us) {
    uint8_t rx1 = (tx == 0) ? 1 : 0;
    uint8_t rx2 = (uint8_t)(3 - tx - rx1);
    /*
     * Reset state BEFORE arming DMA.
     * DECPL_ResetReadyFlag() disables interrupts briefly to prevent a stale
     * TC from an aborted previous measurement from instantly asserting Decpl_RDY.
     * The chan_done bitmask in the callback is cleared by Decpl_RDY going to 0
     * and the callback accumulator resetting on the next full mask match, but
     * the explicit flag reset here is sufficient - the bitmask itself is reset
     * inside the callback when the mask completes.
     */
    Decpl_RDY = 0;
    memset((void *)ToF_Result, 0, sizeof(ToF_Result));
    /* --- Arm TIM2 input-capture DMA: 2 edges per DECPL transducer --- */
    HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)&ToF_Result[0].E0, 2u);
    HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, (uint32_t *)&ToF_Result[1].E0, 2u);
    HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_3, (uint32_t *)&ToF_Result[2].E0, 2u);
    /*
     * Command sequence:
     *
     * 1. Broadcast LISTEN_ONLY P2 to all sensors (noWait=1).
     *    All three PGA460 DECPL pins assert within ~1 UART byte time of each
     *    other.  Each DMA captures this as E0 (reference, discarded later).
     *    The transmitter is now also armed in listen mode for P2 - this is
     *    intentional: its DECPL will fire again when the P1 burst ends (E1).
     *
     * 2. BURST_AND_LISTEN P1 on the transmitter (noWait=1).
     *    Tx fires 6 pulses.  At the end of the Tx decouple period, its DECPL
     *    asserts again ? captured as ToF_Result[tx].E1 (= T_burst reference).
     *    The two receivers listen; when each detects its echo threshold
     *    crossing, its DECPL asserts ? captured as ToF_Result[rx].E1.
     */
    PGA460_UltrasonicCmd(rx1, PGA460_CMD_BROADCAST_LISTEN_ONLY_P2, 1);
    PGA460_UltrasonicCmd(tx,  PGA460_CMD_BURST_AND_LISTEN_PRESET1, 1);
    /* --- Wait for all three DECPL TC interrupts via bitmask callback --- */
    if (wait_decpl_ready(10) != HAL_OK) {
        HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_1);
        HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_2);
        HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_3);
        DEBUG("Tx%u: DECPL timeout - check DECPL wiring and P1/P2 config\n", tx);
        return HAL_TIMEOUT;
    }
    HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_2);
    HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_3);
    /*
     * Extract timestamps:
     *   E1[tx]  = end of Tx burst decouple period  ? acoustic start
     *   E1[rx]  = echo threshold crossing           ? acoustic arrival
     *
     * Unsigned subtraction is wrap-safe for any delta < 2^31 ticks (~12.6 s).
     * The measurement window (< 2 ms) is orders of magnitude shorter.
     */
    uint32_t t_burst  = ToF_Result[tx].E1;
    uint32_t t_echo1  = ToF_Result[rx1].E1;
    uint32_t t_echo2  = ToF_Result[rx2].E1;
    uint32_t delta1 = t_echo1 - t_burst;   /* unsigned wrap-safe subtraction */
    uint32_t delta2 = t_echo2 - t_burst;
    if (delta1 < TOF_MIN_TICKS || delta1 > TOF_MAX_TICKS || delta2 < TOF_MIN_TICKS || delta2 > TOF_MAX_TICKS) {
        DEBUG("Tx%u: ToF ticks out of range (d1=%d, d2=%d) - expected %d..%d\n", tx, delta1, delta2, TOF_MIN_TICKS, TOF_MAX_TICKS);
        return HAL_ERROR;
    }
    *tof_rx1_us = delta1;
    *tof_rx2_us = delta2;
    return HAL_OK;
}

/**
 * @brief Write a float32 calibrated path length to USER_DATA bytes 0-3 of a sensor.
 * @details Splits the IEEE-754 float into 4 bytes using memcpy and writes each byte
 *          to REG_USER_DATA1 + offset (0-3) via PGA460_RegisterWrite().
 * @param sensorID  Sensor UART address (0-7).
 * @param path_um  Calibrated one-way acoustic path length in um.
 * @return HAL status.
 */
static HAL_StatusTypeDef store_path_to_eeprom(uint8_t sensorID, float path_um) {
    uint8_t bytes[4];
    memcpy(bytes, &path_um, 4);
    for (uint8_t i = 0u; i < 4; i++) {
        if (PGA460_RegisterWrite(sensorID, (uint8_t)(REG_USER_DATA1 + i), bytes[i]) != HAL_OK) {
            DEBUG("Sensor %u: failed to write calibration byte %u\n", sensorID, i);
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

/**
 * @brief Read a float32 calibrated path length from USER_DATA bytes 0-3 of a sensor.
 * @details Reads 4 bytes from REG_USER_DATA1 + offset (0-3) and reassembles them into
 *          a float32 using memcpy.  Validates the result against a +/-10% window around
 *          NOMINAL_PATH_UM; returns NOMINAL_PATH_UM if the stored value is out of range.
 * @param sensorID  Sensor UART address (0-7).
 * @return Calibrated path length in um, or NOMINAL_PATH_UM if not stored or invalid.
 */
static float load_path_from_eeprom(uint8_t sensorID) {
    uint8_t bytes[4] = {0, 0, 0, 0};
    uint8_t val;
    for (uint8_t i = 0u; i < 4; i++) {
        if (PGA460_RegisterRead(sensorID, (uint8_t)(REG_USER_DATA1 + i), &val) != HAL_OK) {
            DEBUG("Sensor %u: failed to read calibration byte %u\n", sensorID, i);
            return NOMINAL_PATH_UM;
        }
        bytes[i] = val;
    }
    float result;
    memcpy(&result, bytes, 4);
    /* Sanity check: must be within *10% of nominal */
    if (result < NOMINAL_PATH_UM * 0.90f || result > NOMINAL_PATH_UM * 1.10f) {
        DEBUG("Sensor %u: EEPROM path length %.1f um out of range, using nominal\n",
              sensorID, result);
        return NOMINAL_PATH_UM;
    }
    return result;
}

/**
 * @brief PGA460_LoadCalibration function implementation.
 * @details Calls load_path_from_eeprom() for each of the three sensors and stores the
 *          result in g_pathLen_um[].  Falls back to NOMINAL_PATH_UM on failure.
 *          Call once at boot after PGA460_Init() so prior calibrations survive power-off.
 */
void PGA460_LoadCalibration(void)
{
    for (uint8_t i = 0; i < 3; i++) {
        g_pathLen_um[i] = load_path_from_eeprom(i);
        DEBUG("Sensor %u: loaded D = %.2f um\n", i, g_pathLen_um[i]);
    }
}

/**
 * @brief PGA460_CalibrateReflector function implementation.
 * @details Performs bidirectional still-air calibration for all three transducer pairs.
 *
 *          Repeats N_CAL full measurement cycles (3 transmitter firings each).  For each
 *          bidirectional pair the averaged path length is:
 *            D_ij = c * (t_ij + t_ji) / 2
 *          Wind cancels in the sum, leaving only path geometry.
 *
 *          Pair-to-sensor storage mapping:
 *            D_01 -> sensor 0 USER_DATA (tof[0][0], tof[1][0])
 *            D_12 -> sensor 1 USER_DATA (tof[1][1], tof[2][1])
 *            D_20 -> sensor 2 USER_DATA (tof[2][0], tof[0][1])
 *
 *          Returns HAL_ERROR if fewer than N_CAL/2 cycles produce valid measurements.
 * @param soundSpeed_ms  Speed of sound in m/s from PGA460_UpdateEnvironment() (1 m/s = 1 um/us).
 * @param burnEEPROM  1 = commit calibration to PGA460 EEPROM flash; 0 = RAM only.
 * @return HAL status.
 */
HAL_StatusTypeDef PGA460_CalibrateReflector(float soundSpeed_ms, uint8_t burnEEPROM) {
    const float c = soundSpeed_ms;   /* 1 m/s = 1 um/us exactly */
    DEBUG("Calibration start: c = %.4f m/s, N_CAL = %u\n", c, N_CAL);
    /*
     * Accumulators for the six one-way ToF sums (us).
     * Index mapping mirrors the tof[tx][rx_slot] layout in PGA460_MeasureWind.
     *   sum[0][0] = S t_01,  sum[0][1] = S t_02
     *   sum[1][0] = S t_10,  sum[1][1] = S t_12
     *   sum[2][0] = S t_20,  sum[2][1] = S t_21
     */
    float   sum[3][2] = {{0.0f}};
    uint8_t valid = 0;
    for (uint8_t n = 0; n < N_CAL; n++) {
        float tof[3][2] = {{0.0f}};
        uint8_t all_ok = 1;
        /* Fire each transmitter and capture both receiver ToFs */
        for (uint8_t tx = 0; tx < 3; tx++) {
            if (run_one_measurement(tx, &tof[tx][0], &tof[tx][1]) != HAL_OK) {
                all_ok = 0u;
                break;
            }
            /* Validate: expected range for 316 mm path at 306-496 m/s */
            if (tof[tx][0] < TOF_MIN_TICKS || tof[tx][0] > TOF_MAX_TICKS || tof[tx][1] < TOF_MIN_TICKS || tof[tx][1] > TOF_MAX_TICKS) {
                DEBUG("Cal n=%u Tx%u: ToF out of range (%.1f, %.1f us)\n",
                      n, tx, tof[tx][0], tof[tx][1]);
                all_ok = 0;
                break;
            }
        }
        if (all_ok) {
            for (uint8_t tx = 0; tx < 3; tx++) {
                sum[tx][0] += tof[tx][0];
                sum[tx][1] += tof[tx][1];
            }
            valid++;
        }
        HAL_Delay(CAL_PAIR_DELAY_MS);
    }
    if (valid < (N_CAL / 2)) {
        DEBUG("Calibration failed: only %u/%u valid cycles\n", valid, N_CAL);
        return HAL_ERROR;
    }
    const float inv_valid = 1.0f / (float)valid;
    /*
     * Compute and store the three calibrated path lengths.
     *
     * Bidirectional pair ? transmitter index that owns its USER_DATA:
     *   D_01  stored in sensor 0  (t_01 = sum[0][0], t_10 = sum[1][0])
     *   D_12  stored in sensor 1  (t_12 = sum[1][1], t_21 = sum[2][1])
     *   D_20  stored in sensor 2  (t_20 = sum[2][0], t_02 = sum[0][1])
     */
    /* Average ToF per pair (us) */
    float t_avg_01 = (sum[0][0] + sum[1][0]) * 0.5f * inv_valid;
    float t_avg_12 = (sum[1][1] + sum[2][1]) * 0.5f * inv_valid;
    float t_avg_20 = (sum[2][0] + sum[0][1]) * 0.5f * inv_valid;
    float d[3];
    d[0] = c * t_avg_01;   /* D_01 um, owned by sensor 0 */
    d[1] = c * t_avg_12;   /* D_12 um, owned by sensor 1 */
    d[2] = c * t_avg_20;   /* D_20 um, owned by sensor 2 */
    for (uint8_t i = 0; i < 3; i++) {
        g_pathLen_um[i] = d[i];
        DEBUG("D_%u%u = %.2f um  (%.4f mm, nominal %.2f um, delta %+.1f um, %u samples)\n", i, (i + 1) % 3, d[i], d[i] * 0.001f, NOMINAL_PATH_UM, d[i] - NOMINAL_PATH_UM, valid);
        if (store_path_to_eeprom(i, d[i]) != HAL_OK) {
            return HAL_ERROR;
        }
        if (burnEEPROM) {
            if (PGA460_BurnEEPROM(i) != HAL_OK) {
                DEBUG("Sensor %u: EEPROM burn failed\n", i);
                return HAL_ERROR;
            }
            HAL_Delay(50);   /* PGA460 internal EEPROM write cycle */
        }
    }
    DEBUG("Calibration complete (%u/%u cycles used)\n", valid, N_CAL);
    return HAL_OK;
}

/**
 * @brief PGA460_MeasureWind function implementation.
 * @details Executes a full 3-transmitter measurement cycle and returns the filtered wind vector.
 *
 *          Step 1 - Hardware cycle: fires each transmitter, captures 6 raw one-way ToF
 *          values via TIM2 DECPL input-capture DMA (run_one_measurement).
 *
 *          Step 2 - Filter insert: inserts valid raw ToFs into six per-path sliding-window
 *          circular buffers (N = 5).  On hardware fault keeps prior buffer contents.
 *
 *          Step 3 - Median: insertion sort on <= 5 floats per path.  Returns HAL_ERROR
 *          if any median is outside the physical ToF range (buffer not yet warm).
 *
 *          Step 4 - Wind reconstruction:
 *            b_ij = (D_i/2) * (1/t_ij - 1/t_ji)  [um/us = m/s]
 *            vx = K * (u01_x*b01 + u12_x*b12 + u20_x*b20)  [East m/s]
 *            vy = K * (u01_y*b01 + u12_y*b12 + u20_y*b20)  [North m/s]
 *            K = 20/9 (exact closed-form least-squares for 120-deg symmetric layout)
 *
 *          Step 5 - CORDIC: speed via MODULUS, direction via PHASE + 180 deg (FROM convention).
 * @param out  Output: Speed (m/s) and Direction (deg FROM: 0/360=N, 90=E, 180=S, 270=W).
 * @return HAL_OK if all 3 transmitters succeeded; HAL_ERROR otherwise (filtered output still written).
 */
HAL_StatusTypeDef PGA460_MeasureWind(PGA460_Wind_t *out) {
    if (out == NULL) { return HAL_ERROR; }
    /*=========================================================================
     * Step 1 -- Hardware measurement cycle
     *
     * Fire each transmitter in sequence.  Each firing captures two one-way ToF
     * values via TIM2 DECPL input-capture DMA (see run_one_measurement).
     * Three firings -> six raw ToF values, one per directed path.
     *
     * tof_raw[tx][0] = t_{Tx -> Rx1},  tof_raw[tx][1] = t_{Tx -> Rx2}
     * Receiver assignments:
     *   tx=0: rx1=1, rx2=2    tx=1: rx1=0, rx2=2    tx=2: rx1=0, rx2=1
     *=========================================================================*/
    float tof_raw[3][2] = {{0.0f}};
    HAL_StatusTypeDef hw_status = HAL_OK;
    for (uint8_t tx = 0; tx < 3; tx++) {
        if (run_one_measurement(tx, &tof_raw[tx][0], &tof_raw[tx][1]) != HAL_OK) {
            DEBUG("MeasureWind: Tx%u hardware fault\n", tx);
            hw_status = HAL_ERROR;
            break;
        }
    }
    /*=========================================================================
     * Step 2 -- Insert into per-path sliding-window buffers
     *
     * Even if one transmitter fails, we insert the six values we have (keeping
     * failed paths at their previous buffer state so prior good samples still
     * contribute to the median).  A failed cycle is flagged and the caller
     * receives HAL_ERROR but stale-median output is still written to *out,
     * which is preferable to a hard stop in a meteo application.
     *
     * Path-to-raw-slot mapping:
     *   TOF_01: tof_raw[0][0]   TOF_10: tof_raw[1][0]
     *   TOF_12: tof_raw[1][1]   TOF_21: tof_raw[2][1]
     *   TOF_20: tof_raw[2][0]   TOF_02: tof_raw[0][1]
     *=========================================================================*/
    if (hw_status == HAL_OK) {
        filter_insert(TOF_01, tof_raw[0][0]);
        filter_insert(TOF_10, tof_raw[1][0]);
        filter_insert(TOF_12, tof_raw[1][1]);
        filter_insert(TOF_21, tof_raw[2][1]);
        filter_insert(TOF_20, tof_raw[2][0]);
        filter_insert(TOF_02, tof_raw[0][1]);

        DEBUG("raw t01=%.2f t10=%.2f t12=%.2f t21=%.2f t20=%.2f t02=%.2f us\n",
              tof_raw[0][0], tof_raw[1][0], tof_raw[1][1],
              tof_raw[2][1], tof_raw[2][0], tof_raw[0][1]);
    }

    /*=========================================================================
     * Step 3 -- Compute median ToF for each path
     *
     * filter_median() performs an insertion sort on a local copy of the
     * circular buffer (<=5 elements, <=10 FPU compare-and-swap ops) and
     * returns the middle element.  With a full window of N_FILT=5 samples,
     * up to 2 outliers per path are rejected per measurement call.
     *
     * For the first N_FILT-1 calls the buffer is not yet full; filter_median()
     * returns the median of however many valid samples are present (1 to 4),
     * which degrades gracefully to a single-sample pass-through on the first
     * call.
     *=========================================================================*/
    float t01 = filter_median(TOF_01);
    float t10 = filter_median(TOF_10);
    float t12 = filter_median(TOF_12);
    float t21 = filter_median(TOF_21);
    float t20 = filter_median(TOF_20);
    float t02 = filter_median(TOF_02);

    DEBUG("med t01=%.2f t10=%.2f t12=%.2f t21=%.2f t20=%.2f t02=%.2f us\n", t01, t10, t12, t21, t20, t02);

    /* Guard: if any median is still zero the buffer was never populated */

    if (t01 < TOF_MIN_US || t01 > TOF_MAX_US ||
        t10 < TOF_MIN_US || t10 > TOF_MAX_US ||
        t12 < TOF_MIN_US || t12 > TOF_MAX_US ||
        t21 < TOF_MIN_US || t21 > TOF_MAX_US ||
        t20 < TOF_MIN_US || t20 > TOF_MAX_US ||
        t02 < TOF_MIN_US || t02 > TOF_MAX_US)
    {
        DEBUG("MeasureWind: median ToF out of physical range -- buffer not yet warm\n");
        return HAL_ERROR;
    }
    /*=========================================================================
     * Step 4 -- Wind vector reconstruction
     *
     * Wind projection onto each physical path [um/us = m/s]:
     *   b_ij = (D_i / 2) * (1/t_ij - 1/t_ji)
     *
     * b_ij is the component of the wind velocity along the unit vector
     * u_ij = (pos_j - pos_i) / L.  The units work out to m/s because
     * D_i is in um and t_ij is in us, and 1 um/us = 1 m/s exactly.
     *=========================================================================*/
    float b01 = (g_pathLen_um[0] * 0.5f) * (1.0f/t01 - 1.0f/t10);
    float b12 = (g_pathLen_um[1] * 0.5f) * (1.0f/t12 - 1.0f/t21);
    float b20 = (g_pathLen_um[2] * 0.5f) * (1.0f/t20 - 1.0f/t02);
    DEBUG("b01=%.4f b12=%.4f b20=%.4f m/s\n", b01, b12, b20);
    /*
     * Closed-form least-squares reconstruction (K = 20/9, exact for 120 deg layout):
     *   vx = K * (u01_x*b01 + u12_x*b12 + u20_x*b20)    [East component, m/s]
     *   vy = K * (u01_y*b01 + u12_y*b12 + u20_y*b20)    [North component, m/s]
     */
    float vx = LS_K * (U01_X * b01 + U12_X * b12 + U20_X * b20);
    float vy = LS_K * (U01_Y * b01 + U12_Y * b12 + U20_Y * b20);
    DEBUG("vx=%.4f vy=%.4f m/s\n", vx, vy);
    /*=========================================================================
     * Step 5 -- CORDIC: speed and direction
     *
     * MODULUS: speed = sqrt(vx^2 + vy^2)   [m/s]
     * PHASE:   direction via atan2(vx, vy) + 180 deg (FROM convention)
     *
     * Inputs are scaled to [-1,1] by WIND_CORDIC_INV before the CORDIC write.
     * No circular-mean issue: direction is computed from the (vx,vy) vector
     * which was reconstructed from median ToFs -- no 0/360 wraparound problem.
     *=========================================================================*/
    float speed = cordic_magnitude(vx * WIND_CORDIC_INV, vy * WIND_CORDIC_INV) * WIND_CORDIC_SCALE;
    float angle_to = cordic_atan2_deg(vx, vy);   /* TO direction [-180,+180) */
    float dir_from = angle_to + 180.0f;
    if (dir_from >= 360.0f) { dir_from -= 360.0f; }
    if (dir_from <    0.0f) { dir_from += 360.0f; }
    out->Speed     = speed;
    out->Direction = dir_from;
    return hw_status;   /* HAL_OK if all 3 transmitters succeeded this cycle */
}