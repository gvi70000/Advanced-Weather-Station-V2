/***************************************************************************
 * @file [Wind].h/.c
 * This file contains definitions, data structures, and functions
 * for interfacing with the TI PGA460 ultrasonic signal processor.
 * Copyright (c) [2024] Grozea Ion gvi70000
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 ***************************************************************************/
// TO BE USED WITH PGA460.html to generate register values.

#ifndef WIND_H
#define WIND_H

#include "stm32g4xx_hal.h"

// @brief Wind measurement result returned by PGA460_MeasureWind().
typedef struct __attribute__((packed)) {
    float Speed;      // Wind speed in m/s
    float Direction;  // Wind direction in degrees FROM (0/360 = N, 90 = E, 180 = S, 270 = W)
} Wind_t;

// @brief Environmental data used to compute the speed of sound in moist air.
// @details Populate via BMP581 (Temperature, Pressure), HDC302x (RH), and GPS (Height).
//          Call PGA460_UpdateEnvironment() to update and recompute SoundSpeed.
//          SoundSpeed is then used automatically by PGA460_MeasureWind().
typedef struct __attribute__((packed)) {
    float Height;      // Altitude above sea level in metres (from GPS; 0 to use barometric estimate)
    float Temperature; // Ambient temperature in degrees Celsius (from BMP581 / HDC302x)
    float RH;          // Relative humidity in % (from HDC302x)
    float Pressure;    // Atmospheric pressure in hPa (from BMP581; 0 to estimate from Height)
    float SoundSpeed;  // Speed of sound in m/s - computed by PGA460_UpdateEnvironment()
} Wind_EnvData_t;

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

// Global environmental data instance (defined in PGA460.c).
// Read SoundSpeed after calling PGA460_UpdateEnvironment().
extern Wind_EnvData_t externalData;


// @brief Load previously calibrated path lengths from each sensor's PGA460 USER_DATA EEPROM.
// @details Call once at boot after PGA460_Init() so prior calibrations survive power-off.
//          Falls back to the nominal geometric path length if stored values are out of range.
void Wind_LoadCalibration(void);

// @brief Measure the true acoustic path length for all three transducer pairs in still air.
// @details Fires all three transmitters N_CAL times (identical to PGA460_MeasureWind), computes
//          D_ij = c * (t_ij + t_ji) / 2 for each bidirectional pair (wind cancels in the sum),
//          stores results in RAM and optionally in each sensor's PGA460 USER_DATA EEPROM.
//          Must be called with no wind present (indoor bench or sealed enclosure).
// @param soundSpeed_ms  Speed of sound in m/s from PGA460_UpdateEnvironment() (1 m/s = 1 um/us).
// @param burnEEPROM     1 = commit calibration values to PGA460 EEPROM flash; 0 = RAM only.
// @return HAL status.
HAL_StatusTypeDef Wind_CalibrateReflector(float soundSpeed_ms, uint8_t burnEEPROM);

// @brief Update environmental data and recompute speed of sound.
// @details Writes tempC, rh, hPa, and height_m into externalData and calls
//          PGA460_ComputeSoundSpeed() to refresh externalData.SoundSpeed.
//          Call whenever BMP581, HDC302x, or GPS data is refreshed (typically once per second).
// @param tempC     Ambient temperature in degrees Celsius.
// @param rh        Relative humidity in %.
// @param hPa       Atmospheric pressure in hPa.
// @param height_m  Altitude in metres above sea level (0 to estimate from pressure).
void Wind_UpdateEnvironment(float tempC, float rh, float hPa, float height_m);

// @brief Perform a full 3-transmitter measurement cycle and return filtered wind vector.
// @details Fires each transmitter in turn via TIM2 DECPL hardware timestamping (DMA),
//          inserts the six raw one-way ToF values into per-path sliding-window buffers
//          (N = 5), computes the median of each buffer, reconstructs the wind vector
//          using the closed-form least-squares formula, and returns speed and direction
//          via CORDIC MODULUS and PHASE.
//          Returns HAL_OK when all three transmitters succeeded; HAL_ERROR when a fault
//          occurred (the most recent valid filtered output is still written to *out).
// @param out  Output: Speed (m/s) and Direction (degrees FROM: 0/360 = N, 90 = E, 180 = S, 270 = W).
// @return HAL status.
HAL_StatusTypeDef Wind_Measure(Wind_t *out);

#endif // PGA460_H