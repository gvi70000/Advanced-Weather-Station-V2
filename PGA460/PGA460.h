/***************************************************************************
 * @file [PGA460].h/.c
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

#ifndef PGA460_H
#define PGA460_H

#include "stm32g4xx_hal.h"
#include "tim.h"

// -----------------------------------------------------------------
// Protocol constants
// -----------------------------------------------------------------

#define PGA460_SYNC             0x55  // Sync byte for UART baud-rate auto-detection
#define PGA460_UNLOCK_EEPROM    0x68  // Write to REG_EE_CTRL to unlock EEPROM programming
#define PGA460_LOCK_EEPROM      0x69  // Write to REG_EE_CTRL to trigger EEPROM burn (then locks)
#define ECHO_DATA_DUMP_ENABLE   0x80  // REG_EE_CTRL value that enables Echo Data Dump mode
#define ECHO_DATA_DUMP_DISABLE  0x00  // REG_EE_CTRL value that disables Echo Data Dump mode
#define ECHO_DATA_TOTAL_BYTES   130   // Echo Data Dump response: 2-byte header + 128 amplitude bins

// -----------------------------------------------------------------
// EEPROM register map  (addresses 0x00-0x2B, nonvolatile)
// Content is preserved during power cycles and low-power mode.
// -----------------------------------------------------------------

#define REG_USER_DATA1      0x00  // R/W  User data byte 1   - Reset: 0x00
#define REG_USER_DATA2      0x01  // R/W  User data byte 2   - Reset: 0x00
#define REG_USER_DATA3      0x02  // R/W  User data byte 3   - Reset: 0x00
#define REG_USER_DATA4      0x03  // R/W  User data byte 4   - Reset: 0x00
#define REG_USER_DATA5      0x04  // R/W  User data byte 5   - Reset: 0x00
#define REG_USER_DATA6      0x05  // R/W  User data byte 6   - Reset: 0x00
#define REG_USER_DATA7      0x06  // R/W  User data byte 7   - Reset: 0x00
#define REG_USER_DATA8      0x07  // R/W  User data byte 8   - Reset: 0x00
#define REG_USER_DATA9      0x08  // R/W  User data byte 9   - Reset: 0x00
#define REG_USER_DATA10     0x09  // R/W  User data byte 10  - Reset: 0x00
#define REG_USER_DATA11     0x0A  // R/W  User data byte 11  - Reset: 0x00
#define REG_USER_DATA12     0x0B  // R/W  User data byte 12  - Reset: 0x00
#define REG_USER_DATA13     0x0C  // R/W  User data byte 13  - Reset: 0x00
#define REG_USER_DATA14     0x0D  // R/W  User data byte 14  - Reset: 0x00
#define REG_USER_DATA15     0x0E  // R/W  User data byte 15  - Reset: 0x00
#define REG_USER_DATA16     0x0F  // R/W  User data byte 16  - Reset: 0x00
#define REG_USER_DATA17     0x10  // R/W  User data byte 17  - Reset: 0x00
#define REG_USER_DATA18     0x11  // R/W  User data byte 18  - Reset: 0x00
#define REG_USER_DATA19     0x12  // R/W  User data byte 19  - Reset: 0x00
#define REG_USER_DATA20     0x13  // R/W  User data byte 20  - Reset: 0x00

#define REG_TVGAIN0         0x14  // R/W  TVG time-gain profile byte 0  - Reset: 0xAF
#define REG_TVGAIN1         0x15  // R/W  TVG time-gain profile byte 1  - Reset: 0xFF
#define REG_TVGAIN2         0x16  // R/W  TVG time-gain profile byte 2  - Reset: 0xFF
#define REG_TVGAIN3         0x17  // R/W  TVG time-gain profile byte 3  - Reset: 0x2D
#define REG_TVGAIN4         0x18  // R/W  TVG time-gain profile byte 4  - Reset: 0x68
#define REG_TVGAIN5         0x19  // R/W  TVG time-gain profile byte 5  - Reset: 0x36
#define REG_TVGAIN6         0x1A  // R/W  TVG time-gain profile byte 6  - Reset: 0xFC

#define REG_INIT_GAIN       0x1B  // R/W  Initial AFE gain and BPF bypass - Reset: 0xC0
#define REG_FREQUENCY       0x1C  // R/W  Transducer frequency (f = 0.2*val + 30 kHz) - Reset: 0x8C
#define REG_DEADTIME        0x1D  // R/W  Deglitch window and pulse dead time - Reset: 0x00
#define REG_PULSE_P1        0x1E  // R/W  Preset 1: IO mode, diagnostics, pulse count - Reset: 0x01
#define REG_PULSE_P2        0x1F  // R/W  Preset 2: UART address (bits 7:5) and pulse count - Reset: 0x12
#define REG_CURR_LIM_P1     0x20  // R/W  Current limit for Preset 1 driver - Reset: 0x47
#define REG_CURR_LIM_P2     0x21  // R/W  Current limit for Preset 2 driver - Reset: 0xFF
#define REG_REC_LENGTH      0x22  // R/W  Record-time window length - Reset: 0x1C
#define REG_FREQ_DIAG       0x23  // R/W  Frequency diagnostic enable and threshold - Reset: 0x00
#define REG_SAT_FDIAG_TH    0x24  // R/W  Saturation and frequency diagnostic threshold - Reset: 0xEE
#define REG_FVOLT_DEC       0x25  // R/W  Frequency voltage decay threshold - Reset: 0x7C
#define REG_DECPL_TEMP      0x26  // R/W  AFE gain range (bits 7:6) and temperature compensation - Reset: 0x0A
#define REG_DSP_SCALE       0x27  // R/W  DSP output scaling factor - Reset: 0x00
#define REG_TEMP_TRIM       0x28  // R/W  Temperature measurement offset trim - Reset: 0x00
#define REG_P1_GAIN_CTRL    0x29  // R/W  Preset 1 gain control - Reset: 0x05
#define REG_P2_GAIN_CTRL    0x2A  // R/W  Preset 2 gain control - Reset: 0x05
#define REG_EE_CRC          0x2B  // R    EEPROM CRC (auto-calculated on burn) - Reset: N/A

// -----------------------------------------------------------------
// Volatile register map  (addresses 0x40-0x4D and 0x5F-0x7F)
// Content is lost on power cycle and low-power mode.
// Addresses 0x2C-0x3F and 0x4E-0x5E are TI-internal and inaccessible.
// -----------------------------------------------------------------

#define REG_EE_CTRL         0x40  // R/W  EEPROM control (unlock, burn, data-dump enable) - Reset: 0x00
#define REG_BPF_A2_MSB      0x41  // R/W  Band-pass filter A2 coefficient MSB - Reset: 0x00
#define REG_BPF_A2_LSB      0x42  // R/W  Band-pass filter A2 coefficient LSB - Reset: 0x00
#define REG_BPF_A3_MSB      0x43  // R/W  Band-pass filter A3 coefficient MSB - Reset: 0x00
#define REG_BPF_A3_LSB      0x44  // R/W  Band-pass filter A3 coefficient LSB - Reset: 0x00
#define REG_BPF_B1_MSB      0x45  // R/W  Band-pass filter B1 coefficient MSB - Reset: 0x00
#define REG_BPF_B1_LSB      0x46  // R/W  Band-pass filter B1 coefficient LSB - Reset: 0x00
#define REG_LPF_A2_MSB      0x47  // R/W  Low-pass filter A2 coefficient MSB - Reset: 0x00
#define REG_LPF_A2_LSB      0x48  // R/W  Low-pass filter A2 coefficient LSB - Reset: 0x00
#define REG_LPF_B1_MSB      0x49  // R/W  Low-pass filter B1 coefficient MSB - Reset: 0x00
#define REG_LPF_B1_LSB      0x4A  // R/W  Low-pass filter B1 coefficient LSB - Reset: 0x00
#define REG_TEST_MUX        0x4B  // R/W  Test multiplexer and data-path output select - Reset: 0x00
#define REG_DEV_STAT0       0x4C  // R/W  Device status 0: EEPROM/CRC/wakeup errors - Reset: 0x84
#define REG_DEV_STAT1       0x4D  // R/W  Device status 1: voltage and thermal protection flags - Reset: 0x00

#define REG_P1_THR_0        0x5F  // R/W  Preset 1 threshold map byte  0 - Reset: 0x00
#define REG_P1_THR_1        0x60  // R/W  Preset 1 threshold map byte  1 - Reset: 0x00
#define REG_P1_THR_2        0x61  // R/W  Preset 1 threshold map byte  2 - Reset: 0x00
#define REG_P1_THR_3        0x62  // R/W  Preset 1 threshold map byte  3 - Reset: 0x00
#define REG_P1_THR_4        0x63  // R/W  Preset 1 threshold map byte  4 - Reset: 0x00
#define REG_P1_THR_5        0x64  // R/W  Preset 1 threshold map byte  5 - Reset: 0x00
#define REG_P1_THR_6        0x65  // R/W  Preset 1 threshold map byte  6 - Reset: 0x00
#define REG_P1_THR_7        0x66  // R/W  Preset 1 threshold map byte  7 - Reset: 0x00
#define REG_P1_THR_8        0x67  // R/W  Preset 1 threshold map byte  8 - Reset: 0x00
#define REG_P1_THR_9        0x68  // R/W  Preset 1 threshold map byte  9 - Reset: 0x00
#define REG_P1_THR_10       0x69  // R/W  Preset 1 threshold map byte 10 - Reset: 0x00
#define REG_P1_THR_11       0x6A  // R/W  Preset 1 threshold map byte 11 - Reset: 0x00
#define REG_P1_THR_12       0x6B  // R/W  Preset 1 threshold map byte 12 - Reset: 0x00
#define REG_P1_THR_13       0x6C  // R/W  Preset 1 threshold map byte 13 - Reset: 0x00
#define REG_P1_THR_14       0x6D  // R/W  Preset 1 threshold map byte 14 - Reset: 0x00
#define REG_P1_THR_15       0x6E  // R/W  Preset 1 threshold map byte 15 - Reset: 0x00

#define REG_P2_THR_0        0x6F  // R/W  Preset 2 threshold map byte  0 - Reset: 0x00
#define REG_P2_THR_1        0x70  // R/W  Preset 2 threshold map byte  1 - Reset: 0x00
#define REG_P2_THR_2        0x71  // R/W  Preset 2 threshold map byte  2 - Reset: 0x00
#define REG_P2_THR_3        0x72  // R/W  Preset 2 threshold map byte  3 - Reset: 0x00
#define REG_P2_THR_4        0x73  // R/W  Preset 2 threshold map byte  4 - Reset: 0x00
#define REG_P2_THR_5        0x74  // R/W  Preset 2 threshold map byte  5 - Reset: 0x00
#define REG_P2_THR_6        0x75  // R/W  Preset 2 threshold map byte  6 - Reset: 0x00
#define REG_P2_THR_7        0x76  // R/W  Preset 2 threshold map byte  7 - Reset: 0x00
#define REG_P2_THR_8        0x77  // R/W  Preset 2 threshold map byte  8 - Reset: 0x00
#define REG_P2_THR_9        0x78  // R/W  Preset 2 threshold map byte  9 - Reset: 0x00
#define REG_P2_THR_10       0x79  // R/W  Preset 2 threshold map byte 10 - Reset: 0x00
#define REG_P2_THR_11       0x7A  // R/W  Preset 2 threshold map byte 11 - Reset: 0x00
#define REG_P2_THR_12       0x7B  // R/W  Preset 2 threshold map byte 12 - Reset: 0x00
#define REG_P2_THR_13       0x7C  // R/W  Preset 2 threshold map byte 13 - Reset: 0x00
#define REG_P2_THR_14       0x7D  // R/W  Preset 2 threshold map byte 14 - Reset: 0x00
#define REG_P2_THR_15       0x7E  // R/W  Preset 2 threshold map byte 15 - Reset: 0x00
#define REG_THR_CRC         0x7F  // R/W  Threshold map CRC - Reset: 0x00

// -----------------------------------------------------------------
// Type definitions
// -----------------------------------------------------------------

// @brief UART command codes for the PGA460 communication protocol.
typedef enum {
    PGA460_CMD_BURST_AND_LISTEN_PRESET1       = 0x00,  // Burst + listen using Preset 1 settings
    PGA460_CMD_BURST_AND_LISTEN_PRESET2       = 0x01,  // Burst + listen using Preset 2 settings
    PGA460_CMD_LISTEN_ONLY_PRESET1            = 0x02,  // Listen only using Preset 1 (no burst)
    PGA460_CMD_LISTEN_ONLY_PRESET2            = 0x03,  // Listen only using Preset 2 (no burst)
    PGA460_CMD_TEMP_AND_NOISE_MEASUREMENT     = 0x04,  // Trigger temperature or noise measurement
    PGA460_CMD_ULTRASONIC_MEASUREMENT_RESULT  = 0x05,  // Read back ultrasonic measurement result
    PGA460_CMD_TEMP_AND_NOISE_RESULT          = 0x06,  // Read back temperature or noise result
    PGA460_CMD_TRANSDUCER_ECHO_DATA_DUMP      = 0x07,  // Read 128-sample echo amplitude dump
    PGA460_CMD_SYSTEM_DIAGNOSTICS             = 0x08,  // Request system diagnostics (frequency, decay)
    PGA460_CMD_REGISTER_READ                  = 0x09,  // Read a single register
    PGA460_CMD_REGISTER_WRITE                 = 0x0A,  // Write a single register
    PGA460_CMD_EEPROM_BULK_READ               = 0x0B,  // Bulk-read 44-byte EEPROM image
    PGA460_CMD_EEPROM_BULK_WRITE              = 0x0C,  // Bulk-write 44-byte EEPROM image
    PGA460_CMD_TVG_BULK_READ                  = 0x0D,  // Bulk-read 7 TVG gain bytes
    PGA460_CMD_TVG_BULK_WRITE                 = 0x0E,  // Bulk-write 7 TVG gain bytes
    PGA460_CMD_THRESHOLD_BULK_READ            = 0x0F,  // Bulk-read 33-byte threshold map
    PGA460_CMD_THRESHOLD_BULK_WRITE           = 0x10,  // Bulk-write 33-byte threshold map
    PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P1  = 0x11,  // Broadcast: burst + listen Preset 1 to all sensors
    PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P2  = 0x12,  // Broadcast: burst + listen Preset 2 to all sensors
    PGA460_CMD_BROADCAST_LISTEN_ONLY_P1       = 0x13,  // Broadcast: listen only Preset 1 to all sensors
    PGA460_CMD_BROADCAST_LISTEN_ONLY_P2       = 0x14,  // Broadcast: listen only Preset 2 to all sensors
    PGA460_CMD_BROADCAST_TEMP_NOISE_MEASURE   = 0x15,  // Broadcast: trigger temp/noise measurement on all
    PGA460_CMD_BROADCAST_REGISTER_WRITE       = 0x16,  // Broadcast: write register on all sensors
    PGA460_CMD_BROADCAST_EEPROM_BULK_WRITE    = 0x17,  // Broadcast: bulk-write EEPROM on all sensors
    PGA460_CMD_BROADCAST_TVG_BULK_WRITE       = 0x18,  // Broadcast: bulk-write TVG on all sensors
    PGA460_CMD_BROADCAST_THRESHOLD_BULK_WRITE = 0x19   // Broadcast: bulk-write threshold map on all sensors
} PGA460_Command_t;

// @brief Time-varying gain (TVG) profile bytes (registers 0x14-0x1A, EEPROM).
// @details Seven consecutive bytes encode a piecewise-linear gain ramp applied
//          after the burst decouple period.  Higher values increase AFE gain
//          at the corresponding time slice after transmission.
typedef struct __attribute__((packed)) {
    uint8_t TVGAIN0;  // REG_TVGAIN0 (0x14) - gain at time slice 0
    uint8_t TVGAIN1;  // REG_TVGAIN1 (0x15) - gain at time slice 1
    uint8_t TVGAIN2;  // REG_TVGAIN2 (0x16) - gain at time slice 2
    uint8_t TVGAIN3;  // REG_TVGAIN3 (0x17) - gain at time slice 3
    uint8_t TVGAIN4;  // REG_TVGAIN4 (0x18) - gain at time slice 4
    uint8_t TVGAIN5;  // REG_TVGAIN5 (0x19) - gain at time slice 5
    uint8_t TVGAIN6;  // REG_TVGAIN6 (0x1A) - gain at time slice 6
} TVG_t;

// @brief Core EEPROM configuration registers (0x1B-0x2B).
// @details Written via EEPROM Bulk Write (CMD 0x0C) or individually with
//          Register Write (CMD 0x0A).  Mirrored in PGA460_Regs_t.EEData.Sett.
typedef struct __attribute__((packed)) {
    uint8_t INIT_GAIN;    // 0x1B - Initial AFE gain index and BPF bypass bit
    uint8_t FREQUENCY;    // 0x1C - Transducer frequency: f_kHz = 0.2*val + 30
    uint8_t DEADTIME;     // 0x1D - Deglitch window (bits 7:4) and pulse dead time (bits 3:0)
    uint8_t PULSE_P1;     // 0x1E - Preset 1: IO_IF_SEL (bit 7), DIAG_EN (bit 6), pulse count (bits 5:0)
    uint8_t PULSE_P2;     // 0x1F - Preset 2: UART address (bits 7:5), pulse count (bits 4:0)
    uint8_t CURR_LIM_P1;  // 0x20 - Preset 1 driver current limit
    uint8_t CURR_LIM_P2;  // 0x21 - Preset 2 driver current limit
    uint8_t REC_LENGTH;   // 0x22 - Record-time window length
    uint8_t FREQ_DIAG;    // 0x23 - Frequency diagnostic enable and threshold
    uint8_t SAT_FDIAG_TH; // 0x24 - Saturation and frequency diagnostic threshold
    uint8_t FVOLT_DEC;    // 0x25 - Frequency voltage decay threshold
    uint8_t DECPL_TEMP;   // 0x26 - AFE gain range (bits 7:6) and temperature compensation trim
    uint8_t DSP_SCALE;    // 0x27 - DSP output scaling
    uint8_t TEMP_TRIM;    // 0x28 - Temperature offset trim (sign-magnitude: bit 7 = sign, bits 6:0 = magnitude)
    uint8_t P1_GAIN_CTRL; // 0x29 - Preset 1 gain control
    uint8_t P2_GAIN_CTRL; // 0x2A - Preset 2 gain control
    uint8_t EE_CRC;       // 0x2B - EEPROM CRC (auto-calculated on burn; do not write manually)
} Settings_t;

// @brief EE_CTRL register (0x40) bitfield - EEPROM control and data-dump enable.
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t EE_PRGM     : 1;  // bit 0 - EEPROM program trigger (1 = program)
            uint8_t EE_RLOAD    : 1;  // bit 1 - EEPROM reload trigger (1 = reload from EEPROM into RAM)
            uint8_t EE_PRGM_OK  : 1;  // bit 2 - EEPROM programming status (1 = success)
            uint8_t EE_UNLCK    : 4;  // bits 6:3 - Unlock passcode; valid value = 0xD
            uint8_t DATADUMP_EN : 1;  // bit 7 - Echo data dump enable (1 = enabled)
        } BitField;
    } Val;
} EE_CNTRL_t;

// @brief Volatile digital filter coefficient registers (0x41-0x4A).
// @details Band-pass filter (BPF) and low-pass filter (LPF) biquad coefficients.
//          Written as 16-bit values split across MSB/LSB register pairs.
//          Lost on power cycle; must be re-applied at startup if non-default values are needed.
typedef struct __attribute__((packed)) {
    uint8_t BPF_A2_MSB;  // 0x41 - BPF A2 coefficient MSB
    uint8_t BPF_A2_LSB;  // 0x42 - BPF A2 coefficient LSB
    uint8_t BPF_A3_MSB;  // 0x43 - BPF A3 coefficient MSB
    uint8_t BPF_A3_LSB;  // 0x44 - BPF A3 coefficient LSB
    uint8_t BPF_B1_MSB;  // 0x45 - BPF B1 coefficient MSB
    uint8_t BPF_B1_LSB;  // 0x46 - BPF B1 coefficient LSB
    uint8_t LPF_A2_MSB;  // 0x47 - LPF A2 coefficient MSB
    uint8_t LPF_A2_LSB;  // 0x48 - LPF A2 coefficient LSB
    uint8_t LPF_B1_MSB;  // 0x49 - LPF B1 coefficient MSB
    uint8_t LPF_B1_LSB;  // 0x4A - LPF B1 coefficient LSB
} Filters_t;

// @brief TEST_MUX register (0x4B) - source selection for the TEST pin output.
typedef enum {
    TEST_MUX_GND        = 0x0,  // 0b000 - GND (mux disabled)
    TEST_MUX_AFE_OUTPUT = 0x1,  // 0b001 - Analog front-end output
    TEST_MUX_RESERVED1  = 0x2,  // 0b010 - Reserved
    TEST_MUX_RESERVED2  = 0x3,  // 0b011 - Reserved
    TEST_MUX_8MHZ_CLOCK = 0x4,  // 0b100 - 8 MHz internal clock
    TEST_MUX_ADC_SAMPLE = 0x5   // 0b101 - ADC sample output clock
} TEST_MUX_Enum_t;

// @brief DP_MUX field (bits 2:0 of REG_TEST_MUX) - data-path signal source.
typedef enum {
    DP_MUX_OFF       = 0x0,  // 0b000 - Disabled
    DP_MUX_LPF       = 0x1,  // 0b001 - Low-pass filter output
    DP_MUX_RECTIFIER = 0x2,  // 0b010 - Full-wave rectifier output
    DP_MUX_BPF       = 0x3,  // 0b011 - Band-pass filter output
    DP_MUX_ADC       = 0x4   // 0b100 - Raw ADC output
} DP_MUX_t;

// @brief TEST_MUX register (0x4B) bitfield.
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            DP_MUX_t        DP_MUX     : 3;  // bits 2:0 - data-path source select
            uint8_t         SAMPLE_SEL : 1;  // bit 3 - 0 = 8-bit at 1 us/sample, 1 = 12-bit at 2 us/sample
            uint8_t         RESERVED   : 1;  // bit 4 - reserved, write 0
            TEST_MUX_Enum_t TEST_MUX   : 3;  // bits 7:5 - TEST pin output source
        } BitField;
    } Val;
} TEST_MUX_t;

// @brief DEV_STAT0 register (0x4C) bitfield - EEPROM integrity and wakeup status.
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t TRIM_CRC_ERR : 1;  // bit 0 - 1 = CRC error in EEPROM trim space
            uint8_t EE_CRC_ERR   : 1;  // bit 1 - 1 = CRC error in user EEPROM space
            uint8_t THR_CRC_ERR  : 1;  // bit 2 - 1 = CRC error in threshold map
            uint8_t CMW_WU_ERR   : 1;  // bit 3 - 1 = command received before wakeup complete
            uint8_t OPT_ID       : 2;  // bits 5:4 - device option identification
            uint8_t REV_ID       : 2;  // bits 7:6 - device revision identification
        } BitField;
    } Val;
} DEV_STAT0_t;

// @brief DEV_STAT1 register (0x4D) bitfield - voltage rail and thermal protection status.
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t VPWR_UV  : 1;  // bit 0 - 1 = VPWR under-voltage
            uint8_t VPWR_OV  : 1;  // bit 1 - 1 = VPWR over-voltage
            uint8_t AVDD_UV  : 1;  // bit 2 - 1 = AVDD under-voltage
            uint8_t AVDD_OV  : 1;  // bit 3 - 1 = AVDD over-voltage
            uint8_t IOREG_UV : 1;  // bit 4 - 1 = IOREG under-voltage
            uint8_t IOREG_OV : 1;  // bit 5 - 1 = IOREG over-voltage
            uint8_t TSD_PROT : 1;  // bit 6 - 1 = thermal shutdown has occurred
            uint8_t RESERVED : 1;  // bit 7 - reserved
        } BitField;
    } Val;
} DEV_STAT1_t;

// @brief Detection threshold map for one preset (registers 0x5F-0x7E, 16 bytes each).
// @details P1_THR covers 0x5F-0x6E; P2_THR covers 0x6F-0x7E.
//          THR_CRC at 0x7F is auto-calculated on bulk write.
typedef struct __attribute__((packed)) {
    uint8_t P1_THR[16];  // Preset 1 threshold bytes (0x5F-0x6E)
    uint8_t P2_THR[16];  // Preset 2 threshold bytes (0x6F-0x7E)
    uint8_t THR_CRC;     // Threshold map CRC (0x7F, auto-calculated)
} Thresholds_t;

// @brief Full 44-byte EEPROM image layout (registers 0x00-0x2B).
// @details This is the exact byte sequence sent and received by the EEPROM
//          bulk-write (CMD 0x0C) and bulk-read (CMD 0x0B) commands.
typedef struct __attribute__((packed)) {
    uint8_t    UserData[20];  // 0x00-0x13 - application-specific user data
    TVG_t      TVG;           // 0x14-0x1A - time-varying gain profile
    Settings_t Sett;          // 0x1B-0x2B - core configuration settings
} EEImage_t;

// @brief Complete shadow of all accessible PGA460 registers for one sensor.
// @details Kept in RAM and synchronised with the device via register read/write
//          and bulk read/write commands.  Use PGA460_RegisterRead/Write for
//          individual registers, or the bulk commands for groups.
typedef struct __attribute__((packed)) {
    EEImage_t   EEData;         // 0x00-0x2B - EEPROM image
    EE_CNTRL_t  eeCtrl;         // 0x40      - EEPROM control
    //Filters_t   Filters;        // 0x41-0x4A - digital filter coefficients
    //TEST_MUX_t  TestMux;        // 0x4B      - test mux
    DEV_STAT0_t Stat0;          // 0x4C      - device status 0
    DEV_STAT1_t Stat1;          // 0x4D      - device status 1
    Thresholds_t THR;           // 0x5F-0x7F - threshold map
    uint8_t     eepromUnlocked; // Software flag: 1 = EEPROM currently unlocked
} PGA460_Regs_t;

// @brief AFE gain range selection encoded in DECPL_TEMP[7:6].
// @details Controls the front-end amplifier gain window (min-max dB).
//          Write via PGA460_SetTVG() which preserves the lower 6 bits of REG_DECPL_TEMP.
typedef enum {
    PGA460_GAIN_32_64dB = 0xC0,  // 0b11 << 6 - 32 to  64 dB gain range
    PGA460_GAIN_46_78dB = 0x80,  // 0b10 << 6 - 46 to  78 dB gain range
    PGA460_GAIN_52_84dB = 0x40,  // 0b01 << 6 - 52 to  84 dB gain range
    PGA460_GAIN_58_90dB = 0x00   // 0b00 << 6 - 58 to  90 dB gain range (widest)
} PGA460_GainRange_t;

// @brief Selector for the temperature/noise measurement command (CMD 0x04).
typedef enum {
    PGA460_CMD_GET_TEMP  = 0x00,  // Request internal temperature measurement
    PGA460_CMD_GET_NOISE = 0x01   // Request ambient noise level measurement
} PGA460_CmdType_t;

// @brief Single ultrasonic measurement result decoded from the PGA460 response frame.
typedef struct __attribute__((packed)) {
    uint16_t tof_us;    // Time-of-flight in microseconds (PGA460 UART result, 1 LSB = 1 us)
    uint8_t  width;     // Echo pulse width (raw byte; 1 LSB = 16 us after scaling)
    uint8_t  amplitude; // Echo peak amplitude (raw 8-bit value, higher = stronger echo)
} PGA460_Measure_t;

// @brief Per-sensor state: register shadow and last measurement result.
typedef struct __attribute__((packed)) {
    PGA460_Regs_t    Registers;  // Full register shadow (see PGA460_Regs_t)
    PGA460_Measure_t Measures;   // Most recent ultrasonic measurement result
    // Note: the sensor's UART address equals its index in the sensors[] array.
} PGA460_Sensor_t;

// -----------------------------------------------------------------
// Public function prototypes
// -----------------------------------------------------------------
// @brief Get a pointer to the EEImage_t of a sensor so callers (e.g. Wind.c) can
//        read/write UserData without exposing the internal sensors[] array.
// @param sensorID  Sensor UART address (0-7).
// @return Pointer to sensors[sensorID].Registers.EEData, or NULL if sensorID is out of range.
EEImage_t *PGA460_GetEEData(uint8_t sensorID);

// Initialisation
// @brief Initialise all sensors, write threshold maps, and optionally burn EEPROM.
// @param burnEEPROM  1 = commit configuration to EEPROM flash; 0 = RAM only.
// @return HAL status.
HAL_StatusTypeDef PGA460_Init(uint8_t burnEEPROM);

// @brief Scan the UART bus (addresses 0-7), assign a new UART address, and burn it to EEPROM.
// @param index  Target UART address (0-7) to assign to the responding sensor.
// @return HAL status.
HAL_StatusTypeDef PGA460_WriteUARTAddr(uint8_t index);

// @brief Scan the UART bus and print a report of detected sensors via printf.
void PGA460_ScanAndReport(void);

// Register access
// @brief Read a single register from a sensor.
// @param sensorID  Sensor UART address (0-7).
// @param regAddr   Register address.
// @param regValue  Output: register value.
// @return HAL status.
HAL_StatusTypeDef PGA460_RegisterRead(uint8_t sensorID, uint8_t regAddr, uint8_t *regValue);

// @brief Write a single register on a sensor.
// @param sensorID  Sensor UART address (0-7).
// @param regAddr   Register address.
// @param regValue  Value to write.
// @return HAL status.
HAL_StatusTypeDef PGA460_RegisterWrite(uint8_t sensorID, uint8_t regAddr, uint8_t regValue);

// @brief Read DEV_STAT0 and DEV_STAT1 and log any active fault flags via DEBUG.
// @param sensorID  Sensor UART address (0-7).
// @return HAL status.
HAL_StatusTypeDef PGA460_CheckStatus(uint8_t sensorID);

// EEPROM and configuration
// @brief Bulk-write the 44-byte EEPROM image from sensors[sensorID].Registers.EEData to the device.
// @param sensorID  Sensor UART address (0-7).
// @return HAL status.
HAL_StatusTypeDef PGA460_EEPROMBulkWrite(uint8_t sensorID);

// @brief Bulk-read the 44-byte EEPROM image from device into sensors[sensorID].Registers.EEData.
// @details Sends CMD 0x0B and receives [DIAG][44 EEData bytes][CHK] = 46 bytes.
//          Call this during PGA460_Init() so UserData (path lengths) are available
//          to Wind_LoadCalibration() before any measurement starts.
// @param sensorID  Sensor UART address (0-7).
// @return HAL status.
HAL_StatusTypeDef PGA460_EEPROMBulkRead(uint8_t sensorID);

// @brief Burn the current EEPROM image to non-volatile flash inside the PGA460.
// @details Unlocks, triggers the internal charge pump, waits 1000 ms, and verifies EE_PRGM_OK.
//          Use sparingly: the PGA460 supports a limited number of EEPROM burn cycles.
// @param sensorID  Sensor UART address (0-7).
// @return HAL status.
HAL_StatusTypeDef PGA460_BurnEEPROM(uint8_t sensorID);

// @brief Bulk-read TVG bytes (TVGAIN0-TVGAIN6) from device into sensors[sensorID].Registers.EEData.TVG.
// @param sensorID  Sensor UART address (0-7).
// @return HAL status.
HAL_StatusTypeDef PGA460_GetTVG(uint8_t sensorID);

// @brief Set AFE gain range and write the TVG profile to the device.
// @param sensorID    Sensor UART address (0-7).
// @param gain_range  AFE gain range selection (see PGA460_GainRange_t).
// @param tvg         Pointer to the 7-byte TVG profile to send.
// @return HAL status.
HAL_StatusTypeDef PGA460_SetTVG(uint8_t sensorID, PGA460_GainRange_t gain_range, const TVG_t *tvg);

// @brief Bulk-read the threshold map from the device into sensors[sensorID].Registers.THR.
// @param sensorID  Sensor UART address (0-7).
// @return HAL status.
HAL_StatusTypeDef PGA460_GetThresholds(uint8_t sensorID);

// @brief Bulk-write the threshold map from sensors[sensorID].Registers.THR to the device.
// @param sensorID  Sensor UART address (0-7).
// @return HAL status.
HAL_StatusTypeDef PGA460_SetThresholds(uint8_t sensorID);

// Ultrasonic measurement
// @brief Send a UART command to trigger or read a measurement.
// @param sensorID  Sensor UART address (0-7).
// @param cmd       Command code (see PGA460_Command_t).
// @param noWait    1 = return immediately; 0 = wait PGA460_CAPTURE_DELAY_MS after burst commands.
// @return HAL status.
HAL_StatusTypeDef PGA460_UltrasonicCmd(const uint8_t sensorID, const PGA460_Command_t cmd, const uint8_t noWait);

// @brief Request and parse the ultrasonic measurement result into sensors[sensorID].Measures.
// @param sensorID  Sensor UART address (0-7).
// @return HAL status.
HAL_StatusTypeDef PGA460_GetUltrasonicMeasurement(const uint8_t sensorID);

// @brief Trigger a burst, capture 128 echo amplitude bins, and return them in echoOut.
// @param sensorID  Sensor UART address (0-7).
// @param preset    Preset command to trigger (PGA460_CMD_BURST_AND_LISTEN_PRESET1 or PRESET2).
// @param echoOut   Output buffer: 128 bytes of amplitude data (bins 0-127).
// @return HAL status.
HAL_StatusTypeDef PGA460_GetEchoDataDump(uint8_t sensorID, uint8_t preset, uint8_t *echoOut);

// Diagnostics and calibration
// @brief Compute and write a temperature offset so the PGA460 matches an external reference.
// @details Reads the PGA460 internal temperature, computes (external - internal), clamps the
//          signed offset to [-64, +63] degrees, and writes it to REG_TEMP_TRIM (0x28).
// @param sensorID       Sensor UART address (0-7).
// @param externalTempC  Reference temperature in degrees Celsius (from BMP581 or HDC302x).
// @return HAL status.
HAL_StatusTypeDef PGA460_SetTemperatureOffset(uint8_t sensorID, float externalTempC);

// @brief Read a diagnostic value (frequency, decay period, temperature, or noise).
// @param sensorID   Sensor UART address (0-7).
// @param run        1 = trigger a burst before reading diagnostics; 0 = read last result.
// @param diag       0 = frequency (Hz), 1 = decay period (us), 2 = temperature (degC), 3 = noise (raw).
// @param diagResult Output: diagnostic value in the units described above.
// @return HAL status.
HAL_StatusTypeDef PGA460_GetSystemDiagnostics(const uint8_t sensorID, const uint8_t run, const uint8_t diag, float *diagResult);

// @brief Trigger a temperature or noise measurement and return the result.
// @param sensorID  Sensor UART address (0-7).
// @param mode      PGA460_CMD_GET_TEMP or PGA460_CMD_GET_NOISE.
// @return Temperature in degC, or raw noise byte, or PGA460_TEMP_ERR (999.0f) on failure.
float PGA460_ReadTemperatureOrNoise(const uint8_t sensorID, const PGA460_CmdType_t mode);

#endif // PGA460_H