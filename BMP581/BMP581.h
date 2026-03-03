/**
 * @file BMP581.h
 * @brief Header file for the BMP581 barometric pressure and temperature sensor library.
 *
 * This library contains all the definitions, enumerations, register structures, and
 * function prototypes for initializing, configuring, and reading data from the BMP581
 * sensor over I2C.
 */

#ifndef __BMP581_H
#define __BMP581_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "i2c.h"
#include <stdint.h>

// Conversion coefficients
#define TEMP_COEFF      65536.0f  // Raw-to-float divisor for temperature (2^16)
#define PRESS_COEFF     64.0f     // Raw-to-float divisor for pressure    (2^6)

// I2C address
#define BMP581_ADDRESS  0x8C  // 0x46 << 1

// Chip ID values
#define BMP581_CHIP_ID_PRIM  0x50  // Primary chip ID
#define BMP581_CHIP_ID_SEC   0x51  // Secondary chip ID

// Register addresses
#define BMP581_REG_CHIP_ID        0x01  // Chip ID register
#define BMP581_REG_REV_ID         0x02  // Revision ID register
#define BMP581_REG_CHIP_STATUS    0x11  // ASIC status register
#define BMP581_REG_DRIVE_CONFIG   0x13  // Host interface configuration register
#define BMP581_REG_INT_CONFIG     0x14  // Interrupt configuration register
#define BMP581_REG_INT_SOURCE     0x15  // Interrupt source selection register
#define BMP581_REG_FIFO_CONFIG    0x16  // FIFO configuration register
#define BMP581_REG_FIFO_COUNT     0x17  // Number of frames in FIFO register
#define BMP581_REG_FIFO_SEL       0x18  // FIFO frame selection register
#define BMP581_REG_TEMP_DATA_XLSB 0x1D  // Temperature data XLSB
#define BMP581_REG_TEMP_DATA_LSB  0x1E  // Temperature data LSB
#define BMP581_REG_TEMP_DATA_MSB  0x1F  // Temperature data MSB
#define BMP581_REG_PRESS_DATA_XLSB 0x20 // Pressure data XLSB
#define BMP581_REG_PRESS_DATA_LSB  0x21 // Pressure data LSB
#define BMP581_REG_PRESS_DATA_MSB  0x22 // Pressure data MSB
#define BMP581_REG_INT_STATUS     0x27  // Interrupt status register
#define BMP581_REG_STATUS         0x28  // Sensor status register
#define BMP581_REG_FIFO_DATA      0x29  // FIFO data register
#define BMP581_REG_NVM_ADDR       0x2B  // NVM address register
#define BMP581_REG_NVM_DATA_LSB   0x2C  // NVM data LSB register
#define BMP581_REG_NVM_DATA_MSB   0x2D  // NVM data MSB register
#define BMP581_REG_DSP_CONFIG     0x30  // DSP configuration register
#define BMP581_REG_DSP_IIR        0x31  // DSP IIR filter configuration register
#define BMP581_REG_OOR_THR_P_LSB  0x32  // Pressure OOR threshold LSB
#define BMP581_REG_OOR_THR_P_MSB  0x33  // Pressure OOR threshold MSB
#define BMP581_REG_OOR_RANGE      0x34  // Pressure OOR range register
#define BMP581_REG_OOR_CONFIG     0x35  // OOR configuration register
#define BMP581_REG_OSR_CONFIG     0x36  // Oversampling rate configuration register
#define BMP581_REG_ODR_CONFIG     0x37  // Output data rate configuration register
#define BMP581_REG_OSR_EFF        0x38  // Effective OSR configuration register
#define BMP581_REG_CMD            0x7E  // Command register

/**
 * @brief Host interface mode selection.
 */
typedef enum {
    BMP581_HIF_MODE_I2C_ONLY           = 0x0,  // 0b00: I2C Mode Only (SPI disabled)
    BMP581_HIF_MODE_SPI_MODE1_MODE2    = 0x1,  // 0b01: SPI Mode1 and Mode2
    BMP581_HIF_MODE_SPI_MODE0_MODE3    = 0x2,  // 0b10: SPI Mode0 and Mode3
    BMP581_HIF_MODE_SPI_I2C_AUTOCONFIG = 0x3   // 0b11: SPI and I2C available (autoconfig)
} BMP581_hif_mode_t;

/**
 * @brief I3C SDR parity error status.
 */
typedef enum {
    BMP581_I3C_ERR_0_NO_ERROR = 0x0,  // 0b0: No SDR parity error occurred
    BMP581_I3C_ERR_0_ERROR    = 0x1   // 0b1: SDR parity error occurred
} BMP581_i3c_err_0_t;

/**
 * @brief I3C S0/S1 error status.
 */
typedef enum {
    BMP581_I3C_ERR_3_NO_ERROR = 0x0,  // 0b0: No S0/S1 error occurred
    BMP581_I3C_ERR_3_ERROR    = 0x1   // 0b1: S0/S1 error occurred
} BMP581_i3c_err_3_t;

/**
 * @brief I2C CSB pull-up enable.
 */
typedef enum {
    BMP581_I2C_CSB_PULLUP_DISABLED = 0x0,  // 0b0: Pull-up disabled
    BMP581_I2C_CSB_PULLUP_ENABLED  = 0x1   // 0b1: Pull-up enabled
} BMP581_i2c_csb_pup_en_t;

/**
 * @brief SPI wire mode selection.
 */
typedef enum {
    BMP581_SPI_4WIRE_MODE = 0x0,  // 0b0: SPI 4-wire mode
    BMP581_SPI_3WIRE_MODE = 0x1   // 0b1: SPI 3-wire mode
} BMP581_spi3_en_t;

/**
 * @brief Pad drive strength for serial IO pins.
 */
typedef enum {
    BMP581_PAD_DRV_LOW  = 0x0,  // 0b0: Low drive strength
    BMP581_PAD_DRV_HIGH = 0x1   // 0b1: High drive strength
} BMP581_pad_if_drv_t;

/**
 * @brief Interrupt pin mode (pulsed or latched).
 */
typedef enum {
    BMP581_INT_MODE_PULSED  = 0x0,  // 0b0: Pulsed
    BMP581_INT_MODE_LATCHED = 0x1   // 0b1: Latched
} BMP581_int_mode_t;

/**
 * @brief Interrupt pin polarity.
 */
typedef enum {
    BMP581_INT_POL_ACTIVE_LOW  = 0x0,  // 0b0: Active Low
    BMP581_INT_POL_ACTIVE_HIGH = 0x1   // 0b1: Active High
} BMP581_int_pol_t;

/**
 * @brief Interrupt pin output type.
 */
typedef enum {
    BMP581_INT_PIN_PUSH_PULL  = 0x0,  // 0b0: Push-Pull
    BMP581_INT_PIN_OPEN_DRAIN = 0x1   // 0b1: Open-Drain
} BMP581_int_od_t;

/**
 * @brief Interrupt enable/disable.
 */
typedef enum {
    BMP581_INT_DISABLE = 0x0,  // 0b0: Interrupts disabled
    BMP581_INT_ENABLE  = 0x1   // 0b1: Interrupts enabled
} BMP581_int_en_t;

/**
 * @brief Pad drive strength for the INT pin.
 */
typedef enum {
    BMP581_INT_DRV_LOW  = 0x0,  // 0b0: Low drive strength
    BMP581_INT_DRV_HIGH = 0x1   // 0b1: High drive strength
} BMP581_pad_int_drv_t;

/**
 * @brief Data Ready interrupt enable.
 */
typedef enum {
    BMP581_DRDY_DISABLE = 0x0,  // 0b0: Data Ready interrupt disabled
    BMP581_DRDY_ENABLE  = 0x1   // 0b1: Data Ready interrupt enabled
} BMP581_drdy_data_reg_en_t;

/**
 * @brief FIFO Full interrupt enable.
 */
typedef enum {
    BMP581_FIFO_FULL_DISABLE = 0x0,  // 0b0: FIFO Full interrupt disabled
    BMP581_FIFO_FULL_ENABLE  = 0x1   // 0b1: FIFO Full interrupt enabled
} BMP581_fifo_full_en_t;

/**
 * @brief FIFO Threshold/Watermark interrupt enable.
 */
typedef enum {
    BMP581_FIFO_THS_DISABLE = 0x0,  // 0b0: FIFO Threshold/Watermark interrupt disabled
    BMP581_FIFO_THS_ENABLE  = 0x1   // 0b1: FIFO Threshold/Watermark interrupt enabled
} BMP581_fifo_ths_en_t;

/**
 * @brief Pressure Out-of-Range interrupt enable.
 */
typedef enum {
    BMP581_OOR_P_DISABLE = 0x0,  // 0b0: Pressure Out-of-Range interrupt disabled
    BMP581_OOR_P_ENABLE  = 0x1   // 0b1: Pressure Out-of-Range interrupt enabled
} BMP581_oor_p_en_t;

/**
 * @brief FIFO threshold (watermark) level.
 */
typedef enum {
    BMP581_FIFO_THR_DISABLE   = 0x00, // 0x00: FIFO threshold disabled
    BMP581_FIFO_THR_31_FRAMES = 0x1F  // 0x1F: FIFO threshold set to 31 frames
} BMP581_fifo_threshold_t;

/**
 * @brief FIFO operating mode.
 */
typedef enum {
    BMP581_FIFO_MODE_STREAM       = 0x0,  // 0b0: Stream-to-FIFO Mode
    BMP581_FIFO_MODE_STOP_ON_FULL = 0x1   // 0b1: STOP-on-FULL Mode
} BMP581_fifo_mode_t;

/**
 * @brief FIFO frame data source selection.
 */
typedef enum {
    BMP581_FIFO_NOT_ENABLED      = 0x0,  // 0b00: FIFO not enabled
    BMP581_FIFO_TEMP_DATA        = 0x1,  // 0b01: Temperature data only
    BMP581_FIFO_PRESSURE_DATA    = 0x2,  // 0b10: Pressure data only
    BMP581_FIFO_PRESS_TEMP_DATA  = 0x3   // 0b11: Pressure and temperature data
} BMP581_fifo_frame_s_t;

/**
 * @brief FIFO decimation (downsampling) selection.
 */
typedef enum {
    BMP581_FIFO_NO_DOWNSAMPLING  = 0x0,  // 0b000: No decimation
    BMP581_FIFO_DOWNSAMPLING_2X  = 0x1,  // 0b001: 2x downsampling
    BMP581_FIFO_DOWNSAMPLING_4X  = 0x2,  // 0b010: 4x downsampling
    BMP581_FIFO_DOWNSAMPLING_8X  = 0x3,  // 0b011: 8x downsampling
    BMP581_FIFO_DOWNSAMPLING_16X = 0x4,  // 0b100: 16x downsampling
    BMP581_FIFO_DOWNSAMPLING_32X = 0x5,  // 0b101: 32x downsampling
    BMP581_FIFO_DOWNSAMPLING_64X = 0x6,  // 0b110: 64x downsampling
    BMP581_FIFO_DOWNSAMPLING_128X = 0x7  // 0b111: 128x downsampling
} BMP581_fifo_dec_sel_t;

/**
 * @brief Pressure and Temperature compensation enable.
 */
typedef enum {
    BMP581_COMP_NONE            = 0x0,  // 0b00: No compensation for Pressure or Temperature
    BMP581_COMP_TEMP_ONLY       = 0x1,  // 0b01: Temperature compensation only
    BMP581_COMP_PRESS_TEMP      = 0x2,  // 0b10: Pressure compensation with temperature (uncompensated T output)
    BMP581_COMP_PRESS_TEMP_BOTH = 0x3   // 0b11: Full compensation for both Pressure and Temperature
} BMP581_comp_pt_en_t;

/**
 * @brief IIR flush behaviour in FORCED mode.
 */
typedef enum {
    BMP581_IIR_FLUSH_DISABLED = 0x0,  // 0b0: IIR flush disabled
    BMP581_IIR_FLUSH_ENABLED  = 0x1   // 0b1: IIR flush enabled in FORCED mode
} BMP581_iir_flush_forced_en_t;

/**
 * @brief IIR filter output source selection (before or after filter).
 */
typedef enum {
    BMP581_IIR_BEFORE_FILTER = 0x0,  // 0b0: Value selected before IIR filter
    BMP581_IIR_AFTER_FILTER  = 0x1   // 0b1: Value selected after IIR filter
} BMP581_iir_selection_t;

/**
 * @brief OOR IIR filter source selection.
 */
typedef enum {
    BMP581_OOR_IIR_BEFORE_FILTER = 0x0,  // 0b0: Value selected before IIR filter
    BMP581_OOR_IIR_AFTER_FILTER  = 0x1   // 0b1: Value selected after IIR filter
} BMP581_oor_iir_sel_t;

/**
 * @brief IIR low-pass filter coefficient selection.
 */
typedef enum {
    BMP581_IIR_BYPASS    = 0x0,  // 0b000: Bypass (no filtering)
    BMP581_IIR_COEFF_1   = 0x1,  // 0b001: Filter coefficient 1
    BMP581_IIR_COEFF_3   = 0x2,  // 0b010: Filter coefficient 3
    BMP581_IIR_COEFF_7   = 0x3,  // 0b011: Filter coefficient 7
    BMP581_IIR_COEFF_15  = 0x4,  // 0b100: Filter coefficient 15
    BMP581_IIR_COEFF_31  = 0x5,  // 0b101: Filter coefficient 31
    BMP581_IIR_COEFF_63  = 0x6,  // 0b110: Filter coefficient 63
    BMP581_IIR_COEFF_127 = 0x7   // 0b111: Filter coefficient 127
} BMP581_iir_filter_coeff_t;

/**
 * @brief Out-of-Range event count limit before triggering interrupt.
 */
typedef enum {
    BMP581_OOR_CNT_LIMIT_1  = 0x0,  // Counter limit of 1
    BMP581_OOR_CNT_LIMIT_3  = 0x1,  // Counter limit of 3
    BMP581_OOR_CNT_LIMIT_7  = 0x2,  // Counter limit of 7
    BMP581_OOR_CNT_LIMIT_15 = 0x3   // Counter limit of 15
} BMP581_oor_cnt_limit_t;

/**
 * @brief Oversampling rate selection for temperature and pressure.
 */
typedef enum {
    BMP581_OSR_1X   = 0x0,  // Oversampling rate = 1x
    BMP581_OSR_2X   = 0x1,  // Oversampling rate = 2x
    BMP581_OSR_4X   = 0x2,  // Oversampling rate = 4x
    BMP581_OSR_8X   = 0x3,  // Oversampling rate = 8x
    BMP581_OSR_16X  = 0x4,  // Oversampling rate = 16x
    BMP581_OSR_32X  = 0x5,  // Oversampling rate = 32x
    BMP581_OSR_64X  = 0x6,  // Oversampling rate = 64x
    BMP581_OSR_128X = 0x7   // Oversampling rate = 128x
} BMP581_osr_t;

/**
 * @brief Sensor power mode.
 */
typedef enum {
    BMP581_PWRMODE_STANDBY = 0x0,  // Standby mode: no measurement ongoing
    BMP581_PWRMODE_NORMAL  = 0x1,  // Normal mode: measurements at configured ODR
    BMP581_PWRMODE_FORCED  = 0x2,  // Forced mode: single one-shot measurement
    BMP581_PWRMODE_NONSTOP = 0x3   // Non-Stop mode: continuous measurements without duty-cycling
} BMP581_pwr_mode_t;

/**
 * @brief Output data rate (ODR) selection.
 */
typedef enum {
    BMP581_ODR_240_HZ   = 0x00, // 240.000 Hz
    BMP581_ODR_218_HZ   = 0x01, // 218.537 Hz
    BMP581_ODR_199_HZ   = 0x02, // 199.111 Hz
    BMP581_ODR_179_HZ   = 0x03, // 179.200 Hz
    BMP581_ODR_160_HZ   = 0x04, // 160.000 Hz
    BMP581_ODR_149_HZ   = 0x05, // 149.333 Hz
    BMP581_ODR_140_HZ   = 0x06, // 140.000 Hz
    BMP581_ODR_129_HZ   = 0x07, // 129.855 Hz
    BMP581_ODR_120_HZ   = 0x08, // 120.000 Hz
    BMP581_ODR_110_HZ   = 0x09, // 110.164 Hz
    BMP581_ODR_100_HZ   = 0x0A, // 100.299 Hz
    BMP581_ODR_89_HZ    = 0x0B, //  89.600 Hz
    BMP581_ODR_80_HZ    = 0x0C, //  80.000 Hz
    BMP581_ODR_70_HZ    = 0x0D, //  70.000 Hz
    BMP581_ODR_60_HZ    = 0x0E, //  60.000 Hz
    BMP581_ODR_50_HZ    = 0x0F, //  50.056 Hz
    BMP581_ODR_45_HZ    = 0x10, //  45.025 Hz
    BMP581_ODR_40_HZ    = 0x11, //  40.000 Hz
    BMP581_ODR_35_HZ    = 0x12, //  35.000 Hz
    BMP581_ODR_30_HZ    = 0x13, //  30.000 Hz
    BMP581_ODR_25_HZ    = 0x14, //  25.005 Hz
    BMP581_ODR_20_HZ    = 0x15, //  20.000 Hz
    BMP581_ODR_15_HZ    = 0x16, //  15.000 Hz
    BMP581_ODR_10_HZ    = 0x17, //  10.000 Hz
    BMP581_ODR_5_HZ     = 0x18, //   5.000 Hz
    BMP581_ODR_4_HZ     = 0x19, //   4.000 Hz
    BMP581_ODR_3_HZ     = 0x1A, //   3.000 Hz
    BMP581_ODR_2_HZ     = 0x1B, //   2.000 Hz
    BMP581_ODR_1_HZ     = 0x1C, //   1.000 Hz
    BMP581_ODR_0_5_HZ   = 0x1D, //   0.500 Hz
    BMP581_ODR_0_25_HZ  = 0x1E, //   0.250 Hz
    BMP581_ODR_0_125_HZ = 0x1F  //   0.125 Hz
} BMP581_odr_sel_t;

/**
 * @brief Deep standby mode control.
 */
typedef enum {
    BMP581_DEEP_ENABLED  = 0,  // Enable deep standby mode
    BMP581_DEEP_DISABLED = 1   // Disable deep standby mode
} BMP581_deep_standby_t;

/**
 * @brief Command register values.
 */
typedef enum {
    BMP581_CMD_NO_CMD      = 0x00, // Reserved – no command
    BMP581_CMD_NVM_WRITE_1 = 0x5D, // First CMD in NVM write sequence (0x5D → 0xA0)
    BMP581_CMD_EXT_MODE_1  = 0x69, // Part of extended mode enable sequence
    BMP581_CMD_EXT_MODE_2  = 0x73, // Part of extended mode enable sequence
    BMP581_CMD_NVM_WRITE_2 = 0xA0, // Last CMD in NVM write sequence
    BMP581_CMD_NVM_READ    = 0xA5, // Last CMD in NVM read sequence (0x5D → 0xA5)
    BMP581_CMD_EXT_MODE_3  = 0xB4, // Part of extended mode enable sequence
    BMP581_CMD_RESET       = 0xB6  // Soft reset – resets all registers to default
} BMP581_cmd_t;


// ─── Register bitfield structures ─────────────────────────────────────────────

/**
 * @brief ASIC Status Register (0x11).
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                // Complete 8-bit register value
        struct __attribute__((packed)) {
            uint8_t hif_mode  : 2;   // Bits 0-1: HIF mode (NVM-backed)
            uint8_t i3c_err_0 : 1;   // Bit 2: SDR parity error occurred
            uint8_t i3c_err_3 : 1;   // Bit 3: S0/S1 error occurred
            uint8_t reserved  : 4;   // Bits 4-7: Reserved
        } BitField;
    } Val;
} BMP581_asic_status_t;

/**
 * @brief Host Interface Configuration Register (0x13).
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                   // Complete 8-bit register value
        struct __attribute__((packed)) {
            uint8_t i2c_csb_pup_en : 1;  // Bit 0: CSB pull-up (I2C mode only)
            uint8_t spi3_en        : 1;  // Bit 1: SPI 3-wire mode enable
            uint8_t reserved_1     : 2;  // Bits 2-3: Reserved
            uint8_t pad_if_drv     : 1;  // Bit 4: Serial IO pad drive strength (I2C mode only)
            uint8_t reserved_2     : 3;  // Bits 5-7: Reserved
        } BitField;
    } Val;
} BMP581_host_if_config_t;

/**
 * @brief Interrupt Configuration Register (0x14).
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                // Complete 8-bit register value
        struct __attribute__((packed)) {
            uint8_t int_mode    : 1;  // Bit 0: INT mode (0 = pulsed, 1 = latched)
            uint8_t int_pol     : 1;  // Bit 1: INT polarity (0 = active low, 1 = active high)
            uint8_t int_od      : 1;  // Bit 2: INT pin type (0 = push-pull, 1 = open-drain)
            uint8_t int_en      : 1;  // Bit 3: Interrupt enable
            uint8_t pad_int_drv : 1;  // Bit 4: INT pad drive strength
            uint8_t reserved    : 3;  // Bits 5-7: Reserved
        } BitField;
    } Val;
} BMP581_int_config_t;

/**
 * @brief Interrupt Source Selection Register (0x15).
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                        // Complete 8-bit register value
        struct __attribute__((packed)) {
            uint8_t drdy_data_reg_en : 1;     // Bit 0: Data Ready interrupt enable
            uint8_t fifo_full_en     : 1;     // Bit 1: FIFO Full interrupt enable
            uint8_t fifo_ths_en      : 1;     // Bit 2: FIFO Threshold/Watermark interrupt enable
            uint8_t oor_p_en         : 1;     // Bit 3: Pressure Out-of-Range interrupt enable
            uint8_t reserved         : 4;     // Bits 4-7: Reserved
        } BitField;
    } Val;
} BMP581_int_source_t;

/**
 * @brief FIFO Configuration Register (0x16).
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                  // Complete 8-bit register value
        struct __attribute__((packed)) {
            uint8_t fifo_threshold : 5; // Bits 0-4: FIFO threshold value
            uint8_t fifo_mode      : 1; // Bit 5: FIFO mode (0 = stream, 1 = stop-on-full)
            uint8_t reserved       : 2; // Bits 6-7: Reserved
        } BitField;
    } Val;
} BMP581_fifo_config_t;

/**
 * @brief Number of Frames in FIFO Register (0x17).
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                  // Complete 8-bit register value
        struct __attribute__((packed)) {
            uint8_t fifo_count : 6;     // Bits 0-5: Number of frames currently in FIFO
            uint8_t reserved   : 2;     // Bits 6-7: Reserved
        } BitField;
    } Val;
} BMP581_fifo_count_t;

/**
 * @brief FIFO Frame Selection Register (0x18).
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                   // Complete 8-bit register value
        struct __attribute__((packed)) {
            uint8_t fifo_frame_sel : 2;  // Bits 0-1: FIFO frame data source selection
            uint8_t fifo_dec_sel   : 3;  // Bits 2-4: FIFO decimation selection
            uint8_t reserved       : 3;  // Bits 5-7: Reserved
        } BitField;
    } Val;
} BMP581_fifo_frame_sel_t;

/**
 * @brief Sensor data structure for temperature and pressure readings.
 * @note Temperature in °C, pressure in Pa. Populated by BMP581_Get_TempPressData().
 */
typedef struct __attribute__((packed)) {
    float pressure;     // Pressure in Pa
    float temperature;  // Temperature in degrees Celsius
} BMP581_sensor_data_t;

/**
 * @brief Interrupt Status Register (0x27).
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                  // Complete 8-bit register value
        struct __attribute__((packed)) {
            uint8_t drdy_data_reg : 1;  // Bit 0: Data Ready flag
            uint8_t fifo_full     : 1;  // Bit 1: FIFO Full flag
            uint8_t fifo_ths      : 1;  // Bit 2: FIFO Threshold/Watermark flag
            uint8_t oor_p         : 1;  // Bit 3: Pressure data out-of-range flag
            uint8_t por           : 1;  // Bit 4: POR or software reset complete flag
            uint8_t reserved      : 3;  // Bits 5-7: Reserved
        } BitField;
    } Val;
} BMP581_int_status_t;

/**
 * @brief Status Register (0x28).
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                          // Complete 8-bit register value
        struct __attribute__((packed)) {
            uint8_t status_core_rdy           : 1;  // Bit 0: Digital core domain accessible
            uint8_t status_nvm_rdy            : 1;  // Bit 1: Device ready for NVM operations
            uint8_t status_nvm_err            : 1;  // Bit 2: NVM error flag
            uint8_t status_nvm_cmd_err        : 1;  // Bit 3: Boot command error flag
            uint8_t status_boot_err_corrected : 1;  // Bit 4: ECC-corrected boot error flag
            uint8_t reserved                  : 2;  // Bits 5-6: Reserved
            uint8_t st_crack_pass             : 1;  // Bit 7: Crack check passed
        } BitField;
    } Val;
} BMP581_status_t;

/**
 * @brief NVM Address Register (0x2B).
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                    // Complete 8-bit register value
        struct __attribute__((packed)) {
            uint8_t nvm_row_address : 6;  // Bits 0-5: NVM row address
            uint8_t nvm_prog_en     : 1;  // Bit 6: NVM programming enable
            uint8_t reserved        : 1;  // Bit 7: Reserved
        } BitField;
    } Val;
} BMP581_nvm_addr_t;

/**
 * @brief DSP Configuration Register (0x30).
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                                              // Complete 8-bit register value
        struct __attribute__((packed)) {
            BMP581_comp_pt_en_t          comp_pt_en          : 2;  // Bits 0-1: Pressure/Temperature compensation enable
            BMP581_iir_flush_forced_en_t iir_flush_forced_en : 1;  // Bit 2: IIR flush for FORCED mode
            BMP581_iir_selection_t       shdw_sel_iir_t      : 1;  // Bit 3: Temperature Data Register IIR source
            BMP581_iir_selection_t       fifo_sel_iir_t      : 1;  // Bit 4: FIFO IIR source for temperature
            BMP581_iir_selection_t       shdw_sel_iir_p      : 1;  // Bit 5: Shadow Register IIR source for pressure
            BMP581_iir_selection_t       fifo_sel_iir_p      : 1;  // Bit 6: FIFO IIR source for pressure
            BMP581_oor_iir_sel_t         oor_sel_iir_p       : 1;  // Bit 7: OOR IIR source for pressure
        } BitField;
    } Val;
} BMP581_dsp_config_t;

/**
 * @brief DSP IIR Filter Configuration Register (0x31).
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                                    // Complete 8-bit register value
        struct __attribute__((packed)) {
            BMP581_iir_filter_coeff_t set_iir_p : 3;     // Bits 0-2: Pressure IIR LPF coefficient
            BMP581_iir_filter_coeff_t set_iir_t : 3;     // Bits 3-5: Temperature IIR LPF coefficient
            uint8_t                   reserved  : 2;     // Bits 6-7: Reserved (write 0)
        } BitField;
    } Val;
} BMP581_dsp_iir_config_t;

/**
 * @brief OOR Configuration Register (0x35).
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                             // Complete 8-bit register value
        struct __attribute__((packed)) {
            uint8_t              oor_thr_p_16 : 1; // Bit 0: OOR pressure threshold bit 16 (MSB extension)
            uint8_t              reserved     : 5; // Bits 1-5: Reserved
            BMP581_oor_cnt_limit_t cnt_lim    : 2; // Bits 6-7: OOR event count limit
        } BitField;
    } Val;
} BMP581_oor_config_t;

/**
 * @brief Oversampling Rate Configuration Register (0x36).
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                 // Complete 8-bit register value
        struct __attribute__((packed)) {
            BMP581_osr_t osr_t   : 3;  // Bits 0-2: Temperature oversampling rate
            BMP581_osr_t osr_p   : 3;  // Bits 3-5: Pressure oversampling rate
            uint8_t      press_en : 1; // Bit 6: Pressure measurement enable
            uint8_t      reserved : 1; // Bit 7: Reserved
        } BitField;
    } Val;
} BMP581_osr_config_t;

/**
 * @brief Output Data Rate Configuration Register (0x37).
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                              // Complete 8-bit register value
        struct __attribute__((packed)) {
            BMP581_pwr_mode_t     pwr_mode  : 2;   // Bits 0-1: Power mode
            BMP581_odr_sel_t      odr       : 5;   // Bits 2-6: ODR selection
            BMP581_deep_standby_t deep_dis  : 1;   // Bit 7: Deep standby disable
        } BitField;
    } Val;
} BMP581_odr_config_t;

/**
 * @brief Effective OSR Configuration Register (0x38, read-only).
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;                  // Complete 8-bit register value
        struct __attribute__((packed)) {
            uint8_t osr_t_eff    : 3;   // Bits 0-2: Effective OSR for temperature
            uint8_t osr_p_eff    : 3;   // Bits 3-5: Effective OSR for pressure
            uint8_t reserved     : 1;   // Bit 6: Reserved
            uint8_t odr_is_valid : 1;   // Bit 7: ODR validity flag
        } BitField;
    } Val;
} BMP581_osr_eff_config_t;

/**
 * @brief Complete register map structure for the BMP581.
 * @note  Fields are ordered to match the device register layout. Read-only or
 *        commented-out fields are included for reference but not mapped to I2C reads.
 */
typedef struct __attribute__((packed)) {
    //uint8_t CHIP_ID;                        // BMP581_REG_CHIP_ID     (0x01) — read via BMP581_Get_CHIP_ID()
    //uint8_t CHIP_REV;                       // BMP581_REG_REV_ID      (0x02) — read via BMP581_Get_CHIP_REV()
    BMP581_asic_status_t    CHIP_STATUS;      // BMP581_REG_CHIP_STATUS (0x11)
    BMP581_host_if_config_t DRIVE_CONFIG;     // BMP581_REG_DRIVE_CONFIG (0x13)
    BMP581_int_config_t     INT_CONFIG;       // BMP581_REG_INT_CONFIG  (0x14)
    BMP581_int_source_t     INT_SOURCE;       // BMP581_REG_INT_SOURCE  (0x15)
    BMP581_fifo_config_t    FIFO_CONFIG;      // BMP581_REG_FIFO_CONFIG (0x16)
    BMP581_fifo_count_t     FIFO_COUNT;       // BMP581_REG_FIFO_COUNT  (0x17)
    BMP581_fifo_frame_sel_t FIFO_SEL;         // BMP581_REG_FIFO_SEL    (0x18)
    BMP581_sensor_data_t    TEMP_PRESS;       // BMP581_REG_TEMP_DATA_XLSB–MSB (0x1D–0x1F) + PRESS_DATA_XLSB–MSB (0x20–0x22)
    BMP581_int_status_t     INT_STATUS;       // BMP581_REG_INT_STATUS  (0x27)
    BMP581_status_t         STATUS;           // BMP581_REG_STATUS      (0x28)
    uint8_t                 FIFO_DATA;        // BMP581_REG_FIFO_DATA   (0x29)
    BMP581_nvm_addr_t       NVM_ADDR;         // BMP581_REG_NVM_ADDR    (0x2B)
    uint16_t                NVM_DATA;         // BMP581_REG_NVM_DATA_LSB–MSB (0x2C–0x2D)
    BMP581_dsp_config_t     DSP_CONFIG;       // BMP581_REG_DSP_CONFIG  (0x30)
    BMP581_dsp_iir_config_t DSP_IIR_CONFIG;   // BMP581_REG_DSP_IIR     (0x31)
    uint16_t                OOR_PRESS_THR;    // BMP581_REG_OOR_THR_P   (0x32–0x33)
    uint8_t                 OOR_PRESS_RNG;    // BMP581_REG_OOR_RANGE   (0x34)
    BMP581_oor_config_t     OOR_CONFIG;       // BMP581_REG_OOR_CONFIG  (0x35)
    BMP581_osr_config_t     OSR_CONFIG;       // BMP581_REG_OSR_CONFIG  (0x36)
    BMP581_odr_config_t     ODR_CONFIG;       // BMP581_REG_ODR_CONFIG  (0x37)
    BMP581_osr_eff_config_t OSR_EFF;          // BMP581_REG_OSR_EFF     (0x38)
    //uint8_t cmd;                            // BMP581_REG_CMD         (0x7E) — write via BMP581_SendCommand()
} BMP581_registers_t;

// ─── Function prototypes ───────────────────────────────────────────────────────

/**
 * @brief Initialize the BMP581 sensor (soft reset, verify chip ID, apply default config).
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Init(void);

/**
 * @brief Send a command to the BMP581 command register (0x7E).
 * @param cmd Command value from BMP581_cmd_t.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_SendCommand(BMP581_cmd_t cmd);

/**
 * @brief Read and return the chip ID from register 0x01.
 * @return Chip ID byte (0x50 or 0x51 for BMP581).
 */
uint8_t BMP581_Get_CHIP_ID(void);

/**
 * @brief Read and return the revision ID from register 0x02.
 * @return Revision ID byte.
 */
uint8_t BMP581_Get_CHIP_REV(void);

/**
 * @brief Read the ASIC status register (0x11) into the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_CHIPStatus(void);

/**
 * @brief Write the drive configuration register (0x13) from the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_DriveConfig(void);

/**
 * @brief Read the drive configuration register (0x13) into the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_DriveConfig(void);

/**
 * @brief Write the interrupt configuration register (0x14) from the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_INTConfig(void);

/**
 * @brief Read the interrupt configuration register (0x14) into the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_INTConfig(void);

/**
 * @brief Write the interrupt source register (0x15) from the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_INTSource(void);

/**
 * @brief Read the interrupt source register (0x15) into the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_INTSource(void);

/**
 * @brief Write the FIFO configuration register (0x16) from the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_FIFOConfig(void);

/**
 * @brief Read the FIFO configuration register (0x16) into the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_FIFOConfig(void);

/**
 * @brief Read the FIFO frame count register (0x17) into the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_FIFOCount(void);

/**
 * @brief Read the FIFO frame selection register (0x18) into the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_FIFOFrameSel(void);

/**
 * @brief Write the FIFO frame selection register (0x18) from the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_FIFOFrameSel(void);

/**
 * @brief Read temperature and pressure data (0x1D–0x22) and convert to physical units.
 * @param data Pointer to BMP581_sensor_data_t to store results (pressure in Pa, temperature in °C).
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_TempPressData(BMP581_sensor_data_t *data);

/**
 * @brief Read the interrupt status register (0x27) into the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_INTStatus(void);

/**
 * @brief Read the sensor status register (0x28) into the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_Status(void);

/**
 * @brief Read one frame of FIFO data from register 0x29 into the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_FIFOData(void);

/**
 * @brief Write the NVM address register (0x2B) from the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_NVMAddr(void);

/**
 * @brief Read the NVM address register (0x2B) into the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_NVMAddr(void);

/**
 * @brief Write the NVM data registers (0x2C–0x2D) from the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_NVMData(void);

/**
 * @brief Read the NVM data registers (0x2C–0x2D) into the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_NVMData(void);

/**
 * @brief Write the DSP configuration register (0x30) from the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_DSPConfig(void);

/**
 * @brief Read the DSP configuration register (0x30) into the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_DSPConfig(void);

/**
 * @brief Write the DSP IIR configuration register (0x31) from the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_DSPIIRConfig(void);

/**
 * @brief Read the DSP IIR configuration register (0x31) into the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_DSPIIRConfig(void);

/**
 * @brief Write the pressure OOR threshold registers (0x32–0x33) from the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_OOR_PRESS_THR(void);

/**
 * @brief Read the pressure OOR threshold registers (0x32–0x33) into the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_OOR_PRESS_THR(void);

/**
 * @brief Write the OOR range register (0x34) from the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_OORRange(void);

/**
 * @brief Read the OOR range register (0x34) into the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_OORRange(void);

/**
 * @brief Write the OOR configuration register (0x35) from the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_OORConfig(void);

/**
 * @brief Read the OOR configuration register (0x35) into the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_OORConfig(void);

/**
 * @brief Write the OSR configuration register (0x36) from the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_OSRConfig(void);

/**
 * @brief Read the OSR configuration register (0x36) into the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_OSRConfig(void);

/**
 * @brief Write the ODR configuration register (0x37) from the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_ODRConfig(void);

/**
 * @brief Read the ODR configuration register (0x37) into the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_ODRConfig(void);

/**
 * @brief Read the effective OSR register (0x38) into the register map.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_OSREff(void);

#ifdef __cplusplus
}
#endif

#endif /* __BMP581_H */
