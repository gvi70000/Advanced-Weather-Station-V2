/**
 * @file BMP581.c
 * @brief Implementation file for the BMP581 barometric pressure and temperature sensor library.
 *
 * Contains functions for initializing, configuring, and reading data from the BMP581 sensor.
 * Target: STM32F302, Keil MDK, STM32 HAL.
 */

#include "BMP581.h"
#include "i2c.h"
#include "gpio.h"
#include "stm32g431xx.h"  // Device header for STM32F302 — pulls in core_cm4.h which provides __DSB()

// Global register map instance.
// 'volatile' is required because fields are written in main context and read in ISR context.
static volatile BMP581_registers_t bmp581_regs;

// Helper: reconstruct a 24-bit little-endian signed value (XLSB, LSB, MSB) into int32_t
/**
 * @brief Convert a 3-byte little-endian buffer to a sign-extended 32-bit integer.
 * @param data_ptr Pointer to 3-byte buffer [XLSB, LSB, MSB].
 * @return Sign-extended 32-bit signed integer.
 */
static inline int32_t get_24bit_signed(const uint8_t *data_ptr) {
    uint32_t val = ((uint32_t)data_ptr[2] << 16) | ((uint32_t)data_ptr[1] << 8) | (uint32_t)data_ptr[0];
    if (val & 0x800000) {
        val |= 0xFF000000;  // Sign-extend bit 23 to fill the upper 8 bits
    }
    return (int32_t)val;
}

/**
 * @brief Read all readable BMP581 registers into bmp581_regs.
 * @details Performs a sequential read of every readable register, populating
 *          the global bmp581_regs shadow. Use cases:
 *            - Post-reset snapshot: verify NVM-loaded reset defaults,
 *              enable read-modify-write safety for all subsequent config writes.
 *            - Post-init validation: confirm all written config values were
 *              accepted by the sensor.
 * @note Do not call before POR completion is confirmed (INT_STATUS.por == 1),
 *       as NVM trim values may not yet be stable in the registers.
 * @return HAL status. Returns HAL_ERROR immediately on first I2C failure.
 */
static HAL_StatusTypeDef BMP581_ReadAllRegisters(void) {
    if (BMP581_Get_INTStatus()     != HAL_OK) return HAL_ERROR;  // clears pending INT as side effect
    if (BMP581_Get_INTConfig()     != HAL_OK) return HAL_ERROR;
    if (BMP581_Get_INTSource()     != HAL_OK) return HAL_ERROR;
    if (BMP581_Get_FIFOConfig()    != HAL_OK) return HAL_ERROR;
    if (BMP581_Get_FIFOCount()     != HAL_OK) return HAL_ERROR;
    if (BMP581_Get_DSPConfig()     != HAL_OK) return HAL_ERROR;
    if (BMP581_Get_DSPIIRConfig()  != HAL_OK) return HAL_ERROR;
    if (BMP581_Get_OOR_PRESS_THR() != HAL_OK) return HAL_ERROR;
    if (BMP581_Get_OORRange()      != HAL_OK) return HAL_ERROR;
    if (BMP581_Get_OORConfig()     != HAL_OK) return HAL_ERROR;
    if (BMP581_Get_OSRConfig()     != HAL_OK) return HAL_ERROR;
    if (BMP581_Get_ODRConfig()     != HAL_OK) return HAL_ERROR;
    if (BMP581_Get_OSREff()        != HAL_OK) return HAL_ERROR;
    return HAL_OK;
}

// Initialization
/**
 * @brief BMP581_Init function implementation.
 * @details Performs a full sensor startup sequence per the Bosch BMP5 SensorAPI
 *          (bmp5_configure_interrupt() and bmp5_set_iir_config() reference implementation):
 *          1.  Soft reset via command register.
 *          2.  Verify chip ID (0x50 or 0x51).
 *          3.  Confirm NVM readiness via STATUS register.
 *          4.  Confirm POR completion via INT_STATUS register.
 *          5.  Configure OSR — 64x pressure, 64x temperature (equal accuracy for averaging).
 *          6.  Configure DSP compensation and IIR output routing.
 *              Sensor must be in STANDBY for IIR writes; post-reset state guarantees this.
 *          7.  Set IIR filter coefficients (coefficient 127 = register value 0x07).
 *          8.  Disable all INT sources (INT_SOURCE = 0x00) — mandatory before INT_CONFIG change.
 *          9.  Clear any pending interrupt by reading INT_STATUS.
 *          10. Configure interrupt pin (pulsed, active-high, push-pull).
 *          11. Enable data-ready interrupt source only.
 *          12. Start measurements: set ODR 4 Hz, Normal power mode — always last.
 *          13. Validate ODR/OSR combination via OSR_EFF register.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Init(void) {
    // Step 1: Soft reset
    if (BMP581_SendCommand(BMP581_CMD_RESET) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(2);  // 2 ms — datasheet minimum 2 ms; margin for NVM reload

    // Step 2: Verify chip ID
    uint8_t chip_id = BMP581_Get_CHIP_ID();
    if ((chip_id != BMP581_CHIP_ID_PRIM) && (chip_id != BMP581_CHIP_ID_SEC)) {
        return HAL_ERROR;  // Wrong device or I2C fault
    }

    // Step 3: Confirm NVM ready and error-free
    if (BMP581_Get_Status() != HAL_OK) {
        return HAL_ERROR;
    }
    if (bmp581_regs.STATUS.Val.BitField.status_nvm_rdy != 1 || bmp581_regs.STATUS.Val.BitField.status_nvm_err != 0 || bmp581_regs.STATUS.Val.BitField.status_nvm_cmd_err != 0) {
        return HAL_ERROR;  // NVM not ready or NVM error present
    }
		

		
    // Step 4: Confirm POR completion — INT_STATUS.por must be set after reset
    if (BMP581_Get_INTStatus() != HAL_OK) {
        return HAL_ERROR;
    }
    if (bmp581_regs.INT_STATUS.Val.BitField.por != 1) {
        return HAL_ERROR;  // POR flag not set — reset sequence did not complete
    }
		BMP581_ReadAllRegisters();
		
		bmp581_regs.ODR_CONFIG.Val.BitField.odr      = BMP581_ODR_4_HZ;
    bmp581_regs.ODR_CONFIG.Val.BitField.pwr_mode = BMP581_PWRMODE_STANDBY;
    bmp581_regs.ODR_CONFIG.Val.BitField.deep_dis = BMP581_DEEP_DISABLED;
    if (BMP581_Set_ODRConfig() != HAL_OK) {
        return HAL_ERROR;
    }
		HAL_Delay(3); // standby transition time per datasheet
		
		
    // Step 5: Configure OSR — 64x pressure and 64x temperature for equal accuracy averaging
    // t_meas = 0.5 + 64*0.2415 + 64*0.2415 = 31.4 ms → max ODR ~32 Hz; 4 Hz is well within budget
    bmp581_regs.OSR_CONFIG.Val.BitField.osr_t    = BMP581_OSR_1X;  // 64x temperature OSR
    bmp581_regs.OSR_CONFIG.Val.BitField.osr_p    = BMP581_OSR_1X;  // 64x pressure OSR
    bmp581_regs.OSR_CONFIG.Val.BitField.press_en = 1;               // Enable pressure measurement
    if (BMP581_Set_OSRConfig() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 6: Configure DSP compensation and IIR output routing.
    // comp_pt_en must be set before IIR coefficients take effect.
    // IIR registers are writable only in STANDBY mode (Bosch API note); post-reset state
    // guarantees STANDBY here. shdw_sel_iir routes filtered values to data registers.
    bmp581_regs.DSP_CONFIG.Val.BitField.comp_pt_en          = BMP581_COMP_PRESS_TEMP_BOTH;  // Full P and T compensation
    bmp581_regs.DSP_CONFIG.Val.BitField.iir_flush_forced_en = BMP581_IIR_FLUSH_ENABLED;     // Flush IIR on first sample for clean startup
    bmp581_regs.DSP_CONFIG.Val.BitField.shdw_sel_iir_t      = BMP581_IIR_AFTER_FILTER;      // Data registers show filtered temperature
    bmp581_regs.DSP_CONFIG.Val.BitField.shdw_sel_iir_p      = BMP581_IIR_AFTER_FILTER;      // Data registers show filtered pressure
    if (BMP581_Set_DSPConfig() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 7: Set IIR filter coefficients — 0x07 = coefficient 127 (maximum smoothing)
    bmp581_regs.DSP_IIR_CONFIG.Val.BitField.set_iir_t = BMP581_IIR_COEFF_127;  // Temperature IIR
    bmp581_regs.DSP_IIR_CONFIG.Val.BitField.set_iir_p = BMP581_IIR_COEFF_127;  // Pressure IIR
    if (BMP581_Set_DSPIIRConfig() != HAL_OK) {
        return HAL_ERROR;
    }

    // Steps 8-9: Mandatory Bosch API pre-sequence before writing INT_CONFIG.
    // Any change to int_mode requires INT_SOURCE = 0 first to avoid latching a stale interrupt.
    // Reading INT_STATUS clears any pending interrupt status left from the POR check above.

    // Step 8: Disable all interrupt sources
    bmp581_regs.INT_SOURCE.Val.BitField.drdy_data_reg_en = BMP581_DRDY_DISABLE;
    bmp581_regs.INT_SOURCE.Val.BitField.fifo_full_en     = BMP581_FIFO_FULL_DISABLE;
    bmp581_regs.INT_SOURCE.Val.BitField.fifo_ths_en      = BMP581_FIFO_THS_DISABLE;
    bmp581_regs.INT_SOURCE.Val.BitField.oor_p_en         = BMP581_OOR_P_DISABLE;
    if (BMP581_Set_INTSource() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 9: Read INT_STATUS to clear any pending interrupt
    if (BMP581_Get_INTStatus() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 10: Configure interrupt pin — pulsed, active-high, push-pull, enabled
    bmp581_regs.INT_CONFIG.Val.BitField.int_mode = BMP581_INT_MODE_PULSED;
    bmp581_regs.INT_CONFIG.Val.BitField.int_pol  = BMP581_INT_POL_ACTIVE_HIGH;
    bmp581_regs.INT_CONFIG.Val.BitField.int_od   = BMP581_INT_PIN_PUSH_PULL;
    bmp581_regs.INT_CONFIG.Val.BitField.int_en   = BMP581_INT_ENABLE;
    if (BMP581_Set_INTConfig() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 11: Enable data-ready interrupt source only
    bmp581_regs.INT_SOURCE.Val.BitField.drdy_data_reg_en = BMP581_DRDY_ENABLE;
    bmp581_regs.INT_SOURCE.Val.BitField.fifo_full_en     = BMP581_FIFO_FULL_DISABLE;
    bmp581_regs.INT_SOURCE.Val.BitField.fifo_ths_en      = BMP581_FIFO_THS_DISABLE;
    bmp581_regs.INT_SOURCE.Val.BitField.oor_p_en         = BMP581_OOR_P_DISABLE;
    if (BMP581_Set_INTSource() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 12: Start measurements — ODR and power mode set last per Bosch API examples.
    // 4 Hz is valid for 64x+64x OSR (t_meas 31 ms << 250 ms period).
    bmp581_regs.ODR_CONFIG.Val.BitField.pwr_mode = BMP581_PWRMODE_NONSTOP;
    if (BMP581_Set_ODRConfig() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 13: Validate ODR/OSR combination — sensor sets odr_is_valid = 1 if accepted.
    // __DSB() drains the Cortex-M4 write buffer before the I2C readback.
    __DSB();
    if (BMP581_Get_OSREff() != HAL_OK) {
        return HAL_ERROR;
    }
    if (bmp581_regs.OSR_EFF.Val.BitField.odr_is_valid != 1) {
        return HAL_ERROR;  // ODR/OSR combination rejected by sensor
    }
    BMP581_ReadAllRegisters();
    return HAL_OK;
}

// REG 0x7E - Command Register
/**
 * @brief BMP581_SendCommand function implementation.
 * @details Writes a single command byte to the BMP581 command register (0x7E).
 * @param cmd Command value from BMP581_cmd_t (e.g. BMP581_CMD_RESET).
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_SendCommand(BMP581_cmd_t cmd) {
    return HAL_I2C_Mem_Write(&hi2c3, BMP581_ADDRESS, BMP581_REG_CMD, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&cmd, 1, TIMEOUT_COMM);
}

// REG 0x01 - Chip ID
/**
 * @brief BMP581_Get_CHIP_ID function implementation.
 * @details Reads 1 byte from register 0x01. Returns 0xFF as an error sentinel on I2C failure
 *          (0xFF is not a valid BMP581 chip ID).
 * @return Chip ID byte (0x50 = primary, 0x51 = secondary), or 0xFF on I2C error.
 */
uint8_t BMP581_Get_CHIP_ID(void) {
    uint8_t id = 0;
    if (HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_CHIP_ID, I2C_MEMADD_SIZE_8BIT, &id, 1, TIMEOUT_COMM) == HAL_OK) {
        return id;
    }
    return 0xFF;  // Sentinel: not a valid chip ID
}

// REG 0x02 - Revision ID
/**
 * @brief BMP581_Get_CHIP_REV function implementation.
 * @details Reads 1 byte from register 0x02. Returns 0 on I2C failure.
 * @return Revision ID byte, or 0 on I2C error.
 */
uint8_t BMP581_Get_CHIP_REV(void) {
    uint8_t rev = 0;
    if (HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_REV_ID, I2C_MEMADD_SIZE_8BIT, &rev, 1, TIMEOUT_COMM) == HAL_OK) {
        return rev;
    }
    return 0;
}

// REG 0x11 - ASIC Status
/**
 * @brief BMP581_Get_CHIPStatus function implementation.
 * @details Reads 1 byte from register 0x11 into bmp581_regs.CHIP_STATUS.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_CHIPStatus(void) {
    return HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_CHIP_STATUS, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&bmp581_regs.CHIP_STATUS.Val, 1, TIMEOUT_COMM);
}

// REG 0x13 - Drive Configuration
/**
 * @brief BMP581_Set_DriveConfig function implementation.
 * @details Writes bmp581_regs.DRIVE_CONFIG to register 0x13.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_DriveConfig(void) {
    return HAL_I2C_Mem_Write(&hi2c3, BMP581_ADDRESS, BMP581_REG_DRIVE_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(void*)&bmp581_regs.DRIVE_CONFIG.Val, 1, TIMEOUT_COMM);
}

/**
 * @brief BMP581_Get_DriveConfig function implementation.
 * @details Reads 1 byte from register 0x13 into bmp581_regs.DRIVE_CONFIG.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_DriveConfig(void) {
    return HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_DRIVE_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&bmp581_regs.DRIVE_CONFIG.Val, 1, TIMEOUT_COMM);
}

// REG 0x14 - Interrupt Configuration
/**
 * @brief BMP581_Set_INTConfig function implementation.
 * @details Writes bmp581_regs.INT_CONFIG to register 0x14.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_INTConfig(void) {
    return HAL_I2C_Mem_Write(&hi2c3, BMP581_ADDRESS, BMP581_REG_INT_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(void*)&bmp581_regs.INT_CONFIG.Val, 1, TIMEOUT_COMM);
}

/**
 * @brief BMP581_Get_INTConfig function implementation.
 * @details Reads 1 byte from register 0x14 into bmp581_regs.INT_CONFIG.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_INTConfig(void) {
    return HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_INT_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&bmp581_regs.INT_CONFIG.Val, 1, TIMEOUT_COMM);
}

// REG 0x15 - Interrupt Source
/**
 * @brief BMP581_Set_INTSource function implementation.
 * @details Writes bmp581_regs.INT_SOURCE to register 0x15.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_INTSource(void) {
    return HAL_I2C_Mem_Write(&hi2c3, BMP581_ADDRESS, BMP581_REG_INT_SOURCE, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(void*)&bmp581_regs.INT_SOURCE.Val, 1, TIMEOUT_COMM);
}

/**
 * @brief BMP581_Get_INTSource function implementation.
 * @details Reads 1 byte from register 0x15 into bmp581_regs.INT_SOURCE.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_INTSource(void) {
    return HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_INT_SOURCE, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&bmp581_regs.INT_SOURCE.Val, 1, TIMEOUT_COMM);
}

// REG 0x16 - FIFO Configuration
/**
 * @brief BMP581_Set_FIFOConfig function implementation.
 * @details Writes bmp581_regs.FIFO_CONFIG to register 0x16.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_FIFOConfig(void) {
    return HAL_I2C_Mem_Write(&hi2c3, BMP581_ADDRESS, BMP581_REG_FIFO_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(void*)&bmp581_regs.FIFO_CONFIG.Val, 1, TIMEOUT_COMM);
}

/**
 * @brief BMP581_Get_FIFOConfig function implementation.
 * @details Reads 1 byte from register 0x16 into bmp581_regs.FIFO_CONFIG.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_FIFOConfig(void) {
    return HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_FIFO_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&bmp581_regs.FIFO_CONFIG.Val, 1, TIMEOUT_COMM);
}

// REG 0x17 - FIFO Frame Count
/**
 * @brief BMP581_Get_FIFOCount function implementation.
 * @details Reads 1 byte from register 0x17 into bmp581_regs.FIFO_COUNT.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_FIFOCount(void) {
    return HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_FIFO_COUNT, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&bmp581_regs.FIFO_COUNT.Val, 1, TIMEOUT_COMM);
}

// REG 0x18 - FIFO Frame Selection
/**
 * @brief BMP581_Set_FIFOFrameSel function implementation.
 * @details Writes bmp581_regs.FIFO_SEL to register 0x18.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_FIFOFrameSel(void) {
    return HAL_I2C_Mem_Write(&hi2c3, BMP581_ADDRESS, BMP581_REG_FIFO_SEL, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(void*)&bmp581_regs.FIFO_SEL.Val, 1, TIMEOUT_COMM);
}

/**
 * @brief BMP581_Get_FIFOFrameSel function implementation.
 * @details Reads 1 byte from register 0x18 into bmp581_regs.FIFO_SEL.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_FIFOFrameSel(void) {
    return HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_FIFO_SEL, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&bmp581_regs.FIFO_SEL.Val, 1, TIMEOUT_COMM);
}

// REG 0x1D-0x1F (Temperature) + 0x20-0x22 (Pressure)
/**
 * @brief BMP581_Get_TempPressData function implementation.
 * @details Reads 6 bytes starting at 0x1D (TEMP_XLSB through PRESS_MSB) in a single
 *          burst read, then converts both raw 24-bit little-endian values to float:
 *            Temperature (°C) = raw_T / 2^16
 *            Pressure    (Pa) = raw_P / 2^6
 * @param data Pointer to BMP581_sensor_data_t to receive the converted values.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_TempPressData(BMP581_sensor_data_t *data) {
    uint8_t reg_data[6];

    if (HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_TEMP_DATA_XLSB, I2C_MEMADD_SIZE_8BIT, reg_data, 6, TIMEOUT_COMM) != HAL_OK) {
        return HAL_ERROR;
    }

    int32_t t_raw = get_24bit_signed(&reg_data[0]);  // Bytes 0-2: TEMP_XLSB, LSB, MSB
    int32_t p_raw = get_24bit_signed(&reg_data[3]);  // Bytes 3-5: PRESS_XLSB, LSB, MSB

    data->temperature = (float)t_raw / TEMP_COEFF;   // °C
    data->pressure    = (float)p_raw / PRESS_COEFF;  // Pa

    return HAL_OK;
}

// REG 0x27 - Interrupt Status
/**
 * @brief BMP581_Get_INTStatus function implementation.
 * @details Reads 1 byte from register 0x27 into bmp581_regs.INT_STATUS.
 *          Reading this register clears latched interrupt flags.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_INTStatus(void) {
    return HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_INT_STATUS, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&bmp581_regs.INT_STATUS.Val, 1, TIMEOUT_COMM);
}

// REG 0x28 - Sensor Status
/**
 * @brief BMP581_Get_Status function implementation.
 * @details Reads 1 byte from register 0x28 into bmp581_regs.STATUS.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_Status(void) {
    return HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_STATUS, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&bmp581_regs.STATUS.Val, 1, TIMEOUT_COMM);
}

// REG 0x29 - FIFO Data
/**
 * @brief BMP581_Get_FIFOData function implementation.
 * @details Reads 1 byte from register 0x29 into bmp581_regs.FIFO_DATA.
 *          Call BMP581_Get_FIFOCount() first to determine how many frames are available.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_FIFOData(void) {
    return HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_FIFO_DATA, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&bmp581_regs.FIFO_DATA, 1, TIMEOUT_COMM);
}

// REG 0x2B - NVM Address
/**
 * @brief BMP581_Set_NVMAddr function implementation.
 * @details Writes bmp581_regs.NVM_ADDR to register 0x2B.
 *          Set nvm_prog_en bit to enable NVM programming; clear it for read operations.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_NVMAddr(void) {
    return HAL_I2C_Mem_Write(&hi2c3, BMP581_ADDRESS, BMP581_REG_NVM_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(void*)&bmp581_regs.NVM_ADDR.Val, 1, TIMEOUT_COMM);
}

/**
 * @brief BMP581_Get_NVMAddr function implementation.
 * @details Reads 1 byte from register 0x2B into bmp581_regs.NVM_ADDR.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_NVMAddr(void) {
    return HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_NVM_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&bmp581_regs.NVM_ADDR.Val, 1, TIMEOUT_COMM);
}

// REG 0x2C-0x2D - NVM Data
/**
 * @brief BMP581_Set_NVMData function implementation.
 * @details Writes bmp581_regs.NVM_DATA (2 bytes) to registers 0x2C-0x2D.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_NVMData(void) {
    return HAL_I2C_Mem_Write(&hi2c3, BMP581_ADDRESS, BMP581_REG_NVM_DATA_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(void*)&bmp581_regs.NVM_DATA, 2, TIMEOUT_COMM);
}

/**
 * @brief BMP581_Get_NVMData function implementation.
 * @details Reads 2 bytes from registers 0x2C-0x2D into bmp581_regs.NVM_DATA.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_NVMData(void) {
    return HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_NVM_DATA_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&bmp581_regs.NVM_DATA, 2, TIMEOUT_COMM);
}

// REG 0x30 - DSP Configuration
/**
 * @brief BMP581_Set_DSPConfig function implementation.
 * @details Writes bmp581_regs.DSP_CONFIG to register 0x30.
 *          Must be written before IIR coefficients (0x31) take effect.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_DSPConfig(void) {
    return HAL_I2C_Mem_Write(&hi2c3, BMP581_ADDRESS, BMP581_REG_DSP_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(void*)&bmp581_regs.DSP_CONFIG.Val, 1, TIMEOUT_COMM);
}

/**
 * @brief BMP581_Get_DSPConfig function implementation.
 * @details Reads 1 byte from register 0x30 into bmp581_regs.DSP_CONFIG.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_DSPConfig(void) {
    return HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_DSP_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&bmp581_regs.DSP_CONFIG.Val, 1, TIMEOUT_COMM);
}

// REG 0x31 - DSP IIR Filter Configuration
/**
 * @brief BMP581_Set_DSPIIRConfig function implementation.
 * @details Writes bmp581_regs.DSP_IIR_CONFIG to register 0x31.
 *          Configures the IIR low-pass filter coefficients for pressure and temperature.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_DSPIIRConfig(void) {
    return HAL_I2C_Mem_Write(&hi2c3, BMP581_ADDRESS, BMP581_REG_DSP_IIR, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(void*)&bmp581_regs.DSP_IIR_CONFIG.Val, 1, TIMEOUT_COMM);
}

/**
 * @brief BMP581_Get_DSPIIRConfig function implementation.
 * @details Reads 1 byte from register 0x31 into bmp581_regs.DSP_IIR_CONFIG.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_DSPIIRConfig(void) {
    return HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_DSP_IIR, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&bmp581_regs.DSP_IIR_CONFIG.Val, 1, TIMEOUT_COMM);
}

// REG 0x32-0x33 - OOR Pressure Threshold
/**
 * @brief BMP581_Set_OOR_PRESS_THR function implementation.
 * @details Writes bmp581_regs.OOR_PRESS_THR (2 bytes) to registers 0x32-0x33.
 *          The 17th bit of the threshold is stored separately in OOR_CONFIG (bit 0).
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_OOR_PRESS_THR(void) {
    return HAL_I2C_Mem_Write(&hi2c3, BMP581_ADDRESS, BMP581_REG_OOR_THR_P_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(void*)&bmp581_regs.OOR_PRESS_THR, 2, TIMEOUT_COMM);
}

/**
 * @brief BMP581_Get_OOR_PRESS_THR function implementation.
 * @details Reads 2 bytes from registers 0x32-0x33 into bmp581_regs.OOR_PRESS_THR.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_OOR_PRESS_THR(void) {
    return HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_OOR_THR_P_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&bmp581_regs.OOR_PRESS_THR, 2, TIMEOUT_COMM);
}

// REG 0x34 - OOR Pressure Range
/**
 * @brief BMP581_Set_OORRange function implementation.
 * @details Writes bmp581_regs.OOR_PRESS_RNG to register 0x34.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_OORRange(void) {
    return HAL_I2C_Mem_Write(&hi2c3, BMP581_ADDRESS, BMP581_REG_OOR_RANGE, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(void*)&bmp581_regs.OOR_PRESS_RNG, 1, TIMEOUT_COMM);
}

/**
 * @brief BMP581_Get_OORRange function implementation.
 * @details Reads 1 byte from register 0x34 into bmp581_regs.OOR_PRESS_RNG.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_OORRange(void) {
    return HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_OOR_RANGE, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&bmp581_regs.OOR_PRESS_RNG, 1, TIMEOUT_COMM);
}

// REG 0x35 - OOR Configuration
/**
 * @brief BMP581_Set_OORConfig function implementation.
 * @details Writes bmp581_regs.OOR_CONFIG to register 0x35.
 *          Includes the OOR threshold bit 16 extension and the OOR event count limit.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_OORConfig(void) {
    return HAL_I2C_Mem_Write(&hi2c3, BMP581_ADDRESS, BMP581_REG_OOR_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(void*)&bmp581_regs.OOR_CONFIG.Val, 1, TIMEOUT_COMM);
}

/**
 * @brief BMP581_Get_OORConfig function implementation.
 * @details Reads 1 byte from register 0x35 into bmp581_regs.OOR_CONFIG.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_OORConfig(void) {
    return HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_OOR_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&bmp581_regs.OOR_CONFIG.Val, 1, TIMEOUT_COMM);
}

// REG 0x36 - Oversampling Rate Configuration
/**
 * @brief BMP581_Set_OSRConfig function implementation.
 * @details Writes bmp581_regs.OSR_CONFIG to register 0x36.
 *          Controls OSR for temperature and pressure, and pressure enable.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_OSRConfig(void) {
    return HAL_I2C_Mem_Write(&hi2c3, BMP581_ADDRESS, BMP581_REG_OSR_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(void*)&bmp581_regs.OSR_CONFIG.Val, 1, TIMEOUT_COMM);
}

/**
 * @brief BMP581_Get_OSRConfig function implementation.
 * @details Reads 1 byte from register 0x36 into bmp581_regs.OSR_CONFIG.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_OSRConfig(void) {
    return HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_OSR_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&bmp581_regs.OSR_CONFIG.Val, 1, TIMEOUT_COMM);
}

// REG 0x37 - Output Data Rate Configuration
/**
 * @brief BMP581_Set_ODRConfig function implementation.
 * @details Writes bmp581_regs.ODR_CONFIG to register 0x37.
 *          Controls power mode, ODR selection, and deep standby disable.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_ODRConfig(void) {
    return HAL_I2C_Mem_Write(&hi2c3, BMP581_ADDRESS, BMP581_REG_ODR_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(void*)&bmp581_regs.ODR_CONFIG.Val, 1, TIMEOUT_COMM);
}

/**
 * @brief BMP581_Get_ODRConfig function implementation.
 * @details Reads 1 byte from register 0x37 into bmp581_regs.ODR_CONFIG.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_ODRConfig(void) {
    return HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_ODR_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&bmp581_regs.ODR_CONFIG.Val, 1, TIMEOUT_COMM);
}

// REG 0x38 - Effective OSR (read-only)
/**
 * @brief BMP581_Get_OSREff function implementation.
 * @details Reads 1 byte from register 0x38 into bmp581_regs.OSR_EFF.
 *          Check odr_is_valid after writing ODR/OSR config to confirm the combination
 *          is supported by the sensor. Call __DSB() before this read after config writes.
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_OSREff(void) {
    return HAL_I2C_Mem_Read(&hi2c3, BMP581_ADDRESS, BMP581_REG_OSR_EFF, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&bmp581_regs.OSR_EFF.Val, 1, TIMEOUT_COMM);
}
