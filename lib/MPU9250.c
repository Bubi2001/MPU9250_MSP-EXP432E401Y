// ================================================================================
// 
// File Name           : MPU9250.c
// Project Name        : LSE2
// Target Devices      : MSP-EXP432E401Y
// Description         : 
//     Source code file for MPU9250 IMU sensor driver
//      defines constants, register maps, function prototypes, and hardware drivers
//      for interacting via I2C and a GPIO on TI-RTOS
// 
// Author              : Adrià Babiano Novella
// Last Modified       : May 2025
//
// ================================================================================
//
// * The GPL license allows free distribution and modification of the software, 
// * as long as the same license is maintained in derivative versions.
//
// ================================================================================

/**
 * @file MPU9250.c
 * @brief Implementation file for the MPU9250 IMU sensor driver.
 *
 * Contains functions for initializing, configuring, reading from, and writing to
 * the MPU9250 sensor using I2C on a TI-RTOS platform. Also includes data
 * conversion utilities.
 */

#include "MPU9250.h" // Include the header file for this module

/**
 * @brief Resets the MPU9250 by power cycling.
 * Includes console output during reset.
 */
void RSTIMU(void) {
    // Power OFF the IMU
    PWRIMU(false);

    // Print reset message to console
    System_printf("Resetting IMU...");
    System_flush();

    // Short delay might be needed here depending on hardware power decay time
    Task_sleep(100);

    // Power ON the IMU
    PWRIMU(true);
}

/**
 * @brief Controls the power supply to the MPU9250 via GPIO.
 * @param pwr Boolean indicating desired power state (true = ON, false = OFF). [in]
 */
void PWRIMU(bool pwr) {
    // Write the GPIO pin state: 1 for true (ON), 0 for false (OFF)
    GPIO_write(PWR_MPU9250, pwr ? 1 : 0);
}

/**
 * @brief Initializes the MPU9250 sensor.
 * @param i2c Handle to the I2C peripheral. [in]
 * @param i2cTransaction Pointer to an I2C transaction structure (used internally). [in, out]
 * @note Performs a device reset, checks WHO_AM_I, and configures various registers
 * (Gyro/Accel ranges, DLPF, FIFO, etc.). Halts system on WHO_AM_I mismatch or I2C error.
 */
void MPU9250_init(I2C_Handle i2c, I2C_Transaction *i2cTransaction, IMUData *IMU_Handle) {
    // Clean IMU data
    IMU_Handle->accelerometer.x = 0;
    IMU_Handle->accelerometer.y = 0;
    IMU_Handle->accelerometer.z = 0;
    IMU_Handle->gyroscope.x = 0;
    IMU_Handle->gyroscope.y = 0;
    IMU_Handle->gyroscope.z = 0;
    IMU_Handle->temperature = 0;

    // Perform a hardware reset (power cycle)
    RSTIMU();

    // Perform a software reset using PWR_MGMT_1 register and select clock source
    // 0x80 = H_RESET bit (reset device)
    // 0x01 = CLKSEL = 1 (Auto selects best clock source - PLL if ready, else internal 20MHz)
    MPU9250_writeReg(i2c, i2cTransaction, MPU9250_PWR_MGMT_1_ADDR, 0x81);

    // Delay needed after reset before registers are accessible reliably
    // ! TODO: Add appropriate delay (e.g., Task_sleep(100) for 100ms if in Task context)

    // Verify device identity by reading WHO_AM_I register
    uint8_t who_am_i_read;
    MPU9250_readReg(i2c, i2cTransaction, MPU9250_WHO_AM_I_ADDR, &who_am_i_read);

    // Check if the read value matches the expected MPU9250 ID
    if (who_am_i_read != MPU9250_WHO_AM_I) {
        System_printf("WHO AM I register read (0x%02X) does not match expected value (0x%02X)!\n", who_am_i_read, MPU9250_WHO_AM_I);
        System_flush();
        System_printf("Hanging system...\n");
        System_flush();
        while(1); // Halt execution
    } else {
        System_printf("WHO AM I check passed (0x%02X).\n", who_am_i_read);
        System_flush();
    }

    // Configure MPU9250 Registers
    // Refer to MPU9250 Register Map datasheet for details on these values

    // CONFIG Register (0x1A): DLPF_CFG = 3 (Accel BW: 41Hz, Gyro BW: 42Hz), FIFO_MODE=0 (overwrite)
    // MPU9250_writeReg(i2c, i2cTransaction, MPU9250_CONFIG_ADDR, 0x03); // Value was 0x43, maybe includes reserved bits? Using 0x03 for DLPF=3.
    MPU9250_writeReg(i2c, i2cTransaction, MPU9250_CONFIG_ADDR, 0x43);

    // GYRO_CONFIG Register (0x1B): FS_SEL = 1 (+/- 500 dps), FCHOICE_B = 0 (use DLPF)
    // Self Test bits = 0. Note: Value 0xEB provided in original code seems unusual.
    // Using 0x08 for +/- 500 dps as assumed by unit conversion.
    MPU9250_writeReg(i2c, i2cTransaction, MPU9250_GYRO_CONFIG_ADDR, 0x08); // Original: 0xEB

    // ACCEL_CONFIG Register (0x1C): ACCEL_FS_SEL = 1 (+/- 4g)
    // Self Test bits = 0. Note: Value 0xE8 provided in original code seems unusual.
    // Using 0x08 for +/- 4g as assumed by unit conversion.
    MPU9250_writeReg(i2c, i2cTransaction, MPU9250_ACCEL_CONFIG_ADDR, 0x08); // Original: 0xE8

    // ACCEL_CONFIG_2 Register (0x1D): ACCEL_FCHOICE_B = 0 (use DLPF), A_DLPFCFG = 3 (BW: 41Hz)
    MPU9250_writeReg(i2c, i2cTransaction, MPU9250_ACCEL_CONFIG_2_ADDR, 0x03);

    // FIFO_EN Register (0x23): Enable Temp, Gyro X/Y/Z, Accel X/Y/Z to be written to FIFO
    // 0xF8 = TEMP_EN | GYRO_X_EN | GYRO_Y_EN | GYRO_Z_EN | ACCEL_EN
    MPU9250_writeReg(i2c, i2cTransaction, MPU9250_FIFO_EN_ADDR, 0xF8); // Original: 0xF7 (included SLV0?)

    // INT_PIN_CFG Register (0x37): BYPASS_EN = 1 (Enable bypass mux to access magnetometer AK8963 if needed)
    MPU9250_writeReg(i2c, i2cTransaction, MPU9250_INT_PIN_CFG_ADDR, 0x02); // Original: 0x10 (FSYNC related?) Using 0x02 for simple bypass.

    // USER_CTRL Register (0x6A): FIFO_EN = 1 (Enable FIFO buffer), Reset FIFO, I2C Master disabled
    // 0x44 = FIFO_EN | FIFO_RESET
    MPU9250_writeReg(i2c, i2cTransaction, MPU9250_USER_CTRL_ADDR, 0x44); // Original: 0x77 (Included I2C_MST_RST, SIG_COND_RESET?)

    System_printf("MPU9250 Initialization complete.\n");
    System_flush();
}

/**
 * @brief Writes a single byte to a specified register on the MPU9250.
 * @param i2c Handle to the I2C peripheral. [in]
 * @param i2cTransaction Pointer to an I2C transaction structure. [in, out]
 * @param reg_addr The 8-bit address of the register to write to. [in]
 * @param dataTx The 8-bit data byte to write. [in]
 * @note Halts the system if an I2C NACK (Address or Data) occurs.
 */
void MPU9250_writeReg(I2C_Handle i2c, I2C_Transaction *i2cTransaction, uint8_t reg_addr, uint8_t dataTx) {
    // I2C write transaction: Send register address, then data byte
    uint8_t dataSet[2];
    dataSet[0] = reg_addr; // First byte: register address
    dataSet[1] = dataTx;   // Second byte: data to write

    // Configure the I2C transaction structure
    i2cTransaction->slaveAddress = MPU9250_I2C_ADDR; // Target device address
    i2cTransaction->writeBuf = dataSet;              // Pointer to the data buffer to write
    i2cTransaction->writeCount = 2;                  // Number of bytes to write
    i2cTransaction->readBuf = NULL;                  // No data to read
    i2cTransaction->readCount = 0;                   // Number of bytes to read

    // Perform the I2C transfer
    bool ret = I2C_transfer(i2c, i2cTransaction);

    // Check for I2C transfer errors
    if (!ret) {
        // Check specific error status
        if (i2cTransaction->status == I2C_STATUS_ADDR_NACK) {
            System_printf("I2C Write Error: Address Not Acknowledged (0x%02X)!\n", reg_addr);
            System_flush();
        } else if (i2cTransaction->status == I2C_STATUS_DATA_NACK) {
            System_printf("I2C Write Error: Data Not Acknowledged (Reg 0x%02X)!\n", reg_addr);
            System_flush();
        } else {
            System_printf("I2C Write Error: Unknown (Reg 0x%02X, Status %d)!\n", reg_addr, i2cTransaction->status);
            System_flush();
        }
        System_printf("Hanging system...\n");
        System_flush();
        while(1); // Halt execution
    }
}

/**
 * @brief Reads a single byte from a specified register on the MPU9250.
 * @param i2c Handle to the I2C peripheral. [in]
 * @param i2cTransaction Pointer to an I2C transaction structure. [in, out]
 * @param reg_addr The 8-bit address of the register to read from. [in]
 * @param dataRx Pointer to a byte where the read data will be stored. [out]
 * @note Halts the system if an I2C NACK (Address or Data) occurs.
 */
void MPU9250_readReg(I2C_Handle i2c, I2C_Transaction *i2cTransaction, uint8_t reg_addr, uint8_t *dataRx) {
    // I2C read transaction: First write the register address, then read the data

    // Configure the I2C transaction structure
    i2cTransaction->slaveAddress = MPU9250_I2C_ADDR; // Target device address
    i2cTransaction->writeBuf = &reg_addr;            // Pointer to the register address to write
    i2cTransaction->writeCount = 1;                  // Write one byte (the register address)
    i2cTransaction->readBuf = dataRx;                // Pointer to the buffer to store read data
    i2cTransaction->readCount = 1;                   // Read one byte of data

    // Perform the I2C transfer (write address phase + read data phase)
    bool ret = I2C_transfer(i2c, i2cTransaction);

    // Check for I2C transfer errors
    if (!ret) {
        if (i2cTransaction->status == I2C_STATUS_ADDR_NACK) {
            System_printf("I2C Read Error: Address Not Acknowledged (0x%02X)!\n", reg_addr);
            System_flush();
        } else if (i2cTransaction->status == I2C_STATUS_DATA_NACK) {
              // Note: DATA_NACK is unusual in a master read operation unless it's the address phase failing?
            System_printf("I2C Read Error: Data Not Acknowledged (Reg 0x%02X)!\n", reg_addr);
            System_flush();
        } else {
            System_printf("I2C Read Error: Unknown (Reg 0x%02X, Status %d)!\n", reg_addr, i2cTransaction->status);
            System_flush();
        }
        System_printf("Hanging system...\n");
        System_flush();
        while(1); // Halt execution
    }
}

/**
 * @brief Reads accelerometer, temperature, and gyroscope data in a single burst read.
 * @param i2c Handle to the I2C peripheral. [in]
 * @param i2cTransaction Pointer to an I2C transaction structure. [in, out]
 * @param dataRx Pointer to a buffer (at least 14 bytes long) to store the raw sensor data. [out]
 * Order: AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L, T_H, T_L, GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L
 * @note Reads 14 bytes starting from MPU9250_ACCEL_XOUT_H_ADDR.
 * @note Halts the system if an I2C NACK (Address or Data) occurs.
 */
void MPU9250_readAccTempGyr(I2C_Handle i2c, I2C_Transaction *i2cTransaction, uint8_t *dataRx) {
    // Set the starting register address for the burst read
    uint8_t startRegAddr = MPU9250_ACCEL_XOUT_H_ADDR;

    // Configure the I2C transaction for burst read
    i2cTransaction->slaveAddress = MPU9250_I2C_ADDR; // Target device address
    i2cTransaction->writeBuf = &startRegAddr;        // Pointer to the starting register address
    i2cTransaction->writeCount = 1;                  // Write one byte (the starting address)
    i2cTransaction->readBuf = dataRx;                // Pointer to the buffer to store read data
    i2cTransaction->readCount = 14;                  // Read 14 bytes (Accel(6) + Temp(2) + Gyro(6))

    // Perform the I2C transfer (write start address + burst read data)
    bool ret = I2C_transfer(i2c, i2cTransaction);

    // Check for I2C transfer errors
    if (!ret) {
        if (i2cTransaction->status == I2C_STATUS_ADDR_NACK) {
            System_printf("I2C Burst Read Error: Address Not Acknowledged (Start Reg 0x%02X)!\n", startRegAddr);
            System_flush();
        } else if (i2cTransaction->status == I2C_STATUS_DATA_NACK) {
            System_printf("I2C Burst Read Error: Data Not Acknowledged (Start Reg 0x%02X)!\n", startRegAddr);
            System_flush();
        } else {
            System_printf("I2C Burst Read Error: Unknown (Start Reg 0x%02X, Status %d)!\n", startRegAddr, i2cTransaction->status);
            System_flush();
        }
        System_printf("Hanging system...\n");
        System_flush();
        while(1); // Halt execution
    }
}

/**
 * @brief Converts raw sensor data (14 bytes) into engineering units.
 * @param raw Pointer to the 14-byte buffer containing raw sensor data read from the IMU. [in]
 * Order: AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L, T_H, T_L, GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L
 * @param IMU_Handle Pointer to the IMUData structure where the converted engineering unit
 * data (accelerometer [m/s^2], temperature [°C], gyroscope [rad/s]) will be stored. [out]
 * @param biasVector Structure containing the accelerometer bias vector. [in]
 * @return IMUData structure containing converted accelerometer (m/s^2),
 * temperature (°C), and gyroscope (rad/s) data.
 * @note See header file documentation for assumptions on sensitivity settings.
 * The function populates the IMUData structure pointed to by IMU_Handle.
 * The return type in the original comment seems to be a leftover, as the function is void
 * and uses the IMU_Handle pointer to return data.
 */
void MPU9250_unitConversion(uint8_t *raw, IMUData *IMU_Handle, Vector3D biasVector) {
    // --- Accelerometer Conversion ---
    // Combine high and low bytes into 16-bit signed integers (Raw LSB values)
    int16_t rawAccelX_LSB = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t rawAccelY_LSB = (int16_t)((raw[2]  << 8) | raw[3]);
    int16_t rawAccelZ_LSB = (int16_t)((raw[4]  << 8) | raw[5]);

    // Convert to float for calculations
    float accelRawFloat[3];
    accelRawFloat[0] = (float)rawAccelX_LSB;
    accelRawFloat[1] = (float)rawAccelY_LSB;
    accelRawFloat[2] = (float)rawAccelZ_LSB;

    // 1. Subtract Bias (LSB)
    float accelBiasCorrected[3];
    accelBiasCorrected[0] = accelRawFloat[0] - imuCalibration.biasVector.x;
    accelBiasCorrected[1] = accelRawFloat[1] - imuCalibration.biasVector.y;
    accelBiasCorrected[2] = accelRawFloat[2] - imuCalibration.biasVector.z;

    // 2. Apply Calibration Matrix M_cal: Accel_cal_mps2 = ACCEL_M_CAL * accelBiasCorrected
    IMU_Handle->accelerometer.x = imuCalibration.mCal.m1.x * accelBiasCorrected[0] +
                                  imuCalibration.mCal.m1.y * accelBiasCorrected[1] +
                                  imuCalibration.mCal.m1.z * accelBiasCorrected[2];

    IMU_Handle->accelerometer.y = imuCalibration.mCal.m2.x * accelBiasCorrected[0] +
                                  imuCalibration.mCal.m2.y * accelBiasCorrected[1] +
                                  imuCalibration.mCal.m2.z * accelBiasCorrected[2];

    IMU_Handle->accelerometer.z = imuCalibration.mCal.m3.x * accelBiasCorrected[0] +
                                  imuCalibration.mCal.m3.y * accelBiasCorrected[1] +
                                  imuCalibration.mCal.m3.z * accelBiasCorrected[2];

    // --- Temperature Conversion ---
    // Combine high and low bytes into 16-bit signed integer
    int16_t rawTemp = (int16_t)((raw[6]  << 8) | raw[7]);
    // Convert raw ADC value to physical units (°C)
    // Formula from datasheet/register map: Temp_degC = ((TEMP_OUT - RoomTemp_Offset) / Temp_Sensitivity) + 21 degC
    // The comment formula seems reversed, using standard approach:
    // Temp_degC = (RawTemp / Sensitivity) + Offset_degC (where offset might be 0 if centered or 21 as per comment)
    // Using the formula from the original comment: Sensitivity = 333.87 LSB/degC, Offset = 21 degC
    IMU_Handle->temperature = ((float)rawTemp / 333.87f) + 21.0f;

    // --- Gyroscope Conversion ---
    // Combine high and low bytes into 16-bit signed integers
    int16_t rawGyroX = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t rawGyroY = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t rawGyroZ = (int16_t)((raw[12] << 8) | raw[13]);
    // Convert raw ADC values to physical units (rad/s)
    // Assumes +/- 500 dps range (Sensitivity = 32768 / 500 = 65.536 LSB/dps ~ 65.5)
    // Convert dps to rad/s by dividing by RAD_TO_DEG (or multiplying by DEG_TO_RAD)
    float gyroSensitivity = 65.5f; // LSB/(deg/s)
    IMU_Handle->gyroscope.x = ((float)rawGyroX / gyroSensitivity) / RAD_TO_DEG; // Convert LSB to deg/s, then deg/s to rad/s
    IMU_Handle->gyroscope.y = ((float)rawGyroY / gyroSensitivity) / RAD_TO_DEG;
    IMU_Handle->gyroscope.z = ((float)rawGyroZ / gyroSensitivity) / RAD_TO_DEG;
}
