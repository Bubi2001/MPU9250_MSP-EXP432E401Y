// ================================================================================
// 
// File Name           : MPU9250.c
// Target Devices      : MSP-EXP432E401Y
// Description         : 
//     Source code file for MPU9250 IMU sensor driver
//      defines constants, register maps, function prototypes, and hardware drivers
//      for interacting via I2C and a GPIO on TI-RTOS
// 
// Author              : Adrià Babiano Novella
// Create Date         : 2024-12-01
// Revision            : v1.4
// Revision history:
//   v1.0 - Initial Version
//   v1.1 - Modified initialisation values
//   v1.2 - Added custom data types
//   v1.3 - Added comments Doxygen Style
//   v1.4 - Added calibration
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
    System_printf("Resetting IMU... \n");
    System_flush(); // Ensure message is printed immediately

    // Short delay might be needed here depending on hardware power decay time
    // Task_sleep(100); // Example: Sleep for 100ms (if using TI-RTOS Task context)

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
    Task_sleep(100);

    // Verify device identity by reading WHO_AM_I register
    uint8_t who_am_i_read = 0;
        int max_retries = 10; // Try up to 10 times
        while (max_retries > 0) {
            MPU9250_readReg(i2c, i2cTransaction, MPU9250_WHO_AM_I_ADDR, &who_am_i_read);
            if (who_am_i_read == MPU9250_WHO_AM_I) {
                break; // Success! Exit the loop.
            }
            max_retries--;
            Task_sleep(20); // Wait a little longer before retrying
        }
        // Check if we ever succeeded
        if (who_am_i_read != MPU9250_WHO_AM_I) {
            System_printf("WHO AM I register read (0x%x) does not match expected value (0x%x) after multiple retries!\n", who_am_i_read, MPU9250_WHO_AM_I);
            System_printf("Hanging system...\n");
            System_flush();
            while(1); // Halt execution
        } else {
            System_printf("WHO AM I check passed (0x%x).\n", who_am_i_read);
            System_flush();
        }

    // Configure MPU9250 Registers
    // Refer to MPU9250 Register Map datasheet for details on these values

    // CONFIG Register (0x1A): DLPF_CFG = 3 (Accel BW: 41Hz, Gyro BW: 42Hz), FIFO_MODE=0 (overwrite)
    MPU9250_writeReg(i2c, i2cTransaction, MPU9250_CONFIG_ADDR, 0x02);

    // GYRO_CONFIG Register (0x1B): FS_SEL = 1 (+/- 500 dps), FCHOICE_B = 0 (use DLPF)
    // Using 0x08 for +/- 500 dps.
    MPU9250_writeReg(i2c, i2cTransaction, MPU9250_GYRO_CONFIG_ADDR, 0x08);

    // ACCEL_CONFIG Register (0x1C): ACCEL_FS_SEL = 0 (+/- 2g)
    // Using 0x00 for +/- 2g.
    MPU9250_writeReg(i2c, i2cTransaction, MPU9250_ACCEL_CONFIG_ADDR, 0x00);

    // ACCEL_CONFIG_2 Register (0x1D): ACCEL_FCHOICE_B = 0 (use DLPF), A_DLPFCFG = 3 (BW: 41Hz)
    MPU9250_writeReg(i2c, i2cTransaction, MPU9250_ACCEL_CONFIG_2_ADDR, 0x03);

    // FIFO_EN Register (0x23): Enable Temp, Gyro X/Y/Z, Accel X/Y/Z to be written to FIFO
    // 0xF8 = TEMP_EN | GYRO_X_EN | GYRO_Y_EN | GYRO_Z_EN | ACCEL_EN
    MPU9250_writeReg(i2c, i2cTransaction, MPU9250_FIFO_EN_ADDR, 0xF8);

    // INT_PIN_CFG Register (0x37): BYPASS_EN = 1 (Enable bypass mux to access magnetometer AK8963 if needed)
    MPU9250_writeReg(i2c, i2cTransaction, MPU9250_INT_PIN_CFG_ADDR, 0x02);

    // USER_CTRL Register (0x6A): FIFO_EN = 1 (Enable FIFO buffer), Reset FIFO, I2C Master disabled
    // 0x44 = FIFO_EN | FIFO_RESET
    MPU9250_writeReg(i2c, i2cTransaction, MPU9250_USER_CTRL_ADDR, 0x44);

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
            System_printf("I2C Write Error: Address Not Acknowledged (0x%x)!\n", reg_addr);
            System_flush();
        } else if (i2cTransaction->status == I2C_STATUS_DATA_NACK) {
            System_printf("I2C Write Error: Data Not Acknowledged (Reg 0x%x)!\n", reg_addr);
            System_flush();
        } else {
            System_printf("I2C Write Error: Unknown (Reg 0x%x, Status %d)!\n", reg_addr, i2cTransaction->status);
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
 * @brief Converts raw sensor data (14 bytes) into engineering units with bias correction.
 *
 * This function takes the raw 16-bit ADC values from the sensor, subtracts the
 * pre-calculated bias offsets for both the accelerometer and gyroscope, and then
 * converts the corrected values into standard physical units (m/s^2 and rad/s).
 *
 * @param raw Pointer to the 14-byte buffer containing raw sensor data from the IMU. [in]
 * @param IMU_Handle Pointer to the IMUData structure where the converted data will be stored. [out]
 * @param accelBias Structure containing the accelerometer bias values (in LSB) to subtract. [in]
 * @param gyroBias Structure containing the gyroscope bias values (in LSB) to subtract. [in]
 */
void MPU9250_unitConversion(uint8_t *raw, IMUData *IMU_Handle, Vector3D accelBias, Vector3D gyroBias) {
    // --- Combine high and low bytes into 16-bit signed integers ---
    int16_t rawAccelX = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t rawAccelY = (int16_t)((raw[2]  << 8) | raw[3]);
    int16_t rawAccelZ = (int16_t)((raw[4]  << 8) | raw[5]);

    int16_t rawTemp = (int16_t)((raw[6]  << 8) | raw[7]);

    int16_t rawGyroX = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t rawGyroY = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t rawGyroZ = (int16_t)((raw[12] << 8) | raw[13]);

    // --- Accelerometer Conversion ---
    // 1. Subtract the pre-calculated bias (in LSB) from the raw reading.
    // 2. Convert the corrected LSB value to g's by dividing by the sensitivity.
    // 3. Convert g's to m/s^2 by multiplying by the gravity constant.
    // Assumes the accelerometer is configured for the +/- 2g range.
    const float ACCEL_SENSITIVITY_2G = 16384.0f; // LSB per g
    IMU_Handle->accelerometer.x = (((float)rawAccelX - accelBias.x) / ACCEL_SENSITIVITY_2G) * G_MPS2;
    IMU_Handle->accelerometer.y = (((float)rawAccelY - accelBias.y) / ACCEL_SENSITIVITY_2G) * G_MPS2;
    IMU_Handle->accelerometer.z = (((float)rawAccelZ - accelBias.z) / ACCEL_SENSITIVITY_2G) * G_MPS2;

    // --- Temperature Conversion ---
    // Formula from the datasheet/register map.
    IMU_Handle->temperature = ((float)rawTemp / 333.87f) + 21.0f;

    // --- Gyroscope Conversion ---
    // 1. Subtract the pre-calculated bias (in LSB) from the raw reading.
    // 2. Convert the corrected LSB value to degrees per second (dps) by dividing by the sensitivity.
    // 3. Convert dps to radians per second by multiplying by the conversion factor.
    // Assumes the gyroscope is configured for the +/- 500 dps range.
    const float GYRO_SENSITIVITY_500DPS = 65.5f; // LSB per dps
    IMU_Handle->gyroscope.x = (((float)rawGyroX - gyroBias.x) / GYRO_SENSITIVITY_500DPS) * DEG_TO_RAD;
    IMU_Handle->gyroscope.y = (((float)rawGyroY - gyroBias.y) / GYRO_SENSITIVITY_500DPS) * DEG_TO_RAD;
    IMU_Handle->gyroscope.z = (((float)rawGyroZ - gyroBias.z) / GYRO_SENSITIVITY_500DPS) * DEG_TO_RAD;
}

/**
 * @brief Calculates roll, pitch, and yaw angles from IMU data using a complementary filter.
 * @details
 * This function processes raw accelerometer and gyroscope data to compute the orientation
 * of the IMU in terms of Euler angles (roll, pitch, and yaw).
 * * It performs the following steps:
 * 1.  Applies a digital low-pass filter (DLPF) to the raw gyroscope readings to reduce noise.
 * 2.  Calculates the roll and pitch angles from the accelerometer data. These are accurate
 * for long-term orientation but noisy in the short term.
 * 3.  Integrates the filtered gyroscope data to get an estimate of the change in angle. This is
 * accurate for short-term changes but drifts over time.
 * 4.  A complementary filter is used to fuse the accelerometer angles (low-pass) and integrated
 * gyroscope angles (high-pass) for stable and accurate roll and pitch estimations.
 * 5.  The yaw angle is calculated by integrating the z-axis gyroscope data. Note that this
 * is not corrected by the accelerometer and will drift over time. A magnetometer would be
 * needed for absolute yaw correction.
 * @param[in] IMU_Handle Pointer to the IMUData struct containing the latest accelerometer and gyroscope readings.
 * @param[in,out] anglesDegrees Pointer to the EulerAngles struct to store the calculated angles in degrees. The previous values are used as part of the integration step.
 * @param[in] dt The time delta in seconds since the last function call (i.e., the sampling period).
 * @param[in] cf_alpha The complementary filter coefficient, typically close to 1 (e.g., 0.98). It balances the contribution between the gyroscope and accelerometer.
 * - `cf_alpha` -> 1.0: Trust gyroscope more (faster response, more drift).
 * - `cf_alpha` -> 0.0: Trust accelerometer more (slower response, less drift).
 * @param[in] dlpf_alpha The coefficient for the gyroscope's digital low-pass filter. A smaller value provides more smoothing but adds latency.
 */
void MPU9250_calculateAngle(IMUData *IMU_Handle, EulerAngles *anglesDegrees, float dt, float cf_alpha, float dlpf_alpha) {
    static Vector3D filteredGyro = {.x = 0.0f, .y = 0.0f, .z = 0.0f};

    // Apply a digital low-pass filter to the gyroscope data
    filteredGyro.x = (dlpf_alpha * IMU_Handle->gyroscope.x) + (1.0f - dlpf_alpha) * filteredGyro.x;
    filteredGyro.y = (dlpf_alpha * IMU_Handle->gyroscope.y) + (1.0f - dlpf_alpha) * filteredGyro.y;
    filteredGyro.z = (dlpf_alpha * IMU_Handle->gyroscope.z) + (1.0f - dlpf_alpha) * filteredGyro.z;

    // Calculate angles from the accelerometer data
    float rollAccel = atan2f(IMU_Handle->accelerometer.y, IMU_Handle->accelerometer.z) * RAD_TO_DEG;
    float pitchAccel = atan2f(-IMU_Handle->accelerometer.x, sqrtf(powf(IMU_Handle->accelerometer.y, 2) + powf(IMU_Handle->accelerometer.z, 2))) * RAD_TO_DEG;

    // Complementary filter to combine gyroscope and accelerometer data
    // Roll and Pitch are corrected by the accelerometer
    anglesDegrees->roll = cf_alpha * (anglesDegrees->roll + (filteredGyro.x * RAD_TO_DEG * dt)) + (1.0f - cf_alpha) * rollAccel;
    anglesDegrees->pitch = cf_alpha * (anglesDegrees->pitch + (filteredGyro.y * RAD_TO_DEG * dt)) + (1.0f - cf_alpha) * pitchAccel;
    
    // Yaw is not corrected by the accelerometer, so it's based on gyroscope integration only
    anglesDegrees->yaw = anglesDegrees->yaw + (filteredGyro.z * RAD_TO_DEG * dt);
}
