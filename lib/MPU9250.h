// ================================================================================
// 
// File Name           : MPU9250.h
// Project Name        : LSE2
// Target Devices      : MSP-EXP432E401Y
// Description         : 
//     Header file for MPU9250 IMU sensor driver
//      defines constants, register maps, function prototypes, and hardware drivers
//      for interacting via I2C and a GPIO on TI-RTOS
// 
// Author              : Adrià Babiano Novella
// Last Modifed        : May 2025
//
// ================================================================================
//
// * The GPL license allows free distribution and modification of the software, 
// * as long as the same license is maintained in derivative versions.
//
// ================================================================================

/**
 * @file MPU9250.h
 * @brief Header file for the MPU9250 IMU sensor driver.
 *
 * Defines constants, register maps, function prototypes, and includes necessary
 * for interacting with the MPU9250 sensor via I2C on a TI-RTOS platform.
 */

#ifndef MPU9250_H
#define MPU9250_H

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include <ti/drivers/I2C.h>
#include <ti/drivers/GPIO.h>

#include "ti_drivers_config.h"

#include <xdc/runtime/System.h>

// --- Constants ---
// --- Device Specific ---
#define MPU9250_I2C_ADDR            0x68
#define MPU9250_WHO_AM_I            0x71

// --- Register Addresses (MPU9250 Gyro/Accel) ---
#define MPU9250_SELF_TEST_X_GYRO_ADDR   0x00
#define MPU9250_SELF_TEST_Y_GYRO_ADDR   0x01
#define MPU9250_SELF_TEST_Z_GYRO_ADDR   0x02
#define MPU9250_SELF_TEST_X_ACCEL_ADDR  0x0D
#define MPU9250_SELF_TEST_Y_ACCEL_ADDR  0x0E
#define MPU9250_SELF_TEST_Z_ACCEL_ADDR  0x0F
#define MPU9250_XG_OFFSET_H_ADDR        0x13
#define MPU9250_XG_OFFSET_L_ADDR        0x14
#define MPU9250_YG_OFFSET_H_ADDR        0x15
#define MPU9250_YG_OFFSET_L_ADDR        0x16
#define MPU9250_ZG_OFFSET_H_ADDR        0x17
#define MPU9250_ZG_OFFSET_L_ADDR        0x18
#define MPU9250_SMPLRT_DIV_ADDR         0x19
#define MPU9250_CONFIG_ADDR             0x1A
#define MPU9250_GYRO_CONFIG_ADDR        0x1B
#define MPU9250_ACCEL_CONFIG_ADDR       0x1C
#define MPU9250_ACCEL_CONFIG_2_ADDR     0x1D
#define MPU9250_LP_ACCEL_ODR_ADDR       0x1E
#define MPU9250_WOM_THR_ADDR            0x1F
#define MPU9250_FIFO_EN_ADDR            0x23
#define MPU9250_I2C_MST_CTRL_ADDR       0x24
#define MPU9250_I2C_SLV0_ADDR_ADDR      0x25
#define MPU9250_I2C_SLV0_REG_ADDR       0x26
#define MPU9250_I2C_SLV0_CTRL_ADDR      0x27
#define MPU9250_I2C_SLV1_ADDR_ADDR      0x28
#define MPU9250_I2C_SLV1_REG_ADDR       0x29
#define MPU9250_I2C_SLV1_CTRL_ADDR      0x2A
#define MPU9250_I2C_SLV2_ADDR_ADDR      0x2B
#define MPU9250_I2C_SLV2_REG_ADDR       0x2C
#define MPU9250_I2C_SLV2_CTRL_ADDR      0x2D
#define MPU9250_I2C_SLV3_ADDR_ADDR      0x2E
#define MPU9250_I2C_SLV3_REG_ADDR       0x2F
#define MPU9250_I2C_SLV3_CTRL_ADDR      0x30
#define MPU9250_I2C_SLV4_ADDR_ADDR      0x31
#define MPU9250_I2C_SLV4_REG_ADDR       0x32
#define MPU9250_I2C_SLV4_DO_ADDR        0x33
#define MPU9250_I2C_SLV4_CTRL_ADDR      0x34
#define MPU9250_I2C_SLV4_DI_ADDR        0x35
#define MPU9250_I2C_MST_STATUS_ADDR     0x36
#define MPU9250_INT_PIN_CFG_ADDR        0x37
#define MPU9250_INT_ENABLE_ADDR         0x38
#define MPU9250_INT_STATUS_ADDR         0x3A
#define MPU9250_ACCEL_XOUT_H_ADDR       0x3B
#define MPU9250_ACCEL_XOUT_L_ADDR       0x3C
#define MPU9250_ACCEL_YOUT_H_ADDR       0x3D
#define MPU9250_ACCEL_YOUT_L_ADDR       0x3E
#define MPU9250_ACCEL_ZOUT_H_ADDR       0x3F
#define MPU9250_ACCEL_ZOUT_L_ADDR       0x40
#define MPU9250_TEMP_OUT_H_ADDR         0x41
#define MPU9250_TEMP_OUT_L_ADDR         0x42
#define MPU9250_GYRO_XOUT_H_ADDR        0x43
#define MPU9250_GYRO_XOUT_L_ADDR        0x44
#define MPU9250_GYRO_YOUT_H_ADDR        0x45
#define MPU9250_GYRO_YOUT_L_ADDR        0x46
#define MPU9250_GYRO_ZOUT_H_ADDR        0x47
#define MPU9250_GYRO_ZOUT_L_ADDR        0x48
#define MPU9250_EXT_SENS_DATA_00_ADDR   0x49
#define MPU9250_EXT_SENS_DATA_01_ADDR   0x4A
#define MPU9250_EXT_SENS_DATA_02_ADDR   0x4B
#define MPU9250_EXT_SENS_DATA_03_ADDR   0x4C
#define MPU9250_EXT_SENS_DATA_04_ADDR   0x4D
#define MPU9250_EXT_SENS_DATA_05_ADDR   0x4E
#define MPU9250_EXT_SENS_DATA_06_ADDR   0x4F
#define MPU9250_EXT_SENS_DATA_07_ADDR   0x50
#define MPU9250_EXT_SENS_DATA_08_ADDR   0x51
#define MPU9250_EXT_SENS_DATA_09_ADDR   0x52
#define MPU9250_EXT_SENS_DATA_10_ADDR   0x53
#define MPU9250_EXT_SENS_DATA_11_ADDR   0x54
#define MPU9250_EXT_SENS_DATA_12_ADDR   0x55
#define MPU9250_EXT_SENS_DATA_13_ADDR   0x56
#define MPU9250_EXT_SENS_DATA_14_ADDR   0x57
#define MPU9250_EXT_SENS_DATA_15_ADDR   0x58
#define MPU9250_EXT_SENS_DATA_16_ADDR   0x59
#define MPU9250_EXT_SENS_DATA_17_ADDR   0x5A
#define MPU9250_EXT_SENS_DATA_18_ADDR   0x5B
#define MPU9250_EXT_SENS_DATA_19_ADDR   0x5C
#define MPU9250_EXT_SENS_DATA_20_ADDR   0x5D
#define MPU9250_EXT_SENS_DATA_21_ADDR   0x5E
#define MPU9250_EXT_SENS_DATA_22_ADDR   0x5F
#define MPU9250_EXT_SENS_DATA_23_ADDR   0x60
#define MPU9250_I2C_SLV0_DO_ADDR        0x63
#define MPU9250_I2C_SLV1_DO_ADDR        0x64
#define MPU9250_I2C_SLV2_DO_ADDR        0x65
#define MPU9250_I2C_SLV3_DO_ADDR        0x66
#define MPU9250_I2C_MST_DELAY_CTRL_ADDR 0x67
#define MPU9250_SIGNAL_PATH_RESET_ADDR  0x68
#define MPU9250_MOT_DETECT_CTRL_ADDR    0x69
#define MPU9250_USER_CTRL_ADDR          0x6A
#define MPU9250_PWR_MGMT_1_ADDR         0x6B
#define MPU9250_PWR_MGMT_2_ADDR         0x6C
#define MPU9250_FIFO_COUNTH_ADDR        0x72
#define MPU9250_FIFO_COUNTL_ADDR        0x73
#define MPU9250_FIFO_R_W_ADDR           0x74
#define MPU9250_WHO_AM_I_ADDR           0x75
#define MPU9250_XA_OFFSET_H_ADDR        0x77
#define MPU9250_XA_OFFSET_L_ADDR        0x78
#define MPU9250_YA_OFFSET_H_ADDR        0x7A
#define MPU9250_YA_OFFSET_L_ADDR        0x7B
#define MPU9250_ZA_OFFSET_H_ADDR        0x7D
#define MPU9250_ZA_OFFSET_L_ADDR        0x7E

// --- Physical Constants ---
#define G_MPS2          9.80665f
#define RAD_TO_DEG      57.2957795f

/**
 * @brief Structure to represent a 3-dimensional vector.
 */
typedef struct Vector3D {
    float x;    /**< X-component */
    float y;    /**< Y-component */
    float z;    /**< Z-component */
} Vector3D;

/**
 * @brief Structure to hold data from an Inertial Measurement Unit (IMU).
 */
typedef struct IMUData {
    Vector3D accelerometer; /**< Acceleration vector in m/s^2 */
    Vector3D gyroscope;     /**< Angular velocity vector in rad/s */
    float temperature;      /**< Temperature reading in Celsius */
} IMUData;

// --- Function Prototypes ---
/**
 * @brief Resets the MPU9250 by power cycling.
 * @note Uses PWRIMU function and assumes PWR_MPU9250 GPIO is configured.
 * Includes console output during reset.
 */
void RSTIMU(void);

/**
 * @brief Controls the power supply to the MPU9250 via GPIO.
 * @param pwr Boolean indicating desired power state (true = ON, false = OFF). [in]
 * @note Assumes PWR_MPU9250 is defined and configured as a GPIO output.
 */
void PWRIMU(bool pwr);

/**
 * @brief Initializes the MPU9250 sensor.
 * @param i2c Handle to the I2C peripheral. [in]
 * @param i2cTransaction Pointer to an I2C transaction structure (used internally). [in, out]
 * @note Performs a device reset, checks WHO_AM_I, and configures various registers
 * (Gyro/Accel ranges, DLPF, FIFO, etc.). Halts system on WHO_AM_I mismatch or I2C error.
 * Assumes specific configuration values are desired (see implementation).
 */
void MPU9250_init(I2C_Handle i2c, I2C_Transaction *i2cTransaction, IMUData *IMU_Handle);

/**
 * @brief Writes a single byte to a specified register on the MPU9250.
 * @param i2c Handle to the I2C peripheral. [in]
 * @param i2cTransaction Pointer to an I2C transaction structure. [in, out]
 * @param reg_addr The 8-bit address of the register to write to. [in]
 * @param dataTx The 8-bit data byte to write. [in]
 * @note Halts the system if an I2C NACK (Address or Data) occurs.
 */
void MPU9250_writeReg(I2C_Handle i2c, I2C_Transaction *i2cTransaction, uint8_t reg_addr, uint8_t dataTx);

/**
 * @brief Reads a single byte from a specified register on the MPU9250.
 * @param i2c Handle to the I2C peripheral. [in]
 * @param i2cTransaction Pointer to an I2C transaction structure. [in, out]
 * @param reg_addr The 8-bit address of the register to read from. [in]
 * @param dataRx Pointer to a byte where the read data will be stored. [out]
 * @note Halts the system if an I2C NACK (Address or Data) occurs.
 */
void MPU9250_readReg(I2C_Handle i2c, I2C_Transaction *i2cTransaction, uint8_t reg_addr, uint8_t *dataRx);

/**
 * @brief Reads accelerometer, temperature, and gyroscope data in a single burst read.
 * @param i2c Handle to the I2C peripheral. [in]
 * @param i2cTransaction Pointer to an I2C transaction structure. [in, out]
 * @param dataRx Pointer to a buffer (at least 14 bytes long) to store the raw sensor data. [out]
 * Order: AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L, T_H, T_L, GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L
 * @note Reads 14 bytes starting from MPU9250_ACCEL_XOUT_H_ADDR.
 * @note Halts the system if an I2C NACK (Address or Data) occurs.
 */
void MPU9250_readAccTempGyr(I2C_Handle i2c, I2C_Transaction *i2cTransaction, uint8_t *dataRx);

/**
 * @brief Converts raw sensor data (14 bytes) into engineering units.
 * @param raw Pointer to the 14-byte buffer containing raw sensor data read from the IMU. [in]
 * Order: AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L, T_H, T_L, GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L
 * @param IMU_Handle Pointer to the IMUData structure where the converted engineering unit
 * data (accelerometer [m/s^2], temperature [°C], gyroscope [rad/s]) will be stored. [out]
 * @param imuCalibration Structure containing the accelerometer bias vector and calibration matrix. [in]
 * @return IMUData structure containing converted accelerometer (m/s^2),
 * temperature (°C), and gyroscope (rad/s) data.
 * @note See header file documentation for assumptions on sensitivity settings.
 * The function populates the IMUData structure pointed to by IMU_Handle.
 * The return type in the original comment seems to be a leftover, as the function is void
 * and uses the IMU_Handle pointer to return data.
 */
void MPU9250_unitConversion(uint8_t *raw, IMUData *IMU_Handle, CalibrationData imuCalibration);

#endif // MPU9250_H
