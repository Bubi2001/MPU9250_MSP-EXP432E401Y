# MPU9250 IMU Sensor Driver API Reference

## 1. Overview

This document provides the API reference for the MPU9250 IMU sensor driver. The library is designed to run on an **MSP-EXP432E401Y** development board within the **TI-RTOS** environment. It facilitates communication with the MPU9250 via I2C, providing functions for initialization, configuration, data acquisition, and processing.

**Hardware Dependencies:**

* An I2C peripheral for communication.
* A GPIO pin to control the power supply to the MPU9250.

**Software Dependencies:**

* TI-RTOS Kernel (`Task`, `System`)
* TI Drivers (`I2C`, `GPIO`)

## 2. Data Structures

The library defines several structures to organize sensor data.

### `Vector3D`

Represents a 3-dimensional vector, used for accelerometer and gyroscope data.

```c
typedef struct Vector3D {
    float x;    // X-component
    float y;    // Y-component
    float z;    // Z-component
} Vector3D;
```

* **Members:**
  * `x`: `float` - The value on the X-axis.
  * `y`: `float` - The value on the Y-axis.
  * `z`: `float` - The value on the Z-axis.

### `EulerAngles`

Represents the orientation of the sensor in terms of roll, pitch, and yaw.

```c
typedef struct EulerAngles {
    float pitch;    // Rotation around the Y-axis
    float roll;     // Rotation around the X-axis
    float yaw;      // Rotation around the Z-axis
} EulerAngles;
```

* **Members:**
  * `pitch`: `float` - The pitch angle in degrees.
  * `roll`: `float` - The roll angle in degrees.
  * `yaw`: `float` - The yaw angle in degrees.

### `IMUData`

A container structure that holds all processed data from the IMU.

```c
typedef struct IMUData {
    Vector3D accelerometer; // Acceleration vector in m/s^2
    Vector3D gyroscope;     // Angular velocity vector in rad/s
    float temperature;      // Temperature reading in Celsius
} IMUData;
```

* **Members:**
  * `accelerometer`: `Vector3D` - Stores the acceleration data in m/s².
  * `gyroscope`: `Vector3D` - Stores the angular velocity data in rad/s.
  * `temperature`: `float` - Stores the sensor's internal temperature in degrees Celsius.

## 3. Constants and Macros

### Device and Communication

* `MPU9250_I2C_ADDR`: `0x68` - The I2C slave address of the MPU9250.
* `MPU9250_WHO_AM_I`: `0x70` - The expected value of the WHO_AM_I register (for MPU-9250, this can also be `0x71` or `0x73` depending on the version, but the code expects `0x70`).

### Physical Constants

* `G_MPS2`: `9.80665f` - Standard gravity in meters per second squared.
* `RAD_TO_DEG`: `57.2957795f` - Conversion factor from radians to degrees.
* `DEG_TO_RAD`: `1.0f / RAD_TO_DEG` - Conversion factor from degrees to radians.

### Register Map

The header file `MPU9250.h` contains a complete list of register address macros from `0x00` to `0x7E`. Key registers used in the driver include:

* `MPU9250_CONFIG_ADDR`: `0x1A`
* `MPU9250_GYRO_CONFIG_ADDR`: `0x1B`
* `MPU9250_ACCEL_CONFIG_ADDR`: `0x1C`
* `MPU9250_ACCEL_CONFIG_2_ADDR`: `0x1D`
* `MPU9250_ACCEL_XOUT_H_ADDR`: `0x3B` (Start of sensor data block)
* `MPU9250_PWR_MGMT_1_ADDR`: `0x6B`
* `MPU9250_WHO_AM_I_ADDR`: `0x75`

## 4. Function Reference

### Power and Reset Functions

#### `PWRIMU`

Controls the power supply to the MPU9250 via a GPIO pin.

```c
void PWRIMU(bool pwr);
```

* **Description:** Sets the state of the `PWR_MPU9250` GPIO pin. This function is intended to be connected to the VDD input of the sensor.
* **Parameters:**
  * `pwr` [in]: `bool` - The desired power state. `true` for ON, `false` for OFF.

#### `RSTIMU`

Performs a hardware reset of the MPU9250 by power cycling it.

```c
void RSTIMU(void);
```

* **Description:** This function calls `PWRIMU(false)` followed by `PWRIMU(true)` to cycle the power to the sensor, effectively resetting it. It also prints a reset message to the system console.

### Core Functions

#### `MPU9250_init`

Initializes the MPU9250 sensor with a predefined configuration.

```c
void MPU9250_init(I2C_Handle i2c, I2C_Transaction *i2cTransaction, IMUData *IMU_Handle);
```

* **Description:** This is the primary initialization function. It performs the following sequence:
  1. Resets the `IMU_Handle` data structure to zero.
  2. Performs a hardware reset using `RSTIMU()`.
  3. Performs a software reset and selects the best available clock source.
  4. Verifies the device identity by reading the `WHO_AM_I` register.
  5. Configures the gyroscope range to `±500 dps`.
  6. Configures the accelerometer range to `±2g`.
  7. Sets the Digital Low-Pass Filter (DLPF) for both accelerometer and gyroscope.
  8. Enables the FIFO for sensor data.
* **Parameters:**
  * `i2c` [in]: `I2C_Handle` - The handle to the configured I2C peripheral.
  * `i2cTransaction` [in, out]: `I2C_Transaction *` - A pointer to an I2C transaction structure used for all I2C communications.
  * `IMU_Handle` [out]: `IMUData *` - A pointer to the main IMU data structure to be cleared.
* **Note:** The system will halt execution (`while(1);`) if the `WHO_AM_I` check fails or if any I2C transfer results in an error.

### I2C Communication Functions

#### `MPU9250_writeReg`

Writes a single byte to a specified register.

```c
void MPU9250_writeReg(I2C_Handle i2c, I2C_Transaction *i2cTransaction, uint8_t reg_addr, uint8_t dataTx);
```

* **Parameters:**
  * `i2c` [in]: `I2C_Handle` - The handle to the I2C peripheral.
  * `i2cTransaction` [in, out]: `I2C_Transaction *` - Pointer to the I2C transaction structure.
  * `reg_addr` [in]: `uint8_t` - The address of the register to write to.
  * `dataTx` [in]: `uint8_t` - The data byte to write.
* **Note:** Halts the system on I2C transfer failure.

#### `MPU9250_readReg`

Reads a single byte from a specified register.

```c
void MPU9250_readReg(I2C_Handle i2c, I2C_Transaction *i2cTransaction, uint8_t reg_addr, uint8_t *dataRx);
```

* **Parameters:**
  * `i2c` [in]: `I2C_Handle` - The handle to the I2C peripheral.
  * `i2cTransaction` [in, out]: `I2C_Transaction *` - Pointer to the I2C transaction structure.
  * `reg_addr` [in]: `uint8_t` - The address of the register to read from.
  * `dataRx` [out]: `uint8_t *` - A pointer to a variable where the read data will be stored.
* **Note:** Halts the system on I2C transfer failure.

#### `MPU9250_readAccTempGyr`

Reads all sensor data (accelerometer, temperature, gyroscope) in a single burst read.

```c
void MPU9250_readAccTempGyr(I2C_Handle i2c, I2C_Transaction *i2cTransaction, uint8_t *dataRx);
```

* **Description:** Performs a 14-byte I2C burst read starting from `MPU9250_ACCEL_XOUT_H_ADDR`. This is the most efficient way to get a complete snapshot of the sensor data.
* **Parameters:**
  * `i2c` [in]: `I2C_Handle` - The handle to the I2C peripheral.
  * `i2cTransaction` [in, out]: `I2C_Transaction *` - Pointer to the I2C transaction structure.
  * `dataRx` [out]: `uint8_t *` - A pointer to a buffer of at least 14 bytes to store the raw sensor data.
* **Data Order:** The data is read in the following order: `AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L, T_H, T_L, GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L`.
* **Note:** Halts the system on I2C transfer failure.

### Data Processing Functions

#### `MPU9250_unitConversion`

Converts the raw 14-byte sensor data into engineering units (m/s², °C, rad/s) and applies bias correction.

```c
void MPU9250_unitConversion(uint8_t *raw, IMUData *IMU_Handle, Vector3D accelBias, Vector3D gyroBias);
```

* **Description:**
  1. Combines the high and low bytes for each sensor axis into 16-bit signed integers.
  2. Subtracts the provided bias values (in LSB) from the raw accelerometer and gyroscope readings.
  3. Converts the corrected accelerometer values to `m/s²`.
  4. Converts the raw temperature value to `ºC`.
  5. Converts the corrected gyroscope values to `rad/s`.
* **Parameters:**
  * `raw` [in]: `uint8_t *` - Pointer to the 14-byte buffer of raw sensor data.
  * `IMU_Handle` [out]: `IMUData *` - Pointer to the structure where the converted data will be stored.
  * `accelBias` [in]: `Vector3D` - A structure containing the accelerometer bias offsets (in LSB) for X, Y, and Z axes.
  * `gyroBias` [in]: `Vector3D` - A structure containing the gyroscope bias offsets (in LSB) for X, Y, and Z axes.

#### `MPU9250_calculateAngle`

Calculates roll, pitch, and yaw angles from IMU data using a complementary filter.

```c
void MPU9250_calculateAngle(IMUData *IMU_Handle, EulerAngles *anglesDegrees, float dt, float cf_alpha, float dlpf_alpha);
```

* **Description:** Fuses accelerometer and gyroscope data to provide a stable orientation estimate.
  1. Applies a digital low-pass filter to the gyroscope readings to reduce noise.
  2. Calculates "long-term" roll and pitch from the accelerometer.
  3. Integrates gyroscope data to get "short-term" changes in orientation.
  4. Combines these two estimates using a complementary filter for stable roll and pitch.
  5. Yaw is calculated by integrating the Z-axis gyroscope data only.
* **Parameters:**
  * `IMU_Handle` [in]: `IMUData *` - Pointer to the struct with the latest sensor readings.
  * `anglesDegrees` [in, out]: `EulerAngles *` - Pointer to the struct where the calculated angles (in degrees) will be stored. The previous values are used for integration.
  * `dt` [in]: `float` - The time delta in seconds since the last call (the sampling period).
  * `cf_alpha` [in]: `float` - The complementary filter coefficient (e.g., 0.98). A higher value trusts the gyroscope more.
  * `dlpf_alpha` [in]: `float` - The digital low-pass filter coefficient for the gyroscope. A smaller value provides more smoothing.
* **Note:** The yaw angle is not corrected by an absolute reference (like a magnetometer) and will drift over time.
