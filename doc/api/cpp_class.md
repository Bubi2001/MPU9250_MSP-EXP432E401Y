# MPU6500/9250 DMP Driver API Reference

This document provides an API reference for the `MPU_DMP` driver, which interfaces with the InvenSense MPU6500/9250 sensors and their embedded Digital Motion Processor (DMP). The library is written in C++ and includes a C-compatible wrapper for use in C projects.

## C++ API (`MPU_DMP` Class)

This is the object-oriented interface for C++ applications.

### Public Data Members

Once `dmpUpdateFifo()` is called, the following public members are updated with the latest sensor data.

| **Member** | **Type** | **Description** |
| --- | --- | --- |
| `ax`,`ay`,`az` | `int` | Raw accelerometer data for X, Y, and Z axes. |
| `gx`,`gy`,`gz` | `int` | Raw gyroscope data for X, Y, and Z axes. |
| `qw`,`qx`,`qy`,`qz` | `long` | Raw quaternion data (W, X, Y, Z) in Q30 format from the DMP. |
| `temperature` | `long` | Raw temperature data from the sensor. |
| `time` | `unsigned long` | Timestamp for the last sensor data update (from DMP). |
| `pitch`,`roll`,`yaw` | `float` | Calculated Euler angles. Updated by calling `computeEulerAngles()`. |
| `mx`,`my`,`mz` | `int` | **(MPU9250 only)**Raw magnetometer data for X, Y, and Z axes. |
| `heading` | `float` | **(MPU9250 only)**Calculated compass heading. Updated by `computeCompassHeading()`. |

### Constructor

`MPU_DMP()`
:   Constructs a new `MPU_DMP` object.

### Basic Communication & Initialization

`int begin()`
:   Initializes the MPU sensor and the underlying driver.

* Returns: `INV_SUCCESS` (0) on success, `INV_ERROR` (-1) on failure.

`int selfTest()`
:   Runs the built-in self-test routine.

* Returns: A bitmask of test results. For MPU6500, a value of `0x03` means all sensors passed. For MPU9250, `0x07` means all passed.

### Sensor Configuration

`int setSensors(unsigned char sensors)`
:   Enables or disables specific sensors using a bitmask (e.g., `INV_XYZ_GYRO | INV_XYZ_ACCEL`).

`int setGyroFSR(unsigned short fsr)`
:   Sets the gyroscope's full-scale range (dps). Valid values: 250, 500, 1000, 2000.

`int setAccelFSR(unsigned char fsr)`
:   Sets the accelerometer's full-scale range (g's). Valid values: 2, 4, 8, 16.

`int setLPF(unsigned short lpf)`
:   Sets the digital low-pass filter cutoff frequency (Hz). Valid values: 5, 10, 20, 42, 98, 188.

`int setSampleRate(unsigned short rate)`
:   Sets the sensor sample rate in Hz.

`unsigned short getGyroFSR()`
:   Gets the current gyroscope full-scale range.

`unsigned char getAccelFSR()`
:   Gets the current accelerometer full-scale range.

`unsigned short getLPF()`
:   Gets the current low-pass filter frequency.

`unsigned short getSampleRate()`
:   Gets the current sample rate.

`float getGyroSens()`
:   Gets the gyroscope's sensitivity (conversion factor from raw value to dps).

`unsigned short getAccelSens()`
:   Gets the accelerometer's sensitivity (conversion factor from raw value to g's).

`int setCompassSampleRate(unsigned short rate)`
:   (MPU9250 only) Sets the magnetometer sample rate in Hz.

`unsigned short getCompassSampleRate()`
:   (MPU9250 only) Gets the magnetometer sample rate.

### DMP (Digital Motion Processor) Functions

`int dmpBegin(unsigned short features, unsigned short fifoRate = 100)`
:   Loads the DMP firmware and configures its features.

* features: A bitmask of DMP features to enable (e.g., DMP_FEATURE_6X_LP_QUAT).
* fifoRate: The desired rate in Hz for the DMP to update the FIFO buffer.
* Returns: INV_SUCCESS on success.

`int dmpUpdateFifo()`
:   Reads the latest data packet from the DMP FIFO and updates the public data members. This should be called frequently.

* Returns: INV_SUCCESS if a new packet was read.

`int dmpSetFifoRate(unsigned short rate)`
:   Sets the DMP FIFO update rate in Hz.

`unsigned short dmpGetFifoRate()`
:   Gets the current DMP FIFO update rate.

`unsigned long dmpGetPedometerSteps()`
:   Gets the total steps counted by the DMP's pedometer.

`int dmpSetPedometerSteps(unsigned long steps)`
:   Sets the pedometer's internal step counter.

`int dmpSetTap(unsigned short xThresh, unsigned short yThresh, unsigned short zThresh, unsigned char taps)`
:   Configures the DMP's tap detection feature.

`bool tapAvailable()`
:   Checks if a new tap has been detected.

`unsigned char getTapDir()`
:   Gets the direction and axis of the last detected tap.

`unsigned char getTapCount()`
:   Gets the count of the last detected tap event.

`int dmpSetOrientation(const signed char *orientationMatrix)`
:   Sets the DMP's orientation matrix.

`unsigned char dmpGetOrientation()`
:   Gets the orientation result from the DMP's Android-style orientation feature.

### Data Conversion & Calculation

`float calcAccel(int axis_val)`
:   Converts a raw accelerometer value to g's.

`float calcGyro(int axis_val)`
:   Converts a raw gyroscope value to degrees per second.

`float calcQuat(long quat_val)`
:   Converts a raw Q30 format quaternion value to a floating-point number.

`void computeEulerAngles(bool degrees = true)`
:   Computes pitch, roll, and yaw from the current quaternion data, updating the public members.

`float calcMag(int axis_val)`
:   (MPU9250 only) Converts a raw magnetometer value to microteslas (uT).

`float computeCompassHeading()`
:   (MPU9250 only) Computes the compass heading from raw magnetometer data.

## C-Compatible API (`MPU_DMP_C.h`)

This is a procedural wrapper around the C++ class for use in C projects.

### Handle Management

All C functions operate on an opaque pointer, `MPU_DMP_Handle`.

`MPU_DMP_Handle MPU_DMP_Create()`
:   Creates a new MPU_DMP instance and returns a handle to it.

`void MPU_DMP_Destroy(MPU_DMP_Handle handle)`
:   Destroys an MPU_DMP instance and frees its memory.

### Function Reference

The C API provides functions that directly correspond to the methods in the C++ class. The C++ object's methods are mapped to C functions by replacing `object.method(args)` with `MPU_DMP_method(handle, args)`.

**Example:**

* C++: `myMpu.begin()` becomes C: `MPU_DMP_begin(myMpuHandle)`
* C++: `myMpu.setGyroFSR(2000)` becomes C: `MPU_DMP_setGyroFSR(myMpuHandle, 2000)`

Data Access:

Since C cannot directly access the C++ object's public members, getter functions are provided for all public data members.

**Example:**

* To get the raw accelerometer X value: `int ax = MPU_DMP_get_ax(myMpuHandle);`
* To get the calculated pitch: `float pitch = MPU_DMP_get_pitch(myMpuHandle);`

Please refer to the `MPU_DMP_C.h` header for a complete list of all C functions, as they are a direct mapping of the C++ methods and data getters described above.
