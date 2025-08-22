# MPU9250_DMP C++ Class API Reference

## 1. Overview

This document provides the API reference for the `MPU9250_DMP` C++ class. This class serves as a high-level wrapper around the InvenSense Motion Driver 6.12, abstracting the underlying C functions into a clean, object-oriented interface.

The primary goal of this class is to simplify the integration of the MPU9250's Digital Motion Processor (DMP) into a C++ project, particularly within the TI-RTOS environment. It manages the driver's state, handles data retrieval and parsing, and provides convenient methods for data conversion and calculation.

## 2. Class Definition

### Public Data Members

The class exposes public member variables to store the latest sensor data retrieved from the DMP. After a successful call to `dmpUpdateFifo()`, these variables are populated with the new data.

* **Raw Sensor Data:**
  * `int ax, ay, az`: Raw accelerometer data.
  * `int gx, gy, gz`: Raw gyroscope data (calibrated if the feature is enabled).
  * `long qw, qx, qy, qz`: Raw quaternion data from the DMP in Q30 format.
* **Calculated Data:**
  * `float pitch, roll, yaw`: Calculated Euler angles in degrees. These are populated by calling `computeEulerAngles()`.
* **Metadata:**
  * `unsigned long time`: Timestamp (in milliseconds) of the last FIFO packet.

### Constructor

#### `MPU9250_DMP()`

The default constructor initializes the class.

```cpp
MPU9250_DMP();
```

## 3. Method Reference

### Initialization & Basic Control

#### `begin`

Initializes the MPU-9250 hardware and the underlying C-driver. It sets the default sensor configuration (Gyro and Accel enabled) and prepares the device for use.

```cpp
int begin();
```

* **Returns:** `INV_SUCCESS` (0) on success, or `INV_ERROR` (-1) on failure.

#### `selfTest`

Runs the built-in self-test routine to verify the functionality of the gyroscope, accelerometer, and magnetometer.

```cpp
int selfTest();
```

* **Returns:** A bitmask of test results. A value of `0x07` indicates that all three sensors passed.
  * **Bit 0: Gyro**
  * **Bit 1: Accel**
  * **Bit 2: Compass**

### DMP (Digital Motion Processor) Functions

#### `dmpBegin`

This is the primary function to initialize and start the DMP. It loads the firmware, sets the desired features, registers internal callbacks for gestures, and enables the DMP.

```cpp
int dmpBegin(unsigned short features, unsigned short fifoRate = 100);
```

* **Parameters:**
  * `features` [in]: A bitmask of features to enable (e.g., `DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL`).
  * `fifoRate` [in]: The desired rate (in Hz) for the DMP to generate data packets. The maximum is 200Hz.
* **Returns:** `INV_SUCCESS` on success.

#### `dmpUpdateFifo`

Reads a single packet from the DMP FIFO and updates the class's public data members (`ax`, `ay`, `az`, `gx`, `gy`, `gz`, `qw`, `qx`, `qy`, `qz`, `time`). This should be called whenever an MPU interrupt is detected.

```cpp
int dmpUpdateFifo();
```

* **Returns:** `INV_SUCCESS` if a packet was successfully read.

#### `dmpSetOrientation`

Sets the physical orientation of the sensor relative to the body frame. This is crucial for the DMP's sensor fusion algorithm.

```cpp
int dmpSetOrientation(const signed char *orientationMatrix);
```

* **Parameters:**
  * `orientationMatrix` [in]: A 9-element array representing a 3x3 rotation matrix.
* **Returns:** `INV_SUCCESS` on success.

### Gesture and Feature Functions

#### `dmpGetPedometerSteps`

Retrieves the current step count from the DMP's pedometer.

```cpp
unsigned long dmpGetPedometerSteps();
```

* **Returns:** The total number of steps counted.

#### `tapAvailable`

Checks if a tap gesture has been detected since the last time this function was called.

```cpp
bool tapAvailable();
```

* **Returns:** `true` if a new tap is available, `false` otherwise. This flag is automatically reset after being read.

#### `getTapDir` / `getTapCount`

Retrieves the direction and number of taps for the last detected tap gesture.

```cpp
unsigned char getTapDir();   // Returns direction (e.g., TAP_X_UP)
unsigned char getTapCount(); // Returns number of consecutive taps
```

### Data Calculation & Conversion

#### `computeEulerAngles`

Calculates the pitch, roll, and yaw from the quaternion data stored in the public members. The results are stored in the `pitch`, `roll`, and `yaw` public members.

```cpp
void computeEulerAngles(bool degrees = true);
```

* **Parameters:**
  * `degrees` [in]: If `true` (default), the results are in degrees. If `false`, they are in radians.

#### `calcAccel` / `calcGyro` / `calcQuat`

**Converts raw sensor or quaternion values from the public data members into standard engineering units.**

```cpp
float calcAccel(int axis_val); // Raw to g's
float calcGyro(int axis_val);  // Raw to degrees/sec
float calcQuat(long quat_val); // Raw Q30 to float
```

* **Parameters:**
  * `axis_val` / `quat_val` [in]: A raw sensor value (e.g., `ax`, `gy`, `qw`).
* **Returns:** The converted floating-point value.

## 4. Typical Usage Workflow (C++)

1. Instantiate the Class:

    ```cpp
    MPU9250_DMP myIMU;
    ```

2. Initialize the Hardware:

    ```cpp
    if (myIMU.begin() != INV_SUCCESS) {
        // Handle error
    }
    ```

3. Initialize and Start the DMP:

    ```cpp
    unsigned short dmpFeatures = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_TAP;
    if (myIMU.dmpBegin(dmpFeatures, 100) != INV_SUCCESS) { // 100 Hz rate
        // Handle error
    }
    ```

4. Main Loop (Interrupt-Driven):

    ```cpp
    while (1) {
        // Wait for MPU interrupt signal
        if (myIMU.dmpUpdateFifo() == INV_SUCCESS) {
            // New data is available in public members
            myIMU.computeEulerAngles();
            // Now use myIMU.roll, myIMU.pitch, myIMU.yaw, etc.
        }
        if (myIMU.tapAvailable()) {
            // A tap was detected!
            unsigned char count = myIMU.getTapCount();
            unsigned char dir = myIMU.getTapDir();
        }
    }
    ```
