# MPU9250_DMP C Wrapper API Reference

## 1. Overview

This document provides the API reference for the C wrapper for the `MPU9250_DMP` C++ class. This wrapper exposes the core functionality of the C++ object through a standard C interface, allowing it to be used in pure C projects.

It follows a common C pattern for object-oriented design by using an opaque handle (`MPU9250_DMP_Handle`) to manage the underlying C++ object's lifecycle and state. All operations are performed through functions that accept this handle as their first parameter.

## 2. Opaque Handle

### `MPU9250_DMP_Handle`

This is an opaque pointer to an internal structure that contains the `MPU9250_DMP` C++ object. The user should not attempt to dereference or manipulate this handle directly. It is created by `MPU9250_DMP_Create()` and must be passed to all other functions in this API.

```cpp
typedef struct MPU9250_DMP_t* MPU9250_DMP_Handle;
```

## 3. Lifecycle Functions

These functions are used to create and destroy an instance of the MPU9250 driver.

### `MPU9250_DMP_Create`

Allocates memory and creates an instance of the MPU9250_DMP driver. This must be the first function called.

```cpp
MPU9250_DMP_Handle MPU9250_DMP_Create();
```

* **Returns:** A valid `MPU9250_DMP_Handle` on success, or `NULL` on failure.

### `MPU9250_DMP_Destroy`

Frees the memory associated with a driver instance. This should be called when the driver is no longer needed to prevent memory leaks.

```cpp
void MPU9250_DMP_Destroy(MPU9250_DMP_Handle handle);
```

* **Parameters:**
  * `handle` [in]: The handle to the driver instance to be destroyed.

## 4. Core Functions

These are the primary functions for initializing the hardware and reading data from the DMP.

### `MPU9250_DMP_begin`

Initializes the MPU-9250 hardware and the underlying driver. This function must be called after creating the handle and before attempting to read data.

```cpp
int MPU9250_DMP_begin(MPU9250_DMP_Handle handle);
```

* **Parameters:**
  * `handle` [in]: A valid handle created by `MPU9250_DMP_Create()`.
* **Returns:** `0` on success, `-1` on failure.

### `MPU9250_DMP_dmpUpdateFifo`

Reads a single packet from the DMP FIFO and updates the internal state of the driver object. This should be called whenever an MPU interrupt is detected. After this function successfully returns, the getter functions can be used to retrieve the latest data.

```cpp
int MPU9250_DMP_dmpUpdateFifo(MPU9250_DMP_Handle handle);
```

* **Parameters:**
  * `handle` [in]: A valid handle.
* **Returns:** `0` if a packet was successfully read, `-1` on failure.

## 5. Data Getter Functions

These functions are used to retrieve the individual components of the quaternion data after a successful call to `MPU9250_DMP_dmpUpdateFifo()`.

### `MPU9250_DMP_get_qX`

```cpp
float MPU9250_DMP_get_qw(MPU9250_DMP_Handle handle);
float MPU9250_DMP_get_qx(MPU9250_DMP_Handle handle);
float MPU9250_DMP_get_qy(MPU9250_DMP_Handle handle);
float MPU9250_DMP_get_qz(MPU9250_DMP_Handle handle);
```

* **Parameters:**
  * `handle` [in]: A valid handle.
* **Returns:** The specified component of the quaternion as a floating-point number.

## 6. Typical Usage Workflow (C)

1. Create a driver instance:

    ```c
    MPU9250_DMP_Handle myIMU = MPU9250_DMP_Create();
    if (myIMU == NULL) {
        // Handle memory allocation error
    }
    ```

2. Initialize the hardware:

    ```c
    if (MPU9250_DMP_begin(myIMU) != 0) {
        // Handle initialization error
    }
    ```

3. Main Loop (Interrupt-Driven):

    ```c
    while (1) {
        // Wait for an MPU interrupt signal
        if (MPU9250_DMP_dmpUpdateFifo(myIMU) == 0) {
            // New data is available. Get it using the getter functions.
            float qw = MPU9250_DMP_get_qw(myIMU);
            float qx = MPU9250_DMP_get_qx(myIMU);
            float qy = MPU9250_DMP_get_qy(myIMU);
            float qz = MPU9250_DMP_get_qz(myIMU);
            // ... use the quaternion data ...
        }
    }
    ```

4. Clean up:

    ```c
    MPU9250_DMP_Destroy(myIMU);
    myIMU = NULL;
    ```
