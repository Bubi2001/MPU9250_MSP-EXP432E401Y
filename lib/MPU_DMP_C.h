#ifndef MPU_DMP_C_H
#define MPU_DMP_C_H

#include <stdbool.h> 

#ifdef __cplusplus
extern "C" {
#endif

// Return error codes
#define INV_SUCCESS 0
#define INV_ERROR  -1

/**
 * @file MPU_DMP_C.h
 * @brief A C-style wrapper for the MPU_DMP C++ library.
 *
 * This API provides C functions to access the functionality of the MPU_DMP C++ class.
 * It uses an opaque pointer (MPU_DMP_Handle) to manage the underlying C++ object.
 */

/**
 * @brief An opaque handle representing an instance of the MPU_DMP C++ object.
 *
 * This handle is created by MPU_DMP_Create() and must be passed to all other
 * functions in this API. It should be destroyed using MPU_DMP_Destroy() when
 * no longer needed to release resources.
 */
typedef struct MPU_DMP_t* MPU_DMP_Handle;

// --- Function Declarations ---

// Lifecycle
/**
 * @brief   Creates a new MPU_DMP instance.
 * @return  A handle to the new instance, or NULL on failure.
 */
MPU_DMP_Handle MPU_DMP_Create();

/**
 * @brief   Destroys an MPU_DMP instance and frees its memory.
 * @param   handle The handle to the instance to destroy.
 */
void MPU_DMP_Destroy(MPU_DMP_Handle handle);

// Basic Configuration & Status
/**
 * @brief   Initializes the MPU sensor.
 * @param   handle A valid MPU_DMP_Handle.
 * @return  0 on success, -1 on failure.
 */
int MPU_DMP_begin(MPU_DMP_Handle handle);

/**
 * @brief   Runs the sensor's built-in self-test.
 * @param   handle A valid MPU_DMP_Handle.
 * @return  A bitmask of test results (see MPU_DMP::selfTest for details).
 */
int MPU_DMP_selfTest(MPU_DMP_Handle handle);

/**
 * @brief   Sets which sensors are active.
 * @param   handle A valid MPU_DMP_Handle.
 * @param   sensors A bitmask of sensors to enable.
 * @return  0 on success.
 */
int MPU_DMP_setSensors(MPU_DMP_Handle handle, unsigned char sensors);

/**
 * @brief   Sets the gyroscope's full-scale range (FSR).
 * @param   handle A valid MPU_DMP_Handle.
 * @param   fsr The desired FSR in degrees per second (e.g., 2000).
 * @return  0 on success.
 */
int MPU_DMP_setGyroFSR(MPU_DMP_Handle handle, unsigned short fsr);

/**
 * @brief   Sets the accelerometer's full-scale range (FSR).
 * @param   handle A valid MPU_DMP_Handle.
 * @param   fsr The desired FSR in g's (e.g., 2, 4, 8, 16).
 * @return  0 on success.
 */
int MPU_DMP_setAccelFSR(MPU_DMP_Handle handle, unsigned short fsr);

/**
 * @brief   Sets the digital low-pass filter (DLPF) frequency.
 * @param   handle A valid MPU_DMP_Handle.
 * @param   lpf The cutoff frequency in Hz.
 * @return  0 on success.
 */
int MPU_DMP_setLPF(MPU_DMP_Handle handle, unsigned short lpf);

/**
 * @brief   Sets the sensor sample rate.
 * @param   handle A valid MPU_DMP_Handle.
 * @param   rate The sample rate in Hz.
 * @return  0 on success.
 */
int MPU_DMP_setSampleRate(MPU_DMP_Handle handle, unsigned short rate);

/** @brief  Gets the current gyroscope FSR. @param handle A valid MPU_DMP_Handle. */
unsigned short MPU_DMP_getGyroFSR(MPU_DMP_Handle handle);
/** @brief  Gets the current accelerometer FSR. @param handle A valid MPU_DMP_Handle. */
unsigned char MPU_DMP_getAccelFSR(MPU_DMP_Handle handle);
/** @brief  Gets the current low-pass filter frequency. @param handle A valid MPU_DMP_Handle. */
unsigned short MPU_DMP_getLPF(MPU_DMP_Handle handle);
/** @brief  Gets the current sample rate. @param handle A valid MPU_DMP_Handle. */
unsigned short MPU_DMP_getSampleRate(MPU_DMP_Handle handle);
/** @brief  Gets the gyroscope sensitivity. @param handle A valid MPU_DMP_Handle. */
float MPU_DMP_getGyroSens(MPU_DMP_Handle handle);
/** @brief  Gets the accelerometer sensitivity. @param handle A valid MPU_DMP_Handle. */
unsigned short MPU_DMP_getAceelSens(MPU_DMP_Handle handle);

#ifdef USE_MPU9250
/** @brief Sets the magnetometer sample rate. @param handle A valid MPU_DMP_Handle. @param rate The rate in Hz. */
int MPU_DMP_setCompassSampleRate(MPU_DMP_Handle handle, unsigned short rate);
/** @brief Gets the magnetometer sample rate. @param handle A valid MPU_DMP_Handle. */
unsigned short MPU_DMP_getCompassSampleRate(MPU_DMP_Handle handle);
#endif

// Interrupt Configuration
/** @brief  Enables the MPU's data-ready interrupt. @param handle A valid MPU_DMP_Handle. */
int MPU_DMP_enableInterrupt(MPU_DMP_Handle handle);
/** @brief  Sets the interrupt pin's logic level. @param handle A valid MPU_DMP_Handle. @param active_low 1 for active low, 0 for active high. */
int MPU_DMP_setIntLevel(MPU_DMP_Handle handle, unsigned char active_low);
/** @brief  Configures the interrupt pin as latched or 50us pulse. @param handle A valid MPU_DMP_Handle. @param enable 1 for latched, 0 for 50us pulse. */
int MPU_DMP_setIntLatched(MPU_DMP_Handle handle, unsigned char enable);

// DMP (Digital Motion Processor) specific functions
/**
 * @brief   Initializes the DMP.
 * @param   handle A valid MPU_DMP_Handle.
 * @param   features A bitmask of DMP features to enable.
 * @param   fifoRate The desired DMP FIFO rate in Hz.
 * @return  0 on success, -1 on failure.
 */
int MPU_DMP_dmpBegin(MPU_DMP_Handle handle, unsigned short features, unsigned short fifoRate);

/**
 * @brief   Reads the latest data packet from the DMP FIFO.
 * @param   handle A valid MPU_DMP_Handle.
 * @return  0 if a new packet was read, -1 otherwise.
 */
int MPU_DMP_dmpUpdateFifo(MPU_DMP_Handle handle);

/** @brief Sets the DMP FIFO rate. @param handle A valid MPU_DMP_Handle. @param rate Rate in Hz. */
int MPU_DMP_dmpSetFifoRate(MPU_DMP_Handle handle, unsigned short rate);
/** @brief Gets the DMP FIFO rate. @param handle A valid MPU_DMP_Handle. */
unsigned short MPU_DMP_dmpGetFifoRate(MPU_DMP_Handle handle);

// Pedometer
/** @brief Gets the number of steps from the DMP pedometer. @param handle A valid MPU_DMP_Handle. */
unsigned long MPU_DMP_dmpGetPedometerSteps(MPU_DMP_Handle handle);
/** @brief Sets the number of steps in the DMP pedometer. @param handle A valid MPU_DMP_Handle. @param steps The step count to set. */
int MPU_DMP_dmpSetPedometerSteps(MPU_DMP_Handle handle, unsigned long steps);

// Tap Detection
/** @brief Configures the DMP tap detection. @param handle A valid MPU_DMP_Handle. @param xThresh X-axis threshold. @param yThresh Y-axis threshold. @param zThresh Z-axis threshold. @param taps Number of taps to detect. */
int MPU_DMP_dmpSetTap(MPU_DMP_Handle handle, unsigned short xThresh, unsigned short yThresh, unsigned short zThresh, unsigned char taps);
/** @brief Checks if a new tap is available. @param handle A valid MPU_DMP_Handle. @return 1 if available, 0 otherwise. */
int MPU_DMP_tapAvailable(MPU_DMP_Handle handle);
/** @brief Gets the direction of the last detected tap. @param handle A valid MPU_DMP_Handle. */
unsigned char MPU_DMP_getTapDir(MPU_DMP_Handle handle);
/** @brief Gets the count of the last detected tap event. @param handle A valid MPU_DMP_Handle. */
unsigned char MPU_DMP_getTapCount(MPU_DMP_Handle handle);

// Orientation
/** @brief Sets the DMP's orientation matrix. @param handle A valid MPU_DMP_Handle. @param orientationMatrix A 9-element array. */
int MPU_DMP_dmpSetOrientation(MPU_DMP_Handle handle, const signed char *orientationMatrix);
/** @brief Gets the current Android-style orientation from the DMP. @param handle A valid MPU_DMP_Handle. */
unsigned char MPU_DMP_dmpGetOrientation(MPU_DMP_Handle handle);

// Data Calculation & Conversion
/** @brief Converts raw accelerometer value to g's. @param handle A valid MPU_DMP_Handle. @param axis_val Raw sensor value. */
float MPU_DMP_calcAccel(MPU_DMP_Handle handle, int axis_val);
/** @brief Converts raw gyroscope value to dps. @param handle A valid MPU_DMP_Handle. @param axis_val Raw sensor value. */
float MPU_DMP_calcGyro(MPU_DMP_Handle handle, int axis_val);
/** @brief Converts raw Q30 quaternion to float. @param handle A valid MPU_DMP_Handle. @param quat_val Raw Q30 value. */
float MPU_DMP_calcQuat(MPU_DMP_Handle handle, long quat_val);
/** @brief Computes Euler angles from the current quaternion. @param handle A valid MPU_DMP_Handle. @param degrees True for degrees, false for radians. */
int MPU_DMP_computeEulerAngles(MPU_DMP_Handle handle, bool degrees);

#ifdef USE_MPU9250
/** @brief Converts raw magnetometer value to uT. @param handle A valid MPU_DMP_Handle. @param axis Raw sensor value. */
float MPU_DMP_calcMag(MPU_DMP_Handle handle, int axis);
/** @brief Computes compass heading from raw magnetometer data. @param handle A valid MPU_DMP_Handle. */
float MPU_DMP_computeCompassHeading(MPU_DMP_Handle handle);
#endif

// Raw Data Getters
/** @brief Gets raw accelerometer X-axis value. @param handle A valid MPU_DMP_Handle. */
int MPU_DMP_get_ax(MPU_DMP_Handle handle);
/** @brief Gets raw accelerometer Y-axis value. @param handle A valid MPU_DMP_Handle. */
int MPU_DMP_get_ay(MPU_DMP_Handle handle);
/** @brief Gets raw accelerometer Z-axis value. @param handle A valid MPU_DMP_Handle. */
int MPU_DMP_get_az(MPU_DMP_Handle handle);
/** @brief Gets raw gyroscope X-axis value. @param handle A valid MPU_DMP_Handle. */
int MPU_DMP_get_gx(MPU_DMP_Handle handle);
/** @brief Gets raw gyroscope Y-axis value. @param handle A valid MPU_DMP_Handle. */
int MPU_DMP_get_gy(MPU_DMP_Handle handle);
/** @brief Gets raw gyroscope Z-axis value. @param handle A valid MPU_DMP_Handle. */
int MPU_DMP_get_gz(MPU_DMP_Handle handle);
/** @brief Gets raw quaternion W value. @param handle A valid MPU_DMP_Handle. */
long MPU_DMP_get_qw(MPU_DMP_Handle handle);
/** @brief Gets raw quaternion X value. @param handle A valid MPU_DMP_Handle. */
long MPU_DMP_get_qx(MPU_DMP_Handle handle);
/** @brief Gets raw quaternion Y value. @param handle A valid MPU_DMP_Handle. */
long MPU_DMP_get_qy(MPU_DMP_Handle handle);
/** @brief Gets raw quaternion Z value. @param handle A valid MPU_DMP_Handle. */
long MPU_DMP_get_qz(MPU_DMP_Handle handle);
/** @brief Gets raw temperature value. @param handle A valid MPU_DMP_Handle. */
long MPU_DMP_get_temperature(MPU_DMP_Handle handle);
/** @brief Gets the timestamp of the last DMP update. @param handle A valid MPU_DMP_Handle. */
unsigned long MPU_DMP_get_time(MPU_DMP_Handle handle);

// Processed Data Getters
/** @brief Gets the calculated pitch in degrees/radians. @param handle A valid MPU_DMP_Handle. */
float MPU_DMP_get_pitch(MPU_DMP_Handle handle);
/** @brief Gets the calculated roll in degrees/radians. @param handle A valid MPU_DMP_Handle. */
float MPU_DMP_get_roll(MPU_DMP_Handle handle);
/** @brief Gets the calculated yaw in degrees/radians. @param handle A valid MPU_DMP_Handle. */
float MPU_DMP_get_yaw(MPU_DMP_Handle handle);

#ifdef USE_MPU9250
/** @brief Gets raw magnetometer X-axis value. @param handle A valid MPU_DMP_Handle. */
int MPU_DMP_get_mx(MPU_DMP_Handle handle);
/** @brief Gets raw magnetometer Y-axis value. @param handle A valid MPU_DMP_Handle. */
int MPU_DMP_get_my(MPU_DMP_Handle handle);
/** @brief Gets raw magnetometer Z-axis value. @param handle A valid MPU_DMP_Handle. */
int MPU_DMP_get_mz(MPU_DMP_Handle handle);
/** @brief Gets the calculated compass heading. @param handle A valid MPU_DMP_Handle. */
float MPU_DMP_get_heading(MPU_DMP_Handle handle);
#endif

#ifdef __cplusplus
}
#endif

#endif // MPU_DMP_C_H
