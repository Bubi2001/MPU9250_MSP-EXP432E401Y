#ifndef MPU_DMP_H
#define MPU_DMP_H

// This block tells the C++ compiler to handle these C headers correctly
extern "C" {
#include "util/inv_mpu.h"
#include "util/inv_mpu_dmp_motion_driver.h"
}

// Return error codes
#define INV_SUCCESS 0
#define INV_ERROR  -1

/**
 * @class MPU_DMP
 * @brief C++ class to interface with the InvenSense MPU6050/6500/9250 and its Digital Motion Processor (DMP).
 *
 * This class provides a simplified, object-oriented interface to the InvenSense
 * Motion Driver library, handling initialization, configuration, and data retrieval
 * from the MPU's sensors and the DMP.
 */
class MPU_DMP {
public:
    //-------------------------------------------------------------------------
    // Public Data Members
    //-------------------------------------------------------------------------
    int ax, ay, az;         ///< Raw accelerometer data for X, Y, and Z axes.
    int gx, gy, gz;         ///< Raw gyroscope data for X, Y, and Z axes.
    long qw, qx, qy, qz;    ///< Raw quaternion data (W, X, Y, Z) in Q30 format from the DMP.
    long temperature;       ///< Raw temperature data from the sensor.
    unsigned long time;     ///< Timestamp for the last sensor data update (from DMP).
    float pitch, roll, yaw; ///< Calculated Euler angles in degrees. Updated by calling computeEulerAngles().

    // MPU9250 Magnetometer specific members
    #ifdef USE_MPU9250
    int mx, my, mz;         ///< Raw magnetometer data for X, Y, and Z axes.
    float heading;          ///< Calculated compass heading in degrees. Updated by calling computeCompassHeading().
    #endif

    //-------------------------------------------------------------------------
    // Constructor
    //-------------------------------------------------------------------------
    /**
     * @brief   Constructs a new MPU_DMP object.
     */
    MPU_DMP();

    //-------------------------------------------------------------------------
    // Basic Communication & Initialization
    //-------------------------------------------------------------------------
    /**
     * @brief   Initializes the MPU sensor and underlying driver.
     * @return  `INV_SUCCESS` (0) on success, `INV_ERROR` (-1) on failure.
     */
    int begin();
    
    /**
     * @brief   Runs the built-in self-test routine to verify sensor functionality.
     * @return  A bitmask of test results.
     * - For MPU6500: 0x01=gyro pass, 0x02=accel pass. A value of 0x03 means all sensors passed.
     * - For MPU9250: 0x04=mag pass is added. A value of 0x07 means all sensors passed.
     */
    int selfTest();

    //-------------------------------------------------------------------------
    // Sensor Configuration
    //-------------------------------------------------------------------------
    /**
     * @brief   Enable/disable specific sensors.
     * @param   sensors Bitmask of sensors to enable (e.g., `INV_XYZ_GYRO | INV_XYZ_ACCEL`).
     * @return  `INV_SUCCESS` (0) on success.
     */
    int setSensors(unsigned char sensors);

    /**
     * @brief   Sets the gyroscope's full-scale range (FSR).
     * @param   fsr The desired FSR in degrees per second (e.g., 250, 500, 1000, 2000).
     * @return  `INV_SUCCESS` (0) on success.
     */
    int setGyroFSR(unsigned short fsr);

    /**
     * @brief   Sets the accelerometer's full-scale range (FSR).
     * @param   fsr The desired FSR in g's (e.g., 2, 4, 8, 16).
     * @return  `INV_SUCCESS` (0) on success.
     */
    int setAccelFSR(unsigned char fsr);

    /**
     * @brief   Sets the digital low-pass filter (DLPF) frequency.
     * @param   lpf The cutoff frequency in Hz (e.g., 5, 10, 20, 42, 98, 188).
     * @return  `INV_SUCCESS` (0) on success.
     */
    int setLPF(unsigned short lpf);

    /**
     * @brief   Sets the sensor sample rate.
     * @note    The DMP FIFO rate is set separately. This sets the rate for raw sensor data.
     * @param   rate The sample rate in Hz.
     * @return  `INV_SUCCESS` (0) on success.
     */
    int setSampleRate(unsigned short rate);

    /** @brief  Gets the current gyroscope full-scale range in dps. */
    unsigned short getGyroFSR();
    /** @brief  Gets the current accelerometer full-scale range in g's. */
    unsigned char getAccelFSR();
    /** @brief  Gets the current low-pass filter frequency in Hz. */
    unsigned short getLPF();
    /** @brief  Gets the current sample rate in Hz. */
    unsigned short getSampleRate();
    /** @brief  Gets the gyroscope's sensitivity (raw value to dps conversion factor). */
    float getGyroSens();
    /** @brief  Gets the accelerometer's sensitivity (raw value to g's conversion factor). */
    unsigned short getAccelSens();

    #ifdef USE_MPU9250
    /**
     * @brief   Sets the magnetometer (compass) sample rate. (MPU9250 only)
     * @param   rate The sample rate in Hz.
     * @return  `INV_SUCCESS` (0) on success.
     */
    int setCompassSampleRate(unsigned short rate);

    /** * @brief   Gets the current magnetometer (compass) sample rate in Hz. (MPU9250 only)
     */
    unsigned short getCompassSampleRate();
    #endif

    //-------------------------------------------------------------------------
    // Interrupt Configuration
    //-------------------------------------------------------------------------
    /** @brief  Enables the MPU's data-ready interrupt. */
    int enableInterrupt();
    /** * @brief   Sets the interrupt pin's logic level.
     * @param   active_low 1 for active low, 0 for active high.
     */
    int setIntLevel(unsigned char active_low);
    /** * @brief   Configures the interrupt pin as latched or 50us pulse.
     * @param   enable 1 for latched, 0 for 50us pulse.
     */
    int setIntLatched(unsigned char enable);
    /** @brief  Gets the interrupt status register. */
    short getIntStatus();

    //-------------------------------------------------------------------------
    // DMP (Digital Motion Processor) Functions
    //-------------------------------------------------------------------------
    /**
     * @brief   Loads the DMP firmware and configures its features.
     * @param   features A bitmask of DMP features to enable (e.g., `DMP_FEATURE_6X_LP_QUAT`).
     * @param   fifoRate The desired rate in Hz for the DMP to update the FIFO buffer.
     * @return  `INV_SUCCESS` (0) on success, `INV_ERROR` (-1) on failure.
     */
    int dmpBegin(unsigned short features, unsigned short fifoRate = 100);

    /**
     * @brief   Reads the latest available data packet from the DMP FIFO.
     * @note    This function should be called frequently in a loop to process motion data.
     * It updates the public data members (ax, ay, az, gx, gy, gz, qw, qx, qy, qz, time).
     * @return  `INV_SUCCESS` (0) if a new packet was read, `INV_ERROR` (-1) otherwise.
     */
    int dmpUpdateFifo();

    /**
     * @brief   Sets the rate at which the DMP updates the FIFO buffer.
     * @param   rate The new FIFO rate in Hz.
     * @return  `INV_SUCCESS` (0) on success.
     */
    int dmpSetFifoRate(unsigned short rate);

    /** @brief  Gets the current DMP FIFO update rate in Hz. */
    unsigned short dmpGetFifoRate();

    /** @brief  Gets the total number of steps counted by the DMP's pedometer. */
    unsigned long dmpGetPedometerSteps();
    
    /** * @brief   Sets the pedometer's internal step counter.
     * @param   steps The value to set the step counter to.
     */
    int dmpSetPedometerSteps(unsigned long steps);

    /**
     * @brief   Configures the DMP's tap detection feature.
     * @param   xThresh Threshold for the X-axis.
     * @param   yThresh Threshold for the Y-axis.
     * @param   zThresh Threshold for the Z-axis.
     * @param   taps The number of taps to detect (e.g., 1 for single tap).
     * @return  `INV_SUCCESS` (0) on success.
     */
    int dmpSetTap(unsigned short xThresh, unsigned short yThresh, unsigned short zThresh, unsigned char taps = 1);
    
    /** @brief  Checks if a new tap has been detected since the last call. */
    bool tapAvailable();
    /** @brief  Gets the direction and axis of the last detected tap. */
    unsigned char getTapDir();
    /** @brief  Gets the count of the last detected tap event. */
    unsigned char getTapCount();

    /**
     * @brief   Sets the DMP's orientation matrix.
     * @param   orientationMatrix A 9-element array representing the device's mounting orientation.
     * @return  `INV_SUCCESS` (0) on success.
     */
    int dmpSetOrientation(const signed char *orientationMatrix);

    /** @brief  Gets the orientation result from the DMP's Android-style orientation feature. */
    unsigned char dmpGetOrientation();

    //-------------------------------------------------------------------------
    // Data Conversion & Calculation
    //-------------------------------------------------------------------------
    /**
     * @brief   Converts a raw accelerometer axis value to g's.
     * @param   axis_val The raw integer value from the sensor.
     * @return  The acceleration in g's.
     */
    float calcAccel(int axis_val);

    /**
     * @brief   Converts a raw gyroscope axis value to degrees per second.
     * @param   axis_val The raw integer value from the sensor.
     * @return  The angular velocity in dps.
     */
    float calcGyro(int axis_val);

    /**
     * @brief   Converts a raw Q30 format quaternion value to a floating-point number.
     * @param   quat_val The raw Q30 long integer from the DMP.
     * @return  The floating-point quaternion component.
     */
    float calcQuat(long quat_val);

    /**
     * @brief   Computes pitch, roll, and yaw from the current quaternion data.
     * @note    Updates the public `pitch`, `roll`, and `yaw` data members.
     * @param   degrees If true (default), angles are in degrees. If false, in radians.
     */
    void computeEulerAngles(bool degrees = true);

    #ifdef USE_MPU9250
    /**
     * @brief   Converts a raw magnetometer axis value to microteslas (uT). (MPU9250 only)
     * @param   axis_val The raw integer value from the magnetometer.
     * @return  The magnetic field strength in uT.
     */
    float calcMag(int axis_val);

    /**
     * @brief   Computes the compass heading from raw magnetometer data. (MPU9250 only)
     * @note    Updates the public `heading` data member. Raw mag data (mx, my, mz) must
     * be read separately as it is not part of the DMP FIFO packet.
     * @return  The compass heading in degrees (0-360).
     */
    float computeCompassHeading();
    #endif

private:
    float _gSense;          ///< Cached gyroscope sensitivity.
    unsigned short _aSense; ///< Cached accelerometer sensitivity.
    #ifdef USE_MPU9250
    float _mSense;          ///< Magnetometer sensitivity.
    #endif
    
    /**
     * @brief   Helper function to convert an orientation matrix row to a scalar value for the DMP.
     * @param   row Pointer to a 3-element row of the orientation matrix.
     * @return  The calculated scalar for that row.
     */
    unsigned short orientation_row_2_scale(const signed char *row);
};

#endif // MPU_DMP_H
