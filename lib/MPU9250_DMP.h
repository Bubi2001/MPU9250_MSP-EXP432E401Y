#ifndef MPU9250_DMP_H
#define MPU9250_DMP_H

// This block tells the C++ compiler to handle these C headers correctly
extern "C" {
#include "util/inv_mpu.h"
#include "util/inv_mpu_dmp_motion_driver.h"
}

// Return error codes
#define INV_SUCCESS 0
#define INV_ERROR -1

class MPU9250_DMP {
public:
    //-------------------------------------------------------------------------
    // Public Data Members
    //-------------------------------------------------------------------------
    int ax, ay, az;         // Raw accelerometer data
    int gx, gy, gz;         // Raw gyroscope data
    int mx, my, mz;         // Raw magnetometer data
    long qw, qx, qy, qz;    // Raw quaternion data (Q30 format)
    long temperature;       // Raw temperature data
    unsigned long time;     // Timestamp for last update
    float pitch, roll, yaw; // Calculated Euler angles (degrees)
    float heading;          // Calculated compass heading (degrees)

    //-------------------------------------------------------------------------
    // Constructor
    //-------------------------------------------------------------------------
    MPU9250_DMP();

    //-------------------------------------------------------------------------
    // Basic Communication & Initialization
    //-------------------------------------------------------------------------
    /**
     * @brief Initializes the MPU-9250 hardware and underlying C-driver.
     * Sets default sensor ranges and prepares the device for use.
     * @return INV_SUCCESS on success, INV_ERROR on failure.
     */
    int begin();

    /**
     * @brief Runs the built-in self-test routine to verify sensor functionality.
     * @return A bitmask of test results. 0x01=gyro pass, 0x02=accel pass, 0x04=mag pass.
     * A value of 0x07 means all sensors passed.
     */
    int selfTest();

    //-------------------------------------------------------------------------
    // Sensor Configuration
    //-------------------------------------------------------------------------
    int setSensors(unsigned char sensors);
    int setGyroFSR(unsigned short fsr);
    int setAccelFSR(unsigned char fsr);
    int setLPF(unsigned short lpf);
    int setSampleRate(unsigned short rate);
    int setCompassSampleRate(unsigned short rate);

    unsigned short getGyroFSR();
    unsigned char getAccelFSR();
    unsigned short getLPF();
    unsigned short getSampleRate();
    unsigned short getCompassSampleRate();
    float getGyroSens();
    unsigned short getAccelSens();

    //-------------------------------------------------------------------------
    // Interrupt Configuration
    //-------------------------------------------------------------------------
    int enableInterrupt();
    int setIntLevel(unsigned char active_low);
    int setIntLatched(unsigned char enable);
    short getIntStatus();

    //-------------------------------------------------------------------------
    // DMP (Digital Motion Processor) Functions
    //-------------------------------------------------------------------------
    /**
     * @brief Loads the DMP firmware, configures features, and starts the DMP.
     * @param features A bitmask of features to enable (e.g., DMP_FEATURE_6X_LP_QUAT).
     * @param fifoRate The desired rate (in Hz) for the DMP to generate packets. Max 200.
     * @return INV_SUCCESS on success, INV_ERROR on failure.
     */
    int dmpBegin(unsigned short features, unsigned short fifoRate = 100);

    /**
     * @brief Reads a packet from the DMP FIFO and updates public data members.
     * This is the primary function to call when using the DMP.
     * @return INV_SUCCESS on success, INV_ERROR on failure.
     */
    int dmpUpdateFifo();

    int dmpSetFifoRate(unsigned short rate);
    unsigned short dmpGetFifoRate();

    // Pedometer Functions
    unsigned long dmpGetPedometerSteps();
    int dmpSetPedometerSteps(unsigned long steps);

    // Tap Functions
    int dmpSetTap(unsigned short xThresh, unsigned short yThresh, unsigned short zThresh, unsigned char taps = 1);
    bool tapAvailable();
    unsigned char getTapDir();
    unsigned char getTapCount();

    // Orientation Functions
    int dmpSetOrientation(const signed char *orientationMatrix);
    unsigned char dmpGetOrientation();

    //-------------------------------------------------------------------------
    // Data Conversion & Calculation
    //-------------------------------------------------------------------------
    float calcAccel(int axis_val);
    float calcGyro(int axis_val);
    float calcMag(int axis_val);
    float calcQuat(long quat_val);
    void computeEulerAngles(bool degrees = true);
    float computeCompassHeading();

private:
    // Internal state variables for sensor sensitivity
    float _gSense;
    unsigned short _aSense;
    float _mSense;

    // Internal helper for orientation matrix conversion
    unsigned short orientation_row_2_scale(const signed char *row);
};

#endif // MPU9250_DMP_H