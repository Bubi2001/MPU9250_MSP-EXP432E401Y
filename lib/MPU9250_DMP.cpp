#include "MPU9250_DMP.h"
#include <math.h>

// Define PI for Euler angle calculations
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Global callbacks for C-driver tap and orientation events
static bool _tap_available_cb = false;
static unsigned char _tap_count_cb = 0;
static unsigned char _tap_direction_cb = 0;
static unsigned char _orientation_cb = 0;

static void tap_cb(unsigned char direction, unsigned char count) {
    _tap_available_cb = true;
    _tap_count_cb = count;
    _tap_direction_cb = direction;
}

static void orient_cb(unsigned char orient) {
    _orientation_cb = orient;
}

//-------------------------------------------------------------------------
// Constructor
//-------------------------------------------------------------------------
MPU9250_DMP::MPU9250_DMP() {
    // Initialize sensitivities to their default/constant values
    _gSense = 0.0f; // Will be updated in begin()
    _aSense = 0.0f; // Will be updated in begin()
    _mSense = 0.15f; // Constant for AK8963
}

//-------------------------------------------------------------------------
// Basic Communication & Initialization
//-------------------------------------------------------------------------
int MPU9250_DMP::begin() {
    if (mpu_init(NULL) != 0) return INV_ERROR;
    if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0) return INV_ERROR;
    if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0) return INV_ERROR;

    // Update sensitivity values based on default FSR
    _gSense = getGyroSens();
    _aSense = getAccelSens();

    return INV_SUCCESS;
}

int MPU9250_DMP::selfTest() {
    long gyro_bias[3], accel_bias[3];
    return mpu_run_self_test(gyro_bias, accel_bias);
}

//-------------------------------------------------------------------------
// Sensor Configuration
//-------------------------------------------------------------------------
int MPU9250_DMP::setSensors(unsigned char sensors) {
    return mpu_set_sensors(sensors);
}

int MPU9250_DMP::setGyroFSR(unsigned short fsr) {
    int result = mpu_set_gyro_fsr(fsr);
    if (result == INV_SUCCESS) {
        _gSense = getGyroSens(); // Update sensitivity on change
    }
    return result;
}

int MPU9250_DMP::setAccelFSR(unsigned char fsr) {
    int result = mpu_set_accel_fsr(fsr);
    if (result == INV_SUCCESS) {
        _aSense = getAccelSens(); // Update sensitivity on change
    }
    return result;
}

int MPU9250_DMP::setLPF(unsigned short lpf) { return mpu_set_lpf(lpf); }
int MPU9250_DMP::setSampleRate(unsigned short rate) { return mpu_set_sample_rate(rate); }
int MPU9250_DMP::setCompassSampleRate(unsigned short rate) { return mpu_set_compass_sample_rate(rate); }
unsigned short MPU9250_DMP::getGyroFSR() { unsigned short fsr; mpu_get_gyro_fsr(&fsr); return fsr; }
unsigned char MPU9250_DMP::getAccelFSR() { unsigned char fsr; mpu_get_accel_fsr(&fsr); return fsr; }
unsigned short MPU9250_DMP::getLPF() { unsigned short lpf; mpu_get_lpf(&lpf); return lpf; }
unsigned short MPU9250_DMP::getSampleRate() { unsigned short rate; mpu_get_sample_rate(&rate); return rate; }
unsigned short MPU9250_DMP::getCompassSampleRate() { unsigned short rate; mpu_get_compass_sample_rate(&rate); return rate; }
float MPU9250_DMP::getGyroSens() { float sens; mpu_get_gyro_sens(&sens); return sens; }
unsigned short MPU9250_DMP::getAccelSens() { unsigned short sens; mpu_get_accel_sens(&sens); return sens; }

//-------------------------------------------------------------------------
// Interrupt Configuration
//-------------------------------------------------------------------------
int MPU9250_DMP::enableInterrupt() { return set_int_enable(1); }
int MPU9250_DMP::setIntLevel(unsigned char active_low) { return mpu_set_int_level(active_low); }
int MPU9250_DMP::setIntLatched(unsigned char enable) { return mpu_set_int_latched(enable); }
short MPU9250_DMP::getIntStatus() { short status; mpu_get_int_status(&status); return status; }

//-------------------------------------------------------------------------
// DMP (Digital Motion Processor) Functions
//-------------------------------------------------------------------------
int MPU9250_DMP::dmpBegin(unsigned short features, unsigned short fifoRate) {
    if (dmp_load_motion_driver_firmware() != 0) return INV_ERROR;
    if (dmp_set_orientation(0) != 0) return INV_ERROR;

    // Register callbacks for tap and orientation
    dmp_register_tap_cb(tap_cb);
    dmp_register_android_orient_cb(orient_cb);

    if (dmp_enable_feature(features) != 0) return INV_ERROR;
    if (dmp_set_fifo_rate(fifoRate) != 0) return INV_ERROR;
    if (mpu_set_dmp_state(1) != 0) return INV_ERROR;
    return INV_SUCCESS;
}

int MPU9250_DMP::dmpUpdateFifo() {
    short gyro_raw[3], accel_raw[3], sensors_mask;
    long quat_raw[4];
    unsigned char more_packets;

    if (dmp_read_fifo(gyro_raw, accel_raw, quat_raw, &this->time, &sensors_mask, &more_packets) != 0) {
        return INV_ERROR;
    }

    if (sensors_mask & INV_WXYZ_QUAT) {
        this->qw = quat_raw[0];
        this->qx = quat_raw[1];
        this->qy = quat_raw[2];
        this->qz = quat_raw[3];
    }
    if (sensors_mask & INV_XYZ_ACCEL) {
        this->ax = accel_raw[0];
        this->ay = accel_raw[1];
        this->az = accel_raw[2];
    }
    if (sensors_mask & INV_XYZ_GYRO) {
        this->gx = gyro_raw[0];
        this->gy = gyro_raw[1];
        this->gz = gyro_raw[2];
    }
    return INV_SUCCESS;
}

int MPU9250_DMP::dmpSetFifoRate(unsigned short rate) { return dmp_set_fifo_rate(rate); }
unsigned short MPU9250_DMP::dmpGetFifoRate() { unsigned short rate; dmp_get_fifo_rate(&rate); return rate; }
unsigned long MPU9250_DMP::dmpGetPedometerSteps() { unsigned long steps; dmp_get_pedometer_step_count(&steps); return steps; }
int MPU9250_DMP::dmpSetPedometerSteps(unsigned long steps) { return dmp_set_pedometer_step_count(steps); }

int MPU9250_DMP::dmpSetTap(unsigned short xThresh, unsigned short yThresh, unsigned short zThresh, unsigned char taps) {
    unsigned char axes = 0;
    if (xThresh > 0) axes |= TAP_X;
    if (yThresh > 0) axes |= TAP_Y;
    if (zThresh > 0) axes |= TAP_Z;
    if(dmp_set_tap_axes(axes) != 0) return INV_ERROR;
    if(dmp_set_tap_thresh(TAP_X, xThresh) != 0) return INV_ERROR;
    if(dmp_set_tap_thresh(TAP_Y, yThresh) != 0) return INV_ERROR;
    if(dmp_set_tap_thresh(TAP_Z, zThresh) != 0) return INV_ERROR;
    return dmp_set_tap_count(taps);
}

bool MPU9250_DMP::tapAvailable() {
    bool available = _tap_available_cb;
    _tap_available_cb = false; // Reset flag after checking
    return available;
}
unsigned char MPU9250_DMP::getTapDir() { return _tap_direction_cb; }
unsigned char MPU9250_DMP::getTapCount() { return _tap_count_cb; }

int MPU9250_DMP::dmpSetOrientation(const signed char *orientationMatrix) {
    unsigned short scalar = orientation_row_2_scale(orientationMatrix);
	scalar |= orientation_row_2_scale(orientationMatrix + 3) << 3;
	scalar |= orientation_row_2_scale(orientationMatrix + 6) << 6;
    return dmp_set_orientation(scalar);
}
unsigned char MPU9250_DMP::dmpGetOrientation() { return _orientation_cb; }

//-------------------------------------------------------------------------
// Data Conversion & Calculation
//-------------------------------------------------------------------------
float MPU9250_DMP::calcAccel(int axis_val) { return (float)axis_val / _aSense; }
float MPU9250_DMP::calcGyro(int axis_val) { return (float)axis_val / _gSense; }
float MPU9250_DMP::calcMag(int axis_val) { return (float)axis_val * _mSense; }
float MPU9250_DMP::calcQuat(long quat_val) { return (float)quat_val / 1073741824.0f; } // 2^30

void MPU9250_DMP::computeEulerAngles(bool degrees) {
    float q0 = calcQuat(qw);
    float q1 = calcQuat(qx);
    float q2 = calcQuat(qy);
    float q3 = calcQuat(qz);

    this->roll = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabs(sinp) >= 1)
        this->pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        this->pitch = asin(sinp);
    this->yaw = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));

    if (degrees) {
        this->pitch *= 180.0f / M_PI;
        this->roll  *= 180.0f / M_PI;
        this->yaw   *= 180.0f / M_PI;
    }
}

float MPU9250_DMP::computeCompassHeading() {
    // This requires reading the magnetometer, which is not part of the DMP FIFO packet.
    // You would need a separate mechanism to read mag data if heading is needed with DMP.
    // The following is a placeholder if mag data were available:
    if (my == 0)
        heading = (mx < 0) ? 180.0 : 0;
    else
        heading = atan2((float)mx, (float)my) * 180.0 / M_PI;
    if(heading < 0) heading += 360.0;
    return heading;
}

unsigned short MPU9250_DMP::orientation_row_2_scale(const signed char *row) {
    unsigned short b;
    if (row[0] > 0) b = 0;
    else if (row[0] < 0) b = 4;
    else if (row[1] > 0) b = 1;
    else if (row[1] < 0) b = 5;
    else if (row[2] > 0) b = 2;
    else if (row[2] < 0) b = 6;
    else b = 7;
    return b;
}
