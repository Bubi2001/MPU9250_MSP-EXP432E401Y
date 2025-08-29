#include "MPU_DMP_C.h"
#include "MPU_DMP.h"

// The C struct is just a dummy to allow for the opaque pointer
struct MPU_DMP_t {
    MPU_DMP cpp_object;
};

// --- Implementation of the C functions ---

MPU_DMP_Handle MPU_DMP_Create() {
    // Allocate memory for our handle and use the C++ 'new' operator
    // to create an instance of the MPU_DMP class.
    return new MPU_DMP_t();
}

void MPU_DMP_Destroy(MPU_DMP_Handle handle) {
    if (handle) {
        delete handle;
    }
}

int MPU_DMP_begin(MPU_DMP_Handle handle) {
    if (!handle) return -1;
    return handle->cpp_object.begin();
}

int MPU_DMP_selfTest(MPU_DMP_Handle handle) {
    if (!handle) return -1;
    return handle->cpp_object.selfTest();
}

int MPU_DMP_setSensors(MPU_DMP_Handle handle, unsigned char sensors) {
    if (!handle) return -1;
    return handle->cpp_object.setSensors(sensors);
}

int MPU_DMP_setGyroFSR(MPU_DMP_Handle handle, unsigned short fsr) {
    if (!handle) return -1;
    return handle->cpp_object.setGyroFSR(fsr);
}

int MPU_DMP_setAccelFSR(MPU_DMP_Handle handle, unsigned short fsr) {
    if (!handle) return -1;
    return handle->cpp_object.setAccelFSR(fsr);
}

int MPU_DMP_setLPF(MPU_DMP_Handle handle, unsigned short lpf) {
    if (!handle) return -1;
    return handle->cpp_object.setLPF(lpf);
}

int MPU_DMP_setSampleRate(MPU_DMP_Handle handle, unsigned short rate) {
    if (!handle) return -1;
    return handle->cpp_object.setSampleRate(rate);
}

unsigned short MPU_DMP_getGyroFSR(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.getGyroFSR();
}

unsigned char MPU_DMP_getAccelFSR(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.getAccelFSR();
}

unsigned short MPU_DMP_getLPF(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.getLPF();
}

unsigned short MPU_DMP_getSampleRate(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.getSampleRate();
}

float MPU_DMP_getGyroSens(MPU_DMP_Handle handle) {
    if (!handle) return 0.0f;
    return handle->cpp_object.getGyroSens();
}

unsigned short MPU_DMP_getAceelSens(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.getAccelSens();
}

#ifdef USE_MPU9250
int MPU_DMP_setCompassSampleRate(MPU_DMP_Handle handle, unsigned short rate) {
    if (!handle) return -1;
    return handle->cpp_object.setCompassSampleRate(rate);
}

unsigned short MPU_DMP_getCompassSampleRate(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.getCompassSampleRate();
}
#endif

int MPU_DMP_enableInterrupt(MPU_DMP_Handle handle) {
    if (!handle) return -1;
    return handle->cpp_object.enableInterrupt();
}

int MPU_DMP_setIntLevel(MPU_DMP_Handle handle, unsigned char active_low) {
    if (!handle) return -1;
    return handle->cpp_object.setIntLevel(active_low);
}

int MPU_DMP_setIntLatched(MPU_DMP_Handle handle, unsigned char enable) {
    if (!handle) return -1;
    return handle->cpp_object.setIntLatched(enable);
}

int MPU_DMP_dmpBegin(MPU_DMP_Handle handle, unsigned short features, unsigned short fifoRate) {
    if (!handle) return -1;
    return handle->cpp_object.dmpBegin(features, fifoRate);
}

int MPU_DMP_dmpUpdateFifo(MPU_DMP_Handle handle) {
    if (!handle) return -1;
    return handle->cpp_object.dmpUpdateFifo();
}

int MPU_DMP_dmpSetFifoRate(MPU_DMP_Handle handle, unsigned short rate) {
    if (!handle) return -1;
    return handle->cpp_object.dmpSetFifoRate(rate);
}

unsigned short MPU_DMP_dmpGetFifoRate(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.dmpGetFifoRate();
}

unsigned long MPU_DMP_dmpGetPedometerSteps(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.dmpGetPedometerSteps();
}

int MPU_DMP_dmpSetPedometerSteps(MPU_DMP_Handle handle, unsigned long steps) {
    if(!handle) return -1;
    return handle->cpp_object.dmpSetPedometerSteps(steps);
}

int MPU_DMP_dmpSetTap(MPU_DMP_Handle handle, unsigned short xThresh, unsigned short yThresh, unsigned short zThresh, unsigned char taps) {
    if (!handle) return -1;
    return handle->cpp_object.dmpSetTap(xThresh, yThresh, zThresh, taps);
}

int MPU_DMP_tapAvailable(MPU_DMP_Handle handle) {
    if (!handle) return -1;
    return (handle->cpp_object.tapAvailable()) ? 1 : 0;
}

unsigned char MPU_DMP_getTapDir(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.getTapDir();
}

unsigned char MPU_DMP_getTapCount(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.getTapCount();
}

int MPU_DMP_dmpSetOrientation(MPU_DMP_Handle handle, const signed char *orientationMatrix) {
    if (!handle) return -1;
    return handle->cpp_object.dmpSetOrientation(orientationMatrix);
}

unsigned char MPU_DMP_dmpGetOrientation(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.dmpGetOrientation();
}

float MPU_DMP_calcAccel(MPU_DMP_Handle handle, int axis_val) {
    if (!handle) return 0.0f;
    return handle->cpp_object.calcAccel(axis_val);
}

float MPU_DMP_calcGyro(MPU_DMP_Handle handle, int axis_val) {
    if (!handle) return 0.0f;
    return handle->cpp_object.calcGyro(axis_val);
}

float MPU_DMP_calcQuat(MPU_DMP_Handle handle, long quat_val) {
    if (!handle) return -1;
    return handle->cpp_object.calcQuat(quat_val);
}

int MPU_DMP_computeEulerAngles(MPU_DMP_Handle handle, bool degrees) {
    if (!handle) return -1;
    handle->cpp_object.computeEulerAngles(degrees);
    return 0;
}

#ifdef USE_MPU9250
float MPU_DMP_calcMag(MPU_DMP_Handle handle, int axis) {
    if (!handle) return 0.0f;
    return handle->cpp_object.calcMag(axis_val);
}

float MPU_DMP_computeCompassHeading(MPU_DMP_Handle handle) {
    if (!handle) return 0.0f;
    return handle->cpp_object.computeCompassHeading();
}
#endif

int MPU_DMP_get_ax(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.ax;
}

int MPU_DMP_get_ay(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.ay;
}

int MPU_DMP_get_az(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.az;
}

int MPU_DMP_get_gx(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.gx;
}

int MPU_DMP_get_gy(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.gy;
}

int MPU_DMP_get_gz(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.gz;
}

long MPU_DMP_get_qw(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.qw;
}

long MPU_DMP_get_qx(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.qx;
}

long MPU_DMP_get_qy(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.qy;
}

long MPU_DMP_get_qz(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.qz;
}

long MPU_DMP_get_temperature(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.temperature;
}

unsigned long MPU_DMP_get_time(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.time;
}

float MPU_DMP_get_pitch(MPU_DMP_Handle handle) {
    if (!handle) return 0.0f;
    return handle->cpp_object.pitch;
}

float MPU_DMP_get_roll(MPU_DMP_Handle handle) {
    if (!handle) return 0.0f;
    return handle->cpp_object.roll;
}

float MPU_DMP_get_yaw(MPU_DMP_Handle handle) {
    if (!handle) return 0.0f;
    return handle->cpp_object.yaw;
}

#ifdef USE_MPU9250
int MPU_DMP_get_mx(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.mx;
}

int MPU_DMP_get_my(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.my;
}

int MPU_DMP_get_mz(MPU_DMP_Handle handle) {
    if (!handle) return 0;
    return handle->cpp_object.mz;
}

float MPU_DMP_get_heading(MPU_DMP_Handle handle) {
    if (!handle) return 0.0f;
    return handle->cpp_object.heading;
}
#endif
