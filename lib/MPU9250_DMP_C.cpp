#include "MPU9250_DMP_C.h"
#include "MPU9250_DMP.h"

// The C struct is just a dummy to allow for the opaque pointer
struct MPU9250_DMP_t {
    MPU9250_DMP cpp_object;
};

// --- Implementation of the C functions ---

MPU9250_DMP_Handle MPU9250_DMP_Create() {
    // Allocate memory for our handle and use the C++ 'new' operator
    // to create an instance of the MPU9250_DMP class.
    return new MPU9250_DMP_t();
}

void MPU9250_DMP_Destroy(MPU9250_DMP_Handle handle) {
    if (handle) {
        delete handle;
    }
}

int MPU9250_DMP_begin(MPU9250_DMP_Handle handle) {
    if (!handle) return -1;
    // Call the C++ method on the object inside the handle
    return handle->cpp_object.begin();
}

int MPU9250_DMP_dmpUpdateFifo(MPU9250_DMP_Handle handle) {
    if (!handle) return -1;
    return handle->cpp_object.dmpUpdateFifo();
}

// --- Getter functions ---
float MPU9250_DMP_get_qw(MPU9250_DMP_Handle handle) {
    if (!handle) return 0.0f;
    return handle->cpp_object.qw;
}
float MPU9250_DMP_get_qx(MPU9250_DMP_Handle handle) {
    if (!handle) return 0.0f;
    return handle->cpp_object.qx;
}
float MPU9250_DMP_get_qy(MPU9250_DMP_Handle handle) {
    if (!handle) return 0.0f;
    return handle->cpp_object.qy;
}
float MPU9250_DMP_get_qz(MPU9250_DMP_Handle handle) {
    if (!handle) return 0.0f;
    return handle->cpp_object.qz;
}
