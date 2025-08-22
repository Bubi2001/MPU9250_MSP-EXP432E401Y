#ifndef MPU9250_DMP_C_H
#define MPU9250_DMP_C_H

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle to the C++ object
typedef struct MPU9250_DMP_t* MPU9250_DMP_Handle;

// C-style functions to interact with the C++ object
MPU9250_DMP_Handle MPU9250_DMP_Create();
void MPU9250_DMP_Destroy(MPU9250_DMP_Handle handle);

int MPU9250_DMP_begin(MPU9250_DMP_Handle handle);
int MPU9250_DMP_dmpUpdateFifo(MPU9250_DMP_Handle handle);

// Functions to get data
float MPU9250_DMP_get_qw(MPU9250_DMP_Handle handle);
float MPU9250_DMP_get_qx(MPU9250_DMP_Handle handle);
float MPU9250_DMP_get_qy(MPU9250_DMP_Handle handle);
float MPU9250_DMP_get_qz(MPU9250_DMP_Handle handle);

#ifdef __cplusplus
}
#endif

#endif // MPU9250_DMP_C_H
