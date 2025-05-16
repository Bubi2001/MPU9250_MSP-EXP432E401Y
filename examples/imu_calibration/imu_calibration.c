// ================================================================================
// 
// File Name           : imu_calibration.c
// Project Name        : LSE2
// Target Devices      : MSP-EXP432E401Y
// Description         : 
//     Test file to read the offsets and bias of the accelerometer inside the
//     MPU9250 Inertial Measurment Unit using TI-RTOS in the MSP-EXP432E401Y.
// 
// Author              : Adrià Babiano Novella
// Last Modified       : May 2025
//
// ================================================================================
//
// * The GPL license allows free distribution and modification of the software, 
// * as long as the same license is maintained in derivative versions.
//
// ================================================================================

/* STD header files */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Timestamp.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

/* TI-RTOS Header files */
#include <ti/drivers/I2C.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Board.h>

/* Board */
#include "ti_drivers_config.h"

/* Sensor */
#include "lib/MPU9250.h"

/* Handlers RTOS */
extern Task_Handle task0;
extern Semaphore_Handle semS1;

/* Handlers peripherals */
I2C_Handle IMUSensor;

/* function prototypes */
void switchHwi0ISR(UArg arg);
void taskCollectAccelCalibrationData(UArg arg0, UArg arg1);

int main(void){
    Board_init();

    System_printf("Initialasing GPIOs... \n");
    System_flush();

    GPIO_init();

    GPIO_enableInt(LSW);

    System_printf("Initialising I2C... \n");
    System_flush();

    I2C_init();
    I2C_Params paramsi2c;
    I2C_Params_init(&paramsi2c);
    paramsi2c.bitRate = I2C_400kHz;
    paramsi2c.transferMode = I2C_MODE_BLOCKING;
    IMUSensor = I2C_open(IMU, &paramsi2c);

    System_printf("Sistema inicialitzat!\n");
    System_flush();

    GPIO_write(LED3_GPIO, 1);

    System_printf("Press switch S1 when orientation is set to start reading \n");
    System_flush();

    BIOS_start();

    return 0;
}

void switchHwi0ISR(UArg arg) {
    GPIO_clearInt(LSW);
    Semaphore_post(semS1);
}

void taskCollectAccelCalibrationData(UArg arg0, UArg arg1) {
    I2C_Transaction i2cTransaction; // Local or passed in
    uint8_t rawSensorBuffer[14];    // To store data from MPU9250_readAccTempGyr
    int orientation_idx = 0;
    uint16_t sample_count = 0;

    const uint32_t SAMPLES_PER_ORIENTATION = 33142;

#define ACCEL_CAL_NUM_ORIENTATIONS 6

    // Array to store the names of orientations for user prompts
    static char* orientationPrompts[ACCEL_CAL_NUM_ORIENTATIONS] = {
        "Position IMU with +X axis pointing UP (against gravity)",
        "Position IMU with -X axis pointing UP (X axis along gravity)",
        "Position IMU with +Y axis pointing UP (against gravity)",
        "Position IMU with -Y axis pointing UP (Y axis along gravity)",
        "Position IMU with +Z axis pointing UP (against gravity)",
        "Position IMU with -Z axis pointing UP (Z axis along gravity)"
    };

    System_printf("Starting Accelerometer 6-Point Calibration Data Collection.\n");
    System_printf("For each orientation, %d samples will be collected and averaged.\n", SAMPLES_PER_ORIENTATION);
    System_printf("Ensure the IMU is perfectly stationary for each measurement set.\n");
    System_flush();

    for (orientation_idx = 0; orientation_idx < ACCEL_CAL_NUM_ORIENTATIONS; orientation_idx++) {
        System_printf("\n--- Orientation %d of %d ---\n", orientation_idx + 1, ACCEL_CAL_NUM_ORIENTATIONS);
        System_printf("%s\n", orientationPrompts[orientation_idx]);
        System_printf("\n Press any switch when orientation is ready \n");
        System_flush();

        Semaphore_pend(semS1, BIOS_WAIT_FOREVER);

        System_printf("Collecting data for current orientation...\n");
        System_flush();

        int64_t sumRawAccelX = 0; // Use 64-bit to prevent overflow when summing 33142 * 16-bit values
        int64_t sumRawAccelY = 0;
        int64_t sumRawAccelZ = 0;

        for (sample_count = 0; sample_count < SAMPLES_PER_ORIENTATION; sample_count++) {
            // Read all sensor data (Accel, Temp, Gyro)
            // Your MPU9250_readAccTempGyr already handles the I2C transaction
            MPU9250_readAccTempGyr(IMUSensor, &i2cTransaction, rawSensorBuffer);

            // Extract raw accelerometer data (signed 16-bit integers)
            // Order: AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L, ...
            int16_t rawAccelX = (int16_t)((rawSensorBuffer[0] << 8) | rawSensorBuffer[1]);
            int16_t rawAccelY = (int16_t)((rawSensorBuffer[2] << 8) | rawSensorBuffer[3]);
            int16_t rawAccelZ = (int16_t)((rawSensorBuffer[4] << 8) | rawSensorBuffer[5]);

            sumRawAccelX += rawAccelX;
            sumRawAccelY += rawAccelY;
            sumRawAccelZ += rawAccelZ;

            // Optional: Short delay between samples if needed, or if IMU data rate is very high.
            // If your MPU9250 ODR is 1kHz (due to DLPF settings), reading in a tight loop
            // will get you new samples quickly. A Task_sleep(1) would sample at ~1kHz.
            // For 1000 samples, Task_sleep(1) adds ~1 second to collection time per orientation.
            Task_sleep(1); // Sample at roughly 1kHz if tick is 1ms
        }

        // Calculate averages (as floats, for easier use in Python script)
        float avgRawAccelX = (float)sumRawAccelX / SAMPLES_PER_ORIENTATION;
        float avgRawAccelY = (float)sumRawAccelY / SAMPLES_PER_ORIENTATION;
        float avgRawAccelZ = (float)sumRawAccelZ / SAMPLES_PER_ORIENTATION;

        // Print the averaged raw LSB values for this orientation
        // Format: "(avg_X_LSB, avg_Y_LSB, avg_Z_LSB)"
        System_printf("Orientation '%s' - Averaged Raw LSB: (%.2f, %.2f, %.2f)\n",
                      orientationPrompts[orientation_idx],
                      avgRawAccelX, avgRawAccelY, avgRawAccelZ);
        System_flush();

        uint8_t idx_B0 = ((orientation_idx + 1) & 0x01);
        uint8_t idx_B1 = ((orientation_idx + 1) & 0x02);
        uint8_t idx_B2 = ((orientation_idx + 1) & 0x04);

        GPIO_write(LED0_GPIO, idx_B0);
        GPIO_write(LED1_GPIO, idx_B1);
        GPIO_write(LED2_GPIO, idx_B2);
    }

    System_printf("\nAll 6 orientations collected. Use these averaged LSB values in your offline Python script.\n");
    System_flush();

    // The task can now terminate, pend indefinitely, or signal completion.
}
