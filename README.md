# MPU9250 library for the MSP-EXP432E401Y

I2C Driver for library for the IMU MPU9250 using an MSP-EXP432E401Y.

On the `lib` directory you can find the source and header files for the c library itself.

On the `script` directory you can find a Python script that calculates the bias vector and calibration matrix for the IMU with acceleration readings. On the releases tab you can find a Code Composer Studio project to get this raw accelerometer readings.

The Python script has several dependencies, to have them all run this command:

```bash
pip install termcolor numpy sympy
```

To run the script run this command:

```bash
python ./script/imu_cal.py
```

And follow the printed in console instructions
