#ifndef ARDUINO_MPU9250_I2C_H
#define ARDUINO_MPU9250_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

int arduino_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
int arduino_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);

#ifdef __cplusplus
}
#endif

#endif // ARDUINO_MPU9250_I2C_H
