#include <ti/drivers/I2C.h>
#include "Board.h" // Make sure this includes your I2C configuration

extern I2C_Handle i2cHandle; // This should be initialized in your main application thread

int arduino_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data) {
    I2C_Transaction i2cTransaction;
    unsigned char txBuffer[length + 1];

    txBuffer[0] = reg_addr;
    memcpy(&txBuffer[1], data, length);

    i2cTransaction.slaveAddress = slave_addr;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = length + 1;
    i2cTransaction.readBuf = NULL;
    i2cTransaction.readCount = 0;

    if (I2C_transfer(i2cHandle, &i2cTransaction)) {
        return 0; // Success
    } else {
        return -1; // Failure
    }
}

int arduino_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data) {
    I2C_Transaction i2cTransaction;
    unsigned char txBuffer = reg_addr;

    i2cTransaction.slaveAddress = slave_addr;
    i2cTransaction.writeBuf = &txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = data;
    i2cTransaction.readCount = length;

    if (I2C_transfer(i2cHandle, &i2cTransaction)) {
        return 0; // Success
    } else {
        return -1; // Failure
    }
}
