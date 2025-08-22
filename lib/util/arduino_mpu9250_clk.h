#ifndef ARDUINO_MPU9250_CLK_H
#define ARDUINO_MPU9250_CLK_H

#ifdef __cplusplus
extern "C" {
#endif

void arduino_delay_ms(unsigned long num_ms);
int arduino_get_clock_ms(unsigned long *count);

#ifdef __cplusplus
}
#endif

#endif // ARDUINO_MPU9250_CLK_H
