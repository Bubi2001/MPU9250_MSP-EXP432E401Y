#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

void arduino_delay_ms(unsigned long num_ms) {
    Task_sleep(num_ms * (1000 / Clock_tickPeriod));
}

int arduino_get_clock_ms(unsigned long *count) {
    if (!count) {
        return -1;
    }
    *count = Clock_getTicks() * Clock_tickPeriod / 1000;
    return 0;
}
