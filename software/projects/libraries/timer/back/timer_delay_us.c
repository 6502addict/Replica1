#include <stdio.h>
#include <stdint.h>
#include <timer.h>

/*
 * Delay in microseconds (approximately)
 */
void timer_delay_us(unsigned int microseconds) {
    /* Convert microseconds to ticks */
    /* 1.8432MHz = 1.843 ticks per microsecond */
    unsigned int ticks = (microseconds * 1843UL) / 1000UL;
    timer_delay_ticks(ticks);
}
