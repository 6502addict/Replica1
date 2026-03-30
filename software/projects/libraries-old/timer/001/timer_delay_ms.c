/* timer_delay_ms.c - Reworked to use CPU speed */
#include <stdio.h>
#include <stdint.h>
#include <timer.h>

/*
 * Delay in milliseconds using actual CPU speed
 */

void timer_delay_ms(unsigned int milliseconds) {
    uint8_t cpu_mhz;
    uint32_t ticks_per_ms;
    uint32_t adjusted_ticks;
    int i;
    
    cpu_mhz = timer_cpu_speed();
    ticks_per_ms = (uint32_t)cpu_mhz * 1000UL;
    adjusted_ticks = ticks_per_ms - (590 / cpu_mhz);
    for (i = 0; i < milliseconds; i++)
        timer_delay_ticks((unsigned int)adjusted_ticks);
}

