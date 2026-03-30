#include <stdio.h>
#include <stdint.h>
#include <timer.h>


/*
 * Delay in milliseconds  
 */
void timer_delay_ms(unsigned int milliseconds) {
    unsigned int ticks = milliseconds * 1843UL;
    timer_delay_ticks(ticks);
}
