#include <stdio.h>
#include <stdint.h>
#include <timer.h>

/*
 * Convert timer ticks to milliseconds  
 */
uint16_t timer_ticks_to_ms(uint16_t ticks) {
    /* 1843.2 ticks per millisecond */
    return ticks / 1843;
}
