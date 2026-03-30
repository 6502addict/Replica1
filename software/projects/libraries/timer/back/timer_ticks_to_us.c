#include <stdio.h>
#include <stdint.h>
#include <timer.h>

/*
 * Convert timer ticks to microseconds
 * Assumes timer_clk = phi2 = 1.8432MHz
 */
uint32_t timer_ticks_to_us(uint16_t ticks) {
    /* 1.8432MHz = 1,843,200 ticks per second */
    /* 1 tick = 1/1,843,200 seconds = 0.542 microseconds */
    return (uint32_t) ticks * 542UL / 1000UL;
}
