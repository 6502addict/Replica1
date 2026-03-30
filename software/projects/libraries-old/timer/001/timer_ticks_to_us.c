/* timer_ticks_to_us.c - Reworked to use CPU speed */
#include <stdio.h>
#include <stdint.h>
#include <timer.h>

/*
 * Convert timer ticks to microseconds using actual CPU speed
 */
uint32_t timer_ticks_to_us(uint16_t ticks) {
    uint8_t cpu_mhz = timer_cpu_speed();
    // ticks_per_us = cpu_mhz (e.g., 30 ticks per µs at 30MHz)
    return (uint32_t)ticks / cpu_mhz;
}

