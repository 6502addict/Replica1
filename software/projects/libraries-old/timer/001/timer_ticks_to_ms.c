/* timer_ticks_to_ms.c - Reworked to use CPU speed @@*/
#include <stdio.h>
#include <stdint.h>
#include <timer.h>

/*
 * Convert timer ticks to milliseconds using actual CPU speed
 */
uint16_t timer_ticks_to_ms(uint16_t ticks) {
    uint8_t cpu_mhz = timer_cpu_speed();
    uint32_t ticks_per_ms = (uint32_t)cpu_mhz * 1000UL;
    return (uint16_t)(ticks / ticks_per_ms);
}
