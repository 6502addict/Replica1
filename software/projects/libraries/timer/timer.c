#include <stdio.h>
#include <stdint.h>
#include "../config.h"
#include "timer.h"

/*
 * Get timer cpu speed
 * Returns: the cpu speed in mhz
 */
uint8_t timer_cpu_speed(void) {
    return (*TIMER_CPU_SPEED);
}

/*
 * Start the timer (resets counter to 0)
 */
void timer_start(void) {
    *TIMER_CONTROL = TIMER_START_STOP;
}

/*
 * Stop the timer (preserves current count)
 */
void timer_stop(void) {
    *TIMER_CONTROL = 0x00;
}

/*
 * Read current timer value
 * Returns: 16-bit timer count
 */
uint16_t timer_read(void) {
    uint8_t low, high;
    
    // Read low byte first, then high byte
    low = *TIMER_LOW;
    high = *TIMER_HIGH;
    
    return ((uint16_t)high << 8) | low;
}

/*
 * Get timer status
 * Returns: 1 if running, 0 if stopped
 */
uint8_t timer_is_running(void) {
    return (*TIMER_CONTROL & TIMER_START_STOP) ? 1 : 0;
}

/*
 * Precise delay using hardware timer
 * ticks: number of timer ticks to delay
 */
void timer_delay_ticks(unsigned int ticks) {
    unsigned int start_time, current_time;
    
    timer_start();          // Reset and start timer
    start_time = 0;         // Timer starts at 0
    
    do {
        current_time = timer_read();
    } while (current_time < ticks);
    
    timer_stop();
}

/*
 * Get timer frequency in Hz
 */
uint32_t timer_get_frequency_hz(void) {
    uint8_t cpu_mhz = timer_cpu_speed();
    return (uint32_t)cpu_mhz * 1000000UL;
}

/*
 * Get ticks per millisecond
 */
uint32_t timer_get_ticks_per_ms(void) {
    uint8_t cpu_mhz = timer_cpu_speed();
    return (uint32_t)cpu_mhz * 1000UL;
}

/*
 * Get ticks per microsecond
 */
uint16_t timer_get_ticks_per_us(void) {
    return timer_cpu_speed();  // At nMHz, n ticks per microsecond
}

/*
 * Convert timer ticks to milliseconds using actual CPU speed
 */
uint16_t timer_ticks_to_ms(uint16_t ticks) {
    uint8_t cpu_mhz = timer_cpu_speed();
    uint32_t ticks_per_ms = (uint32_t)cpu_mhz * 1000UL;
    return (uint16_t)(ticks / ticks_per_ms);
}

/*
 * Convert timer ticks to microseconds using actual CPU speed
 * WARNING: Imprecise at high CPU speeds due to integer division!
 *          At 30MHz, loses sub-microsecond precision.
 *          For accurate timing, use ticks directly.
 */
uint32_t timer_ticks_to_us(uint16_t ticks) {
    uint8_t cpu_mhz = timer_cpu_speed();
    return (uint32_t)ticks / cpu_mhz;
}

/*
 * Delay in microseconds using actual CPU speed
 */
void timer_delay_us(unsigned int microseconds) {
    uint8_t cpu_mhz;
    uint32_t ticks_per_us, total_ticks;
    unsigned int ms, remaining_us;
    uint32_t remaining_ticks;
    
    cpu_mhz = timer_cpu_speed();
    ticks_per_us = cpu_mhz;  // At 1MHz, 1 tick per µs; at 30MHz, 30 ticks per µs
    total_ticks = ticks_per_us * microseconds;
    
    // Handle overflow for large delays
    if (total_ticks > 65535UL) {
        // For very large µs delays, convert to ms
        ms = microseconds / 1000;
        remaining_us = microseconds % 1000;
        if (ms > 0) 
            timer_delay_ms(ms);
        if (remaining_us > 0) {
            remaining_ticks = ticks_per_us * remaining_us;
            if (remaining_ticks <= 65535UL) 
                timer_delay_ticks((unsigned int)remaining_ticks);
        }
    } else {
        timer_delay_ticks((unsigned int)total_ticks);
    }
}

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
