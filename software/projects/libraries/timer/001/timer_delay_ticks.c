/* timer_delay_ticks.c - No changes needed, this is the base function */
#include <stdio.h>
#include <stdint.h>
#include <timer.h>

/*
 * Precise delay using hardware timer
 * ticks: number of timer ticks to delay
 */
void timer_delay_ticks(unsigned int ticks) {
    unsigned int start_time, current_time;
    
    timer_start();          /* Reset and start timer */
    start_time = 0;         /* Timer starts at 0 */
    
    do {
        current_time = timer_read();
    } while (current_time < ticks);
    
    timer_stop();
}

/* Additional helper functions */

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
