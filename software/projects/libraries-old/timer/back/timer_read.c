#include <stdio.h>
#include <stdint.h>
#include <timer.h>

/*
 * Read current timer value
 * Returns: 16-bit timer count
 */
uint16_t timer_read(void) {
    uint8_t low, high;
    
    /* Read low byte first, then high byte */
    low = *TIMER_LOW;
    high = *TIMER_HIGH;
    
    return ((uint16_t) high << 8) | low;
}
