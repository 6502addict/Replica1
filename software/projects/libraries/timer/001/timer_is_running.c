#include <stdio.h>
#include <stdint.h>
#include <timer.h>

/*
 * Get timer status
 * Returns: 1 if running, 0 if stopped
 */
unsigned char timer_is_running(void) {
    return (*TIMER_CONTROL & TIMER_START_STOP) ? 1 : 0;
}
