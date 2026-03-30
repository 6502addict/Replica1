#include <stdio.h>
#include <stdint.h>
#include <timer.h>

/*
 * Stop the timer (preserves current count)
 */
void timer_stop(void) {
    *TIMER_CONTROL = 0x00;
}
