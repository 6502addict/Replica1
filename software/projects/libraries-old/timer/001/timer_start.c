#include <stdio.h>
#include <stdint.h>
#include <timer.h>

/*
 * Start the timer (resets counter to 0)
 */
void timer_start(void) {
    *TIMER_CONTROL = TIMER_START_STOP;
}
