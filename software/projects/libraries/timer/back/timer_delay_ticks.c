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


