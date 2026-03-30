#include <stdio.h>
#include <stdint.h>
#include <timer.h>

/*
 * Get timer cpu speed
 * Returns: the cpu speed in mhz
 */
uint8_t timer_cpu_speed(void) {
  return (*TIMER_CPU_SPEED);
}
