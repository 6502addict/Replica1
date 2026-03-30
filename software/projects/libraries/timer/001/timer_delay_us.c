/* timer_delay_us.c - Reworked to use CPU speed */
#include <stdio.h>
#include <stdint.h>
#include <timer.h>

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
  } else 
    timer_delay_ticks((unsigned int)total_ticks);
}
