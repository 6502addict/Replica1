# Timer Library User Manual

## Overview

The Timer Library provides hardware timer functionality for embedded systems. It allows precise timing measurements and delays.

## Installation

Include the timer header in your C source files:

```c
#include <timer.h>
```

Link with the timer library when compiling.

## Basic Usage

### Starting and Stopping the Timer

```c
timer_start();      // Start the timer
timer_stop();       // Stop the timer
```

### Reading Timer Values

```c
uint16_t ticks = timer_read();  // Get current timer count
```

### Checking Timer Status

```c
if (timer_is_running()) {
    // Timer is currently running
}
```

## Timing Functions

### Get Timer Information

```c
uint32_t freq = timer_get_frequency_hz();    // Timer frequency in Hz
uint8_t speed = timer_cpu_speed();           // CPU speed in MHz
uint32_t ticks_ms = timer_get_ticks_per_ms(); // Ticks per millisecond
uint16_t ticks_us = timer_get_ticks_per_us(); // Ticks per microsecond
```

### Convert Timer Ticks

```c
uint32_t microseconds = timer_ticks_to_us(ticks);
uint16_t milliseconds = timer_ticks_to_ms(ticks);
```

### Delay Functions

```c
timer_delay_ticks(1000);    // Delay for 1000 timer ticks
timer_delay_us(500);        // Delay for 500 microseconds
timer_delay_ms(10);         // Delay for 10 milliseconds
```

## Example: Timing a Function

```c
#include <timer.h>

void my_function(void) {
    // Function to time
    for (int i = 0; i < 1000; i++);
}

int main(void) {
    uint16_t start, end, elapsed;
    
    timer_start();
    start = timer_read();
    
    my_function();
    
    end = timer_read();
    timer_stop();
    
    elapsed = end - start;
    printf("Function took %u ticks (%lu us)\n", 
           elapsed, timer_ticks_to_us(elapsed));
    
    return 0;
}
```

## Example: Simple Delays

```c
#include <timer.h>

int main(void) {
    printf("Starting...\n");
    
    timer_delay_ms(1000);   // Wait 1 second
    printf("1 second later\n");
    
    timer_delay_us(500);    // Wait 500 microseconds
    printf("Done\n");
    
    return 0;
}
```

## Notes

- Timer resolution depends on the hardware timer frequency
- Use `timer_get_frequency_hz()` to determine actual timing resolution
- Large delays are automatically split into smaller chunks to avoid overflow
- Always call `timer_stop()` when finished to conserve power