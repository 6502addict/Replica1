/*
 * Timer Interface for C Functions
 * Compatible with cc65 K&R C
 */

#include <stdio.h>
#include <stdint.h>
#include <timer.h>


/*
 * Time a function execution
 * func: pointer to function to time
 * Returns: execution time in timer ticks
 */
uint16_t time_function(void (*func)(void)) {
    uint16_t  end_time;
    
    timer_start();          /* Start timing */
    func();                 /* Execute function */
    end_time = timer_read(); /* Get end time */
    timer_stop();           /* Stop timer */
    
    return end_time;
}


/*
 * Example usage functions
 */

void delay_short(void)
{
    unsigned int i;
    for (i = 0; i < 1000; i++) {
        /* Simple delay loop */
    }
}

/* Test function 1 - simple loop */
void test_function_1(void) {
    unsigned int i;
    volatile unsigned char dummy = 0;
    
    for (i = 0; i < 1000; i++) {
        dummy = i & 0xFF;
    }
}

/* Test function 2 - SPI operation */
void test_spi_transfer(void) {
  //    unsigned char result;
    /* Assuming spi_transfer is available */
    /* result = spi_transfer(0xFF); */
}

/* Timing test example */
void run_timing_tests(void) {
  uint16_t execution_time;
  uint16_t miliseconds;
  uint32_t microseconds;
  
  printf("Function Timing Tests\n");
  printf("====================\n");
  
  /* Time test function 1 */
  execution_time = time_function(delay_short);
  microseconds = timer_ticks_to_us(execution_time);
  miliseconds =  timer_ticks_to_ms(execution_time);    

  
  printf("delay_short():\n");
  printf("  Ticks: %u\n", execution_time);
  printf("  Time: %lu microseconds\n", microseconds);
  printf("  Time: %u milliseconds\n\n", miliseconds);
  
  /* Manual timing example */
  printf("Manual timing example:\n");
  timer_start();
  
  /* Your code to time goes here */
  delay_short();
  
  execution_time = timer_read();
  timer_stop();
  
  printf("  Manual timing: %u ticks\n", execution_time);
  printf("  Manual timing: %lu microseconds\n", timer_ticks_to_us(execution_time));
}

int main() {
  run_timing_tests();
  return 0;
}

/*
 * Benchmark SD card operations
 */
void benchmark_sd_operations(void) {
    unsigned int read_time, write_time;
    //    static unsigned char buffer[512];
    
    printf("SD Card Benchmarks\n");
    printf("==================\n");
    
    /* Time a block read */
    timer_start();
    /* sd_read(0, buffer); */  /* Uncomment when you have sd_read */
    read_time = timer_read();
    timer_stop();
    
    printf("Block read: %u ticks (%lu us)\n", 
           read_time, timer_ticks_to_ms(read_time));
    
    /* Time a block write */
    timer_start();
    /* sd_write(1000, buffer); */ /* Uncomment when you have sd_write */
    write_time = timer_read();
    timer_stop();
    
    printf("Block write: %u ticks (%lu us)\n", 
           write_time, timer_ticks_to_ms(write_time));
}
