/*
 * Timer Test Program using timer library
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
    uint16_t start_time, end_time;
    
    timer_start();
    start_time = timer_read();
    func();
    end_time = timer_read();
    timer_stop();
    
    return end_time - start_time;
}

/*
 * Test functions
 */

void delay_short(void) {
    uint16_t i;
    for (i = 0; i < 1000; i++) {
        // Simple delay loop
    }
}

void test_function_1(void) {
    uint16_t i;
    volatile uint8_t dummy = 0;
    
    for (i = 0; i < 1000; i++) {
        dummy = i & 0xFF;
    }
}

void test_function_2(void) {
    uint16_t i;
    volatile uint16_t sum = 0;
    
    for (i = 0; i < 500; i++) {
        sum += i;
    }
}

void test_uart_output(void) {
    printf("UART test message\n");
}

/*
 * Timing test suite
 */
void run_timing_tests(void) {
    uint16_t execution_time;
    uint32_t microseconds;
    uint16_t milliseconds;
    
    printf("Timer Test Program\n");
    printf("==================\n");
    printf("Timer frequency: %lu Hz\n", timer_get_frequency_hz());
    printf("CPU speed: %u\n", timer_cpu_speed());
    printf("Timer running: %s\n\n", timer_is_running() ? "Yes" : "No");
    
    // Test 1: Short delay
    execution_time = time_function(delay_short);
    microseconds = timer_ticks_to_us(execution_time);
    milliseconds = timer_ticks_to_ms(execution_time);
    
    printf("delay_short():\n");
    printf("  Ticks: %u\n", execution_time);
    printf("  Time: %lu microseconds\n", microseconds);
    printf("  Time: %u milliseconds\n\n", milliseconds);
    
    // Test 2: Function 1
    execution_time = time_function(test_function_1);
    microseconds = timer_ticks_to_us(execution_time);
    milliseconds = timer_ticks_to_ms(execution_time);
    
    printf("test_function_1():\n");
    printf("  Ticks: %u\n", execution_time);
    printf("  Time: %lu microseconds\n", microseconds);
    printf("  Time: %u milliseconds\n\n", milliseconds);
    
    // Test 3: Function 2
    execution_time = time_function(test_function_2);
    microseconds = timer_ticks_to_us(execution_time);
    milliseconds = timer_ticks_to_ms(execution_time);
    
    printf("test_function_2():\n");
    printf("  Ticks: %u\n", execution_time);
    printf("  Time: %lu microseconds\n", microseconds);
    printf("  Time: %u milliseconds\n\n", milliseconds);
    
    // Test 4: UART output
    execution_time = time_function(test_uart_output);
    microseconds = timer_ticks_to_us(execution_time);
    milliseconds = timer_ticks_to_ms(execution_time);
    
    printf("test_uart_output():\n");
    printf("  Ticks: %u\n", execution_time);
    printf("  Time: %lu microseconds\n", microseconds);
    printf("  Time: %u milliseconds\n\n", milliseconds);
    
    // Manual timing example
    printf("Manual timing example:\n");
    timer_start();
    delay_short();
    execution_time = timer_read();
    timer_stop();
    
    printf("  Manual timing: %u ticks\n", execution_time);
    printf("  Time: %lu microseconds\n\n", timer_ticks_to_us(execution_time));
}

/*
 * Test timer accuracy
 */
void test_timer_accuracy(void) {
    uint8_t i;
    uint16_t readings[10];
    
    printf("Timer Accuracy Test\n");
    printf("===================\n");
    
    // Take multiple readings of the same operation
    for (i = 0; i < 10; i++) {
        readings[i] = time_function(delay_short);
        printf("Reading %u: %u ticks (%lu us)\n", 
               i + 1, readings[i], timer_ticks_to_us(readings[i]));
    }
    
    printf("\n");
}

/*
 * Test timer delay functions
 */
void test_timer_delays(void) {
    printf("Timer Delay Test\n");
    printf("================\n");
    
    printf("Delaying 1000 ticks...\n");
    timer_delay_ticks(1000);
    printf("Done.\n");
    
    printf("Delaying 1000 microseconds...\n");
    timer_delay_us(1000);
    printf("Done.\n");
    
    printf("Delaying 10 milliseconds...\n");
    timer_delay_ms(10);
    printf("Done.\n\n");
}

/*
 * Basic timer functionality test
 */
void test_basic_timer(void) {
    uint16_t start_time, end_time, elapsed;
    
    printf("Basic Timer Test\n");
    printf("================\n");
    
    timer_start();
    start_time = timer_read();
    
    delay_short();
    
    end_time = timer_read();
    timer_stop();
    
    elapsed = end_time - start_time;
    
    printf("Start time: %u\n", start_time);
    printf("End time: %u\n", end_time);
    printf("Elapsed: %u ticks (%lu us)\n\n", elapsed, timer_ticks_to_us(elapsed));
}

int main() {
    printf("Starting timer library tests...\n\n");
    
    test_basic_timer();
    run_timing_tests();
    test_timer_accuracy();
    test_timer_delays();
    
    printf("Timer tests completed.\n");
    return 0;
}
