/*
 * File: include/timer.h
 * Timer Library Header File
 */

#ifndef TIMER_H
#define TIMER_H

/* Timer register addresses */
#define TIMER_CONTROL     ((unsigned char*) TIMER_ADDR + 0)  /* Control register  */
#define TIMER_LOW         ((unsigned char*) TIMER_ADDR + 1)  /* Counter low byte  */
#define TIMER_HIGH        ((unsigned char*) TIMER_ADDR + 2)  /* Counter high byte */
#define TIMER_CPU_SPEED   ((unsigned char*) TIMER_ADDR + 3)  /* Counter cpu speed */

/* Control register bits */
#define TIMER_START_STOP  0x01


/* Function prototypes */
void      timer_start(void);
void      timer_stop(void);
uint16_t  timer_read(void);
uint8_t   timer_is_running(void);
uint8_t   timer_cpu_speed(void);
uint32_t  timer_ticks_to_us(unsigned int);
uint16_t  timer_ticks_to_ms(unsigned int);
void      timer_delay_ticks(unsigned int);
void      timer_delay_us(unsigned int);
void      timer_delay_ms(unsigned int);
uint32_t  timer_get_frequency_hz(void);
uint32_t  timer_get_ticks_per_ms(void);
uint16_t  timer_get_ticks_per_us(void);


#endif /* TIMER_H */


