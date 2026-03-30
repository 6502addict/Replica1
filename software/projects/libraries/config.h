#ifndef CONFIG_H
#define CONFIG_H

/* 1. Define internal IDs for your targets */
#define ID_REPLICA1  1
#define ID_MAX6502   2

/* 2. Map the incoming TARGET name to an ID */
/* This works even if TARGET is passed as a bare word from the Makefile */
#if TARGET == replica1
    #define TARGET_ID ID_REPLICA1
#elif TARGET == max6502
    #define TARGET_ID ID_MAX6502
#else
    #error "Unknown or undefined TARGET"
#endif

/* 3. Use the ID to select your configuration */
#if TARGET_ID == ID_REPLICA1
    #define RAM_START    0x0000
    #define RAM_END      0xBFFF
    #define STACK        0x01FF
    #define WOZACI       0xC000
    #define SPI_ADDR     0xC210
    #define TIMER_ADDR   0xC220
    #define UART_ADDR    0xC230
    #define KBD_ADDR     0xC250
    #define SCREEN_ADDR  0xD800
    #define CON_ADDR     0xD010

#elif TARGET_ID == ID_MAX6502
    #define RAM_START    0x0000
    #define RAM_END      0xEFFF
    #define STACK        0x01FF
    #define CON_ADDR     0xF000
    #define SPI_ADDR     0xF010
    #define TIMER_ADDR   0xF020
    #define UART_ADDR    0xF030
#endif

#endif /* CONFIG_H */


