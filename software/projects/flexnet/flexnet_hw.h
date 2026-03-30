#pragma once
#include <stdint.h>

// MC6850 ACIA base address — change here only
#define MC6850_BASE     0xD000

// MC6850 register access
#define MC6850_STAT  (*(volatile uint8_t *)(MC6850_BASE))
#define MC6850_CTRL  (*(volatile uint8_t *)(MC6850_BASE))
#define MC6850_DATA  (*(volatile uint8_t *)(MC6850_BASE + 1))

// MC6850 status bits
#define MC6850_RDRF  0x01   // Receive Data Register Full
#define MC6850_TDRE  0x02   // Transmit Data Register Empty

// MC6850 control register values
#define MC6850_RESET      0x03   // master reset (CR1:CR0 = 11)
// /16 clock divider, 8N1, no interrupt — tune for your clock/baud
#define MC6850_CTRL_INIT  0x15

// Timeout scale: busy-wait loop iterations per millisecond
// At 1 MHz a tight loop is ~10 cycles = ~100 iter/ms; tune as needed
#define FLEX_TIMEOUT_SCALE  100U

