# SPI Library User Manual

## Overview

The SPI Library provides hardware SPI (Serial Peripheral Interface) functionality for embedded systems. It supports configurable clock speed, modes, and chip select control.

## Installation

Include the SPI header in your C source files:

```c
#include <spi.h>
```

Link with the SPI library when compiling.

## Basic Usage

### Initialize SPI

```c
spi_init(divisor, cpol, cpha);
```

Parameters:
- `divisor`: Clock divisor (controls SPI speed)
- `cpol`: Clock polarity (0 or 1)
- `cpha`: Clock phase (0 or 1)

### Data Transfer

```c
uint8_t result = spi_transfer(0xFF);  // Send 0xFF, receive response
```

### Chip Select Control

```c
spi_cs_low();   // Assert chip select (active low)
// ... SPI transfers ...
spi_cs_high();  // Deassert chip select
```

## Configuration Functions

### Set Clock Speed

```c
spi_set_divisor(8);              // Set clock divisor directly
spi_set_frequency_khz(1000);     // Set frequency to 1MHz
uint8_t div = spi_calculate_divisor(500);  // Calculate divisor for 500kHz
```

### Set SPI Mode

```c
spi_set_mode(cpol, cpha);
```

Common SPI modes:
- Mode 0: `cpol=0, cpha=0`
- Mode 1: `cpol=0, cpha=1`
- Mode 2: `cpol=1, cpha=0`
- Mode 3: `cpol=1, cpha=1`

## Example: Reading from SPI Device

```c
#include <spi.h>

int main(void) {
    uint8_t data[4];
    
    // Initialize SPI: divisor=8, mode 0
    spi_init(8, 0, 0);
    
    // Read 4 bytes from device
    spi_cs_low();
    data[0] = spi_transfer(0x00);  // Send command
    data[1] = spi_transfer(0xFF);  // Read byte 1
    data[2] = spi_transfer(0xFF);  // Read byte 2
    data[3] = spi_transfer(0xFF);  // Read byte 3
    spi_cs_high();
    
    printf("Read: %02X %02X %02X %02X\n", 
           data[0], data[1], data[2], data[3]);
    
    return 0;
}
```

## Example: Writing to SPI Device

```c
#include <spi.h>

int main(void) {
    // Initialize for 500kHz, mode 3
    spi_set_frequency_khz(500);
    spi_set_mode(1, 1);
    
    // Write command and data
    spi_cs_low();
    spi_transfer(0x02);    // Write command
    spi_transfer(0x12);    // Address
    spi_transfer(0x34);    // Data byte 1
    spi_transfer(0x56);    // Data byte 2
    spi_cs_high();
    
    return 0;
}
```

## Notes

- Always control chip select manually using `spi_cs_low()` and `spi_cs_high()`
- SPI transfers are full-duplex - data is sent and received simultaneously
- Use `spi_calculate_divisor()` to determine the correct divisor for your desired frequency
- Clock divisor and frequency functions help achieve precise timing requirements