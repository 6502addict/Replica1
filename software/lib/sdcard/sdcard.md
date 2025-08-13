# SD Card Library Manual

## Overview

This library provides SD card functionality for vintage 8-bit processors (6502, 6800, 6809) interfacing through SPI. It is designed to work with FPGA-based vintage computer recreations and supports SDHC cards (SD v2.0+) with plans for SD v1.x support.

The library is built using the cc65 toolchain and provides a clean, error-code-based interface suitable for embedded systems without requiring printf support in the core functions.

## Features

- **SDHC Support**: Full support for SD v2.0+ (SDHC) cards
- **Error Code System**: Comprehensive error reporting without embedded printf statements
- **SPI Interface**: Works with the companion SPI master library
- **Block-Level Access**: Read and write 512-byte blocks
- **CRC7 Support**: Automatic CRC calculation for SD commands
- **Optimized for Vintage CPUs**: Efficient code generation for 8-bit processors

## Installation

### Prerequisites

- cc65 toolchain installed
- SPI master library (companion library)
- FPGA with SPI master interface (mspi_iface)

### Building the Library

```bash
# Build the SD card library
make

# This creates sdcard.lib and compiles all source files
```

### Installation (Optional)

```bash
# Install to cc65 system directories
make install
```

## Header File

Include the library header in your programs:

```c
#include <sdcard.h>
#include <spi.h>      // Required for SPI functions
```

## Constants and Definitions

### Block Size
```c
#define SD_BLOCK_SIZE    512    // Standard SD card block size
```

### Data Tokens
```c
#define DATA_START_TOKEN    0xFE    // Start of data block
#define DATA_ACCEPT_TOKEN   0x05    // Write accepted
#define DATA_REJECT_CRC     0x0B    // Write rejected - CRC error
#define DATA_REJECT_WRITE   0x0D    // Write rejected - write error
```

### Error Codes

#### Initialization Errors (0x00-0x0F)
```c
#define SD_SUCCESS              0x00    // Success
#define SD_ERROR_CMD0           0x01    // CMD0 (GO_IDLE_STATE) failed
#define SD_ERROR_CMD8           0x02    // CMD8 (SEND_IF_COND) failed
#define SD_ERROR_ACMD41         0x03    // ACMD41 initialization failed
#define SD_ERROR_V1_CARD        0x04    // SD v1.x card (not supported)
#define SD_ERROR_UNKNOWN_CMD8   0x05    // CMD8 unexpected response
```

#### Command Errors (0x10-0x1F)
```c
#define SD_ERROR_CMD55          0x10    // CMD55 (APP_CMD) failed
#define SD_ERROR_ACMD41_TIMEOUT 0x11    // ACMD41 timeout
```

#### Read Errors (0x20-0x2F)
```c
#define SD_ERROR_CMD17          0x20    // CMD17 (READ_SINGLE_BLOCK) failed
#define SD_ERROR_READ_TOKEN     0x21    // Read data token timeout/error
#define SD_ERROR_READ_TIMEOUT   0x22    // Read operation timeout
```

#### Write Errors (0x30-0x3F)
```c
#define SD_ERROR_CMD24          0x30    // CMD24 (WRITE_BLOCK) failed
#define SD_ERROR_WRITE_REJECT   0x31    // Write data rejected
#define SD_ERROR_WRITE_TIMEOUT  0x32    // Write operation timeout
```

## Core Functions

### Initialization

#### `uint8_t sd_init(void)`

Initializes the SD card with full initialization sequence.

**Returns:**
- `SD_SUCCESS` (0x00) on success
- Error code on failure

**Description:**
Performs complete SD card initialization:
1. Sends initial clock cycles to wake up card
2. Sends CMD0 to put card in idle state
3. Sends CMD8 to check card version and voltage
4. Performs ACMD41 loop until card is ready

**Usage:**
```c
uint8_t result = sd_init();
if (result != SD_SUCCESS) {
    // Handle initialization error
    // Use sd_error_string(result) for debugging
}
```

### Block I/O Operations

#### `uint8_t sd_read(unsigned long block_num, uint8_t *buffer)`

Reads a single 512-byte block from the SD card.

**Parameters:**
- `block_num`: Block number to read (0-based, block addressing for SDHC)
- `buffer`: Pointer to 512-byte buffer to store data

**Returns:**
- `SD_SUCCESS` on successful read
- Error code on failure

**Usage:**
```c
static uint8_t buffer[SD_BLOCK_SIZE];
uint8_t result = sd_read(0, buffer);  // Read MBR
if (result == SD_SUCCESS) {
    // Process data in buffer
}
```

#### `uint8_t sd_write(unsigned long block_num, uint8_t *buffer)`

Writes a single 512-byte block to the SD card.

**Parameters:**
- `block_num`: Block number to write
- `buffer`: Pointer to 512-byte buffer containing data to write

**Returns:**
- `SD_SUCCESS` on successful write
- Error code on failure

**Usage:**
```c
static uint8_t data[SD_BLOCK_SIZE];
// Fill data buffer...
uint8_t result = sd_write(1000, data);
if (result != SD_SUCCESS) {
    // Handle write error
}
```

### Command Functions

#### `void sd_cmd(uint8_t cmd, uint8_t arg0, uint8_t arg1, uint8_t arg2, uint8_t arg3)`

Sends a complete SD command with automatic CRC7 calculation.

**Parameters:**
- `cmd`: Command byte (with start and transmission bits)
- `arg0-arg3`: 32-bit argument (arg0 = MSB, arg3 = LSB)

**Usage:**
```c
// Send CMD0 manually
sd_cmd(0x40, 0x00, 0x00, 0x00, 0x00);
```

#### `uint8_t sd_cmd_response(uint8_t cmd, uint8_t arg0, uint8_t arg1, uint8_t arg2, uint8_t arg3)`

Sends SD command and returns R1 response.

**Returns:** R1 response byte

### Response Functions

#### `uint8_t sd_r1_response(void)`

Waits for and receives R1 response from SD card.

**Returns:**
- R1 response byte (bit 7 = 0 for valid response)
- 0xFF on timeout

#### `uint8_t sd_r1_data(uint8_t *response_buf, uint8_t data_bytes)`

Gets R1 response plus additional data bytes (used for CMD8, CMD58, etc.).

**Parameters:**
- `response_buf`: Buffer to store additional data
- `data_bytes`: Number of additional bytes to read

**Returns:** R1 response byte

### CRC Functions

#### `uint8_t sd_common_crc(uint8_t cmd, uint8_t arg0, uint8_t arg1, uint8_t arg2, uint8_t arg3)`

Returns pre-calculated CRC7 for common SD commands, or calculates dynamically.

**Optimized Commands:**
- CMD0 with arg 0x00000000
- CMD8 with arg 0x000001AA  
- CMD55 with arg 0x00000000
- ACMD41 with arg 0x40000000

### Utility Functions

#### `void sd_delay(void)`

Simple delay function for timing between SD commands.

#### `const char* sd_error_string(uint8_t error_code)`

Converts error code to human-readable string (for debugging).

**Usage:**
```c
uint8_t error = sd_init();
if (error != SD_SUCCESS) {
    printf("SD Error: %s\n", sd_error_string(error));
}
```

## Programming Examples

### Basic Initialization and Test

```c
#include <stdio.h>
#include <stdint.h>
#include <spi.h>
#include <sdcard.h>

int main(void) {
    uint8_t result;
    static uint8_t buffer[SD_BLOCK_SIZE];
    
    // Initialize SPI (slow speed for SD init)
    spi_init(0x20, 0, 0);  // Slow speed, Mode 0
    spi_cs_low();
    
    // Initialize SD card
    result = sd_init();
    if (result != SD_SUCCESS) {
        printf("SD init failed: %s\n", sd_error_string(result));
        return 1;
    }
    
    // Switch to faster speed for data operations
    spi_set_divisor(0x02);  // Faster speed
    
    // Read MBR (block 0)
    result = sd_read(0, buffer);
    if (result == SD_SUCCESS) {
        printf("MBR signature: %02X %02X\n", buffer[510], buffer[511]);
    }
    
    spi_cs_high();
    return 0;
}
```

### Read/Write Test

```c
void test_read_write(void) {
    static uint8_t write_buffer[SD_BLOCK_SIZE];
    static uint8_t read_buffer[SD_BLOCK_SIZE];
    uint8_t result;
    unsigned int i;
    
    // Create test pattern
    for (i = 0; i < SD_BLOCK_SIZE; i++) {
        write_buffer[i] = (uint8_t)(i & 0xFF);
    }
    
    // Write test block (use safe block number)
    result = sd_write(1000, write_buffer);
    if (result != SD_SUCCESS) {
        printf("Write failed: %s\n", sd_error_string(result));
        return;
    }
    
    // Read back and verify
    result = sd_read(1000, read_buffer);
    if (result != SD_SUCCESS) {
        printf("Read failed: %s\n", sd_error_string(result));
        return;
    }
    
    // Verify data
    for (i = 0; i < SD_BLOCK_SIZE; i++) {
        if (read_buffer[i] != write_buffer[i]) {
            printf("Data mismatch at byte %u\n", i);
            return;
        }
    }
    
    printf("Read/write test PASSED!\n");
}
```

### Error Handling Example

```c
uint8_t safe_sd_read(unsigned long block, uint8_t *buffer) {
    uint8_t result = sd_read(block, buffer);
    
    switch (result) {
        case SD_SUCCESS:
            return SD_SUCCESS;
            
        case SD_ERROR_CMD17:
            printf("CMD17 failed - check SPI connection\n");
            break;
            
        case SD_ERROR_READ_TOKEN:
            printf("Read token error - card may be busy\n");
            break;
            
        case SD_ERROR_READ_TIMEOUT:
            printf("Read timeout - try slower SPI speed\n");
            break;
            
        default:
            printf("Unknown read error: 0x%02X\n", result);
            break;
    }
    
    return result;
}
```

## Integration with SPI Library

The SD card library requires the SPI library. Typical initialization sequence:

```c
// Initialize SPI interface
spi_init(divisor, cpol, cpha);
spi_cs_low();           // Enable SD card

// Initialize SD card
uint8_t result = sd_init();

// Switch to faster SPI speed after initialization
if (result == SD_SUCCESS) {
    spi_set_divisor(fast_divisor);
}

// Perform SD operations...

spi_cs_high();          // Disable SD card when done
```

## Timing Considerations

### SPI Speed Settings

1. **Initialization Phase**: Use slow SPI speed (divisor 0x20 or higher)
   - SD cards require ≤400kHz during initialization
   - Some cards are sensitive to timing during init

2. **Data Transfer Phase**: Can use faster speeds after initialization
   - Most cards support several MHz for data transfer
   - Adjust based on your FPGA clock and card specifications

### Command Timing

- Always check busy status before sending commands
- Use `sd_delay()` between failed command attempts
- Some commands require longer timeout periods

## Troubleshooting

### Common Issues and Solutions

#### Initialization Fails (CMD0 Error)
- Check SPI connections (MOSI, MISO, SCK, CS)
- Verify SD card is properly inserted
- Ensure adequate power supply to SD card
- Try slower SPI speed

#### CMD8 Errors
- May indicate SD v1.x card (not currently supported)
- Check voltage levels (3.3V vs 5V compatibility)
- Verify SPI mode settings

#### Read/Write Errors
- Ensure SPI speed is not too fast for the card
- Check for proper CS (chip select) control
- Verify block numbers are within card capacity
- Some cards require alignment to erase block boundaries

#### Timeout Errors
- Increase timeout values in polling loops
- Check for proper SPI clock generation
- Ensure card is not write-protected (for write operations)

### Debug Tips

1. **Use Error Strings**: Always use `sd_error_string()` for debugging
2. **Logic Analyzer**: Monitor SPI signals for timing issues
3. **Start Simple**: Test with known-good SD cards first
4. **Incremental Speed**: Start slow and increase SPI speed gradually
5. **Check Responses**: Monitor R1 responses for card status

## Memory Usage

The library is designed for memory-constrained vintage systems:

- **Code Size**: Approximately 2-3KB compiled code
- **RAM Usage**: Minimal - no large static buffers
- **Stack Usage**: Moderate - uses local variables for command buffers

## Future Enhancements

### Planned Features

- **SD v1.x Support**: Support for older SD cards using CMD1 initialization
- **Multi-block Operations**: CMD18/CMD25 for faster data transfer
- **Card Information**: CMD9/CMD10 for card identification
- **Macro Optimization**: Convert simple functions to macros for size/speed

### Compatibility

- **Current**: SDHC (SD v2.0+) cards only
- **Planned**: SD v1.x cards
- **Not Supported**: SDXC cards (>32GB), UHS modes

## Building and Deployment

### Makefile Targets

```bash
make all        # Build sdcard.lib
make clean      # Clean build files  
make install    # Install to cc65 directories
```

### Integration in Projects

```bash
# Compile your program with the library
cc65 -O -I. -I../spi -t replica1 myprogram.c
ca65 myprogram.s
ld65 -o myprogram.bin myprogram.o sdcard.lib spi.lib replica1.lib
```

## License and Credits

This library is designed for vintage computer enthusiasts and FPGA-based recreations. It provides a solid foundation for adding modern storage capabilities to classic 8-bit systems while maintaining the authentic programming experience of the era.