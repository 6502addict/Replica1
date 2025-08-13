# W25Qxx Flash Library Manual

## Table of Contents
1. [Introduction](#introduction)
2. [Hardware Requirements](#hardware-requirements)
3. [Library Overview](#library-overview)
4. [API Reference](#api-reference)
5. [Utility Programs](#utility-programs)
6. [Programming Examples](#programming-examples)
7. [Troubleshooting](#troubleshooting)
8. [Apple 1 / Replica 1 Usage](#apple-1--replica-1-usage)

---

## Introduction

The W25Qxx Flash Library is a generic C library designed for interfacing with Winbond W25Q series SPI flash memory chips. It supports multiple chip variants from 2MB to 32MB capacity and is specifically optimized for use with the cc65 compiler on vintage computer systems like the Apple 1 and Replica 1.

### Supported Chips
- **W25Q16** - 2MB (16Mbit)
- **W25Q32** - 4MB (32Mbit)  
- **W25Q64** - 8MB (64Mbit)
- **W25Q128** - 16MB (128Mbit)
- **W25Q256** - 32MB (256Mbit)

### Key Features
- Automatic chip detection and configuration
- Unified API for all supported chip variants
- 3-byte and 4-byte addressing support (automatic for W25Q256)
- Page, sector, and block erase operations
- Read/write operations with automatic page boundary handling
- Power management functions
- cc65 compiler compatibility

---

## Hardware Requirements

### SPI Interface
The library requires a 4-wire SPI interface:
- **MOSI** (Master Out, Slave In) - Data from controller to flash
- **MISO** (Master In, Slave Out) - Data from flash to controller  
- **SCK** (Serial Clock) - Clock signal
- **CS** (Chip Select) - Active low chip select

### SPI Configuration
- **Mode**: SPI Mode 0 (CPOL=0, CPHA=0)
- **Speed**: Up to 104MHz (typically 1-8MHz for vintage systems)
- **Bit Order**: MSB first

### Power Supply
- **Voltage**: 2.7V to 3.6V (typically 3.3V)
- **Current**: ~15mA active, ~1µA deep power-down

---

## Library Overview

### File Structure
```
w25qxx.h        - Library header file
w25qxx.c        - Library implementation
spi.h           - SPI driver header (user provided)
spi.c           - SPI driver implementation (user provided)
```

### Dependencies
The library requires a user-provided SPI driver with these functions:
```c
void spi_init(uint8_t prescaler, uint8_t cpol, uint8_t cpha);
uint8_t spi_transfer(uint8_t data);
void spi_cs_low(void);
void spi_cs_high(void);
```

### Memory Organization
All W25Qxx chips use the same memory organization:
- **Page Size**: 256 bytes (programmable unit)
- **Sector Size**: 4KB (minimum erase unit)
- **Block Size**: 64KB (fast erase unit)

---

## API Reference

### Initialization

#### `uint8_t w25qxx_init(void)`
Initializes the flash chip and detects the chip type.
- **Returns**: 0 on success, 1 on failure
- **Notes**: Must be called before any other library functions

#### `w25qxx_chip_t w25qxx_get_chip_type(void)`
Returns the detected chip type.
- **Returns**: Chip type enumeration (W25Q16, W25Q32, etc.)

#### `uint32_t w25qxx_get_chip_size(void)`
Returns the chip capacity in bytes.
- **Returns**: Chip size in bytes

### Read Operations

#### `void w25qxx_read(uint32_t address, uint8_t* buffer, uint16_t length)`
Reads data from flash memory.
- **address**: Starting address (0 to chip_size-1)
- **buffer**: Destination buffer
- **length**: Number of bytes to read

#### `void w25qxx_fast_read(uint32_t address, uint8_t* buffer, uint16_t length)`
Fast read with dummy byte (higher speed).
- **address**: Starting address
- **buffer**: Destination buffer  
- **length**: Number of bytes to read

### Write Operations

#### `uint8_t w25qxx_write(uint32_t address, const uint8_t* buffer, uint16_t length)`
Writes data to flash memory (handles page boundaries automatically).
- **address**: Starting address
- **buffer**: Source data
- **length**: Number of bytes to write
- **Returns**: 0 on success, 1 on failure

#### `uint8_t w25qxx_write_page(uint32_t address, const uint8_t* buffer, uint16_t length)`
Writes a single page (max 256 bytes).
- **address**: Page-aligned address (recommended)
- **buffer**: Source data
- **length**: Number of bytes to write (max 256)
- **Returns**: 0 on success, 1 on failure

### Erase Operations

#### `uint8_t w25qxx_erase_sector(uint32_t address)`
Erases a 4KB sector.
- **address**: Any address within the sector
- **Returns**: 0 on success, 1 on failure
- **Time**: Up to 400ms

#### `uint8_t w25qxx_erase_block_64k(uint32_t address)`
Erases a 64KB block.
- **address**: Any address within the block
- **Returns**: 0 on success, 1 on failure
- **Time**: Up to 2 seconds

#### `uint8_t w25qxx_erase_chip(void)`
Erases the entire chip.
- **Returns**: 0 on success, 1 on failure
- **Time**: Up to 100 seconds (chip dependent)

### Status and Utility Functions

#### `uint8_t w25qxx_is_busy(void)`
Checks if chip is busy with an operation.
- **Returns**: 1 if busy, 0 if ready

#### `uint8_t w25qxx_get_status(void)`
Reads status register 1.
- **Returns**: Status register value

#### `uint8_t w25qxx_is_valid_address(uint32_t address)`
Validates an address for the current chip.
- **address**: Address to validate
- **Returns**: 1 if valid, 0 if invalid

#### `uint32_t w25qxx_get_max_address(void)`
Returns the maximum valid address.
- **Returns**: Highest valid address (chip_size - 1)

---

## Utility Programs

### flash-id.mon - Flash Identification Tool

**Purpose**: Identifies connected flash chip and displays detailed information.

**Usage on Apple 1/Replica 1**:

starts automatically if not  press reset and type
```
0300R                ; Start program at $8000
```

**Output Example**:
```
W25Qxx Flash Identification Tool
================================

Initializing flash library...
SUCCESS: Flash chip detected and initialized!

CHIP INFORMATION:
-----------------
Chip Model: W25Q32
Capacity: 4194304 bytes (4096 KB / 4 MB)
Addressing: 3-byte mode

MEMORY ORGANIZATION:
--------------------
Total Size: 4194304 bytes
Pages: 16384 (256 bytes each)
Sectors: 1024 (4096 bytes each)
64KB Blocks: 64 (65536 bytes each)

STATUS REGISTERS:
-----------------
Status 1: 0x00 (Busy: NO, WEL: NO)
Status 2: 0x00
Status 3: 0x00

Flash identification complete.
```

### flash-erase.mon - Complete Chip Erase Tool

**Purpose**: Completely erases all data on the flash chip.

**Safety Features**:
- Multiple warning messages
- Two-stage confirmation process
- Progress indication during erase
- Verification of erase completion

**Usage**:
```
R                    ; Reset system
8000R                ; Start program
```

**Interactive Process**:
1. Displays chip information
2. Shows multiple warnings about data loss
3. Requires typing "yes" exactly
4. Requires pressing "Y" for final confirmation
5. Performs erase with progress indication
6. Verifies erase completion

**⚠️ WARNING**: This tool will permanently erase ALL data on the chip!

### flash-dump.mon - Memory Dump Viewer

**Purpose**: Displays flash memory content in hexadecimal format with ASCII representation.

**Features**:
- Block-based viewing (64KB blocks)
- Hexadecimal and ASCII display
- Interactive menu system
- Support for multiple consecutive blocks
- Early exit option when viewing multiple blocks

**Usage**:
```
R                    ; Reset system
8000R                ; Start program
```

**Commands**:
- `0` - Dump block 0 only
- `5 3` - Dump 3 blocks starting from block 5
- `quit` or `exit` - Exit program

**Output Format**:
```
00000000: FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF  |................|
00000010: 48 65 6C 6C 6F 20 57 6F 72 6C 64 21 00 FF FF FF  |Hello World!....|
00000020: FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF  |................|
```

---

## Programming Examples

### Basic Flash Operations

```c
#include "w25qxx.h"

int main(void) {
    uint8_t buffer[256];
    uint8_t result;
    
    // Initialize flash
    if (w25qxx_init() != 0) {
        printf("Flash init failed!\n");
        return 1;
    }
    
    // Read data
    w25qxx_read(0x1000, buffer, 256);
    
    // Erase sector before writing
    result = w25qxx_erase_sector(0x1000);
    if (result != 0) {
        printf("Erase failed!\n");
        return 1;
    }
    
    // Write data
    strcpy((char*)buffer, "Hello, World!");
    result = w25qxx_write(0x1000, buffer, strlen((char*)buffer) + 1);
    if (result != 0) {
        printf("Write failed!\n");
        return 1;
    }
    
    return 0;
}
```

### Chip Information Display

```c
void display_chip_info(void) {
    w25qxx_chip_t chip_type;
    uint32_t chip_size;
    
    chip_type = w25qxx_get_chip_type();
    chip_size = w25qxx_get_chip_size();
    
    printf("Chip: W25Q%d\n", chip_type);
    printf("Size: %lu bytes\n", chip_size);
    printf("Max Address: 0x%08lX\n", w25qxx_get_max_address());
    printf("Ready: %s\n", w25qxx_is_busy() ? "NO" : "YES");
}
```

### Safe Write with Verification

```c
uint8_t safe_write(uint32_t address, const uint8_t* data, uint16_t length) {
    uint8_t verify_buffer[256];
    uint8_t result;
    
    // Validate address
    if (!w25qxx_is_valid_address(address + length - 1)) {
        return 1; // Invalid address range
    }
    
    // Erase sector if writing to start of sector
    if ((address & 0xFFF) == 0) {
        result = w25qxx_erase_sector(address);
        if (result != 0) return result;
    }
    
    // Write data
    result = w25qxx_write(address, data, length);
    if (result != 0) return result;
    
    // Verify write
    w25qxx_read(address, verify_buffer, length);
    if (memcmp(data, verify_buffer, length) != 0) {
        return 1; // Verification failed
    }
    
    return 0; // Success
}
```

---

## Troubleshooting

### Common Issues

#### Flash Not Detected
**Symptoms**: `w25qxx_init()` returns 1
**Causes**:
- SPI wiring problems
- Incorrect power supply voltage
- Wrong SPI mode configuration
- Timing issues

**Solutions**:
1. Check all SPI connections (MOSI, MISO, SCK, CS)
2. Verify 3.3V power supply
3. Ensure SPI Mode 0 configuration
4. Reduce SPI clock speed
5. Check for proper pull-up on CS line

#### Read/Write Failures
**Symptoms**: Data corruption or write failures
**Causes**:
- Address out of range
- Writing to protected sectors
- SPI timing issues
- Power supply noise

**Solutions**:
1. Validate addresses with `w25qxx_is_valid_address()`
2. Check status registers for protection bits
3. Add delay between operations
4. Improve power supply filtering

#### Erase Operation Timeouts
**Symptoms**: Operations never complete
**Causes**:
- Chip write-protection enabled
- Hardware write-protect pin asserted
- Power supply issues during erase

**Solutions**:
1. Check write-protect pin (should be high)
2. Verify status registers
3. Ensure stable power during erase
4. Add timeout detection in application

### Debugging Tips

1. **Use flash-id.mon** to verify chip detection and status
2. **Check status registers** for error conditions
3. **Start with small operations** before attempting large transfers
4. **Add timeouts** to prevent infinite loops
5. **Verify SPI driver** with known-good devices first

---

## Apple 1 / Replica 1 Usage

### Memory Map Considerations

When using on Apple 1 or Replica 1 systems:
- **Program Location**: Typically load at $8000 or higher
- **Variable Storage**: Be mindful of limited RAM
- **Stack Usage**: Monitor stack depth for large operations

### WozMon Loading

The utility programs are compiled as `.mon` files for easy loading with WozMon:

1. **Connect terminal** to Apple 1/Replica 1
2. **Reset system**: Press reset or type `R`
3. **Load program**: Use WozMon's load command or paste hex data
4. **Run program**: Type address followed by `R` (e.g., `8000R`)

### Example Loading Session

```
Apple 1 - WozMon
R

8000: A9 48 8D 00 02 A9 65 8D 01 02 ...
8010: 6C 00 02 00 00 00 00 00 00 00 ...
...
8000R

W25Qxx Flash Identification Tool
================================
```

### Performance Considerations

On vintage systems with limited processing power:
- **Use appropriate SPI speeds** (1-4MHz typically safe)
- **Break large operations** into smaller chunks
- **Add progress indicators** for long operations
- **Consider user interrupt** capabilities for long erase operations

### Building for Apple 1

Using cc65 to build for Apple 1:

```bash
# Compile the library
cc65 -t apple1 -O w25qxx.c
ca65 w25qxx.s

# Compile application
cc65 -t apple1 -O flash-id.c
ca65 flash-id.s

# Link
ld65 -t apple1 -o flash-id.mon flash-id.o w25qxx.o apple1.lib
```

---

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025 | Initial release with W25Q16/32/64/128/256 support |

---

## License

This library is provided as-is for educational and hobbyist use. Suitable for vintage computer restoration and FPGA projects.

---

## Support

For technical support and questions:
- Verify hardware connections first
- Use the diagnostic tools (flash-id.mon) to isolate issues
- Check the troubleshooting section for common problems
- Ensure proper SPI driver implementation for your system