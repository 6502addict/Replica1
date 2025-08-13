# SPI Master Interface Documentation

## Overview

This documentation describes the VHDL component `mspi_iface` designed to provide SPI (Serial Peripheral Interface) master functionality to vintage 8-bit processors: 6502, 6800, and 6809. The interface presents a simple memory-mapped register interface that is compatible with the bus timing and addressing modes of these classic processors.

## Component Summary

- **mspi_iface**: Allows the vintage CPU to act as SPI master (controlling the SPI bus)
- Uses an internal `spi_master` component for actual SPI protocol handling
- Provides 4-register memory-mapped interface
- Supports all SPI modes (CPOL/CPHA combinations)
- Programmable clock divider for SPI speed control

## Pin Compatibility

### CPU Bus Interface

| Signal | 6502 | 6800 | 6809 | Description |
|--------|------|------|------|-------------|
| phi2   | φ2   | E    | E    | System clock |
| reset_n| RES# | RESET# | RESET# | Reset (active low) |
| cs_n   | CS#  | CS#  | CS#  | Chip select (active low) |
| rw     | R/W# | R/W  | R/W  | Read/Write control |
| address| A1-A0| A1-A0| A1-A0| Address bits 1-0 |
| data_in| D0-7 | D0-7 | D0-7 | Data bus input |
| data_out| D0-7| D0-7 | D0-7 | Data bus output |

### SPI Interface

| Signal   | Direction | Description |
|----------|-----------|-------------|
| spi_clk  | Input     | Base clock for SPI timing generation |
| spi_sck  | Output    | SPI Serial Clock |
| spi_cs_n | Output    | SPI Chip Select (active low) |
| spi_mosi | Output    | SPI Master Out, Slave In |
| spi_miso | Input     | SPI Master In, Slave Out |

## Register Map

The interface uses 2-bit address decoding (A1-A0) providing 4 registers:

### Address 00 (0xC200): Command Register

**Write:**
```
Bit 7-3: Reserved
Bit 2:   SPI_ENABLE (1 = enable SPI CS, 0 = disable CS)
Bit 1:   CPHA (Clock Phase)
Bit 0:   CPOL (Clock Polarity)
```

**Read:** Returns current command register value
```
Bit 7-3: Reserved (read as 0)
Bit 2:   SPI_ENABLE current state
Bit 1:   CPHA current setting
Bit 0:   CPOL current setting
```

### Address 01 (0xC201): Status Register

**Write:** Not permitted (writes ignored)

**Read:**
```
Bit 7-2: Reserved (read as 0)
Bit 1:   BUSY_N (1 = SPI idle, 0 = SPI transaction in progress)
Bit 0:   DATA_READY (1 = received data available, 0 = no new data)
```

### Address 10 (0xC202): Data Register

**Write:** Starts SPI transaction with written byte (if SPI not busy)
**Read:** Returns received data from last transaction (clears DATA_READY flag)

### Address 11 (0xC203): Divider Register

**Write:** Sets SPI clock divider (8-bit value)
**Read:** Returns current divider setting

## SPI Clock Generation

The SPI clock frequency is determined by:
```
SPI_Clock = spi_clk / (2 * (divider + 1))
```

Where:
- `spi_clk` is the input base clock (typically 50MHz or similar)
- `divider` is the 8-bit value in the divider register
- Minimum divider value is 0 (fastest clock)
- Maximum divider value is 255 (slowest clock)

### Clock Divider Examples
```
Divider = 0:   SPI_Clock = spi_clk / 2    (fastest)
Divider = 1:   SPI_Clock = spi_clk / 4
Divider = 4:   SPI_Clock = spi_clk / 10
Divider = 24:  SPI_Clock = spi_clk / 50   (1MHz from 50MHz base)
Divider = 255: SPI_Clock = spi_clk / 512  (slowest)
```

## Usage Examples

### 6502 Assembly Example

```assembly
SPI_BASE    = $C200
SPI_CMD     = SPI_BASE      ; Command register
SPI_STATUS  = SPI_BASE+1    ; Status register  
SPI_DATA    = SPI_BASE+2    ; Data register
SPI_DIV     = SPI_BASE+3    ; Divider register

; Initialize SPI (Mode 0, moderate speed)
init_spi:
    LDA #25             ; Set divider for reasonable speed
    STA SPI_DIV
    
    LDA #%00000100      ; Enable SPI CS, Mode 0 (CPOL=0, CPHA=0)
    STA SPI_CMD

; Send byte and receive response
spi_transfer:
    LDA SPI_STATUS      ; Check if busy
    AND #%00000010      ; Check BUSY_N bit
    BEQ spi_transfer    ; Wait until not busy (BUSY_N=1)
    
    LDA #$55            ; Data to send
    STA SPI_DATA        ; Start transaction
    
wait_complete:
    LDA SPI_STATUS      ; Check status
    AND #%00000001      ; Check DATA_READY
    BEQ wait_complete   ; Wait for completion
    
    LDA SPI_DATA        ; Read received data
    RTS

; Disable SPI (release CS)
disable_spi:
    LDA #%00000000      ; Disable SPI CS, keep mode settings
    STA SPI_CMD
    RTS
```

### 6809 Assembly Example

```assembly
SPI_BASE    EQU $C200
SPI_CMD     EQU SPI_BASE      ; Command register
SPI_STATUS  EQU SPI_BASE+1    ; Status register
SPI_DATA    EQU SPI_BASE+2    ; Data register  
SPI_DIV     EQU SPI_BASE+3    ; Divider register

; Initialize SPI for SD card (Mode 0, slow speed for init)
init_sd_spi:
        LDA     #100          ; Slow speed for SD init
        STA     SPI_DIV
        
        LDA     #%00000100    ; Enable CS, Mode 0
        STA     SPI_CMD
        RTS

; Switch to faster speed after init
fast_spi:
        LDA     #2            ; Faster speed for data transfer
        STA     SPI_DIV
        RTS

; SPI transfer routine
spi_xfer:
        ; Input: A = byte to send
        ; Output: A = received byte
        PSHS    A             ; Save byte to send
        
wait_ready:
        LDA     SPI_STATUS    ; Check busy status
        ANDA    #%00000010    ; Isolate BUSY_N bit
        BEQ     wait_ready    ; Wait until ready
        
        PULS    A             ; Restore byte to send
        STA     SPI_DATA      ; Start transaction
        
wait_done:
        LDA     SPI_STATUS    ; Check completion
        ANDA    #%00000001    ; Check DATA_READY
        BEQ     wait_done     ; Wait for data
        
        LDA     SPI_DATA      ; Get received byte
        RTS
```

## Operation Sequence

### Standard SPI Transaction

1. **Initialize:**
   - Write divider register to set SPI clock speed
   - Write command register to set SPI mode and enable CS

2. **Transfer Data:**
   - Check BUSY_N bit in status register (must be 1 for ready)
   - Write data to data register to start transaction
   - Poll DATA_READY bit until set (transaction complete)
   - Read data register to get received byte

3. **Cleanup:**
   - Clear SPI_ENABLE bit to release CS line when done

### Multi-byte Transfers

For multiple byte transfers (like SD card commands):

```assembly
; Send multi-byte command
send_cmd:
    ; Enable SPI first
    LDA #%00000100
    STA SPI_CMD
    
    ; Send each byte
    LDA #$40        ; CMD0
    JSR spi_xfer
    LDA #$00        ; Arg byte 1
    JSR spi_xfer
    LDA #$00        ; Arg byte 2  
    JSR spi_xfer
    LDA #$00        ; Arg byte 3
    JSR spi_xfer
    LDA #$00        ; Arg byte 4
    JSR spi_xfer
    LDA #$95        ; CRC
    JSR spi_xfer
    
    ; Keep CS enabled for response
    RTS
```

## SPI Modes

The interface supports all four standard SPI modes:

| Mode | CPOL | CPHA | Clock Idle | Data Change | Data Sample |
|------|------|------|------------|-------------|-------------|
| 0    | 0    | 0    | Low        | Falling     | Rising      |
| 1    | 0    | 1    | Low        | Rising      | Falling     |
| 2    | 1    | 0    | High       | Rising      | Falling     |
| 3    | 1    | 1    | High       | Falling     | Rising      |

## Hardware Integration

### FPGA Integration

```vhdl
-- Example instantiation in top-level design
spi_master_inst: mspi_iface
port map (
    phi2     => cpu_phi2,
    reset_n  => system_reset_n,
    cs_n     => spi_cs_n,           -- Decoded from CPU address bus
    rw       => cpu_rw,
    address  => cpu_addr(1 downto 0), -- A1, A0
    data_in  => cpu_data_out,
    data_out => spi_data_to_cpu,
    spi_clk  => clk_50mhz,          -- Fast clock for SPI generation
    -- SPI interface to external devices
    spi_sck  => sd_sck,
    spi_cs_n => sd_cs_n,
    spi_mosi => sd_mosi,
    spi_miso => sd_miso
);
```

### Address Decoding

The interface requires 2-bit address decode:

```vhdl
-- Example address decoding for $C200-$C203
spi_cs_n <= '0' when (cpu_addr(15 downto 2) = "11000010000000") 
                 and (phi2 = '1') else '1';
```

## Timing Considerations

- **CPU Bus Timing:** Interface synchronizes to phi2/E clock edge
- **SPI Clock:** Generated from separate high-speed spi_clk input
- **Setup/Hold:** CPU data/address must meet standard bus timing requirements
- **SPI Speed:** Adjust divider for target device compatibility

## Application Examples

### SD Card Interface
Perfect for implementing SD card storage on vintage computers:
- Use slow speed (divider ~100) for initialization
- Switch to fast speed (divider 0-2) for data transfer
- Mode 0 (CPOL=0, CPHA=0) is standard for SD cards

### Display Controllers
Drive SPI-based LCD/OLED displays:
- Moderate speed for control commands
- Fast speed for pixel data transfer
- Mode depends on display controller chip

### Sensor Interfaces
Read from SPI sensors (temperature, accelerometer, etc.):
- Speed depends on sensor specifications
- Usually Mode 0 or Mode 3

## Troubleshooting

### Common Issues

1. **No SPI Clock:** Check spi_clk input and divider register
2. **Wrong Timing:** Verify CPOL/CPHA mode matches target device  
3. **Data Corruption:** Reduce SPI speed (increase divider value)
4. **CS Issues:** Ensure SPI_ENABLE bit is set before transfers
5. **Busy Hangs:** Always check BUSY_N before starting transfers

### Debug Tips

- Use logic analyzer to verify SPI signal timing
- Start with slow speeds and increase gradually
- Verify CS signal is properly controlled
- Check that base spi_clk frequency is appropriate
- Test with simple devices first (like shift registers)