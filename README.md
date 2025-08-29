# Replica1 - Apple 1 Clone in VHDL

A complete VHDL implementation of the Replica 1, which is a faithful clone of the original Apple 1 computer designed by Steve Wozniak in 1976.

## ChangeLog 
- 15/08/25 Added DE1 Support
- 29/08/25 Added John kent 6809 core
- 29/08/25 Added a simple 6809 Monitor in software/mon6809
- 29/08/25 Added Wozmon sources code in software/wozmon    wozmon for 6809 derived from wozmon 6800 is still buggy and non working
       
## Overview

This project recreates the Apple 1 computer using modern FPGA technology while maintaining the original functionality and behavior. The implementation is designed for the DE10-Lite FPGA board and features a configurable CPU core that supports both 6502 and 6800 processors.

## Features

- **Dual CPU Support**: Configurable 6502 or 6800 microprocessor implementation
- **Flexible Memory**: Configurable RAM size from 8KB to 48KB
- **Multiple Clock Speeds**: Debug, 1Hz, 1MHz, 2MHz, 5MHz, 10MHz, 15MHz, 30MHz
- **Serial Interface**: UART communication with configurable baud rates (1200-115200)
- **SD Card Support**: SPI-based storage via Arduino shield connector
- **Apple Cassette Interface (ACI)**: Optional tape interface support
- **Hardware Timer**: Optional timer peripheral
- **Apple BASIC**: Optional BASIC interpreter or Woz Monitor only
- **Real-time Debugging**: 7-segment displays show current address and data bus

## Target Hardware

- **FPGA Board**: Intel DE10-Lite (MAX 10)  or Altera DE1 board (Cyclone 2)
- **Display**: 7-segment displays for address/data bus monitoring
- **Console**: UART terminal interface (requires FTDI USB-to-Serial cable)
- **Storage**: SD card via Arduino-compatible shield
- **Communication**: 115200 baud, 8 bits, no parity, 1 stop bit
- **Input**: Switches and buttons for configuration and control
- **Optional**: Cassette tape interface for authentic program loading

**Important**: This is a headless system with no video output. All interaction is through serial terminal.

## Getting Started

### Prerequisites

- Intel Quartus Prime (Lite Edition recommended)
- DE10-Lite FPGA development board
- SD card and Arduino-compatible shield (optional)
- Serial terminal software (PuTTY, screen, etc.)
- USB cable for programming and serial communication

### Building the Project

1. **Clone the repository**:
   ```bash
   git clone https://github.com/6502addict/Replica1.git
   cd Replica1
   ```

2a. **For DE10-Lite Open in Quartus Prime**:
   - Launch Quartus Prime
   - Navigate to `boards/DE10-Lite/`
   - Open the project file `DE10_Replica1.qpf`
   - All VHDL files and IP cores are properly included

2b. **For DE1 Open in Quartus 13**:
   - Launch Quartus Prime
   - Navigate to `boards/DE1/`
   - Open the project file `DE1_Replica1.qpf`
   - All VHDL files and IP cores are properly included

3. **Compile the project**:
   - Click "Start Compilation" or use Ctrl+L
   - Wait for synthesis and place & route to complete

4. **Program the FPGA**:
   - Connect DE10-Lite via USB
   - Use "Program Device" to load the .sof file
   - The Replica 1 should boot immediately

### Hardware Connections

#### Arduino Shield Connections
The system uses the Arduino-compatible headers on the DE10-Lite:

**Console Interface (UART)**:
- RX ← ARDUINO_IO(0) (D0)
- TX → ARDUINO_IO(1) (D1)
- Connection: FTDI USB-to-Serial cable at 115200 baud, 8N1

**SD Card (SPI Interface)**:
- CS   → ARDUINO_IO(4)  (D4)
- MOSI → ARDUINO_IO(11) (D11)
- MISO ← ARDUINO_IO(12) (D12)
- SCLK → ARDUINO_IO(13) (D13)

**Cassette Interface (Optional)**:
- Tape Out → ARDUINO_IO(3) (D3)
- Tape In  ← ARDUINO_IO(2) (D2)

#### Control Interface
- **KEY(0)**: System reset
- **KEY(1)**: Debug clock step (when in debug mode)
- **SW(2:0)**: Clock speed selection
  - 000: Debug (manual stepping)
  - 001: 1Hz
  - 010: 1MHz
  - 011: 2MHz
  - 100: 5MHz
  - 101: 10MHz
  - 110: 15MHz
  - 111: 30MHz
- **HEX5-0**: Real-time display of address bus (HEX5-2) and data bus (HEX1-0)

## Usage

### Power-On and Reset
1. Program the FPGA using Quartus Prime Programmer
2. Connect FTDI USB-to-Serial cable to ARDUINO_IO(0) and ARDUINO_IO(1)
3. Open serial terminal (115200 baud, 8 bits, no parity, 1 stop bit)
4. Press KEY(0) to reset the system
5. The system will display the familiar Apple 1 prompt in your terminal:
   ```
   \
   ```

**Important**: This system uses a UART console interface like the PIA on the original Replica 1, not a video display. All interaction is through the serial terminal.

### Clock Speed Control
Use switches SW(2:0) to select CPU clock speed:
- Start with 001 (1Hz) for initial testing
- Use 010 (1MHz) for authentic Apple 1 speed
- Higher speeds available for faster program execution
- Use 000 (debug) with KEY(1) for single-step debugging

### Programming and Operation
- **Woz Monitor**: Built-in monitor for memory examination and program entry
- **BASIC**: If enabled, type `E000R` to start Apple BASIC
- **Serial Loading**: Upload programs via terminal
- **SD Card**: Store and load programs from SD card (if SPI enabled)
- **Real-time Monitoring**: Watch address and data buses on 7-segment displays

### Example Programs
The system is compatible with original Apple 1 software:
- Woz Monitor commands
- Apple 1 BASIC programs
- Period-appropriate games and utilities

## Memory Map

| Address Range | Description |
|---------------|-------------|
| $0000-$BFFF   | RAM (48KB default, configurable) |
| $C000-$C1FF   | ACI - Apple Cassette Interface (if enabled) |
| $C200-$C20F   | SPI Master Controller (if enabled) |
| $C210-$C21F   | Hardware Timer (if enabled) |
| $D010-$D01F   | PIA - Console Interface (UART) |
| $D000-$DFFF   | I/O Space |
| $E000-$EFFF   | BASIC ROM (if enabled) |
| $FF00-$FFFF   | Woz Monitor ROM |

**Note**: Memory map may vary based on configuration options and enabled peripherals.

## Technical Details

## Technical Details

### Clock Generation
- **Base Clock**: 50MHz from MAX10_CLK1_50
- **PLL Generated**: 30MHz main clock
- **CPU Clocks**: Selectable from debug to 30MHz via switch-controlled multiplexer
- **Serial Clock**: 1.8432MHz fractional divider for accurate UART timing

### CPU Implementation
- **6502 Mode**: Full 6502 instruction set with cycle-accurate timing
- **6800 Mode**: Complete 6800 instruction set support
- **Bus Interface**: 16-bit address, 8-bit data, standard RW control
- **Interrupt Support**: NMI and IRQ handling

### I/O System
- **Console (PIA)**: UART-based terminal interface at $D010-$D01F
- **ACI**: Apple Cassette Interface at $C000-$C1FF (if enabled)
- **SPI Master**: SD card controller at $C200-$C20F (if enabled)  
- **Timer**: Programmable interval timer at $C210-$C21F (if enabled)
- **Debug Interface**: Real-time bus monitoring via 7-segment displays

**Note**: There is NO video system - all console I/O is handled through the UART interface, just like the PIA on the original Replica 1.

## File Structure

```
Replica1/
├── boards/
│   └── DE10-Lite/         # Complete DE10-Lite build files
│       ├── DE10_Replica1.vhd      # Top-level board file
│       ├── DE10_Replica1.qpf      # Quartus project file
│       ├── DE10_Replica1.qsf      # Quartus settings & pin assignments
│       ├── DE10_Replica1.sdc      # Timing constraints
│       ├── RAM_DE10.vhd           # Board-specific RAM wrapper
│       ├── main_clock.*           # PLL clock generation IP
│       └── ram_8k.*               # 8K RAM block IP
├── rtl/
│   ├── core/
│   │   └── Replica1_CORE.vhd      # Main system core
│   ├── cpu/
│   │   ├── CPU_65XX.vhd           # 6502 CPU implementation
│   │   ├── CPU_6800.vhd           # 6800 CPU implementation
│   │   ├── cpu65xx.vhd            # 6502 variant
│   │   └── cpu68.vhd              # 6800 variant
│   ├── peripherals/
│   │   ├── aci/
│   │   │   └── aci.vhd            # Apple Cassette Interface
│   │   ├── mspi/
│   │   │   └── mspi-iface.vhd     # SPI master controller
│   │   ├── pia/
│   │   │   ├── pia_uart.vhd       # Console UART (PIA)
│   │   │   ├── uart_receive.vhd   # UART RX
│   │   │   └── uart_send.vhd      # UART TX
│   │   └── timer/
│   │       └── simple_timer.vhd   # Hardware timer
│   ├── rom/
│   │   ├── BASIC.vhd              # Apple BASIC ROM
│   │   ├── WOZACI.vhd             # Woz Monitor with ACI
│   │   ├── WOZMON65.vhd           # Woz Monitor for 6502
│   │   └── WOZMON68.vhd           # Woz Monitor for 6800
│   └── utils/
│       ├── clock_divider.vhd      # Clock division utilities
│       ├── debug_clock_button.vhd # Debug clock control
│       ├── fractional_clock_divider.vhd # UART clock gen
│       ├── hexto7seg.vhd          # 7-segment display driver
│       ├── simple_clock_switch.vhd # Clock multiplexer
│       ├── spi-master.vhd         # SPI master implementation
│       └── [other utility modules]
```

## Contributing

Contributions are welcome! Please feel free to submit issues, feature requests, or pull requests.

### Development Guidelines
1. Follow existing VHDL coding style
2. Test thoroughly with simulation before synthesis
3. Document any changes to the memory map or interfaces
4. Maintain compatibility with original Apple 1 software

## Resources

- [Apple 1 Operation Manual](https://www.apple1.org/)
- [6502 Instruction Set Reference](http://www.6502.org/)
- [Woz Monitor Source Code](https://github.com/jefftranter/6502/tree/master/asm/wozmon)
- [Apple 1 Registry](https://apple1registry.com/)

## License

This project is open source. See LICENSE file for details.
for files added in software or firmware included in vhdl files check:
https://github.com/6502addict/Replica1/blob/main/License%20for%20added%20or%20included%20files

## Acknowledgments

- Steve Wozniak for the original Apple 1 design
- Vince Briel for the Replica 1 hardware design
- The 6502.org community for extensive documentation
- All contributors to Apple 1 preservation efforts

## Contact

For questions or support, please open an issue on GitHub or contact the project maintainer.

---

*"The Apple 1 was the foundation of everything that followed. This VHDL implementation keeps that legacy alive in modern FPGAs."*
