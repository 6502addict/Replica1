# SPI Master Module Documentation

## Overview
The `spi_master` module is a configurable SPI (Serial Peripheral Interface) master controller implemented in VHDL. It supports both SPI modes (CPOL/CPHA combinations) and variable clock speeds through a programmable clock divider.

## Entity Declaration
```vhdl
entity spi_master is
    port (
        clk         : in     std_logic;        -- System clock
        reset_n     : in     std_logic;        -- Active low reset
        spi_req     : in     std_logic;        -- SPI transaction request
        spi_divider : in     std_logic_vector(5 downto 0);  -- Clock divider (6 bits)
        spi_busy    : out    std_logic;        -- Busy flag
        data_in     : in     std_logic_vector(7 downto 0);  -- Data to transmit
        data_out    : out    std_logic_vector(7 downto 0);  -- Received data
        cpol        : in     std_logic;        -- Clock polarity
        cpha        : in     std_logic;        -- Clock phase
        spi_sck     : out    std_logic;        -- SPI clock
        spi_cs      : out    std_logic;        -- SPI chip select (active low)
        spi_mosi    : out    std_logic;        -- Master Out Slave In
        spi_miso    : in     std_logic         -- Master In Slave Out
    );
end spi_master;
```

## Port Descriptions

### Input Ports
- **clk**: System clock input - provides the base timing for the module
- **reset_n**: Active low asynchronous reset
- **spi_req**: Transaction request signal. When this goes low while not busy, starts an SPI transaction
- **spi_divider**: 6-bit clock divider value (0-63) that determines SPI clock speed
- **data_in**: 8-bit data to be transmitted via SPI
- **cpol**: Clock polarity bit (SPI Mode configuration)
  - '0': Clock idle state is low
  - '1': Clock idle state is high
- **cpha**: Clock phase bit (SPI Mode configuration)
  - '0': Data sampled on first clock edge, changed on second
  - '1': Data changed on first clock edge, sampled on second
- **spi_miso**: Master In Slave Out - data from SPI slave device

### Output Ports
- **spi_busy**: High when SPI transaction is in progress
- **data_out**: 8-bit received data from SPI slave (valid when transaction completes)
- **spi_sck**: SPI clock output to slave device
- **spi_cs**: SPI chip select (active low during transactions)
- **spi_mosi**: Master Out Slave In - data to SPI slave device

## SPI Modes
The module supports all four standard SPI modes:

| Mode | CPOL | CPHA | Clock Idle | Data Sample Edge |
|------|------|------|------------|------------------|
| 0    | 0    | 0    | Low        | Rising           |
| 1    | 0    | 1    | Low        | Falling          |
| 2    | 1    | 0    | High       | Falling          |
| 3    | 1    | 1    | High       | Rising           |

## Clock Generation
The SPI clock is generated using a programmable clock divider:
- **spi_divider**: 6-bit value allowing division ratios from 1 to 64
- **Base clock frequency**: System clock ÷ (spi_divider + 1)
- **SPI clock frequency**: Base clock ÷ 2 (due to toggle operation)

### Example Clock Calculations
If system clock = 50 MHz:
- spi_divider = 0: SPI clock = 25 MHz
- spi_divider = 1: SPI clock = 12.5 MHz  
- spi_divider = 49: SPI clock = 500 kHz

## Operation Sequence

### Starting a Transaction
1. Ensure `spi_busy` is low
2. Set `data_in` to the byte to transmit
3. Configure `cpol`, `cpha`, and `spi_divider`
4. Pulse `spi_req` low (falling edge triggers transaction)

### During Transaction
1. `spi_busy` goes high
2. `spi_cs` goes low (active)
3. 8 bits are transmitted MSB first on `spi_mosi`
4. 8 bits are received MSB first from `spi_miso`
5. Clock toggles according to CPOL/CPHA settings

### Transaction Completion
1. `spi_busy` goes low
2. `spi_cs` goes high (inactive)
3. `data_out` contains received byte
4. `spi_sck` returns to idle state (CPOL value)

## Timing Diagram
```
spi_req   \_______/‾‾‾‾‾‾‾
spi_busy  _/‾‾‾‾‾‾‾‾‾‾‾‾‾\_
spi_cs    ‾‾\_____________/‾‾
spi_sck   __/‾\_/‾\_/‾\_/‾\__  (CPOL=0, CPHA=0 example)
spi_mosi  --<D7><D6><D5>...--
```

## Internal Architecture

### Components
- **prog_clock_divider**: Generates the base clock for SPI timing
- **State machine**: 17-step counter managing the SPI transaction
- **Shift registers**: `tx_reg` for transmission, `rx_reg` for reception

### State Machine
- **Steps 0-15**: Clock and data phases (8 bits × 2 phases each)
- **Step 16**: Transaction completion and cleanup
- **Step 17**: Reset to idle state

## Usage Notes

### Reset Behavior
- All outputs go to high-impedance ('Z') state
- Busy flag cleared
- Step counter reset to 0

### MOSI/MISO Handling
- MOSI goes high-impedance when not transmitting
- Data is transmitted MSB first
- Data is received MSB first

### Chip Select
- Active low during transactions
- Automatically controlled by busy state
- Goes active when transaction starts
- Goes inactive when transaction completes

## Integration Example
```vhdl
-- Instantiation example
spi_master_inst: spi_master
port map (
    clk         => system_clk,
    reset_n     => reset_n,
    spi_req     => start_transaction,
    spi_divider => "001100",  -- Divide by 13
    spi_busy    => transaction_busy,
    data_in     => tx_data,
    data_out    => rx_data,
    cpol        => '0',       -- SPI Mode 0
    cpha        => '0',       -- SPI Mode 0
    spi_sck     => spi_clock,
    spi_cs      => chip_select_n,
    spi_mosi    => mosi_signal,
    spi_miso    => miso_signal
);
```