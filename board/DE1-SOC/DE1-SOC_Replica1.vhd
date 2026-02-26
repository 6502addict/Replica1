--------------------------------------------------------------------------
-- To configure the machine search "Board Configuration Parameters"
-- and adapt the configuration to your needs 
--------------------------------------------------------------------------

-- Notes: ------------------------------------------------------------------------------
-- if the builtin usb serial port is failing
-- make sure that these lines are in your .qsf file
-- set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to UART_TX
-- set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to UART_RX
-- set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to UART_RTS
-- set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to UART_CTS
--
-- if still failing use a 3.3v ftdi cable connected to GPIO
-- check the SERIAL_PORT constant and set it to "BUILTIN" to use the DE1-SOC serial port
-- or set it to "GPIO" to use a ftdi cable connected to GPIO_1(0) and GPIO_1(1)
----------------------------------------------------------------------------------------
 

library IEEE;
    use IEEE.std_logic_1164.all;
    use ieee.numeric_std.all;
    use ieee.math_real.all;


entity DE1_SoC_Replica1 is
    port (
        --------- ADC ---------
        ADC_CONVST       : out   std_logic;
        ADC_DIN          : out   std_logic;
        ADC_DOUT         : in    std_logic;
        ADC_SCLK         : out   std_logic;

        --------- AUD ---------
        AUD_ADCDAT       : in    std_logic;
        AUD_ADCLRCK      : inout std_logic;
        AUD_BCLK         : inout std_logic;
        AUD_DACDAT       : out   std_logic;
        AUD_DACLRCK      : inout std_logic;
        AUD_XCK          : out   std_logic;

        --------- CLOCKS ---------
        CLOCK2_50        : in    std_logic;
        CLOCK3_50        : in    std_logic;
        CLOCK4_50        : in    std_logic;
        CLOCK_50         : in    std_logic;

        --------- DRAM ---------
        DRAM_ADDR        : out   std_logic_vector(12 downto 0);
        DRAM_BA          : out   std_logic_vector(1 downto 0);
        DRAM_CAS_N       : out   std_logic;
        DRAM_CKE         : out   std_logic;
        DRAM_CLK         : out   std_logic;
        DRAM_CS_N        : out   std_logic;
        DRAM_DQ          : inout std_logic_vector(15 downto 0);
        DRAM_LDQM        : out   std_logic;
        DRAM_RAS_N       : out   std_logic;
        DRAM_UDQM        : out   std_logic;
        DRAM_WE_N        : out   std_logic;

        --------- FAN ---------
        FAN_CTRL         : out   std_logic;

        --------- FPGA I2C ---------
        FPGA_I2C_SCLK    : out   std_logic;
        FPGA_I2C_SDAT    : inout std_logic;

        --------- GPIO ---------
        GPIO_0           : inout std_logic_vector(35 downto 0);
        GPIO_1           : inout std_logic_vector(35 downto 0);

        --------- HEX Displays ---------
        HEX0             : out   std_logic_vector(6 downto 0);
        HEX1             : out   std_logic_vector(6 downto 0);
        HEX2             : out   std_logic_vector(6 downto 0);
        HEX3             : out   std_logic_vector(6 downto 0);
        HEX4             : out   std_logic_vector(6 downto 0);
        HEX5             : out   std_logic_vector(6 downto 0);

        -- Note: VHDL does not have a direct equivalent to Verilog `ifdef.
        -- These HPS ports are included here; comment them out if HPS is disabled.
        --------- HPS ---------
--        HPS_CONV_USB_N   : inout std_logic;
--        HPS_DDR3_ADDR    : out   std_logic_vector(14 downto 0);
--        HPS_DDR3_BA      : out   std_logic_vector(2 downto 0);
--        HPS_DDR3_CAS_N   : out   std_logic;
--        HPS_DDR3_CKE     : out   std_logic;
--        HPS_DDR3_CK_N    : out   std_logic;
--        HPS_DDR3_CK_P    : out   std_logic;
--        HPS_DDR3_CS_N    : out   std_logic;
--        HPS_DDR3_DM      : out   std_logic_vector(3 downto 0);
--        HPS_DDR3_DQ      : inout std_logic_vector(31 downto 0);
--        HPS_DDR3_DQS_N   : inout std_logic_vector(3 downto 0);
--        HPS_DDR3_DQS_P   : inout std_logic_vector(3 downto 0);
--        HPS_DDR3_ODT     : out   std_logic;
--        HPS_DDR3_RAS_N   : out   std_logic;
--        HPS_DDR3_RESET_N : out   std_logic;
--        HPS_DDR3_RZQ     : in    std_logic;
--        HPS_DDR3_WE_N    : out   std_logic;
--        HPS_ENET_GTX_CLK : out   std_logic;
--        HPS_ENET_INT_N   : inout std_logic;
--        HPS_ENET_MDC     : out   std_logic;
--        HPS_ENET_MDIO    : inout std_logic;
--        HPS_ENET_RX_CLK  : in    std_logic;
--        HPS_ENET_RX_DATA : in    std_logic_vector(3 downto 0);
--        HPS_ENET_RX_DV   : in    std_logic;
--        HPS_ENET_TX_DATA : out   std_logic_vector(3 downto 0);
--        HPS_ENET_TX_EN   : out   std_logic;
--        HPS_FLASH_DATA   : inout std_logic_vector(3 downto 0);
--        HPS_FLASH_DCLK   : out   std_logic;
--        HPS_FLASH_NCSO   : out   std_logic;
--        HPS_GSENSOR_INT  : inout std_logic;
--        HPS_I2C1_SCLK    : inout std_logic;
--        HPS_I2C1_SDAT    : inout std_logic;
--        HPS_I2C2_SCLK    : inout std_logic;
--        HPS_I2C2_SDAT    : inout std_logic;
--        HPS_I2C_CONTROL  : inout std_logic;
--        HPS_KEY          : inout std_logic;
--        HPS_LED          : inout std_logic;
--        HPS_LTC_GPIO     : inout std_logic;
--        HPS_SD_CLK       : out   std_logic;
--        HPS_SD_CMD       : inout std_logic;
--        HPS_SD_DATA      : inout std_logic_vector(3 downto 0);
--        HPS_SPIM_CLK     : out   std_logic;
--        HPS_SPIM_MISO    : in    std_logic;
--        HPS_SPIM_MOSI    : out   std_logic;
--        HPS_SPIM_SS      : inout std_logic;
--        HPS_UART_RX      : in    std_logic;
--        HPS_UART_TX      : out   std_logic;
--        HPS_USB_CLKOUT   : in    std_logic;
--        HPS_USB_DATA     : inout std_logic_vector(7 downto 0);
--        HPS_USB_DIR      : in    std_logic;
--        HPS_USB_NXT      : in    std_logic;
--        HPS_USB_STP      : out   std_logic;

        --------- IRDA ---------
        IRDA_RXD         : in    std_logic;
        IRDA_TXD         : out   std_logic;

        --------- KEY ---------
        KEY              : in    std_logic_vector(3 downto 0);

        --------- LEDR ---------
        LEDR             : out   std_logic_vector(9 downto 0);

        --------- PS2 ---------
        PS2_CLK          : inout std_logic;
        PS2_CLK2         : inout std_logic;
        PS2_DAT          : inout std_logic;
        PS2_DAT2         : inout std_logic;

        --------- SW ---------
        SW               : in    std_logic_vector(9 downto 0);

        --------- TD ---------
        TD_CLK27         : in    std_logic;
        TD_DATA          : in    std_logic_vector(7 downto 0);
        TD_HS            : in    std_logic;
        TD_RESET_N       : out   std_logic;
        TD_VS            : in    std_logic;

        --------- VGA ---------
        VGA_B            : out   std_logic_vector(7 downto 0);
        VGA_BLANK_N      : out   std_logic;
        VGA_CLK          : out   std_logic;
        VGA_G            : out   std_logic_vector(7 downto 0);
        VGA_HS           : out   std_logic;
        VGA_R            : out   std_logic_vector(7 downto 0);
        VGA_SYNC_N       : out   std_logic;
        VGA_VS           : out   std_logic;

        --------- UART ---------
        UART_TX          : out    std_logic;
        UART_RX          : in     std_logic;
        UART_RTS         : out   std_logic;
        UART_CTS         : in    std_logic;

        --------- QSPI ---------
        QSPI_FLASH_SCLK  : out   std_logic;
        QSPI_FLASH_DATA  : inout std_logic_vector(3 downto 0);
        QSPI_FLASH_CE_n  : out   std_logic

        --------- RISC-V JTAG ---------
--        RISCV_JTAG_TCK   : in    std_logic;
--        RISCV_JTAG_TDI   : in    std_logic;
--        RISCV_JTAG_TDO   : out   std_logic;
--        RISCV_JTAG_TMS   : in    std_logic
    );
end entity DE1_SoC_Replica1;


architecture top of DE1_SOC_Replica1 is

component hexto7seg is
    generic (SEGMENTS : integer := 8);
    port (
        hex         : in   std_logic_vector(3 downto 0);
        seg         : out  std_logic_vector(SEGMENTS-1 downto 0)
    );
end component;

component fractional_clock_divider is
    generic (
        CLK_FREQ_HZ     : positive := 50_000_000;  
        FREQUENCY_HZ    : positive := 1_843_200      
    );
    port (
        clk_in   : in  std_logic;  
        reset_n  : in  std_logic;  
        clk_out  : out std_logic   
    );
end component;

component clock_divider IS
    generic (divider : integer := 4);
    port (
        reset    : in  std_logic := '1';
        clk_in   : in  std_logic;
        clk_out  : out std_logic
    );
end component;

component pll_clock is
    port (
        refclk   : in  std_logic := 'X'; -- clk
        rst      : in  std_logic := 'X'; -- reset
        outclk_0 : out std_logic;        -- clk
        outclk_1 : out std_logic;        -- clk
        locked   : out std_logic         -- export
    );
end component;

component frac_clk_div is
    port (
        reset_n  : in  std_logic := '1';
        clk_in   : in  std_logic;
        divider  : in  std_logic_vector(15 downto 0);  -- [15:8] integer part, [7:0] fractional part (*256)
        clk_out  : out std_logic
    );
end component;

component Replica1_CORE is
    generic (
        CPU_TYPE        : string  :=  "6502";        -- 6502, 65C02, 6800 or 6809
        CPU_CORE        : string  :=  "65XX";        -- 65XX, T65, MX65 
        ROM             : string  :=  "WOZMON65";    -- default wozmon65
        RAM_SIZE_KB     : integer :=  8;             -- 8 to 48kb
        BAUD_RATE       : integer :=  115200;        -- uart speed 1200 to 115200
        HAS_ACI         : boolean :=  false;         -- add the aci (incomplete)
        HAS_MSPI        : boolean :=  false;         -- add master spi  C200
        HAS_TIMER       : boolean :=  false          -- add basic timer
    );
    port (
        main_clk        : in     std_logic;
        serial_clk      : in     std_logic;
        reset_n         : in     std_logic;
        cpu_reset_n     : in     std_logic;
        bus_phi2        : out    std_logic;
        bus_address     : out    std_logic_vector(15 downto 0);
        bus_data        : out    std_logic_vector(7  downto 0);
        bus_rw          : out    std_logic;
        bus_mrdy        : in     std_logic;
        ext_ram_cs_n    : out    std_logic;
        ext_ram_data    : in     std_logic_vector(7  downto 0);
        ext_tram_cs_n   : out    std_logic;
        ext_tram_data   : in     std_logic_vector(7  downto 0);
        uart_rx         : in     std_logic;
        uart_tx         : out    std_logic;
        spi_cs          : out    std_logic;
        spi_sck         : out    std_logic;
        spi_mosi        : out    std_logic;
        spi_miso        : in     std_logic;
        tape_out        : out    std_logic;
        tape_in         : in     std_logic
    );
end component;

component sdram_controller is
    generic (
        FREQ_MHZ           : integer := 100;   -- Clock frequency in MHz
        ROW_BITS           : integer := 13;    -- 13 for DE10-Lite, 12 for DE1
        COL_BITS           : integer := 10;    -- 10 for DE10-Lite, 8 for DE1
        TRP_NS             : integer := 20;    -- Precharge time (for PRECHARGE wait)
        TRCD_NS            : integer := 20;    -- RAS to CAS delay (for ACTIVE→READ/WRITE)
        TRFC_NS            : integer := 70;    -- Refresh cycle time (for AUTO REFRESH wait)
        CAS_LATENCY        : integer := 2;     -- CAS Latency: 2 or 3 cycles
        USE_AUTO_PRECHARGE : boolean := true;  -- true = READA/WRITEA false = READ/WRITE
        USE_AUTO_REFRESH   : boolean := true   -- true = autorefresh, false = triggered refresh
    );
    port(
        clk                : in    std_logic;  
        reset_n            : in    std_logic;  
        
        -- Simple CPU interface
        req                : in    std_logic;
        wr_n               : in    std_logic;  
        addr               : in    std_logic_vector(ROW_BITS+COL_BITS+1 downto 0); 
        din                : in    std_logic_vector(15 downto 0);
        dout               : out   std_logic_vector(15 downto 0);
        byte_en            : in    std_logic_vector(1 downto 0);  
        ready              : out   std_logic;
        ack                : out   std_logic;
        refresh_req        : in   std_logic;
        refresh_active     : out   std_logic;  
        
        -- SDRAM pins
        sdram_clk          : out   std_logic;
        sdram_cke          : out   std_logic;
        sdram_cs_n         : out   std_logic;
        sdram_ras_n        : out   std_logic;
        sdram_cas_n        : out   std_logic;
        sdram_we_n         : out   std_logic;
        sdram_ba           : out   std_logic_vector(1 downto 0);
        sdram_addr         : out   std_logic_vector(ROW_BITS - 1 downto 0);
        sdram_dq           : inout std_logic_vector(15 downto 0);
        sdram_dqm          : out   std_logic_vector(1 downto 0)
    );
end component;

component sram_sdram_bridge is
    generic (
        ADDR_BITS        : integer := 24;
        SDRAM_MHZ        : integer := 75;
        GENERATE_REFRESH : boolean := true;               -- generate refresh_req  false = don't refresh
        USE_CACHE        : boolean := true;               -- enable/disable cache
        -- Cache parameters
        CACHE_SIZE_BYTES : integer := 4096;               -- 4KB cache
        LINE_SIZE_BYTES  : integer := 16;                 -- 16-byte cache lines
        RAM_BLOCK_TYPE   : string  := "M9K, no_rw_check"  -- "M9K", "M4K", "M10K", "AUTO"
    );
    port (
        sdram_clk        : in  std_logic;
        E                : in  std_logic;
        reset_n          : in  std_logic;
        
        -- SRAM-like interface (CPU side)
        sram_ce_n        : in  std_logic;
        sram_we_n        : in  std_logic;
        sram_oe_n        : in  std_logic;
        sram_addr        : in  std_logic_vector(ADDR_BITS-1 downto 0);
        sram_din         : in  std_logic_vector(7 downto 0);
        sram_dout        : out std_logic_vector(7 downto 0);
        mrdy             : out std_logic;
        
        -- SDRAM controller interface
        sdram_req        : out std_logic;
        sdram_wr_n       : out std_logic;
        sdram_addr       : out std_logic_vector(ADDR_BITS-2 downto 0);  
        sdram_din        : out std_logic_vector(15 downto 0);
        sdram_dout       : in  std_logic_vector(15 downto 0);
        sdram_byte_en    : out std_logic_vector(1 downto 0);
        sdram_ready      : in  std_logic;
        sdram_ack        : in  std_logic;
        refresh_req      : out std_logic;
        cache_hitp       : out unsigned(6 downto 0)
    );
end component;

-- compute fractional divider value in 8.8 fixed point format
-- clock_mhz  : PLL output clock in MHz
-- target_mhz : desired output frequency in MHz  
-- mult       : internal multiplier (e.g. 4 for phi2/phi0, 1 for UART)
function compute_divider(clock_mhz : real; target_mhz : real; mult : positive)
    return std_logic_vector is
    variable ratio     : real;
    variable int_part  : integer;
    variable frac_part : integer;
begin
    ratio     := clock_mhz / (target_mhz * real(mult));
    int_part  := integer(floor(ratio));
    frac_part := integer(floor((ratio - real(int_part)) * 256.0));
    return std_logic_vector(to_unsigned(int_part * 256 + frac_part, 16));
end function;

function kb_to_addr_bits(kb : positive) return integer is
    variable bits  : integer := 0;
    variable bytes : integer;
begin
    bytes := kb * 1024;
    while (2**bits) < bytes loop
        bits := bits + 1;
    end loop;
    return bits;
end function;


--------------------------------------------------------------------------
-- Board Configuration Parameters 
--------------------------------------------------------------------------
constant CPU_TYPE         : string   := "6502";                   -- 6502, 65C02, 6800, 6809
constant CPU_CORE         : string   := "MX65";                   -- 65XX or T65 or MX65
constant ROM              : string   := "INTBASIC";
constant RAM_SIZE_KB      : positive := 48;                       -- DE10-Lite supports up to 48KB
constant BAUD_RATE        : integer  := 115200;
constant FAST_CLK_SPEED   : real     := 120.0;
constant CPU_MULTIPLIER   : integer  := 4;
constant SERIAL_CLK_SPEED : real     := 1.8432;
constant HAS_ACI          : boolean  := false;
constant HAS_MSPI         : boolean  := false;
constant HAS_TIMER        : boolean  := false;
constant SDRAM_MHZ        : integer  := 100;
constant ROW_BITS         : integer  := 13;
constant COL_BITS         : integer  := 10;
constant TRP_NS           : integer  := 20;                       -- Precharge time (for PRECHARGE wait)
constant TRCD_NS          : integer  := 20;                       -- RAS to CAS delay (for ACTIVE→READ/WRITE)
constant TRFC_NS          : integer  := 70;                       -- Refresh cycle time (for AUTO REFRESH wait)
constant CAS_LATENCY      : integer  := 2;                        -- CAS Latency: 2 or 3 cycles
constant AUTO_PRECHARGE   : boolean  := false;
constant AUTO_REFRESH     : boolean  := false;
constant CACHE_DATA       : boolean  := false;
constant CACHE_SIZE_BYTES : integer  := 1024;                     -- 1KB cache
constant LINE_SIZE_BYTES  : integer  := 16;                       -- 16-byte cache lines
constant RAM_BLOCK_TYPE   : string   := "AUTO";                   -- "M9K", "M4K", "M10K", "AUTO"
constant SERIAL_PORT      : string   := "BUILTIN";                -- "GPIO" or "BUILTIN"

-- constant computed from other constants
constant ADDR_BITS        : integer  := kb_to_addr_bits(RAM_SIZE_KB);
constant SDRAM_ADDR_WIDTH : integer  := ROW_BITS + COL_BITS + 2;  -- +2 for BA(1:0)
--------------------------------------------------------------------------


signal  address_bus    : std_logic_vector(15 downto 0);
signal  data_bus       : std_logic_vector(7 downto 0);
signal  ram_data       : std_logic_vector(7 downto 0);
signal  tram_data      : std_logic_vector(7 downto 0);
signal  ram_cs_n       : std_logic;
signal  reset_n        : std_logic;
signal  cpu_reset_n    : std_logic;
signal  cpu_clk        : std_logic;
signal  fast_clk       : std_logic;
signal  serial_clk     : std_logic;
signal  sdram_clk      : std_logic;
signal  pll_locked     : std_logic;
signal  phi2           : std_logic;
signal  rw             : std_logic;
signal  tram_cs_n      : std_logic;
signal  sdcard_cs      : std_logic;
signal  sdcard_sck     : std_logic;
signal  sdcard_mosi    : std_logic;
signal  sdcard_miso    : std_logic;
signal  serial_rx      : std_logic;
signal  serial_tx      : std_logic;
signal  tape_in        : std_logic;
signal  tape_out       : std_logic;


-- SDRAM Controller Interface (bridge side)
signal sdram_req       : std_logic;
signal sdram_wr_n      : std_logic;
signal sdram_addr      : std_logic_vector(14 downto 0);
signal sdram_din       : std_logic_vector(15 downto 0);
signal sdram_dout      : std_logic_vector(15 downto 0);
signal sdram_byte_en   : std_logic_vector(1 downto 0);
signal sdram_ready     : std_logic;
signal sdram_ack       : std_logic;
signal refresh_busy    : std_logic;

signal mrdy            : std_logic;
signal refresh_req     : std_logic;
signal cache_hit      : unsigned(6 downto 0);  -- 0 to 100%
signal cache_hit_tens : unsigned(3 downto 0);  -- 0 à 10
signal cache_hit_ones : unsigned(3 downto 0);  -- 0 à 9	
signal cpu_divider    : std_logic_vector(15 downto 0);
signal display        : std_logic_vector(23 downto 0);


begin
    -- reset_n reset the lower level
    -- cpu_reset_n wake up the cpu when the lsasassaaaower levels are initialized
    reset_n <= KEY(0);
    cpu_reset_n <= '1' when reset_n = '1' and pll_locked = '1' else '0';

               
    -- divider = 120 / phi2 * 4
    -- SW(3 downto 0) selects phi2 frequency in MHz
    cpu_divider <= compute_divider(FAST_CLK_SPEED,  1.0, CPU_MULTIPLIER) when SW(3 downto 0) = "0001" else  --  1 MHz 120/ 4 = 30.000 exact
                   compute_divider(FAST_CLK_SPEED,  2.0, CPU_MULTIPLIER) when SW(3 downto 0) = "0010" else  --  2 MHz 120/ 8 = 15.000 exact
                   compute_divider(FAST_CLK_SPEED,  3.0, CPU_MULTIPLIER) when SW(3 downto 0) = "0011" else  --  3 MHz 120/12 = 10.000 exact
                   compute_divider(FAST_CLK_SPEED,  4.0, CPU_MULTIPLIER) when SW(3 downto 0) = "0100" else  --  4 MHz 120/16 =  7.500 int=7  frac=128
                   compute_divider(FAST_CLK_SPEED,  5.0, CPU_MULTIPLIER) when SW(3 downto 0) = "0101" else  --  5 MHz 120/20 =  6.000 exact
                   compute_divider(FAST_CLK_SPEED,  6.0, CPU_MULTIPLIER) when SW(3 downto 0) = "0110" else  --  6 MHz 120/24 =  5.000 exact
                   compute_divider(FAST_CLK_SPEED,  7.0, CPU_MULTIPLIER) when SW(3 downto 0) = "0111" else  --  7 MHz 120/28 =  4.286 int=4  frac=73
                   compute_divider(FAST_CLK_SPEED,  8.0, CPU_MULTIPLIER) when SW(3 downto 0) = "1000" else  --  8 MHz 120/32 =  3.750 int=3  frac=192
                   compute_divider(FAST_CLK_SPEED,  9.0, CPU_MULTIPLIER) when SW(3 downto 0) = "1001" else  --  9 MHz 120/36 =  3.333 int=3  frac=85
                   compute_divider(FAST_CLK_SPEED, 10.0, CPU_MULTIPLIER) when SW(3 downto 0) = "1010" else  -- 10 MHz 120/40 =  3.000 exact
                   compute_divider(FAST_CLK_SPEED, 11.0, CPU_MULTIPLIER) when SW(3 downto 0) = "1011" else  -- 11 MHz 120/44 =  2.727 int=2  frac=186
                   compute_divider(FAST_CLK_SPEED, 12.0, CPU_MULTIPLIER) when SW(3 downto 0) = "1100" else  -- 12 MHz 120/48 =  2.500 int=2  frac=128
                   compute_divider(FAST_CLK_SPEED, 13.0, CPU_MULTIPLIER) when SW(3 downto 0) = "1101" else  -- 13 MHz 120/52 =  2.308 int=2  frac=79
                   compute_divider(FAST_CLK_SPEED, 14.0, CPU_MULTIPLIER) when SW(3 downto 0) = "1110" else  -- 14 MHz 120/56 =  2.143 int=2  frac=37
                   compute_divider(FAST_CLK_SPEED, 15.0, CPU_MULTIPLIER) when SW(3 downto 0) = "1111" else  -- 15 MHz 120/60 =  2.000 exact
                   compute_divider(FAST_CLK_SPEED, 1.0, 4);                                                 -- default: 1 MHz


    mclk: pll_clock                                   port map(refclk              => CLOCK_50,
                                                               rst                 => not reset_n,
                                                               outclk_0            => fast_clk,
                                                               outclk_1            => sdram_clk,
                                                               locked              => pll_locked);

    cclk: frac_clk_div                                port map(reset_n             => reset_n,
                                                               clk_in              => fast_clk,
                                                               divider             => cpu_divider,
                                                               clk_out             => cpu_clk);

    sclk: frac_clk_div                                port map(reset_n             => reset_n,
                                                               clk_in              => fast_clk,
                                                               divider             => compute_divider(FAST_CLK_SPEED, 1.8432, 1),
                                                               clk_out             => serial_clk);

--    cclk : clock_divider                             generic map(divider             => 120_000_000/12_000_000)
--                                                        port map(reset               => reset_n,
--                                                                 clk_in              => fast_clk,
--                                                                 clk_out             => cpu_clk);
--    
--    sclk:  fractional_clock_divider                  generic map(CLK_FREQ_HZ         => 120_000_000,
--                                                                 FREQUENCY_HZ        => 1_843_200)
--                                                        port map(clk_in              => fast_clk,
--                                                                 reset_n             => reset_n,
--                                                                 clk_out             => serial_clk);
    
    cache_hit_tens <= resize(cache_hit / 10, 4);
    cache_hit_ones <= resize(cache_hit mod 10, 4);    

    display <= address_bus & data_bus                                                          when SW(9) = '0' else
               x"0000" & std_logic_vector(cache_hit_tens) & std_logic_vector(cache_hit_ones); 
    
    h0 : hexto7seg                                  generic map(SEGMENTS           => 7) 
                                                       port map(hex                => display(3  downto  0),
                                                                seg                => HEX0); 
    h1 : hexto7seg                                  generic map(SEGMENTS           => 7) 
                                                       port map(hex                => display(7  downto  4),
                                                                seg                => HEX1); 
    h2 : hexto7seg                                  generic map(SEGMENTS           => 7) 
                                                       port map(hex                => display(11 downto  8),
                                                                seg                => HEX2); 
    h3 : hexto7seg                                  generic map(SEGMENTS           => 7) 
                                                       port map(hex                => display(15 downto 12),
                                                                seg                => HEX3); 
    h4 : hexto7seg                                  generic map(SEGMENTS           => 7) 
                                                       port map(hex                => display(19 downto 16),
                                                                seg                => HEX4); 
    h5 : hexto7seg                                 generic map(SEGMENTS            => 7) 
                                                       port map(hex                => display(23 downto 20),
                                                                seg                => HEX5); 

    ap1: Replica1_CORE                              generic map(CPU_TYPE           =>  CPU_TYPE,    -- 6502, 65C02, 6800 or 6809
                                                                CPU_CORE           =>  CPU_CORE,    -- "65XX", "T65", MX65"
                                                                ROM                =>  ROM,         -- default wozmon65
                                                                RAM_SIZE_KB        =>  RAM_SIZE_KB, -- 8 to 48Kb 
                                                                BAUD_RATE          =>  BAUD_RATE,   -- uart speed 1200 to 115200
                                                                HAS_ACI            =>  HAS_ACI,     -- add the aci (incomplete)
                                                                HAS_MSPI           =>  HAS_MSPI,    -- add master spi  C200
                                                                HAS_TIMER          =>  HAS_TIMER)   -- add basic timer C210
                                                    port map(main_clk              =>  cpu_clk,
                                                                serial_clk         =>  serial_clk,
                                                                reset_n            =>  reset_n,
                                                                cpu_reset_n        =>  cpu_reset_n,
                                                                bus_phi2           =>  phi2,    
                                                                bus_address        =>  address_bus,
                                                                bus_data           =>  data_bus,
                                                                bus_rw             =>  rw,
                                                                bus_mrdy           =>  mrdy,
                                                                ext_ram_cs_n       =>  ram_cs_n,
                                                                ext_ram_data       =>  ram_data,
                                                                ext_tram_cs_n      =>  tram_cs_n,
                                                                ext_tram_data      =>  tram_data,
                                                                uart_rx            =>  serial_rx,
                                                                uart_tx            =>  serial_tx,
                                                                spi_cs             =>  sdcard_cs,
                                                                spi_sck            =>  sdcard_sck,
                                                                spi_mosi           =>  sdcard_mosi,
                                                                spi_miso           =>  sdcard_miso,
                                                                tape_out           =>  tape_out,
                                                                tape_in            =>  tape_in);

    bridge : sram_sdram_bridge                      generic map(ADDR_BITS          => ADDR_BITS,
                                                                SDRAM_MHZ          => SDRAM_MHZ,
                                                                GENERATE_REFRESH   => not AUTO_REFRESH,
                                                                USE_CACHE          => CACHE_DATA,
                                                                -- Cache parameters
                                                                CACHE_SIZE_BYTES   => CACHE_SIZE_BYTES, 
                                                                LINE_SIZE_BYTES    => LINE_SIZE_BYTES,  
                                                                RAM_BLOCK_TYPE     => RAM_BLOCK_TYPE)  
                                                       port map(sdram_clk          => sdram_clk,
                                                                E                  => phi2,
                                                                reset_n            => reset_n,
                                                                -- SRAM interface 
                                                                sram_ce_n          => ram_cs_n,
                                                                sram_we_n          => rw,
                                                                sram_oe_n          => not rw,
                                                                sram_addr          => address_bus(ADDR_BITS - 1 downto 0),
                                                                sram_din           => data_bus,
                                                                sram_dout          => ram_data,
                                                                mrdy               => mrdy,
                                                                -- SDRAM controller interface
                                                                sdram_req          => sdram_req,
                                                                sdram_wr_n         => sdram_wr_n,
                                                                sdram_addr         => sdram_addr,
                                                                sdram_din          => sdram_din,
                                                                sdram_dout         => sdram_dout,
                                                                sdram_byte_en      => sdram_byte_en,
                                                                sdram_ready        => sdram_ready,
                                                                sdram_ack          => sdram_ack,
                                                                refresh_req        => refresh_req,
                                                                cache_hitp         => cache_hit);

    sdram : sdram_controller                        generic map(FREQ_MHZ           => SDRAM_MHZ,
                                                                ROW_BITS           => ROW_BITS, 
                                                                COL_BITS           => COL_BITS,
                                                                TRP_NS             => TRP_NS,
                                                                TRCD_NS            => TRCD_NS,
                                                                TRFC_NS            => TRFC_NS,
                                                                CAS_LATENCY        => CAS_LATENCY,
                                                                USE_AUTO_PRECHARGE => AUTO_PRECHARGE,
                                                                USE_AUTO_REFRESH   => AUTO_REFRESH)
                                                       port map(clk                => sdram_clk,
                                                                reset_n            => reset_n,
                                                                req                => sdram_req,
                                                                wr_n               => sdram_wr_n,
                                                                addr               => std_logic_vector(resize(unsigned(sdram_addr), SDRAM_ADDR_WIDTH)),
                                                                din                => sdram_din,
                                                                dout               => sdram_dout,
                                                                byte_en            => sdram_byte_en,
                                                                ready              => sdram_ready,
                                                                ack                => sdram_ack,
                                                                refresh_req        => refresh_req,
                                                                refresh_active     => refresh_busy,
                                                                sdram_clk          => DRAM_CLK,
                                                                sdram_cke          => DRAM_CKE,
                                                                sdram_cs_n         => DRAM_CS_N,
                                                                sdram_ras_n        => DRAM_RAS_N,
                                                                sdram_cas_n        => DRAM_CAS_N,
                                                                sdram_we_n         => DRAM_WE_N,
                                                                sdram_ba           => DRAM_BA,
                                                                sdram_addr         => DRAM_ADDR,
                                                                sdram_dq           => DRAM_DQ,
                                                                sdram_dqm(1)       => DRAM_UDQM,
                                                                sdram_dqm(0)       => DRAM_LDQM);

                                                            
    VGA_B       <= (others => '0');
    VGA_G       <= (others => '0'); 
    VGA_R       <= (others => '0');  
    VGA_HS      <= '0';
    VGA_VS	    <= '0';

    LEDR(0)     <= refresh_busy;    
    LEDR(1)     <= serial_rx;
    LEDR(2)     <= serial_tx;
  
gen_builtin_serial: if SERIAL_PORT = "BUILTIN" generate
    UART_TX         <= serial_tx;
    serial_rx       <= UART_RX;
    UART_RTS        <= UART_CTS;
end generate gen_builtin_serial;


gen_gpio_serial: if SERIAL_PORT = "GPIO" generate
    serial_rx       <= GPIO_1(0);
    GPIO_1(1)       <= serial_tx;
end generate gen_gpio_serial;

    GPIO_1(2)       <= tape_out;
    tape_in         <= GPIO_1(3);

    GPIO_1(4)       <= sdcard_cs;
    GPIO_1(5)       <= sdcard_sck;
    GPIO_1(6)       <= sdcard_mosi;
    sdcard_miso     <= GPIO_1(7);

end top;

