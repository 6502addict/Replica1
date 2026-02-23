--------------------------------------------------------------------------
-- To configure the machine search "Board Configuration Parameters"
-- and adapt the configuration to your needs 
--------------------------------------------------------------------------
--------------------------------------------------------------------------------
-- MAX1000 SDRAM Test Top Level
-- Device: 10M16SAU169C8G
--------------------------------------------------------------------------------

library IEEE;
    use IEEE.std_logic_1164.all;
    use IEEE.numeric_std.all;
    use ieee.math_real.all;
        
entity MAX1000_Replica1 is
    port(
        -- Clock and Reset
        CLK12M      : in    std_logic;  -- 12 MHz input clock
        RESET       : in    std_logic;  -- Active low reset
        USER_BTN    : in    std_logic;  -- User button
        
        -- LEDs for status
        LED         : out   std_logic_vector(7 downto 0);
        
        -- Accelerometer (SPI interface)
        SEN_INT1    : in    std_logic;
        SEN_INT2    : in    std_logic;
        SEN_SDI     : out   std_logic;  -- MOSI (Master Out Slave In)
        SEN_SDO     : in    std_logic;  -- MISO (Master In Slave Out)
        SEN_SPC     : out   std_logic;  -- SPI Clock
        SEN_CS      : out   std_logic;  -- Chip Select
        
        -- PMOD
        PIO         : inout std_logic_vector(7 downto 0);
        
        -- UART
        BDBUS       : inout std_logic_vector(5 downto 0);   -- BDBUS(0) = TX BDBUS(1) = RX
        
        -- FLASH (SPI)
        F_S         : out   std_logic;  -- Chip Select
        F_CLK       : out   std_logic;  -- SPI Clock
        F_DI        : out   std_logic;  -- MOSI (to flash)
        F_DO        : in    std_logic;  -- MISO (from flash)
        
        -- ARDUINO
        D           : inout std_logic_vector(14 downto 0);
        AIN         : in    std_logic_vector(6 downto 0);
        
        
        -- SDRAM Interface (14-bit address version)
        DQ          : inout std_logic_vector(15 downto 0);
        DQM         : out   std_logic_vector(1  downto 0);
        A           : out   std_logic_vector(13 downto 0);
        BA          : out   std_logic_vector(1  downto 0);
        CLK         : out   std_logic;
        CKE         : out   std_logic;
        RAS         : out   std_logic;
        WE          : out   std_logic;
        CS          : out   std_logic;
        CAS         : out   std_logic
    );
end MAX1000_Replica1;

architecture rtl of MAX1000_Replica1 is

component pll_clock is
    port (
        areset      : in  std_logic  := '0';   
        inclk0      : in  std_logic  := '0';
        c0          : out std_logic;              -- fast_clk   120Mhz
        c1          : out std_logic;              -- sdram_clk  120/100Mhz
        locked      : out std_logic 
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

--------------------------------------------------------------------------
-- Board Configuration Parameters 
--------------------------------------------------------------------------
constant BOARD            : string   := "DE10_Lite";
constant CPU_TYPE         : string   := "6502";                   -- 6502, 65C02, 6800, 6809
constant CPU_CORE         : string   := "MX65";                   -- 65XX or T65 or MX65
constant CPU_CLK_SPEED    : real     := 15.0;                     -- cpu speed 1 to  15 Mhz
constant CPU_MULTIPLIER   : integer  := 4;                        -- cpu cores uses 4x clock
constant ROM              : string   := "WOZMON65";
constant RAM_SIZE_KB      : positive := 48;                       -- DE10-Lite supports up to 48KB
constant BAUD_RATE        : integer  := 115200;
constant FAST_CLK_SPEED   : real     := 120.0;
constant SERIAL_CLK_SPEED : real     := 1.8432;
constant HAS_ACI          : boolean  := false;
constant HAS_MSPI         : boolean  := false;
constant HAS_TIMER        : boolean  := false;
constant USE_EBR_RAM      : boolean  := true;                     -- true for DE10-Lite/DE1-SOC, false for DE1
constant SDRAM_MHZ        : integer  := 120;
constant ROW_BITS         : integer  := 13;
constant COL_BITS         : integer  := 9;
constant TRP_NS           : integer  := 18;                       -- Precharge time (for PRECHARGE wait)
constant TRCD_NS          : integer  := 18;                       -- RAS to CAS delay (for ACTIVE→READ/WRITE)
constant TRFC_NS          : integer  := 60;                       -- Refresh cycle time (for AUTO REFRESH wait)
constant CAS_LATENCY      : integer  := 2;                        -- CAS Latency: 2 or 3 cycles
constant ADDR_BITS        : integer  := 16; 
constant AUTO_PRECHARGE   : boolean  := false;
constant AUTO_REFRESH     : boolean  := false;
constant CACHE_DATA       : boolean  := false;                    -- actually only works fine on DE10-Lite
constant CACHE_SIZE_BYTES : integer  := 1024;                     -- 1KB cache
constant LINE_SIZE_BYTES  : integer  := 16;                       -- 16-byte cache lines
constant SDRAM_ADDR_WIDTH : integer  := ROW_BITS + COL_BITS + 2;  -- +2 pour BA(1:0)
constant RAM_BLOCK_TYPE   : string   := "M9K, no_rw_check";       -- "M9K", "M4K", "M10K", "AUTO"


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

begin
    
    reset_n <= RESET;
    cpu_reset_n <= '1' when reset_n = '1' and pll_locked = '1' else '0';
    

    mclk: pll_clock                                   port map(areset              => not reset_n,
                                                               inclk0              => CLK12M,
                                                               c0                  => fast_clk,
                                                               c1                  => sdram_clk,
                                                               locked              => pll_locked);
                                        
    -- Note on board with switches the clock is selected with the first 4 switches
    -- on ne bare MAX1000 no switches in that case set CPU_CLK_SPEED in the constants sections
    cclk: frac_clk_div                                port map(reset_n             => reset_n,
                                                               clk_in              => fast_clk,
                                                               divider             => compute_divider(FAST_CLK_SPEED, CPU_CLK_SPEED, CPU_MULTIPLIER),
                                                               clk_out             => cpu_clk);

    sclk: frac_clk_div                                port map(reset_n             => reset_n,
                                                               clk_in              => fast_clk,
                                                               divider             => compute_divider(FAST_CLK_SPEED, SERIAL_CLK_SPEED, 1),
                                                               clk_out             => serial_clk);
    

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
                                                                cache_hitp         => open);

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
                                                                sdram_clk          => CLK,
                                                                sdram_cke          => CKE,
                                                                sdram_cs_n         => CS,
                                                                sdram_ras_n        => RAS,
                                                                sdram_cas_n        => CAS,
                                                                sdram_we_n         => WE,
                                                                sdram_ba           => BA,
                                                                sdram_addr         => A(12 downto 0),
                                                                sdram_dq           => DQ,
                                                                sdram_dqm          => DQM);

    LED(0)             <= serial_rx;
    LED(1)             <= serial_tx;
    LED(2)             <= tape_in;
    LED(3)             <= tape_out;
    LED(4)             <= cpu_reset_n;
    LED(5)             <= '0';
    LED(6)             <= '0';
    LED(7)             <= refresh_busy;

    BDBUS(0)           <= serial_tx;
    serial_rx          <= BDBUS(1);

    -- same thing for the sdcard both are mapped to the PMOD connector    
    PIO(0)             <= sdcard_cs;
    PIO(1)             <= sdcard_sck;
    PIO(2)             <= sdcard_mosi;
    sdcard_miso        <= PIO(3);

    -- Note the mapping for the tape is purely virtual  no hardware interface
    PIO(4)             <= tape_out;
    tape_in            <= PIO(5);

end rtl;

