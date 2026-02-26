--------------------------------------------------------------------------
-- AX4010 Replica1 Top Level
-- Board: Alinx AX4010 - Cyclone IV E EP4CE10F17C8
-- SDRAM: Hynix H57V2562GTR (256Mbit = 32MB, 16Mx16, ROW=12, COL=8)
--------------------------------------------------------------------------
-- To configure the machine search "Board Configuration Parameters"
-- and adapt the configuration to your needs 
--------------------------------------------------------------------------

library IEEE;
    use IEEE.std_logic_1164.all;
    use ieee.numeric_std.all;
    use ieee.math_real.all;
    
entity AX4010_Replica1 is
    port (
        --------- CLOCK ---------
        CLOCK_50         : in    std_logic;  -- 50MHz

        --------- RESET & KEYS ---------
        KEY              : in    std_logic_vector(3 downto 0);

        --------- LEDs ---------
        LED              : out   std_logic_vector(3 downto 0);

        --------- 7-SEGMENT DISPLAY ---------
        SEG_DATA         : out   std_logic_vector(7 downto 0);
        SEG_SEL          : out   std_logic_vector(5 downto 0);

        --------- SDRAM ---------
        DRAM_ADDR        : out   std_logic_vector(12 downto 0);
        DRAM_BA          : out   std_logic_vector(1 downto 0);
        DRAM_CAS_N       : out   std_logic;
        DRAM_CKE         : out   std_logic;
        DRAM_CLK         : out   std_logic;
        DRAM_CS_N        : out   std_logic;
        DRAM_DQ          : inout std_logic_vector(15 downto 0);
        DRAM_DQM         : out   std_logic_vector(1 downto 0);
        DRAM_RAS_N       : out   std_logic;
        DRAM_WE_N        : out   std_logic;

        --------- UART ---------
        UART_TX          : out   std_logic;
        UART_RX          : in    std_logic;

        --------- VGA ---------
        VGA_R            : out   std_logic_vector(4 downto 0);
        VGA_G            : out   std_logic_vector(5 downto 0);
        VGA_B            : out   std_logic_vector(4 downto 0);
        VGA_HS           : out   std_logic;
        VGA_VS           : out   std_logic;

        --------- SD CARD ---------
        SD_SCK           : out   std_logic;
        SD_MISO          : in    std_logic;
        SD_MOSI          : out   std_logic;
        SD_CS            : out   std_logic;

        --------- SPI FLASH ---------
        FLASH_SCK        : out   std_logic;
        FLASH_MISO       : in    std_logic;
        FLASH_MOSI       : out   std_logic;
        FLASH_CS         : out   std_logic;

        --------- I2C EEPROM ---------
        I2C_SCL          : inout std_logic;
        I2C_SDA          : inout std_logic;

        --------- RTC DS1302 ---------
        DS1302_CLK       : out   std_logic;
        DS1302_DATA      : inout std_logic;
        DS1302_CE        : out   std_logic;

        --------- BUZZER ---------
        BUZZER           : out   std_logic;
        
        --------- CAMERA ---------
--        CAM              : inout std_logic_vector(15 downto 0);  -- camera port used as GPIO

        --------- GPIO   ---------
        GPIO             : inout std_logic_vector(33 downto 0);  -- J1 connector used as GPIO
        
        --------- AN 970 LCD -----
        LCD_INT          : out   std_logic;
        LCD_PWM          : out   std_logic;
        LCD_SCL          : inout std_logic;
        LCD_SDA          : inout std_logic;
        LCD_HSYNC        : out   std_logic;
        LCD_VSYNC        : out   std_logic;
        LCD_DCLK         : out   std_logic;
        LCD_DE           : out   std_logic;
        LCD_B            : out   std_logic_vector(7 downto 0); 
        LCD_G            : out   std_logic_vector(7 downto 0); 
        LCD_R            : out   std_logic_vector(7 downto 0) 
    );
end entity AX4010_Replica1;

architecture top of AX4010_Replica1 is

component mux7seg is
    port (
        clk        : in  std_logic;
        reset_n    : in  std_logic;
        seg_sel    : out std_logic_vector(5 downto 0);  -- Digit select (active low)
        seg_data   : out std_logic_vector(7 downto 0);  -- Segment data (MSB = decimal point)
        display    : in  std_logic_vector(23 downto 0)
    );
end component;

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
constant CPU_TYPE         : string   := "6502";
constant CPU_CORE         : string   := "MX65";
constant ROM              : string   := "INTBASIC";
constant RAM_SIZE_KB      : positive := 48;
constant BAUD_RATE        : integer  := 115200;
constant FAST_CLK_SPEED   : real     := 120.0;
constant CPU_CLK_SPEED    : real     := 12.0;
constant CPU_MULTIPLIER   : integer  := 4;
constant SERIAL_CLK_SPEED : real     := 1.8432;
constant HAS_ACI          : boolean  := false;
constant HAS_MSPI         : boolean  := false;
constant HAS_TIMER        : boolean  := false;
constant USE_EBR_RAM      : boolean  := true;
constant SDRAM_MHZ        : integer  := 100;                      -- 100MHz SDRAM clock
constant ROW_BITS         : integer  := 13;                       -- H57V2562GTR: 13 row bits
constant COL_BITS         : integer  := 9;                        -- H57V2562GTR: 9 col bits
constant TRP_NS           : integer  := 20;                       -- Precharge time (for PRECHARGE wait)
constant TRCD_NS          : integer  := 20;                       -- RAS to CAS delay (for ACTIVE→READ/WRITE)
constant TRFC_NS          : integer  := 70;                       -- Refresh cycle time (for AUTO REFRESH wait)
constant CAS_LATENCY      : integer  := 2;                        -- CAS Latency: 2 or 3 cycles
constant ADDR_BITS        : integer  := 16;                       -- 4KB test window
constant AUTO_PRECHARGE   : boolean  := false;
constant AUTO_REFRESH     : boolean  := true;
constant CACHE_DATA       : boolean  := false;                    -- WARNING: Cache uses LUTs on this small FPGA!
constant CACHE_SIZE_BYTES : integer  := 512;                      -- Small cache (512B) to save resources
constant LINE_SIZE_BYTES  : integer  := 16;
constant SDRAM_ADDR_WIDTH : integer  := ROW_BITS + COL_BITS + 2;
constant RAM_BLOCK_TYPE   : string   := "M9K";


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
signal  disp_clk       : std_logic;
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

    reset_n <= KEY(0);
    cpu_reset_n <= '1' when reset_n = '1' and pll_locked = '1' else '0';
    

    mclk: pll_clock                                   port map(areset              => not reset_n,
                                                               inclk0              => CLOCK_50,
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

    dclk: frac_clk_div                                port map(reset_n             => reset_n,
                                                               clk_in              => fast_clk,
                                                               divider             => compute_divider(FAST_CLK_SPEED, 10000.0, 1),
                                                               clk_out             => disp_clk);
    -- 7-segment display: show cache hit rate
    cache_hit_tens <= resize(cache_hit / 10, 4);
    cache_hit_ones <= resize(cache_hit mod 10, 4);

    display <= address_bus & data_bus                                                          when KEY(3) = '1' else
               x"0000" & std_logic_vector(cache_hit_tens) & std_logic_vector(cache_hit_ones);         
    
    sevenseg: mux7seg                                 port map(clk                 => disp_clk,
                                                               reset_n             => reset_n,
                                                               seg_sel             => seg_sel,
                                                               seg_data            => seg_data,
                                                               display             => display);

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
                                                                sdram_clk          => DRAM_CLK,
                                                                sdram_cke          => DRAM_CKE,
                                                                sdram_cs_n         => DRAM_CS_N,
                                                                sdram_ras_n        => DRAM_RAS_N,
                                                                sdram_cas_n        => DRAM_CAS_N,
                                                                sdram_we_n         => DRAM_WE_N,
                                                                sdram_ba           => DRAM_BA,
                                                                sdram_addr         => DRAM_ADDR,
                                                                sdram_dq           => DRAM_DQ,
                                                                sdram_dqm          => DRAM_DQM);


    -- LEDs
    LED(0)     <= refresh_busy;
    LED(1)     <= serial_rx;
    LED(2)     <= serial_tx;
    LED(3)     <= pll_locked;

    -- VGA: not used yet (for future TVI925 terminal)
    VGA_R      <= (others => '0');
    VGA_G      <= (others => '0');
    VGA_B      <= (others => '0');
    VGA_HS     <= '0';
    VGA_VS     <= '0';
    
    UART_TX    <= serial_tx;
    serial_rx  <= UART_RX;

    
    SD_CS   <= sdcard_cs;
    SD_SCK  <= sdcard_sck;
    SD_MOSI <= sdcard_mosi;
    sdcard_miso <= SD_MISO;
    
    -- Unused peripherals
    BUZZER <= '1';
    

end top;
