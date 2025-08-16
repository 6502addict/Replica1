library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity RAM_DE1 is
    generic (
        RAM_SIZE_KB : integer := 32  -- 8, 16, 24, 32, 40, or 48
    );
    port (
        clock     : in    std_logic;
        cs_n      : in    std_logic;
        we_n      : in    std_logic;
        address   : in    std_logic_vector(15 downto 0);
        data_in   : in    std_logic_vector(7 downto 0);
        data_out  : out   std_logic_vector(7 downto 0);
        -- SRAM interface
        SRAM_DQ   : inout std_logic_vector(15 downto 0);  -- SRAM Data bus 16 Bits
        SRAM_ADDR : out   std_logic_vector(17 downto 0);  -- SRAM Address bus 18 Bits
        SRAM_UB_N : out   std_logic;                      -- SRAM High-byte Data Mask 
        SRAM_LB_N : out   std_logic;                      -- SRAM Low-byte Data Mask 
        SRAM_WE_N : out   std_logic;                      -- SRAM Write Enable
        SRAM_CE_N : out   std_logic;                      -- SRAM Chip Enable
        SRAM_OE_N : out   std_logic                       -- SRAM Output Enable
    );
end entity;

architecture rtl of RAM_DE1 is
    
    signal valid_access : std_logic;
    signal sram_data_write : std_logic_vector(15 downto 0);
    signal sram_data_read  : std_logic_vector(15 downto 0);
    
begin
    
    -- Check if address is within configured RAM size
    valid_access <= '1' when unsigned(address) < (RAM_SIZE_KB * 1024) else '0';
    
    -- SRAM address uses bits 16:1 of the CPU address (bit 0 selects byte)
    -- This gives us 256K words x 16 bits = 512KB total
    SRAM_ADDR <= "000" & address(15 downto 1);  -- Word address (ignoring bit 0)
    
    -- Prepare 16-bit data for writing (duplicate 8-bit data on both bytes)
    sram_data_write <= data_in & data_in;
    
    -- Byte selection based on address bit 0
    -- address(0) = 0 -> use lower byte (bits 7:0)
    -- address(0) = 1 -> use upper byte (bits 15:8)
    SRAM_LB_N <= address(0) or cs_n or not valid_access;     -- Low byte enabled when addr(0)=0
    SRAM_UB_N <= not address(0) or cs_n or not valid_access; -- High byte enabled when addr(0)=1
    
    -- Control signals
    SRAM_CE_N <= cs_n or not valid_access;  -- Chip enable
    SRAM_OE_N <= not we_n;                  -- Output enable (active when reading)
    SRAM_WE_N <= we_n;                      -- Write enable (active low when writing)
    
    -- Bidirectional data bus
    SRAM_DQ <= sram_data_write when (we_n = '0' and cs_n = '0' and valid_access = '1') else 
               (others => 'Z');
    
    -- Read data mux - select byte based on address bit 0
    sram_data_read <= SRAM_DQ;
    data_out <= sram_data_read(7 downto 0) when address(0) = '0' else
                sram_data_read(15 downto 8);
    
end rtl;