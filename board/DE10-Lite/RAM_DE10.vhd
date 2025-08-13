library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity RAM_DE10 is
    generic (
        RAM_SIZE_KB : integer := 32  -- 8, 16, 24, 32, 40, or 48
    );
    port (
        clock:      in std_logic;
        cs_n:       in std_logic;
        we_n:       in std_logic;
        address:    in std_logic_vector(15 downto 0);
        data_in:    in std_logic_vector(7 downto 0);
        data_out:   out std_logic_vector(7 downto 0)
    );
end entity;

architecture rtl of RAM_DE10 is
    
    component ram_8k IS
        PORT (
            address : IN STD_LOGIC_VECTOR (12 DOWNTO 0);  -- 13-bit for 8KB
            clock   : IN STD_LOGIC := '1';
            data    : IN STD_LOGIC_VECTOR (7 DOWNTO 0);
            wren    : IN STD_LOGIC;
            q       : OUT STD_LOGIC_VECTOR (7 DOWNTO 0)
        );
    end component;
    
    -- Calculate number of 8K blocks needed
    constant NUM_BLOCKS : integer := (RAM_SIZE_KB + 7) / 8;  -- Ceiling division
    
    -- Array types for scalable signals
    type ram_data_array is array (0 to NUM_BLOCKS-1) of std_logic_vector(7 downto 0);
    type ram_wren_array is array (0 to NUM_BLOCKS-1) of std_logic;
    
    signal ram_data : ram_data_array := (others => (others => '0'));
    signal ram_wren : ram_wren_array := (others => '0');
    
    -- Address decode
    signal block_select : integer range 0 to NUM_BLOCKS-1;
    signal valid_address : std_logic;
    
begin
    
    -- Address decoding
    block_select <= to_integer(unsigned(address(15 downto 13)));
    valid_address <= '1' when block_select < NUM_BLOCKS else '0';
    
    -- Generate write enables for each block
    gen_wren: for i in 0 to NUM_BLOCKS-1 generate
        ram_wren(i) <= '1' when (we_n = '0' and cs_n = '0' and 
                                block_select = i and valid_address = '1') else '0';
    end generate gen_wren;
    
    -- Generate RAM blocks
    gen_ram_blocks: for i in 0 to NUM_BLOCKS-1 generate
        ram_block: ram_8k port map(
            address => address(12 downto 0),
            clock   => clock,
            data    => data_in,
            wren    => ram_wren(i),
            q       => ram_data(i)
        );
    end generate gen_ram_blocks;
    
    -- Output multiplexer
    process(block_select, valid_address, ram_data)
    begin
        if valid_address = '1' then
            data_out <= ram_data(block_select);
        else
            data_out <= (others => '1');  -- Return 0 for invalid addresses
        end if;
    end process;
    
end rtl;