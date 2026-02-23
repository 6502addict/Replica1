library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity sdram_controller is
    generic (
        FREQ_MHZ : integer := 100;
        ROW_BITS : integer := 13;
        COL_BITS : integer := 10
    );
    port(
        clk            : in    std_logic;
        reset_n        : in    std_logic;
        
        -- Simple CPU interface
        req            : in    std_logic;
        wr_n           : in    std_logic;  -- 1=write, 0=read
        addr           : in    std_logic_vector(ROW_BITS+COL_BITS+1 downto 0); 
        din            : in    std_logic_vector(15 downto 0);
        dout           : out   std_logic_vector(15 downto 0);
        byte_en        : in    std_logic_vector(1 downto 0); -- Active low
        ready          : out   std_logic;
        ack            : out   std_logic;
        
        -- Debug outputs
        debug_state    : out   std_logic_vector(3 downto 0);
        debug_cmd      : out   std_logic_vector(3 downto 0);
        refresh_active : out   std_logic;
        
        -- SDRAM pins (Dummy)
        sdram_clk      : out   std_logic;
        sdram_cke      : out   std_logic;
        sdram_cs_n     : out   std_logic;
        sdram_ras_n    : out   std_logic;
        sdram_cas_n    : out   std_logic;
        sdram_we_n     : out   std_logic;
        sdram_ba       : out   std_logic_vector(1 downto 0);
        sdram_addr     : out   std_logic_vector(12 downto 0);
        sdram_dq       : inout std_logic_vector(15 downto 0);
        sdram_dqm      : out   std_logic_vector(1 downto 0)
    );
end sdram_controller;

architecture behavioral of sdram_controller is

    component zz
        port (
            address : IN STD_LOGIC_VECTOR (10 DOWNTO 0);
            byteena : IN STD_LOGIC_VECTOR (1 DOWNTO 0) := (OTHERS => '1');
            clock   : IN STD_LOGIC := '1';
            data    : IN STD_LOGIC_VECTOR (15 DOWNTO 0);
            wren    : IN STD_LOGIC;
            q       : OUT STD_LOGIC_VECTOR (15 DOWNTO 0)
        );
    end component;

    type state_type is (IDLE, BUSY, DONE);
    signal state : state_type := IDLE;
    
    signal ram_we   : std_logic := '0';
    signal ram_q    : std_logic_vector(15 downto 0);
    signal req_reg  : std_logic := '0';

begin

    -- Instantiate the Block RAM
    -- We use the lowest 11 bits of your addr bus to map to 2048 words
    u_ram : zz
    port map (
        address => addr(10 downto 0),
        byteena => byte_en, -- Converting active-low byte_en to BRAM byteena (if needed)
        clock   => clk,
        data    => din,
        wren    => ram_we,
        q       => ram_q
    );

    -- Simple FSM to mimic SDRAM handshaking
    process(clk, reset)
    begin
        if reset_n = '0' then
            state <= IDLE;
            ready <= '1';
            ack <= '0';
            ram_we <= '0';
        elsif rising_edge(clk) then
            req_reg <= req;
            
            case state is
                when IDLE =>
                    ack <= '0';
                    ram_we <= '0';
                    if req = '1' and req_reg = '0' then -- Edge detect
                        ready <= '0';
                        ram_we <= not wr_n; -- Trigger write if wr=1
                        state <= BUSY;
                    else
                        ready <= '1';
                    end if;

                when BUSY =>
                    -- BRAM takes 1 clock to process. 
                    -- We stay here one cycle to ensure data is latched/read
                    ram_we <= '0'; 
                    state <= DONE;

                when DONE =>
                    dout <= ram_q;
                    ack <= '1';
                    if req = '0' then
                        state <= IDLE;
                    end if;
            end case;
        end if;
    end process;

    -- Dummy assignments for SDRAM pins
    sdram_clk   <= clk;
    sdram_cke   <= '1';
    sdram_cs_n  <= '1';
    sdram_ras_n <= '1';
    sdram_cas_n <= '1';
    sdram_we_n  <= '1';
    sdram_ba    <= (others => '0');
    sdram_addr  <= (others => '0');
    sdram_dq    <= (others => 'Z');
    sdram_dqm   <= (others => '1');

    -- Debug Logic
    debug_state <= x"0" when state = IDLE else
                   x"1" when state = BUSY else
                   x"2" when state = DONE else
                   x"F";
    debug_cmd <= (others => '0');
    refresh_active <= '0';

end behavioral;