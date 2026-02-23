library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity bus_tester is
    generic (
        ADDR_BITS : integer := 24;
        SDRAM_MHZ : integer := 100
    );
    port (
        sdram_clk    : in   std_logic;
        E            : in   std_logic;
        Q            : in   std_logic;
        reset_n      : in   std_logic;
        
        -- SRAM-like interface (CPU side)
        sram_ce_n     : in  std_logic;
        sram_we_n     : in  std_logic;
        sram_oe_n     : in  std_logic;
        sram_addr     : in  std_logic_vector(ADDR_BITS-1 downto 0);
        sram_din      : in  std_logic_vector(7 downto 0);
        sram_dout     : out std_logic_vector(7 downto 0);
        
        -- Memory ready output (for clock stretching)
        mrdy          : out std_logic;
        
        -- SDRAM controller interface
        sdram_req     : out std_logic;
        sdram_wr      : out std_logic;
        sdram_addr    : out std_logic_vector(ADDR_BITS-2 downto 0);
        sdram_din     : out std_logic_vector(15 downto 0);
        sdram_dout    : in  std_logic_vector(15 downto 0);
        sdram_byte_en : out std_logic_vector(1 downto 0);
        sdram_ready   : in  std_logic;
        sdram_ack     : in  std_logic;
        debug         : out std_logic_vector(2 downto 0)
    );
end bus_tester;

architecture rtl of sram_sdram_bridge is
    type state_type is (IDLE, PERFORM_READ, PERFORM_WRITE, WAIT_E_FALLING, UNDEFINED);
    signal state             : state_type := IDLE;
    signal E_prev            : std_logic;
    signal sdram_ack_prev    : std_logic;
    signal loopback          : std_logic_vector(7 downto 0);
	 signal delay             : integer range 0 to 7;
		
begin

	debug <= "000" when state = IDLE           else
            "001" when state = PERFORM_READ   else
            "010" when state = PERFORM_WRITE  else
            "011" when state = WAIT_E_FALLING else
            "111";

	process(sdram_clk)
   begin
		if rising_edge(sdram_clk) then
			E_prev <= E;
			sdram_ack_prev <= sdram_ack;
			
         if reset_n = '0' then
				state    <= IDLE;
            loopback <= x"FF";
            mrdy     <= '1';
            sdram_req <= '0';
         else
				case state is
					when IDLE =>
						mrdy <= '1';
                  sdram_req <= '0';
                  -- trigger on E rising while cs is low
						if sram_ce_n = '0' and E = '1' and E_prev = '0' then
							mrdy <= '0';
							if sram_we_n = '1' then
								state <= PERFORM_READ;
							else
								state <= PERFORM_WRITE;
							end if;
							delay <= 4;
						end if;
						
					when PERFORM_READ =>
						if delay = 0 then
							if sram_we_n = '1'then 
								sram_dout <= loopback;
							end if;
							mrdy <= '1';
							state <= WAIT_E_FALLING;
						else
							delay <= delay - 1;
						end if;
						
					when PERFORM_WRITE => 
						if delay = 0 then
							if sram_we_n = '0' then
								loopback <= sram_din;
							end if;
							mrdy <= '1';
							state <= WAIT_E_FALLING;
						else
							delay <= delay - 1;
						end if;

               when WAIT_E_FALLING =>
						if E = '0' and E_prev = '1' then
							state <= IDLE;
                  end if;

					when UNDEFINED =>
						state <= IDLE;
						
            end case;
			end if;
		end if;
	end process;

end rtl;

