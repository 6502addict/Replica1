library IEEE;
	use ieee.std_logic_1164.all;
	use ieee.numeric_std.all;
	

entity spi_master is
	port (
		clk         : in     std_logic;        
		reset_n     : in     std_logic;     
		spi_req     : in     std_logic;
		spi_divider : in     std_logic_vector(7 downto 0);
		spi_busy    : out    std_logic;      
		data_in     : in     std_logic_vector(7 downto 0);  
		data_out    : out    std_logic_vector(7 downto 0);
		spi_enable  : in     std_logic := '0';
		cpol        : in     std_logic;  									
		cpha        : in     std_logic;  									
		spi_sck     : out    std_logic := '1';
		spi_cs_n    : out    std_logic := '1';
		spi_mosi    : out    std_logic := '1';
		spi_miso    : in     std_logic
	);
end spi_master;

architecture behavioral of spi_master is

	component prog_clock_divider IS
		generic (bits : integer := 8);
		port (
			reset_n  : in  std_logic := '1';  				
			clk_in   : in  std_logic;
			divider  : in  std_logic_vector(bits -1 downto 0);  
			clk_out  : out std_logic
		);
	end component;

	signal rx_reg     : std_logic_vector(7 downto 0) := x"00";
	signal tx_reg     : std_logic_vector(7 downto 0) := x"00";
	signal step       : integer range 0 to 17;
	signal sck        : std_logic;
	signal busy       : std_logic := '0';
	signal base_clock : std_logic;

begin
  
	clock : prog_clock_divider  generic map (bits    => 8)  
										    port map (reset_n => reset_n,
														  clk_in  => clk,
														  divider => spi_divider,
														  clk_out => base_clock);

	spi_sck  <= sck;
	spi_cs_n <= '0' when spi_enable = '1' else '1';
	spi_busy <= busy;
	
	process(base_clock, reset_n, cpol, cpha)
	begin
		if reset_n = '0' then
			busy      <= '0';                
			step      <= 0;
			sck       <= cpol;
			spi_mosi  <= not cpol;
		elsif falling_edge(base_clock) then
			if busy = '0' and spi_req ='0' then
				busy     <= '1';                
				step     <= 0;
				sck      <= cpol;
				spi_mosi <= not cpol;
				tx_reg   <= data_in;
			elsif busy = '1' then
				if step < 16 then
					if cpha = '0' then
						if step mod 2 = 0 then
							spi_mosi <= tx_reg(7);
							tx_reg   <= tx_reg(6 downto 0) & '0';
						else 
							spi_mosi <= not cpol;
							rx_reg   <= rx_reg(6 downto 0) & spi_miso;
						end if;
					else
						if step mod 2 = 0 then
							rx_reg   <= rx_reg(6 downto 0) & spi_miso;
							spi_mosi <= not cpol;
						else 
							spi_mosi <= tx_reg(7);
							tx_reg   <= tx_reg(6 downto 0) & '0';
						end if;
					end if;
					sck      <= not sck; 
				end if;
				if step = 16 then 
					data_out <= rx_reg;
					busy     <= '0';
					sck      <= cpol;
					spi_mosi <= not cpol;
				end if;
				if step < 17 then
					step <= step + 1;
				else
					step <= 0;
				end if;
			end if;
		end if;
	end process;

end behavioral;