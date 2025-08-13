library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity mspi_iface is
    port (
        phi2        : in  std_logic;                     -- E on 6800/6809
        reset_n     : in  std_logic;                     -- reset_n active low
        cs_n        : in  std_logic;                     -- Chip select (active low)
        rw          : in  std_logic;                     -- Read/Write (low = write)
        address     : in  std_logic_vector(1 downto 0);  -- Address bit 0
        data_in     : in  std_logic_vector(7 downto 0);  -- Data from CPU
        data_out    : out std_logic_vector(7 downto 0);  -- Data to CPU
        spi_clk     : in  std_logic;                     -- spi base clock
        spi_sck     : out std_logic;
        spi_cs_n    : out std_logic;
        spi_mosi    : out std_logic;
        spi_miso    : in  std_logic
    );
end mspi_iface;

architecture rtl of mspi_iface is

    component spi_master is
        port (
            clk         : in  std_logic;
            reset_n     : in  std_logic;
            spi_req     : in  std_logic;
            spi_divider : in  std_logic_vector(7 downto 0);
            spi_busy    : out std_logic;
            data_in     : in  std_logic_vector(7 downto 0);
            data_out    : out std_logic_vector(7 downto 0);
				spi_enable  : in  std_logic := '0';
            cpol        : in  std_logic;
            cpha        : in  std_logic;
            spi_sck     : out std_logic;
            spi_cs_n    : out std_logic;
            spi_mosi    : out std_logic;
            spi_miso    : in  std_logic
        );
    end component;
    
    signal data_out_reg     : std_logic_vector(7 downto 0) := (others => '0');
    signal data_in_reg      : std_logic_vector(7 downto 0) := (others => '0');
    signal spi_req          : std_logic := '1';  -- Start at idle
    signal spi_busy         : std_logic;
    signal spi_enable       : std_logic;
    signal spi_data_in      : std_logic_vector(7 downto 0);
    signal spi_data_out     : std_logic_vector(7 downto 0);
    signal spi_divider      : std_logic_vector(7 downto 0);
    signal cpol             : std_logic;
    signal cpha             : std_logic;
    signal data_ready       : std_logic;
    signal spi_busy_last    : std_logic;
    signal spi_done         : std_logic := '0';
    signal spi_start        : std_logic := '0';
	 signal data_pending     : std_logic := '0';
	 
begin
	 
    SPI: spi_master   port map (clk         => spi_clk,
										  reset_n     => reset_n,
										  spi_req     => spi_req,
										  spi_divider => spi_divider,
										  spi_busy    => spi_busy,
										  data_in     => spi_data_in,
										  data_out    => spi_data_out,
										  spi_enable  => spi_enable,
										  cpol        => cpol,
										  cpha        => cpha,
										  spi_sck     => spi_sck,
										  spi_cs_n    => spi_cs_n,
										  spi_mosi    => spi_mosi,
										  spi_miso    => spi_miso);
										  
	process(phi2, reset_n)
	begin
		if reset_n = '0' then
			data_in_reg   <= (others => '0');
			data_out_reg  <= (others => '0');
         spi_req       <= '1'; 
         data_ready    <= '0';
         spi_busy_last <= '1';
         spi_done      <= '1';
			spi_enable    <= '0';
         spi_start     <= '0';
         cpol          <= '0';
         cpha          <= '0';
         spi_divider   <= (others => '1');
		elsif rising_edge(phi2) then
			spi_busy_last <= spi_busy;
            
         if spi_start = '1' then
				if spi_busy = '0' and spi_done = '0' then
					spi_data_in <= data_in_reg;
					spi_req     <= '0';
               spi_start   <= '0';
            end if;
         end if;
            
         if spi_busy_last = '0' and spi_busy = '1' and spi_done = '0' then
				spi_req  <= '1';
            spi_done <= '1';
         end if;
            
         if spi_busy_last = '1' and spi_busy = '0' and spi_done = '1' then
				data_out_reg  <= spi_data_out;
            data_ready    <= '1';
            spi_done      <= '0';
         end if;		
		
			if cs_n = '0' then
				case address is
					when "00" => -- 0xC200  COMMAND REGISTER
						if rw = '0' then
							-- Writing Command Register
							cpol        <= data_in(0);
                     cpha        <= data_in(1);
                     spi_enable  <= data_in(2);
							else	
							-- Reading Command Register
							data_out    <= "00000" & spi_enable & cpha & cpol;
						end if;

					when "01" => -- 0xC201  STATUS Register
						if rw = '0' then
							-- Writing Status Register (not permitted)
							null;
						else
							-- Reading Status Register
							data_out <= "000000" & not spi_busy & data_ready;
						end if;

					when "10" => -- 0xC202  DATA Register
						if rw = '0' then
							data_in_reg  <= data_in;
                     spi_start    <= '1';
                     data_ready   <= '0'; 
						else 
							data_out <= data_out_reg; 
							data_ready <= '0';
						end if;

					when "11" => -- 0xC203  DIVIDER Register
						if rw = '0' then
							spi_divider <= data_in;
						else
							data_out    <= spi_divider;
						end if;
                        
				end case;
         end if;
      end if;
	end process;

end architecture rtl;	 


