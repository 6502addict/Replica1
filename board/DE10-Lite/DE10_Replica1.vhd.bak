--------------------------------------------------------------------------
-- To configure the machine search "Board Configuration Parameters"
-- and adapt the configuration to your needs 
--------------------------------------------------------------------------


library IEEE;
	use IEEE.std_logic_1164.all;
   use ieee.numeric_std.all; 
	
entity DE10_Replica1 is
  port (
		ADC_CLK_10      :	in	    std_logic;
	
		MAX10_CLK1_50   :	in     std_logic;
		MAX10_CLK2_50   :	in     std_logic;

		DRAM_ADDR       :	out    std_logic_vector(12 downto 0);
		DRAM_BA         :	out    std_logic_vector(1 downto 0);
		DRAM_CAS_N      :	out    std_logic;
		DRAM_CKE        :	out    std_logic;
		DRAM_CLK        :	out    std_logic;
		DRAM_CS_N       :	out    std_logic;
		DRAM_DQ         :	inout  std_logic_vector(15 downto 0);
		DRAM_LDQM       :	out    std_logic;
		DRAM_RAS_N      :	out	 std_logic;
		DRAM_UDQM       :	out    std_logic;
		DRAM_WE_N       :	out	 std_logic;

		HEX0				 :	out    std_logic_vector(7 downto 0);
		HEX1				 :	out    std_logic_vector(7 downto 0);
		HEX2				 :	out    std_logic_vector(7 downto 0);
		HEX3				 :	out    std_logic_vector(7 downto 0);
		HEX4				 :	out    std_logic_vector(7 downto 0);
		HEX5				 :	out    std_logic_vector(7 downto 0);

		KEY				 :	in		 std_logic_vector(1 downto 0);

		LEDR				 :	out	 std_logic_vector(9 downto 0);

		SW					 :	in		 std_logic_vector(9 downto 0);
		
		VGA_B				 :	out	 std_logic_vector(3 downto 0);
		VGA_G				 :	out    std_logic_vector(3 downto 0);
		VGA_HS			 :	out	 std_logic;
		VGA_R				 : out	 std_logic_vector(3 downto 0);
		VGA_VS			 :	out	 std_logic;
		
		GSENSOR_CS_N	 :	out	 std_logic;
		GSENSOR_INT		 :	in		 std_logic_vector(2 downto 1);
		GSENSOR_SCLK	 : out    std_logic;
		GSENSOR_SDI		 :	inout  std_logic;
		GSENSOR_SDO	 	 :	inout  std_logic;

		ARDUINO_IO		 : inout  std_logic_vector(15 downto 0);
		ARDUINO_RESET_N : inout  std_logic;

		GPIO            :	inout  std_logic_vector(35 downto 0)
  );
end entity;	

architecture top of DE10_Replica1 is

component hexto7seg is
  port (
	   hex           : in   std_logic_vector(3 downto 0);
		seg           : out  std_logic_vector(7 downto 0)
	);
end component;	

component main_clock is
	port (
		areset		: in  std_logic  := '0';
		inclk0		: in  std_logic  := '0';
		c0	  	      : out std_logic;
		c1	  	      : out std_logic;
		locked		: out std_logic 
	);
end component;

component clock_divider is
    generic (divider : integer := 4);
    port (
        reset    : in  std_logic := '1';
        clk_in   : in  std_logic;
        clk_out  : out std_logic
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
	
component RAM_DE10 is
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
end component;	

component Replica1_CORE is
  generic (
		 BOARD           : string   := "DE1_Lite";
		 CPU_TYPE        : string   := "6502";     -- 6502 or 6800
  	    CPU_SPEED       : string   := "1mhz";     -- "debug", "1hz", "1Mhz", "2Mhz" "5Mhz", "10Mhz", "30Mhz"
  	    ROM             : string   := "WOZMON65"; -- default monitor
		 RAM_SIZE_KB     : positive := 8;          -- 8kb to 48kb
  	    BAUD_RATE       : integer  := 9600;       -- uart speed 1200 to 115200
		 HAS_ACI         : boolean  := false;      -- add the aci (incomplete)
		 HAS_MSPI        : boolean  := false;      -- add master spi  C200
		 HAS_TIMER       : boolean  := false       -- add basic timer
	);
  port (
  		main_clk       : in     std_logic;
  		serial_clk     : in     std_logic;
		reset_n        : in     std_logic;
		cpu_reset_n    : in     std_logic;
		bus_phi2       : out    std_logic;
		bus_address    : out    std_logic_vector(15 downto 0);
		bus_data       : out    std_logic_vector(7  downto 0);
		bus_rw         : out    std_logic;
		ext_ram_cs_n   : out    std_logic;		
		ext_ram_data   : in     std_logic_vector(7  downto 0);
		uart_rx        : in     std_logic;
		uart_tx        : out    std_logic;
		spi_cs         : out    std_logic;
		spi_sck        : out    std_logic;
		spi_mosi       : out    std_logic;
		spi_miso       : in     std_logic;
		tape_out       : out    std_logic;
		tape_in        : in     std_logic
  );
end component;	

component debug_clock_button is
    port (
        clk_1hz_in   : in  STD_LOGIC;         
        debug_btn    : in  STD_LOGIC;         
        debug_clk    : out STD_LOGIC         
    );
end component;

component simple_clock_switch is
    port (
        -- Available clocks
        clock_debug : in  STD_LOGIC;
        clock_1hz   : in  STD_LOGIC;
        clock_1mhz  : in  STD_LOGIC;
        clock_2mhz  : in  STD_LOGIC;
        clock_5mhz  : in  STD_LOGIC;
        clock_10mhz : in  STD_LOGIC;
        clock_15mhz : in  STD_LOGIC;
        clock_30mhz : in  STD_LOGIC;
        
        -- Switch selection (only 3 switches needed)
        SW          : in  STD_LOGIC_VECTOR(2 downto 0);  -- SW(2 downto 0)
        
        -- Output
        main_clock  : out STD_LOGIC
    );
end component;

--------------------------------------------------------------------------
-- Board Configuration Parameters 
--------------------------------------------------------------------------
constant BOARD          : string   := "DE10_Lite";
constant CPU_TYPE       : string   := "6809";
constant CPU_SPEED      : string   := "1mhz";
constant ROM            : string   := "MON6809";
constant RAM_SIZE_KB    : positive := 48;        -- DE10-Lite supports up to 48KB
constant BAUD_RATE      : integer  := 115200;
constant HAS_ACI        : boolean  := false;
constant HAS_MSPI       : boolean  := true;
constant HAS_TIMER      : boolean  := true;

signal  address_bus    : std_logic_vector(15 downto 0);
signal  data_bus       : std_logic_vector(7 downto 0);
signal  sw_prev        : std_logic_vector(2 downto 0);
signal  ram_data       : std_logic_vector(7 downto 0);
signal  ram_cs_n       : std_logic;
signal  reset_n        : std_logic;
signal  cpu_reset_n    : std_logic;
signal  main_clk       : std_logic;
signal  raw_clk        : std_logic;
signal  clock_debug    : std_logic;
signal  clock_1hz      : std_logic;
signal  clock_1mhz     : std_logic;
signal  clock_2mhz     : std_logic;
signal  clock_5mhz     : std_logic;
signal  clock_10mhz    : std_logic;
signal  clock_15mhz    : std_logic;
signal  clock_30mhz    : std_logic;
signal  serial_clk     : std_logic;
signal  main_locked    : std_logic;
signal  phi2           : std_logic;
signal  rw             : std_logic;
signal  ram_cs         : std_logic;
signal  rom_cs         : std_logic;
signal  spi_cs         : std_logic;
signal  spi_sck        : std_logic;
signal  spi_mosi       : std_logic;
signal  spi_miso       : std_logic;
	
begin
	-- on the DE10 Lite the 7 seg display show the current address and data 
	h0 : hexto7seg port map  (hex => data_bus(3  downto 0),     seg => HEX0); 
	h1 : hexto7seg port map  (hex => data_bus(7  downto 4),     seg => HEX1); 
	h2 : hexto7seg port map  (hex => address_bus(3 downto 0),   seg => HEX2); 
	h3 : hexto7seg port map  (hex => address_bus(7 downto 4),   seg => HEX3); 
	h4 : hexto7seg port map  (hex => address_bus(11 downto 8),  seg => HEX4); 
	h5 : hexto7seg port map  (hex => address_bus(15 downto 12), seg => HEX5); 

	-- reset_n is mapped to KEY 0 of the DE10 Lite
	-- reset_n is used to reset low level layers of the fpga modules
	reset_n <= KEY(0);
	
	
	-- cpu_reset_n can only go high if reset_n is high and the PLL locked
	-- cpu_reset_n only reset the cpu and the peripherals
	cpu_reset_n <= '1' when reset_n = '1' and main_locked = '1' else '0';
	
	-- MAIN clock  note on 6502 core main clock is at least twice phi2
	mclk: main_clock                port map(areset		     => not reset_n,
		                                      inclk0		     => MAX10_CLK1_50,
		                                      c0	  	        => clock_30mhz,
		                                      c1	  	        => open,
		                                      locked	  	     => main_locked);
														  

	-- not these clock are the main clock used by the core
	-- this clock is internaly divided by 2 so the main clock is twice the cpu clock

	dbg: debug_clock_button              port map(clk_1hz_in      => clock_1hz,
														       debug_btn       => KEY(1),
														       debug_clk       => clock_debug);
																 
	clk1hz: clock_divider             generic map(divider        => 50_000_000/2)
	 									  	       port map(reset          => '1',
																 clk_in         => MAX10_CLK1_50,
														       clk_out        => clock_1hz);

	clk1mhz: clock_divider            generic map(divider        => 50_000_000/2_000_000)
	 									  	       port map(reset          => '1',
																 clk_in         => MAX10_CLK1_50,
														       clk_out        => clock_1mhz);

	clk2mhz: clock_divider            generic map(divider        => 50_000_000/4_000_000)
	 									  	       port map(reset          => '1',
																 clk_in         => MAX10_CLK1_50,
														       clk_out        => clock_2mhz);
															
	clk5mhz: clock_divider             generic map(divider       => 50_000_000/10_000_000)
													  port map(reset         => '1',
													           clk_in        => MAX10_CLK1_50,
														        clk_out       => clock_5mhz);

	clk10mhz: clock_divider            generic map(divider       => 60_000_000/20_000_000)
	 									  	        port map(reset         => '1',
														        clk_in        => clock_30mhz,
												   		     clk_out       => clock_10mhz);

	clk15mhz: clock_divider            generic map(divider       => 60_000_000/30_000_000)
	 									  	        port map(reset         => '1',
														        clk_in        => clock_30mhz,
														        clk_out       => clock_15mhz);

	csw: simple_clock_switch              port map(clock_debug   => clock_debug, -- 000
																  clock_1hz     => clock_1hz,   -- 001
																  clock_1mhz    => clock_1mhz,  -- 010
																  clock_2mhz    => clock_2mhz,  -- 011
																  clock_5mhz    => clock_5mhz,  -- 100
																  clock_10mhz   => clock_10mhz, -- 101 
																  clock_15mhz   => clock_15mhz, -- 110 
																  clock_30mhz   => clock_30mhz, -- 111 
																  SW            => SW(2 downto 0),
																  main_clock    => raw_clk);
	process(clock_30mhz) 
	begin
		if rising_edge(clock_30mhz)  then
			sw_prev <= SW(2 downto 0);
		end if;
	end process;

	main_clk <= raw_clk when SW(2 downto 0) = sw_prev else '0';
			
																	
	
	-- UART baud rate clock  1.8432Mhz base serial clock
	uclk: fractional_clock_divider   generic map(CLK_FREQ_HZ    => 50_000_000,
										 	    	  		   FREQUENCY_HZ   => 1_843_200)
	 									  	      port map(clk_in         => MAX10_CLK1_50,
												   		   reset_n        => reset_n,
														      clk_out        => serial_clk);
															 
	ap1: Replica1_CORE               generic map(BOARD          =>  BOARD,
										  				      CPU_TYPE       =>  CPU_TYPE,    -- 6502 or 6800
														      CPU_SPEED      =>  CPU_SPEED,   -- "debug", "1hz", "1Mhz", "2Mhz" "5Mhz", "10Mhz", "30Mhz"
															   ROM            =>  ROM,         -- default wozmon65
														      RAM_SIZE_KB    =>  RAM_SIZE_KB, -- 8 to 48Kb 
														      BAUD_RATE      =>  BAUD_RATE,   -- uart speed 1200 to 115200
												  		      HAS_ACI        =>  HAS_ACI,     -- add the aci (incomplete)
		                                          HAS_MSPI       =>  HAS_MSPI,    -- add master spi  C200
		                                          HAS_TIMER      =>  HAS_TIMER)   -- add basic timer C210
									            port map(main_clk       =>  main_clk,
											               serial_clk     =>  serial_clk,
													   	   reset_n        =>  reset_n,
														      cpu_reset_n    =>  cpu_reset_n,
														      bus_phi2       =>  phi2,    
														      bus_address    =>  address_bus,
														      bus_data       =>  data_bus,
														      bus_rw         =>  rw,
																ext_ram_cs_n   =>  ram_cs_n,
																ext_ram_data   =>  ram_data,
														      uart_rx        =>  ARDUINO_IO(0),
														      uart_tx        =>  ARDUINO_IO(1),
														      spi_cs         =>  ARDUINO_IO(4),   -- SD Card Data 3          CS
														      spi_sck        =>  ARDUINO_IO(13),  -- SD Card Clock           SCLK
														      spi_mosi       =>  ARDUINO_IO(11),  -- SD Card Command Signal  MOSI
														      spi_miso       =>  ARDUINO_IO(12),  -- SD Card Data            MISO
														      tape_out       =>  ARDUINO_IO(3),
														      tape_in        =>  ARDUINO_IO(2));


--de10_gen_ram: if BOARD = "DE10_Lite" generate
	ram: RAM_DE10                    generic map(RAM_SIZE_KB   => RAM_SIZE_KB)
								               port map(clock           => phi2,
					   	                           cs_n            => ram_cs_n,
											               we_n            => rw,
											               address         => address_bus,
											               data_in         => data_bus,
							                           data_out        => ram_data);
--end generate de10_gen_ram;																

																
	DRAM_ADDR      <= (others => 'Z');
	DRAM_BA        <= (others => 'Z');
	DRAM_CAS_N     <= 'Z';  
	DRAM_CKE       <= 'Z';  
	DRAM_CLK       <= 'Z';  
	DRAM_CS_N      <= 'Z';  
	DRAM_DQ        <= (others => 'Z');  
	DRAM_LDQM      <= 'Z';  
	DRAM_RAS_N     <= 'Z';  
	DRAM_UDQM      <= 'Z';  
	DRAM_WE_N      <= 'Z';  
	
	VGA_B		   	<= (others => 'Z');
	VGA_G	   		<= (others => 'Z'); 
	VGA_HS	   	<= 'Z';
	VGA_R		   	<= (others => 'Z');  
	VGA_VS	   	<= 'Z';
		
	GSENSOR_CS_N	<= 'Z';
	GSENSOR_SCLK	<= 'Z';
	GSENSOR_SDI		<= 'Z';
	GSENSOR_SDO	 	<= 'Z';


	
end top;

