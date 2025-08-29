library IEEE;
	use IEEE.std_logic_1164.all;
   use ieee.numeric_std.all; 
	
entity Replica1_CORE is
  generic (
		BOARD           : string  :=  "DE10_Lite";   -- DE10_Lite or DE1
		CPU_TYPE        : string  :=  "6502";        -- 6502 or 6800
 	   CPU_SPEED       : string  :=  "1mhz";        -- "debug", "1hz", "1Mhz", "2Mhz" "5Mhz", "10Mhz", "30Mhz"
		ROM             : string  :=  "WOZMON65";    -- default wozmon65
		RAM_SIZE_KB     : integer :=  8;             -- 8 to 48kb
	   BAUD_RATE       : integer :=  115200;        -- uart speed 1200 to 115200
		HAS_ACI         : boolean :=  false;         -- add the aci (incomplete)
		HAS_MSPI        : boolean :=  false;         -- add master spi  C200
		HAS_TIMER       : boolean :=  false          -- add basic timer
--  	   HAS_BASIC       : boolean :=  false;         -- true basic installed, false only wozmon		
--		HAS_MON6809     : boolean :=  false          -- alternative enhanced 6809 monitor
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
		ext_ram_cs_n    : out    std_logic;		
		ext_ram_data    : in     std_logic_vector(7  downto 0);
		uart_rx         : in     std_logic;
		uart_tx         : out    std_logic;
		spi_cs          : out    std_logic;
		spi_sck         : out    std_logic;
		spi_mosi        : out    std_logic;
		spi_miso        : in     std_logic;
		tape_out        : out    std_logic;
		tape_in         : in     std_logic
  );
end entity;	

architecture rtl of Replica1_CORE is


-- PROCESSOR MODULES

component CPU_65XX is
	port (
		-- Clock and Reset
		main_clk : in  std_logic;       -- Main system clock
		reset_n  : in  std_logic;       -- Active low reset
		phi2     : out std_logic;       -- Phase 2 clock enable
		
		-- CPU Control Interface
		rw       : out std_logic;       -- Read/Write (1=Read, 0=Write)
		vma      : out std_logic;       -- Valid Memory Access
		sync     : out std_logic;       -- Instruction fetch cycle
		
		-- Address and Data Bus
		addr     : out std_logic_vector(15 downto 0);  -- Address bus
		data_in  : in  std_logic_vector(7 downto 0);   -- Data input
		data_out : out std_logic_vector(7 downto 0);   -- Data output
		
		-- Interrupt Interface  
		nmi_n    : in  std_logic;       -- Non-maskable interrupt (active low)
		irq_n    : in  std_logic;       -- Interrupt request (active low)
		so_n     : in  std_logic := '1' -- Set overflow (active low)
	);
end component;

component CPU_6800 is
	port (
		-- Clock and Reset
		main_clk : in  std_logic;       -- Main system clock
		reset_n  : in  std_logic;       -- Active low reset
		phi2     : out std_logic;       -- Phase 2 clock output (divided from main_clk)
		
		-- CPU Control Interface
		rw       : out std_logic;       -- Read/Write (1=Read, 0=Write)
		vma      : out std_logic;       -- Valid Memory Access
		sync     : out std_logic;       -- Instruction fetch cycle
		
		-- Address and Data Bus
		addr     : out std_logic_vector(15 downto 0);  -- Address bus
		data_in  : in  std_logic_vector(7 downto 0);   -- Data input
		data_out : out std_logic_vector(7 downto 0);   -- Data output
		
		-- Interrupt Interface  
		nmi_n    : in  std_logic;       -- Non-maskable interrupt (active low)
		irq_n    : in  std_logic;       -- Interrupt request (active low)
		so_n     : in  std_logic := '1' -- Set overflow (not used by 6800)
	);
end component;


component CPU_6809 is
	port (
		-- Clock and Reset
		main_clk : in  std_logic;       -- Main system clock
		reset_n  : in  std_logic;       -- Active low reset
		phi2     : out std_logic;       -- Phase 2 clock output (divided from main_clk)
		
		-- CPU Control Interface
		rw       : out std_logic;       -- Read/Write (1=Read, 0=Write)
		vma      : out std_logic;       -- Valid Memory Access
		sync     : out std_logic;       -- Instruction fetch cycle
		
		-- Address and Data Bus
		addr     : out std_logic_vector(15 downto 0);  -- Address bus
		data_in  : in  std_logic_vector(7 downto 0);   -- Data input
		data_out : out std_logic_vector(7 downto 0);   -- Data output
		
		-- Interrupt Interface  
		nmi_n    : in  std_logic;       -- Non-maskable interrupt (active low)
		irq_n    : in  std_logic;       -- Interrupt request (active low)
		so_n     : in  std_logic := '1' -- Set overflow (not used by 6800)
	);
end component;


-- END OF PROCESSOR MODULES

-- ROM MODULES
	
component WOZMON65 
	port (
		clock    : in std_logic;
		cs_n     : in std_logic;
		address  : in  std_logic_vector(7 downto 0); 
		data_out : out std_logic_vector(7 downto 0)
	);
end component;

component WOZMON68 
	port (
		clock    : in std_logic;
		cs_n     : in std_logic;
		address  : in  std_logic_vector(7 downto 0); 
		data_out : out std_logic_vector(7 downto 0)
	);
end component;

component WOZMON69
	port (
		clock    : in std_logic;
		cs_n     : in std_logic;
		address  : in  std_logic_vector(7 downto 0); 
		data_out : out std_logic_vector(7 downto 0)
	);
end component;

component WOZACI is
    port (
        clock:    in std_logic;
        cs_n:     in std_logic;
        address:  in std_logic_vector(7 downto 0);
        data_out: out std_logic_vector(7 downto 0)
    );
end component;

component BASIC
	port (
		clock    : in std_logic;
		cs_n     : in std_logic;
		address  : in  std_logic_vector(13 downto 0); 
		data_out : out std_logic_vector(7 downto 0)
	);
end component;

component MON6809 is
    port (
        clock:    in std_logic;
        address:  in std_logic_vector(11 downto 0);
        cs_n:     in std_logic;
        data_out: out std_logic_vector(7 downto 0)
    );
end component;

-- END ROM MODULES

-- RAM MODULES

--component RAM_DE10 is
--    generic (
--        RAM_SIZE_KB : integer := 32  -- 8, 16, 24, 32, 40, or 48
--    );
--    port (
--        clock:      in std_logic;
--        cs_n:       in std_logic;
--        we_n:       in std_logic;
--        address:    in std_logic_vector(15 downto 0);
--        data_in:    in std_logic_vector(7 downto 0);
--        data_out:   out std_logic_vector(7 downto 0)
--    );
--end component;

-- END RAM MODULES

-- PERIPHERAL COMPONENTS

component ACI is
    port (
		  reset_n   : in  std_logic;                           -- reset
        phi2      : in  std_logic;                           -- clock named phi2 on the 6502
        cs_n      : in  std_logic;                           -- CXXX chip select²
        address   : in  std_logic_vector(15 downto 0);       -- addresses
        data_out  : out std_logic_vector(7 downto 0);        -- data output
		  tape_in   : in  std_logic;                           -- tape input
		  tape_out  : out std_logic                            -- tape output
    );
end component;

component PIA_UART is
  generic (
     CLK_FREQ_HZ     : positive := 50000000;  
     BAUD_RATE       : positive := 9600;      
     BITS            : positive := 8          
  );
  port (
    -- System interface
    clock       : in  std_logic;    -- CPU clock
    serial_clk  : in  std_logic;    -- Serial clock
    reset_n     : in  std_logic;    -- Active low reset
    
    -- CPU interface
    cs_n        : in  std_logic;                     -- Chip select
    rw          : in  std_logic;                     -- Read/Write: 1=read, 0=write
    address     : in  std_logic_vector(1 downto 0);  -- Register select (for 4 registers)
    data_in     : in  std_logic_vector(7 downto 0);  -- Data from CPU
    data_out    : out std_logic_vector(7 downto 0);  -- Data to CPU
    
    -- Physical UART interface
    rx          : in  std_logic;    -- Serial input
    tx          : out std_logic     -- Serial output
  );
end component;

component mspi_iface is
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
end component;

component simple_timer is
    port (
        phi2        : in  std_logic;                     -- 6502 clock
        reset_n     : in  std_logic;                     -- reset active low
        cs_n        : in  std_logic;                     -- Chip select (active low)
        rw          : in  std_logic;                     -- Read/Write (low = write)
        address     : in  std_logic_vector(1 downto 0);  -- Address bits A1,A0
        data_in     : in  std_logic_vector(7 downto 0);  -- Data from CPU
        data_out    : out std_logic_vector(7 downto 0);  -- Data to CPU
        timer_clk   : in  std_logic                      -- Timer clock (can be same as phi2 or faster)
    );
end component;

	attribute keep : string;

	constant RAM_LIMIT  : integer := RAM_SIZE_KB * 1024;

	signal data_bus	  : std_logic_vector(7 downto 0);
	signal address_bus  : std_logic_vector(15 downto 0);
	signal cpu_data	  : std_logic_vector(7 downto 0);
	signal pia_data	  : std_logic_vector(7 downto 0);
	signal rom_data	  : std_logic_vector(7 downto 0);
	signal ram_data	  : std_logic_vector(7 downto 0);
	signal mspi_data	  : std_logic_vector(7 downto 0);
	signal aci_data	  : std_logic_vector(7 downto 0);
	signal timer_data	  : std_logic_vector(7 downto 0);
	signal ram_addr 	  : std_logic_vector(18 downto 0);
	signal rw			  : std_logic;
	signal vma  		  : std_logic;
	signal nmi_n        : std_logic := '1';
	signal irq_n        : std_logic := '1';
	signal so_n         : std_logic := '1';
	signal ram_cs_n     : std_logic;
	signal rom_cs_n     : std_logic;
	signal aci_cs_n     : std_logic;
	signal mspi_cs_n    : std_logic;
	signal sspi_cs_n    : std_logic;
	signal timer_cs_n   : std_logic;
	signal pia_cs_n     : std_logic;
	signal fast_clk     : std_logic;
	signal phi2         : std_logic;
	signal sync         : std_logic;
	signal aci_in       : std_logic;
	signal aci_out      : std_logic;
	
	
	signal spi_sck_ext  : std_logic;
   signal spi_cs_ext   : std_logic;
   signal spi_mosi_ext : std_logic;
   signal spi_miso_ext : std_logic;

	attribute keep of nmi_n    : signal is "true";
	attribute keep of irq_n    : signal is "true";
	attribute keep of sync     : signal is "true";
	attribute keep of so_n     : signal is "true";
	
begin
--	assert false
--		report "RAM_LIMIT calculated as: " & integer'image(RAM_LIMIT) & " bytes (" & integer'image(RAM_SIZE_KB) & "KB configured)";

	bus_address    <= address_bus;
	bus_data       <= data_bus;
	bus_phi2       <= phi2;
	bus_rw         <= rw;
	ext_ram_cs_n   <= ram_cs_n;
	ram_data       <= ext_ram_data;
						
-- Apple 1 CPU can be either CPU65XX for the 6502 or  CPU68 for the 6800

-- CPU PORT MAP

gen_cpu0: if CPU_TYPE = "6502" generate
	cpu: CPU_65XX          port map(main_clk        => main_clk,
	                                reset_n         => cpu_reset_n,
											  phi2            => phi2,
											  rw              => rw,
											  vma             => vma,
											  sync            => sync,
											  addr            => address_bus,
											  data_in         => data_bus,
											  data_out        => cpu_data,
											  nmi_n           => nmi_n,
											  irq_n           => irq_n,
											  so_n            => so_n);
end generate gen_cpu0;
											  
gen_cpu1: if CPU_TYPE = "6800" generate
	cpu: CPU_6800          port map(main_clk        => main_clk,
	                                reset_n         => cpu_reset_n,
											  phi2            => phi2,
											  rw              => rw,
											  vma             => vma,
											  sync            => sync,
											  addr            => address_bus,
											  data_in         => data_bus,
											  data_out        => cpu_data,
											  nmi_n           => nmi_n,
											  irq_n           => irq_n,
											  so_n            => so_n);
end generate gen_cpu1;

gen_cpu2: if CPU_TYPE = "6809" generate
	cpu: CPU_6809          port map(main_clk        => main_clk,
	                                reset_n         => cpu_reset_n,
											  phi2            => phi2,
											  rw              => rw,
											  vma             => vma,
											  sync            => sync,
											  addr            => address_bus,
											  data_in         => data_bus,
											  data_out        => cpu_data,
											  nmi_n           => nmi_n,
											  irq_n           => irq_n,
											  so_n            => so_n);
end generate gen_cpu2;


-- END OF CPU PORT MAP

-- ROM PORT MAP
											  
--gen_wozmon: if HAS_BASIC = false generate
--	gen_woz65: if CPU_TYPE = "6502" generate
--	rom: WOZMON65          port map(clock           => phi2,
--							              cs_n            => rom_cs_n,
--	                                address         => address_bus(7 downto 0),
--							              data_out        => rom_data);
--	end generate gen_woz65;
--	gen_woz68: if CPU_TYPE = "6800" generate
--	rom: WOZMON68          port map(clock           => phi2,
--							              cs_n            => rom_cs_n,
--	                                address         => address_bus(7 downto 0),
--							              data_out        => rom_data);
--	end generate gen_woz68;
--	gen_woz69: if CPU_TYPE = "6809" generate
--	rom: WOZMON69          port map(clock           => phi2,
--							              cs_n            => rom_cs_n,
--	                                address         => address_bus(7 downto 0),
--							              data_out        => rom_data);
--	end generate gen_woz69;
--end generate gen_wozmon;

woz65: if ROM = "WOZMON65"  generate
	rom: WOZMON65          port map(clock           => phi2,
							              cs_n            => rom_cs_n,
	                                address         => address_bus(7 downto 0),
							              data_out        => rom_data);
end generate woz65;

basic65: if ROM = "BASIC65"  generate
	rom: BASIC             port map(clock           => phi2,
							              cs_n            => rom_cs_n,
	                                address         => address_bus(13 downto 0),
							              data_out        => rom_data);
end generate basic65;

woz68: if ROM = "WOZMON68"  generate
	rom: WOZMON68          port map(clock           => phi2,
							              cs_n            => rom_cs_n,
	                                address         => address_bus(7 downto 0),
							              data_out        => rom_data);
end generate woz68;

woz69: if ROM = "WOZMON69"  generate
	rom: WOZMON69          port map(clock           => phi2,
							              cs_n            => rom_cs_n,
	                                address         => address_bus(7 downto 0),
							              data_out        => rom_data);
end generate woz69;

mon69: if ROM = "MON6809" generate
	rom: MON6809           port map(clock           => phi2,
							              cs_n            => rom_cs_n,
	                                address         => address_bus(11 downto 0),
							              data_out        => rom_data);
end generate mon69;

-- END ROM PORT MAP

											  
gen_aci:  if HAS_ACI and CPU_TYPE = "6502" generate
	tape: ACI              port map(reset_n         => cpu_reset_n,
	                                phi2            => phi2,
	                                cs_n            => aci_cs_n,
											  address         => address_bus,
											  data_out        => aci_data,
											  tape_in         => aci_in,
											  tape_out        => aci_out);
end generate gen_aci;											  

										 
	pia: PIA_UART       generic map(CLK_FREQ_HZ     => 1843200, 
								 		     BAUD_RATE       => BAUD_RATE,
											  BITS            => 8)
								  port map(clock           => phi2,
 								           serial_clk      => serial_clk,
									   	  reset_n         => cpu_reset_n,
									  	     cs_n            => pia_cs_n,
									        rw              => rw,
									        address         => address_bus(1 downto 0),
										     data_in         => data_bus,
									        data_out        => pia_data,
								           rx              => uart_rx,
								           tx              => uart_tx);
	
							
gen_mspi: if HAS_MSPI = true generate
	mspi: mspi_iface       port map(phi2            => phi2, 
											  reset_n         => cpu_reset_n,
											  cs_n            => mspi_cs_n,
											  rw              => rw,
											  address         => address_bus(1 downto 0),
											  data_in         => data_bus,
											  data_out        => mspi_data,
											  spi_clk         => phi2,
											  spi_sck         => spi_sck,   
											  spi_cs_n        => spi_cs,    
											  spi_mosi        => spi_mosi,  
  											  spi_miso        => spi_miso); 
end generate gen_mspi;


gen_timer: if HAS_TIMER = true generate
	timer: simple_timer     port map(phi2          => phi2,
									  		   reset_n       => cpu_reset_n,
											   cs_n          => timer_cs_n,
											   rw            => rw,
											   address       => address_bus(1 downto 0),
											   data_in       => data_bus,
											   data_out      => timer_data,
											   timer_clk     => phi2);
end generate gen_timer;

											
   aci_cs_n     <= '0' when vma = '1' and address_bus(15 downto 9)   = x"C" & "000"      else '1';   -- IF WOZACI
   mspi_cs_n    <= '0' when vma = '1' and address_bus(15 downto 4)   = x"C20"            else '1';   -- IF MASTER SPI CONTROLLER
   timer_cs_n   <= '0' when vma = '1' and address_bus(15 downto 4)   = x"C21"            else '1';   -- IF TIMER
	pia_cs_n     <= '0' when vma = '1' and address_bus(15 downto 4)   = x"D01"            else '1';   -- REPLICA CONSOLE PIA
	
	data_bus <= cpu_data      when rw          = '0' else
		         rom_data      when rom_cs_n    = '0' else 
		         aci_data      when aci_cs_n    = '0' else 
		         mspi_data     when mspi_cs_n   = '0' else 
		         timer_data    when timer_cs_n  = '0' else 
		         ram_data      when ram_cs_n    = '0' else 
					pia_data      when pia_cs_n    = '0' else
		         address_bus(15 downto 8);                 
					
	
	process(vma, address_bus)
	begin
		 rom_cs_n <= '1';  -- Default inactive
		 
		 if vma = '1' then
			if ROM = "WOZMON65" then
				if address_bus(15 downto 8) = x"FF" then
					rom_cs_n <= '0';
				end if;
			elsif ROM = "WOZMON68" then
				if address_bus(15 downto 8) = x"FF" then
					rom_cs_n <= '0';
				end if;
			elsif ROM = "WOZMON69" then
				if address_bus(15 downto 8) = x"FF" then
					rom_cs_n <= '0';
				end if;
			elsif ROM = "BASIC65" then
				if address_bus(15 downto 13) = "111" then
					rom_cs_n <= '0';
				end if;
			elsif ROM = "MON6809" then
				if address_bus(15 downto 11) = "11111" then
					rom_cs_n <= '0';
				end if;
			else
				rom_cs_n <= '1';  
			end if;
		end if;
	end process;	


	
	-- Generalized bit-pattern based chip select
	process(vma, address_bus)
	begin
		 ram_cs_n <= '1';  -- Default inactive
		 
		 if vma = '1' then
			  case RAM_SIZE_KB is
					when 8 =>
						 if address_bus(15 downto 13) = "000" then
							  ram_cs_n <= '0';
						 end if;
						 
					when 16 =>
						 if address_bus(15 downto 14) = "00" then
							  ram_cs_n <= '0';
						 end if;
						 
					when 24 =>
						 if (address_bus(15 downto 14) = "00") or 
							 (address_bus(15 downto 14) = "01" and address_bus(13) = '0') then
							  ram_cs_n <= '0';
						 end if;
						 
					when 32 =>
						 if address_bus(15) = '0' then
							  ram_cs_n <= '0';
						 end if;
						 
					when 40 =>
						 if (address_bus(15 downto 14) /= "11") and 
							 not (address_bus(15 downto 14) = "10" and address_bus(13) = '1') then
							  ram_cs_n <= '0';
						 end if;
						 
					when 48 =>
						 if address_bus(15 downto 14) /= "11" then
							  ram_cs_n <= '0';
						 end if;
						 
					when others =>
						 ram_cs_n <= '1';  -- Invalid size
			  end case;
		 end if;
	end process;	
	
end rtl;

