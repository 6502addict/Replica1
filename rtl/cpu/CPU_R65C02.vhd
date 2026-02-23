-- ============================================================================
-- CPU_65C02.vhd
-- ============================================================================

library IEEE;
use IEEE.std_logic_1164.all;
use ieee.numeric_std.all;

entity CPU_R65C02 is
	port (
		-- Clock and Reset
		main_clk    : in  std_logic;        -- Main system clock
		reset_n     : in  std_logic;        -- Active low reset
		cpu_reset_n : in  std_logic;        -- cpu reset low
		phi2        : out std_logic;        -- Phase 2 clock enable
		
		-- CPU Control Interface
		rw          : out std_logic;        -- Read/Write (1=Read, 0=Write)
		vma         : out std_logic;        -- Valid Memory Access
		sync        : out std_logic;        -- Instruction fetch cycle
		
		-- Address and Data Bus
		addr        : out std_logic_vector(15 downto 0);  -- Address bus
		data_in     : in  std_logic_vector(7 downto 0);   -- Data input
		data_out    : out std_logic_vector(7 downto 0);   -- Data output
		
		-- Interrupt Interface   
		nmi_n       : in  std_logic;        -- Non-maskable interrupt (active low)
		irq_n       : in  std_logic;        -- Interrupt request (active low)
		so_n        : in  std_logic := '1'; -- Set overflow (active low)
		
		-- wait states
		mrdy        : in  std_logic
	);
end CPU_R65C02;

architecture CPU_R65C02_impl of CPU_R65C02 is

	component cpu_clock_gen is
		 Port (
			  clk_4x  : in  STD_LOGIC;
			  reset_n : in  STD_LOGIC;
			  mrdy    : in  STD_LOGIC;       -- Memory Ready
			  clk_1x  : out STD_LOGIC;       -- CPU clock (stretched)
			  clk_2x  : out STD_LOGIC;       -- 2x clock for 6502 cores
			  stretch : out STD_LOGIC        -- '1' only when actually stretching
		 );
	end component;

	component R65C02 is
		port (
			
			reset : in std_logic;
			clk : in std_logic;
			enable : in std_logic;
			nmi_n : in std_logic;
			irq_n : in std_logic;
			di : in unsigned(7 downto 0);
			do : out unsigned(7 downto 0);
			do_next : out unsigned(7 downto 0);
			addr : out unsigned(15 downto 0);
			addr_next : out unsigned(15 downto 0);
			nwe : out std_logic;
			nwe_next : out std_logic;
			sync : out std_logic;
			sync_irq : out std_logic
				
		);
	end component;	

	-- Internal signals
	signal data_bus      : std_logic_vector(7 downto 0);
	signal address_bus   : std_logic_vector(15 downto 0);
	signal cpu_data_out  : std_logic_vector(7 downto 0);
	signal phi2_internal : std_logic;

	-- CPU65XX specific signals
	signal r6502_do    : unsigned(7 downto 0);
	signal r6502_addr  : unsigned(15 downto 0);
	signal r6502_we    : std_logic;
	signal r6502_clk   : std_logic;
   
begin

	-- Input data bus assignment
	data_bus <= data_in;
	phi2     <= phi2_internal;

	clk: cpu_clock_gen port map(clk_4x  => main_clk,
						 			    reset_n => reset_n,
									    mrdy    => mrdy,
									    clk_1x  => phi2_internal,
									    clk_2x  => r6502_clk,
									    stretch => open);
											  
	-- R65C02 instantiation
	cpu65c02_inst: R65C02
		port map (
			reset     => cpu_reset_n,
			clk       => r6502_clk,
			enable    => not phi2_internal,
			nmi_n     => nmi_n,
			irq_n     => irq_n,
			di        => unsigned(data_bus),
			do        => r6502_do,
			do_next   => open,
			addr      => r6502_addr,
			addr_next => open,
			nwe       => rw,
			nwe_next  => open,
			sync      => sync,
			sync_irq  => open
		);
		
	-- Signal assignments for R6502
	cpu_data_out  <= std_logic_vector(r6502_do);
	address_bus   <= std_logic_vector(r6502_addr);

	-- Output assignments
	addr     <= address_bus;
	data_out <= cpu_data_out;
	vma      <= '1';

end CPU_R65C02_impl;