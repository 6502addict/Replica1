-- ============================================================================
-- Apple1_CPU_MX65.vhd
-- Derived from Apple1_CPU_Template.vhd
-- 
-- CPU: MX65 (Alternative 6502 implementation)
-- Dependencies: None (uses standard IEEE libraries only)
-- https://github.com/Steve-Teal/mx65
-- ============================================================================

library IEEE;
use IEEE.std_logic_1164.all;
use ieee.numeric_std.all;

entity CPU_MX65 is
	port (
		-- Clock and Reset
		main_clk    : in  std_logic;       -- Main system clock
		reset_n     : in  std_logic;       -- Active low reset
		cpu_reset_n : in  std_logic;       -- Active low cpu reset
		phi2        : out std_logic;       -- Phase 2 clock output (divided from main_clk)
		
		-- CPU Control Interface
		rw          : out std_logic;       -- Read/Write (1=Read, 0=Write)
		vma         : out std_logic;       -- Valid Memory Access
		sync        : out std_logic;       -- Instruction fetch cycle
		
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
end CPU_MX65;

architecture MX65_impl of CPU_MX65 is

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
	
	-- MX65 Component
	component mx65 is
		port (
			clock		: in  std_logic;                      -- Clock
			reset		: in  std_logic;                      -- Reset (active high)
			ce			: in  std_logic;                      -- Clock enable
			data_in  : in  std_logic_vector(7 downto 0);   -- Data input
			data_out	: out std_logic_vector(7 downto 0);   -- Data output
			address	: out std_logic_vector(15 downto 0);  -- Address bus
			rw			: out std_logic;                      -- Read/Write
			sync		: out std_logic;                      -- Sync
			nmi		: in  std_logic;                      -- NMI (active high)
			irq		: in  std_logic                       -- IRQ (active high)
		);
	end component;

	-- Internal signals
	signal data_bus      : std_logic_vector(7 downto 0);
	signal address_bus   : std_logic_vector(15 downto 0);
	signal cpu_data_out  : std_logic_vector(7 downto 0);
	signal phi2_internal : std_logic;

	-- MX65 specific signals
	signal mx65_rw       : std_logic;
	signal mx65_sync     : std_logic;
	signal mx65_clk      : std_logic;
	
	
	begin
	
	clk: cpu_clock_gen port map(clk_4x  => main_clk,
						 	          reset_n => reset_n,
							          mrdy    => mrdy,
							          clk_1x  => phi2_internal,
							          clk_2x  => mx65_clk,
							          stretch => open);	
	
	
	phi2 <= phi2_internal;

	-- Input data bus assignment
	data_bus <= data_in;

	-- MX65 Instantiation
	mx65_inst: mx65 
		port map(
			clock           => mx65_clk,
			reset           => not cpu_reset_n,         -- MX65 uses active high reset
			ce              => not phi2_internal,
			data_in         => data_bus,
			data_out        => cpu_data_out,
			address         => address_bus,
			rw              => mx65_rw,
			sync            => sync,
			nmi             => nmi_n,              -- MX65 uses active low
			irq             => irq_n               -- MX65 uses active low
		);

	-- Output assignments
	addr     <= address_bus;
	data_out <= cpu_data_out;
	rw       <= mx65_rw;
	vma      <= '1';

end MX65_impl;