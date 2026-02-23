-- ============================================================================
-- Apple1_CPU_65XX.vhd
-- Derived from Apple1_CPU_Template.vhd
-- 
-- CPU: CPU65XX (Modern 6502 implementation)
-- Dependencies: None (uses standard IEEE libraries only)
-- https://www.syntiac.com/fpga64.html
-- https://github.com/emard/apple2fpga/blob/master/cpu6502.vhd
-- ============================================================================

library IEEE;
use IEEE.std_logic_1164.all;
use ieee.numeric_std.all;

entity CPU_65XX is
	port (
		-- Clock and Reset
		main_clk    : in  std_logic;        -- Main system clock
		reset_n     : in  std_logic;        -- Active low reset
		cpu_reset_n : in  std_logic;        -- Active low cpu reset
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
end CPU_65XX;

architecture CPU65XX_impl of CPU_65XX is

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

	-- CPU65XX Component
	component cpu65xx is
		generic (
			pipelineOpcode : boolean;
			pipelineAluMux : boolean;
			pipelineAluOut : boolean
		);
		port (
			clk : in std_logic;
			enable : in std_logic;
			reset : in std_logic;
			nmi_n : in std_logic;
			irq_n : in std_logic;
			so_n : in std_logic := '1';
			di : in unsigned(7 downto 0);
			do : out unsigned(7 downto 0);
			addr : out unsigned(15 downto 0);
			we : out std_logic;
			debugOpcode : out unsigned(7 downto 0);
			debugPc : out unsigned(15 downto 0);
			debugA : out unsigned(7 downto 0);
			debugX : out unsigned(7 downto 0);
			debugY : out unsigned(7 downto 0);
			debugS : out unsigned(7 downto 0)
		);
	end component;

	-- Internal signals
	signal data_bus      : std_logic_vector(7 downto 0);
	signal address_bus   : std_logic_vector(15 downto 0);
	signal cpu_data_out  : std_logic_vector(7 downto 0);
	signal rw_internal   : std_logic;
	signal sync_internal : std_logic;
	signal vma_internal  : std_logic;
	signal phi2_internal : std_logic;

	-- CPU65XX specific signals
	signal cpu65xx_do    : unsigned(7 downto 0);
	signal cpu65xx_addr  : unsigned(15 downto 0);
	signal cpu65xx_we    : std_logic;
	signal cpu65xx_clk   : std_logic;

begin

	-- Input data bus assignment
	data_bus <= data_in;
	phi2     <= phi2_internal;
	
	clk: cpu_clock_gen port map(clk_4x  => main_clk,
						 			    reset_n => reset_n,
									    mrdy    => mrdy,
									    clk_1x  => phi2_internal,
									    clk_2x  => cpu65xx_clk,
									    stretch => open);

	-- CPU65XX Instantiation
	cpu65xx_inst: cpu65xx 
		generic map(
			pipelineOpcode  => false,
			pipelineAluMux  => false,
			pipelineAluOut  => false
		)
		port map(
			clk             => cpu65xx_clk,
			enable          => not phi2_internal,
			reset           => not cpu_reset_n,         -- CPU65XX uses active high reset
			nmi_n           => nmi_n,
			irq_n           => irq_n,
			so_n            => so_n,
			di              => unsigned(data_bus),
			do              => cpu65xx_do,
			addr            => cpu65xx_addr,
			we              => cpu65xx_we,
			debugOpcode     => open,
			debugPc         => open,
			debugA          => open,
			debugX          => open,
			debugY          => open,
			debugS          => open
		);
		
	-- Signal assignments for CPU65XX
	cpu_data_out  <= std_logic_vector(cpu65xx_do);
	address_bus   <= std_logic_vector(cpu65xx_addr);
	rw_internal   <= not cpu65xx_we;              -- Convert WE to RW
	sync_internal <= '0';                         -- CPU65XX doesn't have sync
	vma_internal  <= '1';                         -- on a 6502 the addresses are valid when phi2 is high

	-- Output assignments
	addr     <= address_bus;
	data_out <= cpu_data_out;
	rw       <= rw_internal;
	sync     <= sync_internal;
	vma      <= vma_internal;

end CPU65XX_impl;