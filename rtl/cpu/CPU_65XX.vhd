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
end CPU_65XX;

architecture CPU65XX_impl of CPU_65XX is

	component clock_divider is
		 generic (divider : integer := 4);
		 port (
			  reset    : in  std_logic := '1';
			  clk_in   : in  std_logic;
			  clk_out  : out std_logic
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

begin

	-- Input data bus assignment
	data_bus <= data_in;
	phi2     <= phi2_internal;
	
	clk:  clock_divider generic map(divider         => 2)  
								  port map(reset           => '1',
									  	     clk_in          => main_clk,
											  clk_out         => phi2_internal);

	-- CPU65XX Instantiation
	cpu65xx_inst: cpu65xx 
		generic map(
			pipelineOpcode  => false,
			pipelineAluMux  => false,
			pipelineAluOut  => false
		)
		port map(
			clk             => main_clk,
			enable          => phi2_internal,
			reset           => not reset_n,         -- CPU65XX uses active high reset
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
	vma_internal  <= '1'; --phi2_internal;               -- on a 6502 the addresses are valid when phi2 is high

	-- Output assignments
	addr     <= address_bus;
	data_out <= cpu_data_out;
	rw       <= rw_internal;
	sync     <= sync_internal;
	vma      <= vma_internal;

end CPU65XX_impl;