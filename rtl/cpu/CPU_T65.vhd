-- ============================================================================
-- CPU_T65.vhd
-- Derived from Apple1_CPU_Template.vhd
-- 
-- CPU: T65 (FPGA-optimized 6502 core)
-- Dependencies: T65_Pack package, M6809_Clock_Gen
-- https://opencores.org/projects/t65
-- ============================================================================
library IEEE;
use IEEE.std_logic_1164.all;
use ieee.numeric_std.all;
use work.T65_Pack.all;      -- Required for T65 CPU

entity CPU_T65 is
	port (
		-- Clock and Reset
		main_clk    : in  std_logic;       -- Main system clock (4x CPU)
		reset_n     : in  std_logic;       -- Active low reset
		cpu_reset_n : in  std_logic;       -- Active low reset
		phi2        : out std_logic;       -- E clock
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
		mrdy        : in  std_logic         -- Memory Ready (Low = stretch clock)
	);
end CPU_T65;

architecture T65_impl of CPU_T65 is

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

	-- Internal signals
	signal t65_addr      : std_logic_vector(23 downto 0);
	signal t65_rw_n      : std_logic;
	signal t65_clk       : std_logic;
	signal phi2_internal : std_logic;

begin

	clk: cpu_clock_gen port map(clk_4x  => main_clk,
						 			    reset_n => reset_n,
									    mrdy    => mrdy,
									    clk_1x  => phi2_internal,
									    clk_2x  => t65_clk,
									    stretch => open);
	
	-- T65 Instantiation
	t65_inst: work.T65 
		port map(
			Mode    => "00",                        -- 6502 mode
			BCD_en  => '1',                         -- Enable BCD mode
			Res_n   => cpu_reset_n,                 -- T65 uses active low reset
			Enable  => not phi2_internal,           -- E as enable
			Clk     => t65_clk,
			Rdy     => '1',                         -- hardcoded for now, mrdy added once basic flow works
			Abort_n => '1',                         -- No abort
			IRQ_n   => irq_n,
			NMI_n   => nmi_n,
			SO_n    => so_n,
			R_W_n   => t65_rw_n,
			Sync    => sync,
			EF      => open,
			MF      => open,
			XF      => open, 
			ML_n    => open, 
			VP_n    => open,
			VDA     => open,
			VPA     => open,
			A       => t65_addr,
			DI      => data_in,
			DO      => data_out,
			Regs    => open,
			DEBUG   => open,
			NMI_ack => open
		);
		
	-- Output assignments
	addr  <= t65_addr(15 downto 0);
	rw    <= t65_rw_n;                    -- T65 R_W_n is already 1=Read, 0=Write
	phi2  <= phi2_internal;
	vma   <= '1';                         -- on a 6502 addresses are valid when phi2 is high

end T65_impl;

