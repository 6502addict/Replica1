library IEEE;
	use IEEE.std_logic_1164.all;
   use ieee.numeric_std.all; 

entity sram is
  port (
--    8 bits bus interface 
      address       : in    std_logic_vector(18 downto 0);
		data_in       : in    std_logic_vector(7  downto 0);
		data_out      : out   std_logic_vector(7  downto 0);
		cs_n          : in    std_logic;
		rw            : in    std_logic;
--    16 bits sram interface   		
 	   SRAM_DQ       : inout std_logic_vector(15 downto 0);  --	SRAM Data bus 16 Bits
		SRAM_ADDR     : out   std_logic_vector(17 downto 0);  --	SRAM Address bus 18 Bits
		SRAM_UB_N     : out   std_logic; 						   --	SRAM High-byte Data Mask 
		SRAM_LB_N	  : out   std_logic;                      --	SRAM Low-byte Data Mask 
		SRAM_WE_N     : out	 std_logic;								--	SRAM Write Enable
		SRAM_CE_N	  : out	 std_logic;								--	SRAM Chip Enable
		SRAM_OE_N	  : out	 std_logic								--	SRAM Output Enable
	);
end entity;	

architecture rtl of sram is
begin
	SRAM_ADDR            <= address(18 DOWNTO 1);
	SRAM_CE_N            <= '0' when cs_n = '0' else '1';
	SRAM_OE_N            <= '0' when cs_n = '0' and rw = '1' else '1';
	SRAM_WE_N            <= '0' when cs_n = '0' and rw = '0' else '1';
	SRAM_UB_N            <= '0' when address(0) = '1' else '1';
	SRAM_LB_N            <= '0' when address(0) = '0' else '1';
	SRAM_DQ(7  downto 0) <= data_in  when rw = '0' and address(0) = '0' else (others => 'Z');
	SRAM_DQ(15 downto 8) <= data_in  when rw = '0' and address(0) = '1' else (others => 'Z');
	data_out             <= SRAM_DQ(7 downto 0) when (address(0) = '0') else SRAM_DQ(15 downto 8);                     
end rtl;



