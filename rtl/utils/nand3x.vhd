library ieee;
	use ieee.std_logic_1164.all;
	use ieee.numeric_std.all;
	
entity nand3x is
	port (a  : in  std_logic;
		   b  : in  std_logic;
		   c  : in  std_logic;
			y  : out std_logic);
end entity;

architecture rtl of nand3x is
begin
	y <= '0' when (a = '1') and (b = '1') and (c = '1') else '1';
end rtl;