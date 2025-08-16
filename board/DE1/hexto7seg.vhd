library IEEE;
	use IEEE.std_logic_1164.all;
   use ieee.numeric_std.all; 

entity hexto7seg is
  port (
	   hex           : in   std_logic_vector(3 downto 0);
		seg           : out  std_logic_vector(6 downto 0)
	);
end entity;	

architecture rtl of hexto7seg is
begin
		process(hex)
		begin
		   case hex is 
				when x"0"   => seg <= "1000000"; 
				when x"1"   => seg <= "1111001";
				when x"2"   => seg <= "0100100";
				when x"3"   => seg <= "0110000";
				when x"4"   => seg <= "0011001";
				when x"5"   => seg <= "0010010";
				when x"6"   => seg <= "0000010";
				when x"7"   => seg <= "1111000";
				when x"8"   => seg <= "0000000";
				when x"9"   => seg <= "0010000";
				when x"A"   => seg <= "0001000";
				when x"B"   => seg <= "0000011";
				when x"C"   => seg <= "1000110";
				when x"D"   => seg <= "0100001";
				when x"E"   => seg <= "0000110";
				when x"F"   => seg <= "0001110";
				when others => seg <= "1111111";                            
			end case;	
	 end process;

end rtl;


