library IEEE;
	use IEEE.std_logic_1164.all;
   use ieee.numeric_std.all; 

entity hexto7seg is
  port (
	   hex           : in   std_logic_vector(3 downto 0);
		seg           : out  std_logic_vector(7 downto 0)
	);
end entity;	

architecture rtl of hexto7seg is
begin
		process(hex)
		begin
		   case hex is 
				when x"0"   => seg <= "01000000"; 
				when x"1"   => seg <= "01111001";
				when x"2"   => seg <= "00100100";
				when x"3"   => seg <= "00110000";
				when x"4"   => seg <= "00011001";
				when x"5"   => seg <= "00010010";
				when x"6"   => seg <= "00000010";
				when x"7"   => seg <= "01111000";
				when x"8"   => seg <= "00000000";
				when x"9"   => seg <= "00010000";
				when x"A"   => seg <= "00001000";
				when x"B"   => seg <= "00000011";
				when x"C"   => seg <= "01000110";
				when x"D"   => seg <= "00100001";
				when x"E"   => seg <= "00000110";
				when x"F"   => seg <= "00001110";
				when others => seg <= "01111111";                            
			end case;	
	 end process;

end rtl;


