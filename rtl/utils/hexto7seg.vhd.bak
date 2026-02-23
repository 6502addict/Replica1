library IEEE;
	use IEEE.std_logic_1164.all;
   use ieee.numeric_std.all; 

entity hexto7seg is
  generic (SEGMENTS : integer := 8);
  port (
	   hex           : in   std_logic_vector(3 downto 0);
	   seg           : out  std_logic_vector(SEGMENTS-1 downto 0)
	);
end entity;	

architecture rtl of hexto7seg is
begin
		process(hex)
		begin
		   if SEGMENTS = 8 then
			   case hex is 
					when x"0"   => seg <= "11000000"; 
					when x"1"   => seg <= "11111001";
					when x"2"   => seg <= "10100100";
					when x"3"   => seg <= "10110000";
					when x"4"   => seg <= "10011001";
					when x"5"   => seg <= "10010010";
					when x"6"   => seg <= "10000010";
					when x"7"   => seg <= "11111000";
					when x"8"   => seg <= "10000000";
					when x"9"   => seg <= "10010000";
					when x"A"   => seg <= "10001000";
					when x"B"   => seg <= "10000011";
					when x"C"   => seg <= "11000110";
					when x"D"   => seg <= "10100001";
					when x"E"   => seg <= "10000110";
					when x"F"   => seg <= "10001110";
					when others => seg <= "11111111";                            
				end case;	
			else
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
			end if;
	 end process;

end rtl;


