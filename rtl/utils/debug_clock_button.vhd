library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity debug_clock_button is
    port (
        clk_1hz_in   : in  STD_LOGIC;
        debug_btn    : in  STD_LOGIC;
        debug_clk    : out STD_LOGIC
    );
end debug_clock_button;

architecture Behavioral of debug_clock_button is
    
begin
    
	 debug_clk <=  clk_1hz_in when debug_btn = '0' else '0';
    
end Behavioral;

