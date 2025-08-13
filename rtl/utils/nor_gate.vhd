library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity nor_gate is
    Port ( 
        a : in STD_LOGIC;
        b : in STD_LOGIC;
        y : out STD_LOGIC
    );
end nor_gate;

architecture Behavioral of nor_gate is
begin
    y <= not (a or b);
end Behavioral;
