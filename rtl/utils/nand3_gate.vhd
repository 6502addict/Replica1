library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity nand3_gate is
    Port ( 
        a : in STD_LOGIC;
        b : in STD_LOGIC;
        c : in STD_LOGIC;
        y : out STD_LOGIC
    );
end nand3_gate;

architecture Behavioral of nand3_gate is
begin
    y <= not (a and b and c);
end Behavioral;


