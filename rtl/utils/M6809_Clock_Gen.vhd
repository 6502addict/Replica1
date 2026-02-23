library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity M6809_Clock_Gen is
    Port (
        clk_4x : in  STD_LOGIC;
        reset  : in  STD_LOGIC;
        mrdy   : in  STD_LOGIC;  -- Memory Ready
        E      : out STD_LOGIC;
        Q      : out STD_LOGIC
    );
end M6809_Clock_Gen;

architecture Behavioral of M6809_Clock_Gen is
    signal count : unsigned(1 downto 0) := "00";
begin

    process(clk_4x, reset)
    begin
        if reset = '1' then
            count <= "00";
        elsif rising_edge(clk_4x) then
            -- Logic: Advance the counter UNLESS we are in the stretch state
            -- (count = 3) AND mrdy is low.
            if not (count = "11" and mrdy = '0') then
                count <= count + 1;
            end if;
        end if;
    end process;

    -- Map states to the 6809 Quadrature sequence
    process(count)
    begin
        case count is
            when "00" => Q <= '0'; E <= '0';
            when "01" => Q <= '1'; E <= '0';
            when "10" => Q <= '1'; E <= '1';
            when "11" => Q <= '0'; E <= '1'; -- The Stretch State
            when others => Q <= '0'; E <= '0';
        end case;
    end process;

end Behavioral;