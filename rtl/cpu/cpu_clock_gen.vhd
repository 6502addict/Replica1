library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity cpu_clock_gen is
    Port (
        clk_4x  : in  STD_LOGIC;
        reset_n : in  STD_LOGIC;
        mrdy    : in  STD_LOGIC;       -- Memory Ready
        clk_1x  : out STD_LOGIC;       -- CPU clock (stretched)
        clk_2x  : out STD_LOGIC;       -- 2x clock for 6502 cores
        stretch : out STD_LOGIC        -- '1' only when actually stretching
    );
end cpu_clock_gen;

architecture Behavioral of cpu_clock_gen is
    signal count      : unsigned(1 downto 0) := "00";
    signal count_prev : unsigned(1 downto 0) := "00";
begin
    process(clk_4x, reset_n)
    begin
        if reset_n = '0' then
            count      <= "00";
            count_prev <= "00";
        elsif rising_edge(clk_4x) then
            count_prev <= count;
            
            -- Advance counter UNLESS we're at stretch state (count=3) and mrdy='0'
            if not (count = "11" and mrdy = '0') then
                count <= count + 1;
            end if;
        end if;
    end process;
    
    -- Generate clk_1x (like E in 6809)
    -- High during counts 2 and 3, low during 0 and 1
    clk_1x <= '1' when (count(1) = '1') else '0';
    
    -- Generate clk_2x (toggles every count)
    -- High during counts 1 and 3, low during 0 and 2
    clk_2x <= count(0);
    
    -- Stretch signal: '1' only when we're actually holding the counter
    -- This happens when count=3, mrdy='0', AND we were already at count=3
    stretch <= '1' when (count = "11" and count_prev = "11" and mrdy = '0') else '0';
    
end Behavioral;