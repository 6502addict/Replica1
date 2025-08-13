-- Simple Clock Selector
-- Uses only 3 switches (SW2, SW1, SW0) to select from 8 clocks
-- Binary encoding: 000=debug, 001=1Hz, 010=1MHz, etc.

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity simple_clock_switch is
    port (
        -- Available clocks
        clock_debug : in  STD_LOGIC;
        clock_1hz   : in  STD_LOGIC;
        clock_1mhz  : in  STD_LOGIC;
        clock_2mhz  : in  STD_LOGIC;
        clock_5mhz  : in  STD_LOGIC;
        clock_10mhz : in  STD_LOGIC;
        clock_15mhz : in  STD_LOGIC;
        clock_30mhz : in  STD_LOGIC;
        
        -- Switch selection (only 3 switches needed)
        SW          : in  STD_LOGIC_VECTOR(2 downto 0);  -- SW(2 downto 0)
        
        -- Output
        main_clock  : out STD_LOGIC
    );
end simple_clock_switch;

architecture Behavioral of simple_clock_switch is
begin
    
    -- Simple multiplexer using 3-bit binary selection
    with SW select main_clock <=
        clock_debug when "000",  -- SW = 0: Debug clock
        clock_1hz   when "001",  -- SW = 1: 1 Hz
        clock_1mhz  when "010",  -- SW = 2: 1 MHz  
        clock_2mhz  when "011",  -- SW = 3: 2 MHz
        clock_5mhz  when "100",  -- SW = 4: 5 MHz
        clock_10mhz when "101",  -- SW = 5: 10 MHz
        clock_15mhz when "110",  -- SW = 6: 15 MHz
        clock_30mhz when "111",  -- SW = 7: 30 MHz
        clock_debug when others; -- Default: debug clock
        
end Behavioral;

--
-- Usage in your top-level entity:
--
-- entity vintage_cpu_system is
--     port (
--         CLOCK_50 : in  STD_LOGIC;
--         SW       : in  STD_LOGIC_VECTOR(9 downto 0);
--         KEY      : in  STD_LOGIC_VECTOR(3 downto 0);
--         LEDR     : out STD_LOGIC_VECTOR(9 downto 0);
--         HEX0     : out STD_LOGIC_VECTOR(6 downto 0)
--     );
-- end vintage_cpu_system;
--
-- architecture Behavioral of vintage_cpu_system is
--     signal main_clock : STD_LOGIC;
--     -- ... other signals
-- begin
--
--     -- Clock selector using only SW(2 downto 0)
--     clock_sel: entity work.simple_clock_switch
--         port map (
--             clk_50mhz   => CLOCK_50,
--             clock_debug => clock_debug,
--             clock_1hz   => clock_1hz,
--             clock_1mhz  => clock_1mhz,
--             clock_2mhz  => clock_2mhz,
--             clock_5mhz  => clock_5mhz,
--             clock_10mhz => clock_10mhz,
--             clock_15mhz => clock_15mhz,
--             clock_30mhz => clock_30mhz,
--             SW          => SW(2 downto 0),
--             main_clock  => main_clock
--         );
--
--     -- Your CPU using selected clock
--     cpu_6502: entity work.cpu_6502_core
--         port map (
--             clk => main_clock,
--             reset_n => KEY(0)
--             -- ... other connections
--         );
--
--     -- Display current selection on LEDs
--     LEDR(2 downto 0) <= SW(2 downto 0);      -- Show switch position
--     LEDR(3) <= main_clock;                   -- Show clock activity
--     LEDR(9 downto 4) <= (others => '0');     -- Unused LEDs off
--
--     -- Display selection on 7-segment (optional)
--     with SW(2 downto 0) select HEX0 <=
--         "1000000" when "000",  -- 0: Debug
--         "1111001" when "001",  -- 1: 1Hz
--         "0100100" when "010",  -- 2: 1MHz
--         "0110000" when "011",  -- 3: 2MHz
--         "0011001" when "100",  -- 4: 5MHz
--         "0010010" when "101",  -- 5: 10MHz
--         "0000010" when "110",  -- 6: 15MHz
--         "1111000" when "111",  -- 7: 30MHz
--         "1111111" when others; -- Blank
--
-- end Behavioral;

--
-- Switch Settings:
-- SW(2) SW(1) SW(0) | Clock Selected
-- ------------------+---------------
--   0     0     0   | Debug clock (button controlled)
--   0     0     1   | 1 Hz
--   0     1     0   | 1 MHz
--   0     1     1   | 2 MHz  
--   1     0     0   | 5 MHz
--   1     0     1   | 10 MHz
--   1     1     0   | 15 MHz
--   1     1     1   | 30 MHz
--
-- Only 3 switches used: SW2, SW1, SW0
-- Remaining switches SW(9 downto 3) available for other functions
--
-- Visual feedback:
-- - LEDR(2:0) shows current switch setting
-- - LEDR(3) blinks with selected clock
-- - HEX0 displays clock number (0-7)
--
-- Perfect for vintage CPU debugging - easy to switch between
-- slow debug speeds and full operational speeds!