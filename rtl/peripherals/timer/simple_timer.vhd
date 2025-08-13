library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity simple_timer is
    port (
        phi2        : in  std_logic;                     -- 6502 clock
        reset_n     : in  std_logic;                     -- reset active low
        cs_n        : in  std_logic;                     -- Chip select (active low)
        rw          : in  std_logic;                     -- Read/Write (low = write)
        address     : in  std_logic_vector(1 downto 0);  -- Address bits A1,A0
        data_in     : in  std_logic_vector(7 downto 0);  -- Data from CPU
        data_out    : out std_logic_vector(7 downto 0);  -- Data to CPU
        timer_clk   : in  std_logic                      -- Timer clock (can be same as phi2 or faster)
    );
end simple_timer;

architecture rtl of simple_timer is

    signal timer_counter : unsigned(15 downto 0) := (others => '0');
    signal start_stop    : std_logic := '0';
    signal start_stop_sync : std_logic := '0';
    signal start_stop_prev : std_logic := '0';
    
begin

    -- Timer counter process (runs on timer_clk)
    TIMER_PROCESS: process(timer_clk, reset_n)
    begin
        if reset_n = '0' then
            timer_counter <= (others => '0');
            start_stop_sync <= '0';
            start_stop_prev <= '0';
        elsif rising_edge(timer_clk) then
            -- Synchronize start_stop to timer_clk domain
            start_stop_sync <= start_stop;
            start_stop_prev <= start_stop_sync;
            
            -- Detect rising edge of start_stop (initialize counter)
            if start_stop_prev = '0' and start_stop_sync = '1' then
                timer_counter <= (others => '0');
            -- Count when start_stop is high
            elsif start_stop_sync = '1' then
                timer_counter <= timer_counter + 1;
            end if;
            -- When start_stop is low, counter holds its value
        end if;
    end process TIMER_PROCESS;

    -- CPU interface process (runs on phi2)
    CPU_INTERFACE: process(phi2, reset_n)
    begin
        if reset_n = '0' then
            start_stop <= '0';
        elsif rising_edge(phi2) then
            if cs_n = '0' then
                case address is
                    when "00" => -- 0xC204 Control Register
                        if rw = '0' then
                            -- Writing Control Register
                            start_stop <= data_in(0);
                        else
                            -- Reading Control Register
                            data_out <= "0000000" & start_stop;
                        end if;

                    when "01" => -- 0xC205 Counter Low Byte
                        if rw = '0' then
                            -- Writing to counter not permitted
                            null;
                        else
                            -- Reading Counter Low Byte
                            data_out <= std_logic_vector(timer_counter(7 downto 0));
                        end if;
                        
                    when "10" => -- 0xC206 Counter High Byte
                        if rw = '0' then
                            -- Writing to counter not permitted
                            null;
                        else
                            -- Reading Counter High Byte
                            data_out <= std_logic_vector(timer_counter(15 downto 8));
                        end if;
                
                    when "11" => -- 0xC207 Reserved/Status
                        if rw = '0' then
                            -- Reserved
                            null;
                        else
                            -- Could return timer status
                            data_out <= "0000000" & start_stop;
                        end if;

                    when others =>
                        data_out <= (others => '0');
                end case;
            else
                data_out <= (others => '0');
            end if;
        end if;
    end process CPU_INTERFACE;

end architecture rtl;