library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity clock_divider IS
    generic (divider : integer := 4);
    port (
        reset    : in  std_logic := '1';
        clk_in   : in  std_logic;
        clk_out  : out std_logic
    );
end clock_divider;

architecture rtl of clock_divider is
    signal cnt : integer range 0 to divider - 1;
    signal clk_out_reg : std_logic := '0';
    
    -- Synthesis attributes to prevent optimization
    attribute keep : string;
    attribute keep of clk_out_reg : signal is "true";
    attribute preserve : boolean;
    attribute preserve of clk_out_reg : signal is true;
begin
    process (clk_in, reset)
    begin
        if reset = '0' then
            cnt <= 0;
            clk_out_reg <= '0';
        elsif (rising_edge(clk_in)) then
            if cnt < (divider / 2) then
                clk_out_reg <= '0';
            else    
                clk_out_reg <= '1';
            end if;
            
            if cnt = divider - 1 then
                cnt <= 0;
            else    
                cnt <= cnt + 1;
            end if;
        end if;
    end process;
    
    -- Connect the registered signal to the output
    clk_out <= clk_out_reg;
end rtl;