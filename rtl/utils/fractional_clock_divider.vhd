library IEEE;
	use IEEE.std_logic_1164.all;
	use IEEE.numeric_std.all;
	use IEEE.math_real.all;

entity fractional_clock_divider is
    generic (
        CLK_FREQ_HZ     : positive := 50_000_000;  
        FREQUENCY_HZ    : positive := 1_843_200      
    );
    port (
        clk_in   : in  std_logic;  
        reset_n  : in  std_logic;  
        clk_out  : out std_logic   
    );
end entity fractional_clock_divider;

architecture rtl of fractional_clock_divider is
    -- Calculate the required division ratio
    constant TARGET_FREQ_HZ : real := real(FREQUENCY_HZ);
    constant DIVIDE_RATIO   : real := real(CLK_FREQ_HZ) / TARGET_FREQ_HZ;
    
    constant INT_DIVIDE     : natural := natural(floor(DIVIDE_RATIO));
    constant FRAC_DIVIDE    : natural := natural(round((DIVIDE_RATIO - real(INT_DIVIDE)) * 256.0));
     constant DIV_VALUE      : unsigned(15 downto 0) := to_unsigned(INT_DIVIDE * 256 + FRAC_DIVIDE, 16);
    
    -- Counter and accumulator registers
    signal counter          : unsigned(15 downto 0) := (others => '0'); 
    signal accumulator      : unsigned(15 downto 0) := (others => '0'); 
    signal clk_i            : std_logic := '0';     
    
begin
    clk_out <= clk_i;
    
    process(clk_in, reset_n)
    begin
        if reset_n = '0' then
            counter <= (others => '0');
            accumulator <= (others => '0');
            clk_i <= '0';
            
        elsif rising_edge(clk_in) then
            if counter = 0 then
                clk_i <= not clk_i;
                if accumulator(15) = '1' then
                    counter <= to_unsigned(INT_DIVIDE/2 - 1, 16);
                    accumulator <= accumulator + to_unsigned(FRAC_DIVIDE, 16) - x"0100";
                else
                    counter <= to_unsigned((INT_DIVIDE + 1)/2 - 1, 16);
                    accumulator <= accumulator + to_unsigned(FRAC_DIVIDE, 16);
                end if;
            else
                counter <= counter - 1;
            end if;
        end if;
    end process;
    
end architecture rtl;