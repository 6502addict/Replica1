library IEEE;
    use IEEE.std_logic_1164.all;
    use IEEE.numeric_std.all;

entity mux7seg is
    port (
        clk        : in  std_logic;
        reset_n    : in  std_logic;
        seg_sel    : out std_logic_vector(5 downto 0);  -- Digit select (active low)
        seg_data   : out std_logic_vector(7 downto 0);  -- Segment data (MSB = decimal point)
        display    : in  std_logic_vector(23 downto 0)
    );
end mux7seg;

architecture rtl of mux7seg is
    signal scan_sel   : integer range 0 to 5 := 0;
    signal digit      : std_logic_vector(3 downto 0);
    
begin

   seg_data <= "11000000" when digit = x"0" else
               "11111001" when digit = x"1" else
               "10100100" when digit = x"2" else
               "10110000" when digit = x"3" else
               "10011001" when digit = x"4" else
               "10010010" when digit = x"5" else
               "10000010" when digit = x"6" else
               "11111000" when digit = x"7" else
               "10000000" when digit = x"8" else
               "10010000" when digit = x"9" else
               "10001000" when digit = x"A" else
               "10000011" when digit = x"B" else
               "11000110" when digit = x"C" else
               "10100001" when digit = x"D" else
               "10000110" when digit = x"E" else
               "10001110" when digit = x"F" else
               "11111111";
               
               
    process(clk, reset_n)
        variable phase : std_logic := '0'; -- Toggle between blanking and showing
    begin
        if reset_n = '0' then
            scan_sel <= 0;
            seg_sel  <= "111111";
            phase    := '0';
        elsif rising_edge(clk) then
            if phase = '0' then
                -- BLANKING PHASE: Turn all digits off to let transistors recover
                seg_sel <= "111111"; 
                phase := '1'; 
            else
                -- DISPLAY PHASE: Turn on the current digit
                case scan_sel is
                    when 0 => seg_sel <= "111110"; digit <= display(23 downto 20);
                    when 1 => seg_sel <= "111101"; digit <= display(19 downto 16);
                    when 2 => seg_sel <= "111011"; digit <= display(15 downto 12);
                    when 3 => seg_sel <= "110111"; digit <= display(11 downto 8);
                    when 4 => seg_sel <= "101111"; digit <= display(7 downto 4);
                    when 5 => seg_sel <= "011111"; digit <= display(3 downto 0);
                    when others => seg_sel <= "111111";
                end case;
                        
                -- Increment to next digit for the next cycle
                if scan_sel = 5 then
                    scan_sel <= 0;
                else
                    scan_sel <= scan_sel + 1;
                end if;
                        
                phase := '0';
            end if;
        end if;
    end process;               
        
end rtl;

