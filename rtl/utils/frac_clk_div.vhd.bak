library IEEE;
    use IEEE.std_logic_1164.all;
    use IEEE.numeric_std.all;

entity frac_clk_div is
    port (
        reset_n  : in  std_logic := '1';
        clk_in   : in  std_logic;
        divider  : in  std_logic_vector(15 downto 0);  -- [15:8] integer part, [7:0] fractional part (*256)
        clk_out  : out std_logic
    );
end entity frac_clk_div;

architecture rtl of frac_clk_div is

    component altclkctrl is
        port (inclk : in std_logic; outclk : out std_logic);
    end component;

    signal int_div      : unsigned(7 downto 0);
    signal frac_val     : unsigned(7 downto 0);
    signal half_a       : unsigned(7 downto 0);   -- int_div / 2
    signal half_b       : unsigned(7 downto 0);   -- (int_div + 1) / 2
    signal counter      : unsigned(7 downto 0) := (others => '0');
    signal accum        : unsigned(15 downto 0) := (others => '0');
    signal next_counter : unsigned(7 downto 0);
    signal next_accum   : unsigned(15 downto 0);
    signal clk_i        : std_logic := '0';
    signal clk_mux      : std_logic;

begin
    int_div  <= unsigned(divider(15 downto 8));
    frac_val <= unsigned(divider(7 downto 0));
    half_a   <= shift_right(int_div,     1);
    half_b   <= shift_right(int_div + 1, 1);

    -- pre-compute next values to shorten critical path
    next_accum   <= accum + resize(frac_val, 16) - x"0100" when accum(15) = '1'
                    else accum + resize(frac_val, 16);
    next_counter <= half_a - 1 when accum(15) = '1' else half_b - 1;

    process(clk_in, reset_n)
    begin
        if reset_n = '0' then
            counter <= (others => '0');
            accum   <= (others => '0');
            clk_i   <= '0';
        elsif rising_edge(clk_in) then
            if counter = 0 then
                clk_i   <= not clk_i;
                counter <= next_counter;
                accum   <= next_accum;
            else
                counter <= counter - 1;
            end if;
        end if;
    end process;

    -- bypass divider if int_div < 2, else use divided clock
    -- force onto global clock network via ALTCLKCTRL
    clk_mux <= clk_in when int_div < 2 else clk_i;
    clk_buf : altclkctrl port map (inclk => clk_mux, outclk => clk_out);

end architecture rtl;






--library IEEE;
--    use IEEE.std_logic_1164.all;
--    use IEEE.numeric_std.all;
--
--entity frac_clk_div is
--    port (
--        reset_n  : in  std_logic := '1';
--        clk_in   : in  std_logic;
--        divider  : in  std_logic_vector(15 downto 0);  -- [15:8] integer part, [7:0] fractional part (*256)
--        clk_out  : out std_logic
--    );
--end entity frac_clk_div;
--
--architecture rtl of frac_clk_div is
--    signal int_div  : unsigned(7 downto 0);
--    signal frac_val : unsigned(7 downto 0);
--    signal half_a   : unsigned(7 downto 0);   -- int_div / 2
--    signal half_b   : unsigned(7 downto 0);   -- (int_div + 1) / 2
--    signal counter  : unsigned(7 downto 0) := (others => '0');
--    signal accum    : unsigned(15 downto 0) := (others => '0');
--    signal clk_i    : std_logic := '0';
--begin
--    int_div  <= unsigned(divider(15 downto 8));
--    frac_val <= unsigned(divider(7 downto 0));
--    half_a   <= shift_right(int_div,     1);
--    half_b   <= shift_right(int_div + 1, 1);
--
--    clk_out <= clk_in when int_div < 2 else clk_i;
--
--    process(clk_in, reset_n)
--    begin
--        if reset_n = '0' then
--            counter <= (others => '0');
--            accum   <= (others => '0');
--            clk_i   <= '0';
--        elsif rising_edge(clk_in) then
--            if counter = 0 then
--                clk_i <= not clk_i;
--                if accum(15) = '1' then
--                    counter <= half_a - 1;
--                    accum   <= accum + resize(frac_val, 16) - x"0100";
--                else
--                    counter <= half_b - 1;
--                    accum   <= accum + resize(frac_val, 16);
--                end if;
--            else
--                counter <= counter - 1;
--            end if;
--        end if;
--    end process;
--end architecture rtl;