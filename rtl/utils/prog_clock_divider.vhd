library IEEE;
	use ieee.std_logic_1164.all;
	use ieee.numeric_std.all;

entity prog_clock_divider IS
	 generic (bits : integer := 8);
    port (
        reset_n  : in  std_logic := '1';  				
        clk_in   : in  std_logic;
        divider  : in  std_logic_vector(bits -1 downto 0);  
        clk_out  : out std_logic
    );
end prog_clock_divider;

architecture Behavioral of prog_clock_divider IS
    signal cnt  : unsigned(bits - 1 downto 0) := (others => '0');
    signal fdiv : unsigned(bits - 1 downto 0);
    signal hdiv : unsigned(bits - 1 downto 0);
	 signal div  : std_logic;
	 
begin
	fdiv <= unsigned(divider) - 1;
	hdiv <= unsigned(divider) / 2;

	clk_out <= clk_in when unsigned(divider) < 2 else div; 
	
	process (clk_in, reset_n)
	begin
		if reset_n = '0' then
			cnt <= (others => '0');
		elsif (rising_edge(clk_in)) then
			if cnt < hdiv then
				div <= '0';
			else	
				div <= '1';
			end if;
			if cnt = fdiv then
				cnt  <= (others => '0');
			else	
				cnt <= cnt + 1;
			end if;
		end if;
	end process;	 
	 
end Behavioral;