library ieee;
	use ieee.std_logic_1164.all;
	use ieee.numeric_std.all;
	
entity jk_ff is
	port (clk    : in  std_logic;
		   preset : in  std_logic;
		   clear  : in  std_logic;
			j	    : in  std_logic;
			k      : in  std_logic;
		   d		 : in  std_logic;
			q      : out std_logic;
		   qbar   : out std_logic);
end entity;

architecture rtl of jk_ff is
signal tq : std_logic;
begin
	process(clk, clear, preset)
	begin
		if (clear = '0') then
			tq <= '0';
		elsif (preset = '0') then
			tq <= '1';
		elsif falling_edge(clk) then
			if (j = '1' and k = '0') then
				tq <= '1';
			elsif (j = '0' and k = '1') then
				tq <= '0';
			elsif (j = '1' and k = '1') then
				tq <= not tq;
			else
				tq <= tq;
			end if;
		end if;
	end process;
	q <= tq;
	qbar <= not tq;
end rtl;