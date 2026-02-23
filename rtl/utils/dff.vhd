library ieee;
	use ieee.std_logic_1164.all;
	use ieee.numeric_std.all;
	
entity d_ff is
	port (clk    : in  std_logic;
		   preset : in  std_logic;
		   clear  : in  std_logic;
		   d		 : in  std_logic;
			q      : out std_logic;
		   qbar   : out std_logic);
end entity;

architecture rtl of d_ff is
signal tq : std_logic;
begin
	process(clk, clear, preset)
	begin
		if (clear = '0') then
			tq <= '0';
		elsif (preset = '0') then
			tq <= '1';
		elsif rising_edge(clk) then
			tq <= d;
		end if;
	end process;
	q <= tq;
	qbar <= not tq;
end rtl;