library ieee;
	use ieee.std_logic_1164.all;
	use ieee.numeric_std.all;
library altera;
	use altera.altera_primitives_components.all;
	
entity clock6809e is
	port (clock    : in  std_logic;
		   mrdy	   : in  std_logic;
		   e		   : out std_logic;
		   q        : out std_logic);
end entity;

architecture rtl of clock6809e is
signal cnt     : integer range 0 to 49;
signal fbes    : std_logic;
signal fbep    : std_logic;
signal stretch : std_logic;
signal qj      : std_logic;
signal qbk     : std_logic;
signal trig    : std_logic;

component d_ff is
	port (clk    : in  std_logic;
		   preset : in  std_logic;
		   clear  : in  std_logic;
		   d      : in  std_logic;
		   q      : out std_logic;
		   qbar   : out std_logic);
end component;

component jk_ff is
	port (clk    : in  std_logic;
		   preset : in  std_logic;
		   clear  : in  std_logic;
		   j      : in  std_logic;
		   k      : in  std_logic;
		   q      : out std_logic;
		   qbar   : out std_logic);
end component;

component nand3x is
	port (a  : in  std_logic;
		   b  : in  std_logic;
		   c  : in  std_logic;
			y  : out std_logic);
end component;

begin
	u0 : nand3x PORT MAP (qbk, not mrdy, fbes, trig);
	
	u1 : d_ff  PORT MAP (clk => clock,
						     preset => '1',
							  clear => '1', 
							  d => trig,
							  q => stretch);

   u2 : jk_ff PORT MAP (j => fbep,
							   k => fbes,
							   clk => clock,
							   clear => stretch,
							   preset => '1',
							   q => qj,
							   qbar => qbk);
   							  
   u3 : jk_ff PORT MAP (j => qj,
							   k => qbk,
							   clk => clock,
							   clear => '1',
							   preset => stretch,
							   q => fbes,
							   qbar => fbep);

	q <= qj;
	e <= fbes;
	
end rtl;