library ieee;
	use ieee.std_logic_1164.all;
	use ieee.numeric_std.all;

entity aci is
    port (
		  reset_n   : in std_logic;                            -- reset
        phi2      : in std_logic;                            -- clock named phi2 on the 6502
        cs_n      : in std_logic;                            -- CXXX chip select²
        address   : in std_logic_vector(15 downto 0);        -- addresses
        data_out  : out std_logic_vector(7 downto 0);        -- data output
		  tape_in   : in  std_logic;                           -- tape input
		  tape_out  : out std_logic                            -- tape output
    );
end entity;

architecture rtl of aci is

component nor_gate is
    Port ( 
        a : in STD_LOGIC;
        b : in STD_LOGIC;
        y : out STD_LOGIC
    );
end component;


component nand3_gate is
    Port ( 
        a : in STD_LOGIC;
        b : in STD_LOGIC;
        c : in STD_LOGIC;
        y : out STD_LOGIC
    );
end component;

    type rom_type is array(0 to 255) of std_logic_vector(7 downto 0);
    signal rom : rom_type := (
	     X"A9", X"AA", X"20", X"EF", X"FF", X"A9", X"8D", X"20", 
        X"EF", X"FF", X"A0", X"FF", X"C8", X"AD", X"11", X"D0", 
        X"10", X"FB", X"AD", X"10", X"D0", X"99", X"00", X"02", 
        X"20", X"EF", X"FF", X"C9", X"9B", X"F0", X"E1", X"C9", 
        X"8D", X"D0", X"E9", X"A2", X"FF", X"A9", X"00", X"85", 
        X"24", X"85", X"25", X"85", X"26", X"85", X"27", X"E8", 
        X"BD", X"00", X"02", X"C9", X"D2", X"F0", X"56", X"C9", 
        X"D7", X"F0", X"35", X"C9", X"AE", X"F0", X"27", X"C9", 
        X"8D", X"F0", X"20", X"C9", X"A0", X"F0", X"E8", X"49", 
        X"B0", X"C9", X"0A", X"90", X"06", X"69", X"88", X"C9", 
        X"FA", X"90", X"AD", X"0A", X"0A", X"0A", X"0A", X"A0", 
        X"04", X"0A", X"26", X"24", X"26", X"25", X"88", X"D0", 
        X"F8", X"F0", X"CC", X"4C", X"1A", X"FF", X"A5", X"24", 
        X"85", X"26", X"A5", X"25", X"85", X"27", X"B0", X"BF", 
        X"A9", X"40", X"20", X"CC", X"C1", X"88", X"A2", X"00", 
        X"A1", X"26", X"A2", X"10", X"0A", X"20", X"DB", X"C1", 
        X"D0", X"FA", X"20", X"F1", X"C1", X"A0", X"1E", X"90", 
        X"EC", X"A6", X"28", X"B0", X"98", X"20", X"BC", X"C1", 
        X"A9", X"16", X"20", X"CC", X"C1", X"20", X"BC", X"C1", 
        X"A0", X"1F", X"20", X"BF", X"C1", X"B0", X"F9", X"20", 
        X"BF", X"C1", X"A0", X"3A", X"A2", X"08", X"48", X"20", 
        X"BC", X"C1", X"68", X"2A", X"A0", X"39", X"CA", X"D0", 
        X"F5", X"81", X"26", X"20", X"F1", X"C1", X"A0", X"35", 
        X"90", X"EA", X"B0", X"CD", X"20", X"BF", X"C1", X"88", 
        X"AD", X"81", X"C0", X"C5", X"29", X"F0", X"F8", X"85", 
        X"29", X"C0", X"80", X"60", X"86", X"28", X"A0", X"42", 
        X"20", X"E0", X"C1", X"D0", X"F9", X"69", X"FE", X"B0", 
        X"F5", X"A0", X"1E", X"20", X"E0", X"C1", X"A0", X"2C", 
        X"88", X"D0", X"FD", X"90", X"05", X"A0", X"2F", X"88", 
        X"D0", X"FD", X"BC", X"00", X"C0", X"A0", X"29", X"CA", 
        X"60", X"A5", X"26", X"C5", X"24", X"A5", X"27", X"E5", 
        X"25", X"E6", X"26", X"D0", X"02", X"E6", X"27", X"60"
    );

	 signal  nor1_out      : std_logic;
	 signal  nor2_out      : std_logic;
	 signal  nand2_out     : std_logic;
	 signal  nand3_out     : std_logic;
	 signal  dff_feedback  : std_logic;
	 signal  aci_sel       : std_logic;
	 signal  flip_sel      : std_logic;
	 signal  rom_a0        : std_logic;
	 signal  tape_ff       : std_logic;
	 signal  rom_address   : std_logic_vector(7 downto 0);
	 
begin

	nor1:  nor_gate   port map(a     => cs_n, 
	                           b     => address(11),
							  	  	   y     => nor1_out);

	nor2:  nor_gate   port map(a     => address(10), 
	                           b     => address(9),
									   y     => nor2_out);
									  
	nand1: nand3_gate port map(a     => nor1_out,
                              b     => phi2,
									   c     => nor2_out,
									   y     => aci_sel);	
										
   nor3: nor_gate    port map(a     => aci_sel,
                              b     => address(8),
										y     => flip_sel);

                              	
	nand2: nand3_gate port map(a     => flip_sel,
	                           b     => address(7),
										c     => tape_in,
										y     => nand2_out);
										
  nand3: nand3_gate  port map(a     => nand2_out,
                              b     => nand2_out,
									   c     => address(0),
									   y     => nand3_out);
									
	nor4: nor_gate    port map(a     => nand3_out,
                              b     => nand3_out,
									   y     => rom_a0);
										
	process(reset_n, phi2)
	begin
		if reset_n <= '0' then
			tape_ff <= '0';
		elsif rising_edge(phi2) then
			if flip_sel = '1' then
				tape_ff <= not tape_ff;
			end if;
		end if;
	end process;

	tape_out <= tape_ff;
										
										
	rom_address <= address(7 downto 1) & rom_a0;
								
								
	process(phi2)
   begin
		if rising_edge(phi2) then
			if aci_sel = '0' then
				data_out <= rom(to_integer(unsigned(rom_address)));
         end if;
      end if;
   end process;

end rtl;

