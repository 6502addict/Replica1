library ieee;
	use ieee.std_logic_1164.all;
	use ieee.numeric_std.all;

entity WOZACI is
    port (
        clock:    in std_logic;
        cs_n:     in std_logic;
        address:  in std_logic_vector(7 downto 0);
        data_out: out std_logic_vector(7 downto 0)
    );
end entity;

architecture rtl of WOZACI is
    -- ROM from $C100 to $C1FF (256 bytes)
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

begin

    process(clock)
    begin
        if rising_edge(clock) then
            if cs_n = '0' then
                data_out <= rom(to_integer(unsigned(address)));
            end if;
        end if;
    end process;

end rtl;

