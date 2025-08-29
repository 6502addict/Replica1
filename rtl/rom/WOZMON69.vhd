library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity WOZMON69 is
    port (
        clock:    in std_logic;
        cs_n:     in std_logic;
        address:  in std_logic_vector(7 downto 0);
        data_out: out std_logic_vector(7 downto 0)
    );
end entity;

architecture rtl of WOZMON69 is
    type rom_type is array(0 to 255) of std_logic_vector(7 downto 0);
    signal rom : rom_type := (
        (X"C6"),(X"7F"),(X"F7"),(X"D0"),(X"12"),(X"C6"),(X"A7"),(X"F7"),
        (X"D0"),(X"11"),(X"F7"),(X"D0"),(X"13"),(X"8E"),(X"01"),(X"FF"),
        (X"81"),(X"DF"),(X"27"),(X"18"),(X"81"),(X"9B"),(X"27"),(X"05"),
        (X"30"),(X"01"),(X"5C"),(X"2A"),(X"14"),(X"86"),(X"DC"),(X"BD"),
        (X"FF"),(X"B0"),(X"86"),(X"8D"),(X"BD"),(X"FF"),(X"B0"),(X"CE"),
        (X"02"),(X"01"),(X"C6"),(X"01"),(X"30"),(X"1F"),(X"5A"),(X"2B"),
        (X"F1"),(X"B6"),(X"D0"),(X"11"),(X"2A"),(X"FB"),(X"B6"),(X"D0"),
        (X"10"),(X"A7"),(X"84"),(X"8D"),(X"73"),(X"81"),(X"8D"),(X"26"),
        (X"CF"),(X"CE"),(X"02"),(X"FF"),(X"9F"),(X"2E"),(X"4F"),(X"48"),
        (X"97"),(X"2B"),(X"0C"),(X"2F"),(X"DE"),(X"2E"),(X"A6"),(X"84"),
        (X"81"),(X"8D"),(X"27"),(X"CE"),(X"81"),(X"AE"),(X"27"),(X"EF"),
        (X"23"),(X"F0"),(X"81"),(X"BA"),(X"27"),(X"EA"),(X"81"),(X"D2"),
        (X"27"),(X"57"),(X"0F"),(X"29"),(X"0F"),(X"28"),(X"9F"),(X"2C"),
        (X"DE"),(X"2E"),(X"A6"),(X"84"),(X"88"),(X"B0"),(X"81"),(X"09"),
        (X"23"),(X"06"),(X"8B"),(X"89"),(X"81"),(X"F9"),(X"23"),(X"12"),
        (X"48"),(X"48"),(X"48"),(X"48"),(X"C6"),(X"04"),(X"48"),(X"09"),
        (X"29"),(X"09"),(X"28"),(X"5A"),(X"26"),(X"F8"),(X"0C"),(X"2F"),
        (X"20"),(X"DE"),(X"9C"),(X"2C"),(X"27"),(X"8F"),(X"0D"),(X"2B"),
        (X"2A"),(X"2B"),(X"DE"),(X"26"),(X"96"),(X"29"),(X"A7"),(X"80"),
        (X"9F"),(X"26"),(X"20"),(X"B0"),(X"34"),(X"02"),(X"44"),(X"44"),
        (X"44"),(X"44"),(X"8D"),(X"02"),(X"35"),(X"02"),(X"84"),(X"0F"),
        (X"8A"),(X"B0"),(X"81"),(X"B9"),(X"23"),(X"02"),(X"8B"),(X"07"),
        (X"7D"),(X"D0"),(X"12"),(X"2B"),(X"FB"),(X"B7"),(X"D0"),(X"12"),
        (X"39"),(X"DE"),(X"24"),(X"6E"),(X"84"),(X"26"),(X"23"),(X"DE"),
        (X"28"),(X"9F"),(X"26"),(X"9F"),(X"24"),(X"4F"),(X"26"),(X"10"),
        (X"86"),(X"8D"),(X"8D"),(X"E4"),(X"96"),(X"24"),(X"8D"),(X"CC"),
        (X"96"),(X"25"),(X"8D"),(X"C8"),(X"86"),(X"BA"),(X"8D"),(X"D8"),
        (X"86"),(X"A0"),(X"8D"),(X"D4"),(X"DE"),(X"24"),(X"A6"),(X"84"),
        (X"8D"),(X"BA"),(X"0F"),(X"2B"),(X"DE"),(X"24"),(X"9C"),(X"28"),
        (X"27"),(X"B0"),(X"30"),(X"01"),(X"9F"),(X"24"),(X"96"),(X"25"),
        (X"84"),(X"07"),(X"20"),(X"D2"),(X"00"),(X"00"),(X"00"),(X"00"),
        (X"00"),(X"00"),(X"00"),(X"00"),(X"00"),(X"FF"),(X"FF"),(X"00"));
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
