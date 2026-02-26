library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity INTBASIC is
    Port (
       clk      : in  std_logic;
       cs       : in  std_logic;
       rw       : in  std_logic;
       addr     : in  std_logic_vector (10 downto 0);
       data_in  : in  std_logic_vector (7 downto 0);
       data_out : out std_logic_vector (7 downto 0)
    );
end INTBASIC;

architecture RTL of INTBASIC is

    component rom IS
            GENERIC
            (
                    INIT_FILE     : string        := "";
                    WORD_COUNT    : natural       := 2048;
                    ADDR_WIDTH    : natural       := 11;
                    DATA_WIDTH    : natural       := 8;
                    REG_OUT       : string        := "UNREGISTERED"
            );
            PORT
            (
                    clk           : IN  STD_LOGIC ;
                    addr          : IN  STD_LOGIC_VECTOR (ADDR_WIDTH-1 DOWNTO 0);
                    data_in       : IN  STD_LOGIC_VECTOR (DATA_WIDTH-1 DOWNTO 0);
                    data_out      : OUT STD_LOGIC_VECTOR (DATA_WIDTH-1 DOWNTO 0)
            );
    end component;

begin

        rom_inst : rom   generic map(INIT_FILE  => "INTBASIC.HEX",
                                     WORD_COUNT => 8192,
                                     ADDR_WIDTH => 13)
                            port map(clk         => clk,
                                     addr        => addr,
                                     data_in     => data_in,
                                     data_out    => data_out);

end RTL;