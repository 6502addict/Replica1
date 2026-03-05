-------------------------------------------------
-- 6504 wrapper (Commodore floppy)
-- 12-bit address, no NMI, has IRQ, no SO
-------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;

entity cpu6504 is
    port (
        clock    : in  std_logic;
        reset    : in  std_logic;
        ce       : in  std_logic;
        data_in  : in  std_logic_vector(7 downto 0);
        data_out : out std_logic_vector(7 downto 0);
        address  : out std_logic_vector(11 downto 0);
        rw       : out std_logic;
        sync     : out std_logic;
        irq      : in  std_logic);
end entity;

architecture rtl of cpu6504 is
    signal full_addr : std_logic_vector(15 downto 0);
begin
    core: entity work.mx65
        port map (
            clock    => clock,
            reset    => reset,
            ce       => ce,
            data_in  => data_in,
            data_out => data_out,
            address  => full_addr,
            rw       => rw,
            sync     => sync,
            so       => '1',
            nmi      => '1',
            irq      => irq);

    address <= full_addr(11 downto 0);
end architecture;