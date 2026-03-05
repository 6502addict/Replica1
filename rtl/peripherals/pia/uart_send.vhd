library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;

entity uart_send is
    port (
        clk     : in  std_logic;   -- x16 baud clock
        reset_n : in  std_logic;
        tx      : out std_logic;
        req     : in  std_logic;   -- assert to start transmission
        ready   : out std_logic;   -- high when idle, can accept new byte
        ack     : out std_logic;   -- pulses one cycle when transmission complete
        format  : in  std_logic_vector(2 downto 0);  -- CR4:CR3:CR2 MC6850 mapping
        data_in : in  std_logic_vector(7 downto 0)
    );
end uart_send;

-- MC6850 format mapping:
-- CR4 CR3 CR2
--  0   0   0  = 7 bits + Even parity + 2 stops
--  0   0   1  = 7 bits + Odd  parity + 2 stops
--  0   1   0  = 7 bits + Even parity + 1 stop
--  0   1   1  = 7 bits + Odd  parity + 1 stop
--  1   0   0  = 8 bits + 2 stops
--  1   0   1  = 8 bits + 1 stop
--  1   1   0  = 8 bits + Even parity + 1 stop
--  1   1   1  = 8 bits + Odd  parity + 1 stop

architecture rtl of uart_send is

    type uart_state_t is (UART_IDLE, UART_START, UART_DATA, UART_PARITY, UART_STOP, UART_STOP2);
    signal uart_state : uart_state_t := UART_IDLE;

    signal clocks     : integer range 0 to 15 := 0;
    signal bitno      : integer range 0 to 7  := 0;
    signal byte       : std_logic_vector(7 downto 0) := (others => '0');

    -- decoded format signals
    signal len        : std_logic;  -- '0'=7bits  '1'=8bits
    signal parity_en  : std_logic;  -- '1'=parity enabled
    signal parity_odd : std_logic;  -- '0'=even   '1'=odd
    signal two_stops  : std_logic;  -- '1'=2 stops

begin

    -- MC6850 CR4:CR3:CR2 decoding
    len        <= format(2);
    parity_en  <= not (format(2) and not format(1));
    parity_odd <= format(0);
    two_stops  <= not format(1) and (not format(2) or not format(0));

    transmit : process(clk)
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                uart_state <= UART_IDLE;
                tx         <= '1';
                ready      <= '1';
                ack        <= '0';
                clocks     <= 0;
                bitno      <= 0;
            else
                ack <= '0';  -- default: no ack

                case uart_state is

                    when UART_IDLE =>
                        tx     <= '1';
                        ready  <= '1';
                        clocks <= 0;
                        bitno  <= 0;
                        if req = '1' then
                            byte       <= data_in;
                            ready      <= '0';
                            uart_state <= UART_START;
                        end if;

                    when UART_START =>
                        tx <= '0';
                        if clocks < 15 then
                            clocks <= clocks + 1;
                        else
                            clocks     <= 0;
                            uart_state <= UART_DATA;
                        end if;

                    when UART_DATA =>
                        tx <= byte(bitno);
                        if clocks < 15 then
                            clocks <= clocks + 1;
                        else
                            clocks <= 0;
                            if (len = '0' and bitno = 6) or (len = '1' and bitno = 7) then
                                bitno <= 0;
                                if parity_en = '1' then
                                    uart_state <= UART_PARITY;
                                else
                                    uart_state <= UART_STOP;
                                end if;
                            else
                                bitno <= bitno + 1;
                            end if;
                        end if;

                    when UART_PARITY =>
                        tx <= byte(7) xor byte(6) xor byte(5) xor byte(4) xor
                              byte(3) xor byte(2) xor byte(1) xor byte(0) xor parity_odd;
                        if clocks < 15 then
                            clocks <= clocks + 1;
                        else
                            clocks     <= 0;
                            uart_state <= UART_STOP;
                        end if;

                    when UART_STOP =>
                        tx <= '1';
                        if clocks < 15 then
                            clocks <= clocks + 1;
                        else
                            clocks <= 0;
                            if two_stops = '1' then
                                uart_state <= UART_STOP2;
                            else
                                ack        <= '1';  -- transmission complete
                                ready      <= '1';
                                uart_state <= UART_IDLE;
                            end if;
                        end if;

                    when UART_STOP2 =>
                        tx <= '1';
                        if clocks < 15 then
                            clocks <= clocks + 1;
                        else
                            clocks     <= 0;
                            ack        <= '1';  -- transmission complete
                            ready      <= '1';
                            uart_state <= UART_IDLE;
                        end if;

                    when others =>
                        uart_state <= UART_IDLE;

                end case;
            end if;
        end if;
    end process;

end architecture rtl;