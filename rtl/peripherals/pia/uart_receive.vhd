library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;

entity uart_receive is
    port (
        clk          : in  std_logic;
        reset_n      : in  std_logic;
        rx           : in  std_logic;
        format       : in  std_logic_vector(2 downto 0);  -- CR4:CR3:CR2 MC6850 mapping
        req          : in  std_logic;                     -- pulse to consume received byte
        ready        : out std_logic;                     -- high when byte available
        ack          : out std_logic;                     -- pulses one cycle when byte delivered
        data_out     : out std_logic_vector(7 downto 0);
        parity_error : out std_logic
    );
end uart_receive;

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

architecture rtl of uart_receive is

    constant BIT_CLOCKS : integer := 16;  -- input clock must be 16x baud rate

    type uart_state_t is (UART_IDLE, UART_START, UART_DATA, UART_PARITY, UART_STOP, UART_STOP2);
    signal uart_state : uart_state_t := UART_IDLE;

    signal clocks       : integer range 0 to BIT_CLOCKS - 1 := 0;
    signal bitno        : integer range 0 to 7 := 0;
    signal byte         : std_logic_vector(7 downto 0) := (others => '0');
    signal r            : std_logic := '1';
    signal rxd          : std_logic := '1';
    signal counter      : integer range 0 to 1 := 0;

    -- format decoded signals (MC6850 CR4:CR3:CR2)
    signal len          : std_logic;  -- '0'=7bits  '1'=8bits
    signal parity_en    : std_logic;  -- '1'=parity enabled
    signal parity_odd   : std_logic;  -- '0'=even   '1'=odd
    signal two_stops    : std_logic;  -- '1'=2 stops

    -- parity
    signal calc_parity  : std_logic := '0';

    -- internal handshake
    signal rx_valid      : std_logic := '0';
    signal rx_valid_prev : std_logic := '0';
    signal tmp_ready     : std_logic := '0';

begin

    -- MC6850 CR4:CR3:CR2 decoding (same as uart_send)
    len        <= format(2);
    parity_en  <= not (format(2) and not format(1));
    parity_odd <= format(0);
    two_stops  <= not format(1) and (not format(2) or not format(0));

    -- 2-stage synchronizer
    sample : process(clk)
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                r <= '1'; rxd <= '1';
            else
                r <= rx; rxd <= r;
            end if;
        end if;
    end process;

    -- receive state machine
    receive : process(clk)
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                uart_state   <= UART_IDLE;
                rx_valid     <= '0';
                clocks       <= 0;
                bitno        <= 0;
                counter      <= 0;
                calc_parity  <= '0';
                parity_error <= '0';
                byte         <= (others => '0');
            else
                rx_valid <= '0';  -- default

                case uart_state is

                    when UART_IDLE =>
                        clocks       <= 0;
                        bitno        <= 0;
                        counter      <= 0;
                        calc_parity  <= '0';
                        parity_error <= '0';
                        if rxd = '0' then
                            uart_state <= UART_START;
                        end if;

                    when UART_START =>
                        if clocks = (BIT_CLOCKS - 1) / 2 then
                            if rxd = '0' then
                                clocks     <= 0;
                                uart_state <= UART_DATA;
                            else
                                uart_state <= UART_IDLE;  -- false start
                            end if;
                        else
                            clocks <= clocks + 1;
                        end if;

                    when UART_DATA =>
                        if clocks < BIT_CLOCKS - 1 then
                            clocks <= clocks + 1;
                        else
                            clocks      <= 0;
                            byte(bitno) <= rxd;
                            calc_parity <= calc_parity xor rxd;
                            if (len = '0' and bitno = 6) or (len = '1' and bitno = 7) then
                                bitno <= 0;
                                if len = '0' then
                                    byte(7) <= '0';  -- clear unused bit in 7-bit mode
                                end if;
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
                        if clocks < BIT_CLOCKS - 1 then
                            clocks <= clocks + 1;
                        else
                            clocks <= 0;
                            -- even: calc_parity xor rxd should be 0
                            -- odd:  calc_parity xor rxd should be 1
                            if (calc_parity xor rxd) /= parity_odd then
                                parity_error <= '1';
                            end if;
                            uart_state <= UART_STOP;
                        end if;

                    when UART_STOP =>
                        if clocks < BIT_CLOCKS - 1 then
                            clocks <= clocks + 1;
                        else
                            clocks <= 0;
                            if two_stops = '1' then
                                uart_state <= UART_STOP2;
                            else
                                rx_valid   <= '1';
                                uart_state <= UART_IDLE;
                            end if;
                        end if;

                    when UART_STOP2 =>
                        if clocks < BIT_CLOCKS - 1 then
                            clocks <= clocks + 1;
                        else
                            clocks     <= 0;
                            rx_valid   <= '1';
                            uart_state <= UART_IDLE;
                        end if;

                    when others =>
                        uart_state <= UART_IDLE;

                end case;
            end if;
        end if;
    end process;

    -- read handshake: rx_valid rising edge sets ready, req clears it and pulses ack
    read_proc : process(clk)
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                tmp_ready     <= '0';
                rx_valid_prev <= '0';
                ack           <= '0';
            else
                ack           <= '0';  -- default: no ack
                rx_valid_prev <= rx_valid;
                if req = '1' and tmp_ready = '1' then
                    tmp_ready <= '0';
                    ack       <= '1';  -- byte consumed, confirm to caller
                elsif rx_valid = '1' and rx_valid_prev = '0' then
                    tmp_ready <= '1';  -- new byte available
                end if;
            end if;
        end if;
    end process;

    ready    <= tmp_ready;
    data_out <= byte;

end architecture rtl;