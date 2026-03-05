library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- note:  fake mc6821 connected to serial transmitter / receiver
--        CRA, CRB, DDRA and DDRB are just there to make the software happy

entity pia_uart is
    port (
        -- Note: serial_clk must be exactly 16x the desired baud rate
        --       e.g. MC14411 at 153600 Hz for 9600 baud
        clock      : in  std_logic;
        serial_clk : in  std_logic;   -- x16 baud clock
        reset_n    : in  std_logic;
        format     : in  std_logic_vector(2 downto 0);
        cs_n       : in  std_logic;
        rw         : in  std_logic;
        address    : in  std_logic_vector(1 downto 0);
        data_in    : in  std_logic_vector(7 downto 0);
        data_out   : out std_logic_vector(7 downto 0);
        rx         : in  std_logic;
        tx         : out std_logic
    );
end entity pia_uart;

architecture rtl of pia_uart is

    -- "101" = 8 bits + 1 stop + no parity (MC6850 CR4:CR3:CR2 encoding)
--    constant FORMAT : std_logic_vector(2 downto 0) := "101";

    component uart_send is
        port (
            clk     : in  std_logic;
            reset_n : in  std_logic;
            tx      : out std_logic;
            req     : in  std_logic;
            ready   : out std_logic;
            ack     : out std_logic;
            format  : in  std_logic_vector(2 downto 0);
            data_in : in  std_logic_vector(7 downto 0)
        );
    end component;

    component uart_receive is
        port (
            clk          : in  std_logic;
            reset_n      : in  std_logic;
            rx           : in  std_logic;
            format       : in  std_logic_vector(2 downto 0);
            req          : in  std_logic;
            ready        : out std_logic;
            ack          : out std_logic;
            data_out     : out std_logic_vector(7 downto 0);
            parity_error : out std_logic
        );
    end component;

    -- TX signals (serial_clk domain)
    signal tx_req        : std_logic := '0';
    signal tx_ready      : std_logic;
    signal tx_ack        : std_logic;
    signal tx_data       : std_logic_vector(7 downto 0) := (others => '0');
    -- tx_ready synchronized from serial_clk to clock domain
    signal tx_ready_sync : std_logic_vector(1 downto 0) := "11";
    signal tx_ready_prev : std_logic := '1';

    -- RX signals (serial_clk domain)
    signal rx_req        : std_logic := '0';
    signal rx_ready      : std_logic;
    signal rx_ack        : std_logic;
    signal rx_data       : std_logic_vector(7 downto 0);
    signal rx_parity_err : std_logic;
    -- rx_ready synchronized from serial_clk to clock domain
    signal rx_ready_sync : std_logic_vector(1 downto 0) := "00";
    signal rx_ready_prev : std_logic := '0';

    -- PIA registers
    signal ddra          : std_logic_vector(7 downto 0) := (others => '0');
    signal cra           : std_logic_vector(7 downto 0) := (others => '0');
    signal ddrb          : std_logic_vector(7 downto 0) := (others => '0');
    signal crb           : std_logic_vector(7 downto 0) := (others => '0');

    -- Keyboard buffer (clock domain)
    signal kbd_ready     : std_logic := '0';
    signal kbd_data      : std_logic_vector(7 downto 0) := (others => '0');

begin

    send: uart_send
        port map (
            clk     => serial_clk,
            reset_n => reset_n,
            tx      => tx,
            req     => tx_req,
            ready   => tx_ready,
            ack     => tx_ack,
            format  => format,
            data_in => tx_data
        );

    -- rx_req tied low: pia_uart manages its own kbd_ready flag independently
    recv: uart_receive
        port map (
            clk          => serial_clk,
            reset_n      => reset_n,
            rx           => rx,
            format       => format,
            req          => rx_req,
            ready        => rx_ready,
            ack          => rx_ack,
            data_out     => rx_data,
            parity_error => rx_parity_err
        );

    -- Synchronize tx_ready from serial_clk domain to clock domain
    process(clock)
    begin
        if rising_edge(clock) then
            if reset_n = '0' then
                tx_ready_sync <= "11";
                tx_ready_prev <= '1';
            else
                tx_ready_sync <= tx_ready_sync(0) & tx_ready;
                tx_ready_prev <= tx_ready_sync(1);
            end if;
        end if;
    end process;

    -- Synchronize rx_ready from serial_clk domain to clock domain
    process(clock)
    begin
        if rising_edge(clock) then
            if reset_n = '0' then
                rx_ready_sync <= "00";
                rx_ready_prev <= '0';
                rx_req        <= '0';
                kbd_ready     <= '0';
                kbd_data      <= (others => '0');
            else
                rx_ready_sync <= rx_ready_sync(0) & rx_ready;
                rx_ready_prev <= rx_ready_sync(1);

                -- rising edge: new byte available, latch and ack to uart_receive
                if rx_ready_prev = '0' and rx_ready_sync(1) = '1' then
                    if kbd_ready = '0' then
                        kbd_data  <= rx_data;
                        kbd_ready <= '1';
                    end if;
                    rx_req <= '1';  -- tell uart_receive to clear its ready
                end if;

                -- deassert rx_req once uart_receive has cleared ready (falling edge)
                if rx_ready_prev = '1' and rx_ready_sync(1) = '0' then
                    rx_req <= '0';
                end if;

                -- CPU read of keyboard data clears kbd_ready
                if cs_n = '0' and address = "00" and rw = '1' and cra(2) = '1' then
                    kbd_ready <= '0';
                end if;
            end if;
        end if;
    end process;

    -- CPU bus interface + TX req management
    process(clock)
    begin
        if rising_edge(clock) then
            if reset_n = '0' then
                tx_data  <= (others => '0');
                tx_req   <= '0';
                ddra     <= (others => '0');
                ddrb     <= (others => '0');
                cra      <= (others => '0');
                crb      <= (others => '0');
            else
                -- Clear tx_req as soon as uart_send has accepted the byte
                -- (tx_ready falls meaning uart_send has latched the data and started)
                -- This ensures tx_req is low well before uart_send returns to IDLE
                if tx_ready_sync(1) = '0' and tx_ready_prev = '1' then
                    tx_req <= '0';
                end if;

                if cs_n = '0' then
                    case address is

                        when "00" => -- 0xD010 - KEYBOARD Data register
                            if rw = '0' then
                                if cra(2) = '0' then
                                    ddra <= data_in;
                                end if;
                            else
                                if cra(2) = '0' then
                                    data_out <= ddra;
                                else
                                    -- return keyboard data, force upper case, set bit 7
                                    if kbd_data >= x"61" and kbd_data <= x"7A" then
                                        data_out <= '1' & (kbd_data(6 downto 0) and "1011111");
                                    else
                                        data_out <= '1' & kbd_data(6 downto 0);
                                    end if;
                                end if;
                            end if;

                        when "01" => -- 0xD011 - KEYBOARD Control register
                            if rw = '0' then
                                cra <= data_in;
                            else
                                data_out <= kbd_ready & '0' & cra(5 downto 0);
                            end if;

                        when "10" => -- 0xD012 - SCREEN Data Register
                            if rw = '0' then
                                if crb(2) = '0' then
                                    ddrb <= data_in;
                                else
                                    -- send byte (strip bit 7, Apple-1 convention)
                                    tx_data <= '0' & data_in(6 downto 0);
                                    tx_req  <= '1';
                                end if;
                            else
                                if crb(2) = '0' then
                                    data_out <= ddrb;
                                else
                                    -- bit 7 = busy (uart_send not ready)
                                    data_out <= (not tx_ready_sync(1)) & "0000000";
                                end if;
                            end if;

                        when "11" => -- 0xD013 - SCREEN Control Register
                            if rw = '0' then
                                crb <= data_in;
                            else
                                data_out <= crb;
                            end if;

                        when others => null;

                    end case;
                end if;
            end if;
        end if;
    end process;

end architecture rtl;