library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

-- note:  create a fake mc6821 connected to serial transmitter / receiver
--        CRA, CRB, DDRA and DDRB are just there to make the software happy

entity pia_uart is
  generic (
     CLK_FREQ_HZ     : positive := 50000000;  
     BAUD_RATE       : positive := 9600;      
     BITS            : positive := 8          
  );
  port (
    -- System interface
    clock       : in  std_logic;    -- CPU clock
    serial_clk  : in  std_logic;    -- Serial clock
    reset_n     : in  std_logic;    -- Active low reset
    
    -- CPU interface
    cs_n        : in  std_logic;                     -- Chip select
    rw          : in  std_logic;                     -- Read/Write: 1=read, 0=write
    address     : in  std_logic_vector(1 downto 0);  -- Register select (for 4 registers)
    data_in     : in  std_logic_vector(7 downto 0);  -- Data from CPU
    data_out    : out std_logic_vector(7 downto 0);  -- Data to CPU
    
    -- Physical UART interface
    rx          : in  std_logic;    -- Serial input
    tx          : out std_logic     -- Serial output
  );
end entity pia_uart;

architecture rtl of pia_uart is

component uart_send is
  generic (
      CLK_FREQ_HZ : integer := 1000000;
      BAUD_RATE   : integer := 1200;
      BITS        : integer := 8
  );
  port (
    clk      : in  std_logic;
    tx       : out std_logic;
    strobe_n : in  std_logic;        -- Active low strobe to start transmission
    busy     : out std_logic;
    data_in  : in  std_logic_vector(7 downto 0)
    );
end component;

component uart_receive is
  generic (
      CLK_FREQ_HZ : integer := 1000000;
      BAUD_RATE   : integer := 1200;
      BITS        : integer := 8
    );
  port (
    clk      : in  std_logic;
    rx       : in  std_logic;
    strobe_n : out std_logic;
    data_out : out std_logic_vector(7 downto 0)
    );
end component;

    signal tx_busy     : std_logic := '0';
    signal tx_done     : std_logic := '0';	
    signal tx_strobe_n : std_logic := '0';
    signal tx_data     : std_logic_vector(7 downto 0);

    signal rx_strobe_n : std_logic := '0';
    signal rx_data     : std_logic_vector(7 downto 0);
    
    -- Clock domain crossing synchronizers
    signal rx_strobe_sync : std_logic_vector(2 downto 0) := (others => '1');
    signal rx_strobe_prev : std_logic := '1';
		
    signal ddra        : std_logic_vector(7 downto 0);
    signal cra         : std_logic_vector(7 downto 0);
    signal ddrb        : std_logic_vector(7 downto 0);
    signal crb         : std_logic_vector(7 downto 0);
	 
    signal kbd_ready   : std_logic := '0';  
    signal kbd_data    : std_logic_vector(7 downto 0) := (others => '0');
	 
begin
    send:  uart_send    generic map(CLK_FREQ_HZ      => CLK_FREQ_HZ,
                                    BAUD_RATE        => BAUD_RATE,
                                    BITS             => 8)
                        port map(clk              => serial_clk,
                                tx               => tx,
                                strobe_n         => tx_strobe_n,
                                busy             => tx_busy,
                                data_in          => tx_data);
												
    recv:  uart_receive generic map(CLK_FREQ_HZ      => CLK_FREQ_HZ,
                                    BAUD_RATE        => BAUD_RATE,
                                    BITS             => 8)
                        port map(clk              => serial_clk,
                                rx               => rx,
                                strobe_n         => rx_strobe_n,
                                data_out         => rx_data);
											
    -- Synchronize rx_strobe_n from serial_clk domain to clock domain
    process(clock, reset_n)
    begin
        if reset_n = '0' then
            rx_strobe_sync <= (others => '1');
            rx_strobe_prev <= '1';
            kbd_ready <= '0';
            kbd_data <= (others => '0');
        elsif rising_edge(clock) then
            -- Synchronizer chain for rx_strobe_n
            rx_strobe_sync <= rx_strobe_sync(1 downto 0) & rx_strobe_n;
            rx_strobe_prev <= rx_strobe_sync(2);
            
            -- Detect falling edge of synchronized rx_strobe_n (character received)
            -- Only accept new character if previous one has been read (kbd_ready = '0')
            if rx_strobe_prev = '1' and rx_strobe_sync(2) = '0' and kbd_ready = '0' then
                kbd_data <= rx_data;
                kbd_ready <= '1';
            elsif cs_n = '0' and address = "00" and rw = '1' and cra(2) = '1' then
                kbd_ready <= '0';
            end if;
        end if;
    end process;
	 

    process(clock, reset_n)
    begin
        if reset_n = '0' then
            tx_data     <= (others => '0');
            ddra        <= (others => '0');
            ddrb        <= (others => '0');
            cra         <= (others => '0');
            crb         <= (others => '0');
            tx_strobe_n <=  '1';
            tx_done     <=  '0';
        elsif rising_edge(clock) then
            if tx_busy = '1' and tx_done = '0' then
                tx_strobe_n <= '1';
                tx_done <= '1';
            end if;
            
            if tx_busy = '0' and tx_done = '1' then
                tx_done <= '0';
            end if;		
		  
            if cs_n = '0' then
                case address is
                    when "00" => -- 0xD010 - KEYBOARD Data register
                        if rw = '0' then
                            if cra(2) = '0' then
                                -- if ddra flag set in cra write ddra content
                                ddra <= data_in;
                            else
                                null; -- ora data ignored
                            end if;	
                        else	
                            if cra(2) = '0' then
                                -- if ddra flag set in cra return ddra content
                                data_out <= ddra;
                            else
                                -- if ora selected return keyboard data with high bit set
                                -- force input to upper case
                                if kbd_data >= x"61" and kbd_data <= x"7A" then
                                    data_out <= kbd_data and x"DF"; 
                                    data_out(7) <= '1';
                                else
                                    data_out <= kbd_data;
                                    data_out(7) <= '1';
                                end if;
                            end if;
                        end if;

                    when "01" => -- 0xD011 - KEYBOARD Control register
                        if rw = '0' then
                            -- write cra content
                            cra <= data_in;
                        else
                            -- return irqa flag set if keyboard data are ready (from serial line)
                            data_out <= kbd_ready & '0' & cra(5 downto 0);
                        end if;
                        
                    when "10" => -- 0xD012 - SCREEN Data Register
                        if rw = '0' then
                            if crb(2) = '0' then
                                -- if ddrb flag set in crb write ddrb
                                ddrb <= data_in;
                            else
                                -- write data to the serial transmiter with high bit set to 0
                                tx_data <= '0' & data_in(6 downto 0);
                                -- kick the tx_strobe_n;
                                tx_strobe_n <= '0';
                            end if;
                        else 
                            if crb(2) = '0' then
                                -- if ddrb flag set in crb return ddrb register;
                                data_out <= ddrb;
                            else
                                -- return the tx busy flag when device not ready
                                data_out <= tx_done & "0000000";
                            end if;
                        end if;
				
				
                    when "11" => -- 0xD013 - SCREEN Control Register
                        if rw = '0' then
                            -- write crb register
                            crb <= data_in;
                        else
                            -- return crb register
                            data_out <= crb;
                        end if;

                end case;
            end if;
        end if;
    end process;

end architecture rtl;