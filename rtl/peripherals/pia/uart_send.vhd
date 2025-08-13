library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;
 
entity uart_send is
  generic (
      CLK_FREQ_HZ : integer := 1000000;
		BAUD_RATE   : integer := 1200;
		BITS        : integer := 8
    );
  port (
    clk      : in  std_logic;
    tx       : out std_logic;
    strobe_n : in  std_logic;        
	 busy     : out std_logic;
    data_in  : in  std_logic_vector(7 downto 0)
    );
end uart_send;
 
 
architecture rtl of uart_send is
 
  type uart_state_t is (UART_IDLE, UART_START, UART_DATA, UART_STOP);
  signal uart_state : uart_state_t := UART_IDLE;
 
  signal clocks  : integer range 0 to CLK_FREQ_HZ/BAUD_RATE - 1 := 0;
  signal bitno   : integer range 0 to BITS - 1 := 0;  
  signal byte    : std_logic_vector(7 downto 0) := (others => '0');
  
begin
  transmit : process (clk)
  begin
    if rising_edge(clk) then
      case uart_state is
        when UART_IDLE =>
          tx <= '1';               -- Idle line is high
          clocks <= 0;
          bitno  <= 0;
			 busy   <= '0';
          
          if strobe_n = '0' then   -- Start transmission on active low strobe
            byte <= data_in;       -- Latch input data
            uart_state <= UART_START;
				busy <= '1';
          end if;
 
        when UART_START =>
          tx <= '0';               -- Start bit is low
			 busy <= '1';
          if clocks < CLK_FREQ_HZ/BAUD_RATE - 1 then
            clocks <= clocks + 1;
          else
            clocks <= 0;
            uart_state <= UART_DATA;
          end if;
         
        when UART_DATA =>
          tx <= byte(bitno);       -- Send data bits
			 busy <= '1';
          if clocks < CLK_FREQ_HZ/BAUD_RATE - 1 then
            clocks <= clocks + 1;
          else
            clocks <= 0;
            if bitno < BITS - 1 then
              bitno <= bitno + 1;
            else
              bitno <= 0;
              uart_state <= UART_STOP;
            end if;
          end if;
           
        when UART_STOP =>
          tx <= '1';               
			 busy <= '1';
          if clocks < CLK_FREQ_HZ/BAUD_RATE - 1 then
            clocks <= clocks + 1;
          else
            clocks <= 0;
            uart_state <= UART_IDLE;
          end if;
            
        when others =>
          uart_state <= UART_IDLE;
 
      end case;
    end if;
  end process;
     
end architecture rtl;