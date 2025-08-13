library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;
 
entity uart_receive is
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
end uart_receive;
 
 
architecture rtl of uart_receive is
 
  type uart_state_t is (UART_IDLE, UART_START, UART_DATA, UART_STOP, UART_STROBE);
  signal uart_state : uart_state_t := UART_IDLE;
 
  signal clocks  : integer range 0 to CLK_FREQ_HZ/BAUD_RATE - 1 := 0;
  signal bitno   : integer range 0 to BITS-1 := 0;  
  signal byte    : std_logic_vector(7 downto 0) := (others => '0');
  signal r       : std_logic := '0';
  signal rxd     : std_logic := '0';
  signal counter : integer range 0 to BITS-1 := 0;
  signal stretch : integer range 0 to 4 := 0;
   
begin

  sample : process (clk)
  begin
    if rising_edge(clk) then
      r   <= rx;
      rxd <= r; 
    end if; 
  end process;
 
  receive : process (clk)
  begin
    if rising_edge(clk) then
      case uart_state is
        when UART_IDLE =>
          strobe_n <= '1';
			 stretch  <= 0;
          clocks   <= 0;
          bitno    <= 0;
          if rxd = '0' then       
            uart_state <= UART_START;
          end if;
 
          when UART_START =>
          if clocks = (CLK_FREQ_HZ/BAUD_RATE-1)/2 then
            if rxd = '0' then
              clocks <= 0;  
              uart_state <= UART_DATA;
            else
              uart_state <= UART_IDLE;
            end if;
          else
            clocks <= clocks + 1;
          end if;
         
        when UART_DATA =>
          if clocks < CLK_FREQ_HZ/BAUD_RATE - 1 then
            clocks <= clocks + 1;
          else
            clocks <= 0;
            byte(bitno) <= rxd;
            if bitno < BITS-1 then
              bitno <= bitno + 1;
            else
              bitno <= 0;
              uart_state <= UART_STOP;
            end if;
          end if;
           
        when UART_STOP =>
          if clocks < CLK_FREQ_HZ/BAUD_RATE-1 then
            clocks <= clocks + 1;
          else
            strobe_n   <= '0';
            clocks <= 0;
            uart_state <= UART_STROBE;
          end if;
            
        when UART_STROBE =>
			 if stretch < 4 then 
				stretch <= stretch + 1; 
				strobe_n <= '0';
			 else
			   stretch <= 0;
				strobe_n <= '1';
				uart_state <= UART_IDLE;
			end if;
			 
		  when others =>
          uart_state <= UART_IDLE;
 
      end case;
    end if;
  end process;
  
  data_out <= byte;
     
end architecture rtl;