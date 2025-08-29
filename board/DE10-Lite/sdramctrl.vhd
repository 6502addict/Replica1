-- Simple 8-bit CPU to SDRAM Interface
-- Works with any Motorola-style bus (6502, 6800, 6809)
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity cpu_sdram_wrapper is
   port(
      -- CPU bus interface (Motorola style)
      cpu_addr : in std_logic_vector(22 downto 0);  -- Full byte address bus (8MB)
      cpu_data_in : out std_logic_vector(7 downto 0);
      cpu_data_out : in std_logic_vector(7 downto 0);
      cpu_rw : in std_logic;        -- '1' = read, '0' = write
      cpu_vma : in std_logic;       -- Valid memory address
      cpu_ready : out std_logic;    -- Ready output to CPU
      
      -- SDRAM physical interface
      dram_addr : out std_logic_vector(12 downto 0);
      dram_dq : inout std_logic_vector(15 downto 0);
      dram_ba_1 : out std_logic;
      dram_ba_0 : out std_logic;
      dram_udqm : out std_logic;
      dram_ldqm : out std_logic;
      dram_ras_n : out std_logic;
      dram_cas_n : out std_logic;
      dram_we_n : out std_logic;
      dram_cs_n : out std_logic;
      dram_clk : out std_logic;
      dram_cke : out std_logic;
      dram_addr13 : out std_logic;
      
      -- System interface
      reset : in std_logic;
      ext_reset : in std_logic;
      sys_clk : in std_logic
   );
end cpu_sdram_wrapper;

architecture implementation of cpu_sdram_wrapper is

   component sdram
      port(
         addr: in std_logic_vector(21 downto 0);
         dati: out std_logic_vector(15 downto 0);
         dato: in std_logic_vector(15 downto 0);
         control_dati : in std_logic;
         control_dato : in std_logic;
         control_datob : in std_logic;
         dram_match : in std_logic;
         
         dram_addr : out std_logic_vector(12 downto 0);
         dram_dq : inout std_logic_vector(15 downto 0);
         dram_ba_1 : out std_logic;
         dram_ba_0 : out std_logic;
         dram_udqm : out std_logic;
         dram_ldqm : out std_logic;
         dram_ras_n : out std_logic;
         dram_cas_n : out std_logic;
         dram_we_n : out std_logic;
         dram_cs_n : out std_logic;
         dram_clk : out std_logic;
         dram_cke : out std_logic;
         dram_addr13 : out std_logic;
         
         reset : in std_logic;
         ext_reset : in std_logic;
         cpureset : out std_logic;
         cpuclk : out std_logic;
         c0 : in std_logic
      );
   end component;

   signal sdram_addr : std_logic_vector(21 downto 0);
   signal sdram_dati : std_logic_vector(15 downto 0);
   signal sdram_dato : std_logic_vector(15 downto 0);
   signal cpu_access : std_logic;
   signal access_active : std_logic := '0';

begin

   -- Address mapping: CPU 23-bit byte address → SDRAM 22-bit word address
   -- Drop LSB to convert byte address to word address
   sdram_addr <= cpu_addr(22 downto 1);
   
   -- CPU access detection
   cpu_access <= cpu_vma;
   
   -- Always byte access, duplicate data on both bytes for writes
   sdram_dato <= cpu_data_out & cpu_data_out;
   
   -- Read data selection based on address bit 0
   cpu_data_in <= sdram_dati(7 downto 0) when cpu_addr(0) = '0' else
                  sdram_dati(15 downto 8);

   -- Instantiate SDRAM controller
   sdram_inst: sdram
      port map(
         addr => sdram_addr,
         dati => sdram_dati,
         dato => sdram_dato,
         control_dati => cpu_access and cpu_rw,
         control_dato => cpu_access and not cpu_rw,
         control_datob => '1',  -- Always byte mode
         dram_match => cpu_access,
         
         dram_addr => dram_addr,
         dram_dq => dram_dq,
         dram_ba_1 => dram_ba_1,
         dram_ba_0 => dram_ba_0,
         dram_udqm => dram_udqm,
         dram_ldqm => dram_ldqm,
         dram_ras_n => dram_ras_n,
         dram_cas_n => dram_cas_n,
         dram_we_n => dram_we_n,
         dram_cs_n => dram_cs_n,
         dram_clk => dram_clk,
         dram_cke => dram_cke,
         dram_addr13 => dram_addr13,
         
         reset => reset,
         ext_reset => ext_reset,
         cpureset => open,  -- Not used
         cpuclk => open,    -- Not used  
         c0 => sys_clk
      );

   -- Simple ready control - stretch CPU cycle during SDRAM access
   process(sys_clk)
   begin
      if rising_edge(sys_clk) then
         if reset = '1' then
            access_active <= '0';
            cpu_ready <= '1';
         else
            if cpu_access = '1' and access_active = '0' then
               access_active <= '1';
               cpu_ready <= '0';  -- Stretch CPU cycle
            elsif access_active = '1' then
               access_active <= '0';
               cpu_ready <= '1';  -- Release CPU
            else
               cpu_ready <= '1';
            end if;
         end if;
      end if;
   end process;

end implementation;