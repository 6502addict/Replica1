
--------------------------------------------------------------------------------
-- SDRAM Controller for IS42S16320F (and compatible)
-- Copyright (c) 2026 Didier Derny
--
-- This work is licensed under the Creative Commons
-- Attribution-NonCommercial-ShareAlike 4.0 International License.
--
-- You are free to:
--   - Share: copy and redistribute the material
--   - Adapt: remix, transform, and build upon the material
--
-- Under the following terms:
--   - Attribution: You must give appropriate credit
--   - NonCommercial: You may not use for commercial purposes
--   - ShareAlike: Distribute derivatives under the same license
--
--
-- Full license: https://creativecommons.org/licenses/by-nc-sa/4.0/
--------------------------------------------------------------------------------
-- Tested Boards and SDRAM Chips
--------------------------------------------------------------------------------
--
--  Board           FPGA                         SDRAM chip          Status
--  --------------- -----------------------     ------------------- ---------------
--  DE10-Lite       Intel MAX10 10M50            ISSI IS42S16320F    OK
--  DE1             Intel Cylclone 2 EP2C20      ISSI IS42S16400F    OK
--  DE1-SoC         Intel Cyclone V  5CSEMA      ISSI IS42S16320F    OK
--  AX4010          Intel Cyclone 4  EP4CE10F17  Hynix H57V2562GTR   OK
--  MAX1000         Intel MAX10 10M16            Winbond W9825G6KB   OK
--
--------------------------------------------------------------------------------
-- SDRAM Controller - Theory of Operation
--------------------------------------------------------------------------------
-- This controller implements a complete SDRAM interface with initialization,
-- refresh management, and read/write operations for 16-bit SDRAM chips.
--
-- 1. SDRAM Architecture Overview
--    - 4 independent banks (selectable via BA1:0)
--    - Each bank contains an array of rows and columns
--    - 16-bit data bus (can access individual bytes via DQM)
--    - Address multiplexing: Row address, then Column address
--
-- 2. SDRAM Access Protocol (Page Mode)
--    Standard access sequence:
--      a) ACTIVATE (ACT): Open a row in a bank
--         - Latches entire row into bank's sense amplifiers
--         - Takes tRCD time (RAS-to-CAS delay)
--      b) READ/WRITE: Access column(s) within the open row
--         - Fast access to multiple columns in same row
--         - CAS latency for reads (2 cycles in this design)
--      c) PRECHARGE (PRE): Close the row
--         - Required before accessing different row
--         - Takes tRP time (precharge time)
--         - Can precharge single bank or all banks (A10)
--
-- 3. Auto-Precharge Mode
--    - When USE_AUTO_PRECHARGE = true:
--      * READ and WRITE commands issued with A10=1
--      * Row automatically closes after operation
--      * Simpler state machine, no explicit precharge needed
--      * Slightly slower for sequential same-row accesses
--    - When USE_AUTO_PRECHARGE = false:
--      * READ and WRITE commands issued with A10=0
--      * Row remains open for next access
--      * Controller tracks active row per bank
--      * Faster for sequential accesses to same row
--      * Must explicitly precharge before changing rows
--
--    Note: READ and WRITE commands are unified in the state machine.
--    The only hardware difference between a plain access and an
--    auto-precharge access is address bit A10 (0=normal, 1=auto-precharge).
--    All other command bits and timing are identical.
--
-- 4. Initialization Sequence (Per JEDEC Standard)
--    Power-up sequence required before normal operation:
--      a) Wait 100µs minimum with CKE high and NOP commands
--      b) Issue PRECHARGE ALL command (A10=1)
--      c) Issue 8 AUTO REFRESH commands (separated by tRFC)
--      d) Load MODE REGISTER with operating parameters:
--         - CAS Latency = 2 cycles
--         - Burst Length = 1 (single access)
--         - Burst Type = Sequential
--         - Write Burst Mode = Single Location
--      e) Wait tMRD cycles, then ready for normal operation
--
-- 5. Refresh Management
--    SDRAM requires periodic refresh to retain data (every 7.8µs typical):
--
--    - When USE_AUTO_REFRESH = true:
--      * Controller automatically generates refresh requests
--      * Internal counter triggers refresh every 7.8µs
--      * Refresh postponed if CPU access in progress
--      * Automatic and transparent to external logic
--
--    - When USE_AUTO_REFRESH = false:
--      * External logic provides refresh_req signal
--      * Useful for multi-master systems or custom scheduling
--      * Controller still handles timing and protocol
--
--    Refresh sequence:
--      1. If row active: PRECHARGE first
--      2. Issue AUTO REFRESH command
--      3. Wait tRFC (refresh cycle time ~70ns)
--      4. Return to IDLE
--
-- 6. Two-Process FSM Architecture
--    Process 1 (Synchronous):
--      - Updates all state registers on clock edge
--      - Transfers next values to current values
--      - Handles reset logic
--
--    Process 2 (Combinational):
--      - Computes all next values based on current state
--      - Implements state machine logic
--      - Generates SDRAM commands
--      - No registered outputs (pure combinational)
--
-- 7. Address Mapping
--    CPU Address breakdown:
--      [BANK(1:0)][ROW(12:0 or 11:0)][COL(9:0 or 7:0)]
--
--    Example for DE10-Lite (13-bit row, 10-bit col):
--      addr(24:23) = Bank select (BA1:0)
--      addr(22:10) = Row address (13 bits)
--      addr(9:0)   = Column address (10 bits)
--
-- 8. Clock Domain
--    - Single clock domain (no CDC required)
--    - SDRAM clock = inverted system clock
--    - All timing calculated from FREQ_MHZ generic
--
-- 9. Data Masking (DQM)
--    - byte_en(1:0) controls which bytes to access:
--      * "00" = both bytes (word access)
--      * "01" = lower byte only
--      * "10" = upper byte only
--      * "11" = no bytes (masked)
--    - DQM registered one cycle before data
--
-- 10. Timing Parameters (Configurable via FREQ_MHZ)
--     - tRP:   Precharge time (~20ns)
--     - tRCD:  RAS-to-CAS delay (~20ns)
--     - tRFC:  Refresh cycle time (~70ns)
--     - tMRD:  Mode register delay (2 cycles fixed)
--     - CAS Latency: 2 cycles (configurable in MODE_REG)
--
-- 11. Debug Interface
--     Provides visibility into controller state via debug outputs:
--     - debug_state: Current FSM state (4 bits)
--     - debug_cmd: Current SDRAM command being issued
--     - debug_addr_[10,9,0]: Key address bits
--     - debug_dqm: Data mask signals
--     - refresh_active: High during refresh operation
--
-- Performance Characteristics:
--     With AUTO_PRECHARGE = true:
--       - Read:  ACT + tRCD + READ + CAS_LAT + tRP = ~8-10 cycles
--       - Write: ACT + tRCD + WRITE + tRP = ~6-8 cycles
--
--     With AUTO_PRECHARGE = false and same-row access:
--       - Read:  READ + CAS_LAT = ~4 cycles (much faster!)
--       - Write: WRITE + 1 = ~2 cycles (much faster!)
--
--------------------------------------------------------------------------------

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity sdram_controller is
    generic (
        FREQ_MHZ           : integer := 100;  -- Clock frequency in MHz
        ROW_BITS           : integer := 13;   -- 13 for DE10-Lite, 12 for DE1
        COL_BITS           : integer := 10;   -- 10 for DE10-Lite, 8 for DE1
        TRP_NS             : integer := 20;   -- Precharge time (for PRECHARGE wait)
        TRCD_NS            : integer := 20;   -- RAS to CAS delay (for ACTIVE→READ/WRITE)
        TRFC_NS            : integer := 70;   -- Refresh cycle time (for AUTO REFRESH wait)
        CAS_LATENCY        : integer := 2;    -- CAS Latency: 2 or 3 cycles
        USE_AUTO_PRECHARGE : boolean := true; -- true = READA/WRITEA false = READ/WRITE
        USE_AUTO_REFRESH   : boolean := true  -- true = autorefresh, false = triggered refresh
    );
    port(
        clk            : in    std_logic;  
        reset_n        : in    std_logic;  
        
        -- Simple CPU interface
        req            : in    std_logic;
        wr_n           : in    std_logic;  
        addr           : in    std_logic_vector(ROW_BITS+COL_BITS+1 downto 0); 
        din            : in    std_logic_vector(15 downto 0);
        dout           : out   std_logic_vector(15 downto 0);
        byte_en        : in    std_logic_vector(1 downto 0); 
        ready          : out   std_logic;
        ack            : out   std_logic;
        refresh_req    : in    std_logic;
        refresh_active : out   std_logic;
        
        -- SDRAM pins
        sdram_clk      : out   std_logic;
        sdram_cke      : out   std_logic;
        sdram_cs_n     : out   std_logic;
        sdram_ras_n    : out   std_logic;
        sdram_cas_n    : out   std_logic;
        sdram_we_n     : out   std_logic;
        sdram_ba       : out   std_logic_vector(1 downto 0);
        sdram_addr     : out   std_logic_vector(ROW_BITS-1 downto 0);  		
        sdram_dq       : inout std_logic_vector(15 downto 0);
        sdram_dqm      : out   std_logic_vector(1 downto 0)
    );
end sdram_controller;

architecture rtl of sdram_controller is

    -- SDRAM Commands
    -- CS_N  RAS_N  CAS_N  WE_N  | Command               | Notes
    -- -------------------------------------------------------------------------
    --  1     x      x      x    | DESELECT (DESL)       | Chip deselected
    --  0     1      1      1    | NOP                   | No operation
    --  0     1      1      0    | BURST STOP (BST)      | Terminate burst read/write
 	 --  0     1      0      1    | READ                  | A10=0: normal, A10=1: auto-precharge
  	 --  0     1      0      0    | WRITE                 | A10=0: normal, A10=1: auto-precharge
	 --  0     0      1      1    | BANK ACTIVATE (ACT)   | Open row, BA1:0 selects bank
	 --  0     0      1      0    | PRECHARGE (PRE/PALL)  | A10=0: selected bank, A10=1: all banks
	 --  0     0      0      1    | AUTO REFRESH (REF)    | CKE stays high
	 --  0     0      0      1    | SELF REFRESH (SELF)   | CKE: H→L (entry), L→H (exit)
	 --  0     0      0      0    | MODE REGISTER SET     | BA="00" for standard mode register
	  
	 constant CMD_DESL             : std_logic_vector(3 downto 0) := "1111";  -- Device Deselect (CS=H)
	 constant CMD_NOP              : std_logic_vector(3 downto 0) := "0111";  -- No Operation
    constant CMD_BST              : std_logic_vector(3 downto 0) := "0110";  -- Burst Stop
    constant CMD_READ             : std_logic_vector(3 downto 0) := "0101";  -- Read (A10=L for normal, A10=H for auto-precharge)
    constant CMD_WRITE            : std_logic_vector(3 downto 0) := "0100";  -- Write (A10=L for normal, A10=H for auto-precharge)
    constant CMD_ACT              : std_logic_vector(3 downto 0) := "0011";  -- Bank Activate
    constant CMD_PRE              : std_logic_vector(3 downto 0) := "0010";  -- Precharge (A10=L for selected bank, A10=H for all banks)
    constant CMD_REF              : std_logic_vector(3 downto 0) := "0001";  -- CBR Auto-Refresh
    constant CMD_SELF             : std_logic_vector(3 downto 0) := "0001";  -- Self-Refresh (same as REF, differentiated by CKE transition)
    constant CMD_MRS              : std_logic_vector(3 downto 0) := "0000";  -- Mode Register Set
	 
    -- States (visible on logic analyzer via debug_state)
    constant ST_INIT              : std_logic_vector(3 downto 0) := "0000";
    constant ST_INIT_PRECHARGE    : std_logic_vector(3 downto 0) := "0001";
    constant ST_INIT_REFRESH      : std_logic_vector(3 downto 0) := "0010";
    constant ST_INIT_MODE         : std_logic_vector(3 downto 0) := "0011";
    constant ST_IDLE              : std_logic_vector(3 downto 0) := "0100";
    constant ST_REFRESH           : std_logic_vector(3 downto 0) := "0101";
    constant ST_ACTIVATE          : std_logic_vector(3 downto 0) := "0110";
    constant ST_READ              : std_logic_vector(3 downto 0) := "0111";
    constant ST_WRITE             : std_logic_vector(3 downto 0) := "1000";
    constant ST_PRECHARGE         : std_logic_vector(3 downto 0) := "1001";
    
    constant DQM_IDLE             : std_logic_vector(1 downto 0) := "00";
    
	 -- Calculated cycles:
	 constant TRP_CYCLES           : integer := ((TRP_NS * FREQ_MHZ) + 999) / 1000;
	 constant TRCD_CYCLES          : integer := ((TRCD_NS * FREQ_MHZ) + 999) / 1000;
	 constant TRFC_CYCLES          : integer := ((TRFC_NS * FREQ_MHZ) + 999) / 1000;  -- Use THIS for AUTO REFRESH!
	 constant TMRD_CYCLES          : integer := 2;  -- Mode register delay (2 cycles fixed)
    constant TWR_CYCLES           : integer := 2;  -- Write recovery time
    
    -- Mode Register Parameters (JEDEC Standard bit fields)
    constant MR_BURST_LENGTH      : std_logic_vector(2 downto 0) := "000";  -- Bits [2:0]: Burst Length = 1
    constant MR_BURST_TYPE        : std_logic := '0';                       -- Bit [3]: Sequential (0) or Interleaved (1)
    constant MR_OPERATING_MODE    : std_logic_vector(1 downto 0) := "00";   -- Bits [8:7]: Standard operation
    constant MR_WRITE_BURST_MODE  : std_logic := '0';                       -- Bit [9]: Single location access (1) or Burst (0)
    constant MR_RESERVED          : std_logic_vector(2 downto 0) := "000";  -- Bits [12:10]: Must be 000
    
    -- CAS Latency encoding for Mode Register bits [6:4]
    function cas_latency_bits return std_logic_vector is
    begin
        if CAS_LATENCY = 2 then
            return "010";
        elsif CAS_LATENCY = 3 then
            return "011";
        else
            return "010";  -- Default to 2 if invalid
        end if;
    end function;
 
 
    -- Build complete Mode Register from individual fields
    -- Format: [12:10] Reserved | [9] WBM | [8:7] Mode | [6:4] CAS | [3] BT | [2:0] BL
    constant MODE_REG             : std_logic_vector(12 downto 0) := 
			 MR_RESERVED          &  -- [12:10] Reserved (must be 000)
			 MR_WRITE_BURST_MODE  &  -- [9] Write burst mode
			 MR_OPERATING_MODE    &  -- [8:7] Operating mode  
			 cas_latency_bits     &  -- [6:4] CAS Latency (auto from constant)
			 MR_BURST_TYPE        &  -- [3] Burst type
			 MR_BURST_LENGTH;        -- [2:0] Burst length   

	 -- ISSI datatasheet at least 100µs delay 
	 -- before issing a command other than NOP or INHIBIT
    constant INIT_WAIT            : integer := FREQ_MHZ * 200;      -- 200µs
	 constant REFRESH_INTERVAL     : integer := (FREQ_MHZ * 78) / 10; -- 7.8µs

    signal state                  : std_logic_vector(3 downto 0) := ST_INIT;
    signal state_next             : std_logic_vector(3 downto 0) := ST_INIT;
	 signal seq_count              : integer range 0 to INIT_WAIT + 50 := 0;
	 signal seq_count_next         : integer range 0 to INIT_WAIT + 50 := 0;
	 signal row_active             : std_logic := '0';  -- '1' when a row is open
  	 signal row_active_next        : std_logic := '0';  -- '1' when a row is open
    signal need_refresh           : std_logic := '0';  -- '1' when a refresh is needed
    signal need_refresh_next      : std_logic := '0';  -- '1' when a refresh is needed
    signal refresh_postponed      : std_logic := '0';  -- '1' when a refresh is postponed
    signal refresh_postponed_next : std_logic := '0';  -- '1' when a refresh is postponed
    signal ack_next               : std_logic := '0';
    signal ready_next             : std_logic := '0';
    signal dout_next              : std_logic_vector(15 downto 0);
    
    signal refresh_counter        : integer range 0 to REFRESH_INTERVAL := 0;
    signal init_done              : std_logic := '0';  -- Flag: initialization complete
    signal init_done_next         : std_logic := '0';  -- Flag: initialization complete
    
    -- Address latches
    signal addr_bank_latched      : std_logic_vector(1 downto 0);
    signal addr_row_latched       : std_logic_vector(ROW_BITS-1 downto 0);
    signal addr_col_latched       : std_logic_vector(COL_BITS-1 downto 0);
    signal byte_en_latched        : std_logic_vector(1 downto 0);
    signal din_latched            : std_logic_vector(15 downto 0);
    signal wr_n_latched           : std_logic;
	 
	signal active_row             : std_logic_vector(ROW_BITS-1 downto 0) := (others => '0');
	signal active_row_next        : std_logic_vector(ROW_BITS-1 downto 0) := (others => '0');
	signal active_bank            : std_logic_vector(1 downto 0) := "00";
	signal active_bank_next       : std_logic_vector(1 downto 0) := "00";
  
    -- Command outputs

    signal cmd_next               : std_logic_vector(3 downto 0) := CMD_NOP;

    -- next values
	 signal sdram_cke_next         : std_logic;
    signal sdram_ba_next          : std_logic_vector(1 downto 0);
    signal sdram_addr_next        : std_logic_vector(ROW_BITS-1 downto 0);
    signal sdram_dqm_next         : std_logic_vector(1 downto 0);
    signal sdram_dq_next          : std_logic_vector(15 downto 0) := (others => 'Z');  -- ADD THIS! Default tri-state;1
	 
	 -- init refresh counter
	 signal refresh_count         : integer range 0 to 15; 
	 signal refresh_count_next    : integer range 0 to 15; 
	 

begin

    
    -- Output assignments
    sdram_clk  <= not clk;
    sdram_dq   <= sdram_dq_next when (state = ST_WRITE) else (others => 'Z');
    
    process(clk)
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
				sdram_cke         <= '0';
                state             <= ST_INIT;
                seq_count         <= INIT_WAIT;
                sdram_cs_n        <= CMD_NOP(3);
                sdram_ras_n       <= CMD_NOP(2);
                sdram_cas_n       <= CMD_NOP(1);
                sdram_we_n        <= CMD_NOP(0);		  
                sdram_ba          <= "00";
                sdram_addr        <= (others => '0');
                sdram_dqm         <= "11";
                ready             <= '0';
                ack               <= '0';
                refresh_counter   <= 0;
                need_refresh      <= '0';
                init_done         <= '0';
                
            else
                state             <= state_next;
                seq_count         <= seq_count_next;
                row_active        <= row_active_next;
                active_row        <= active_row_next;
                active_bank       <= active_bank_next;
                need_refresh      <= need_refresh_next;
                init_done         <= init_done_next;
                refresh_count     <= refresh_count_next;
                refresh_postponed <= refresh_postponed_next;
            
                -- transfer next values to controller
                ack               <= ack_next;
                ready             <= ready_next;
                dout              <= dout_next;
                
                -- transfer next values to sdram pins
                sdram_cke         <= sdram_cke_next;
                sdram_cs_n        <= cmd_next(3);
                sdram_ras_n       <= cmd_next(2);
                sdram_cas_n       <= cmd_next(1);
                sdram_we_n        <= cmd_next(0);		  
                sdram_addr        <= sdram_addr_next;
                sdram_ba          <= sdram_ba_next;
                sdram_dqm         <= sdram_dqm_next;
            
                if USE_AUTO_REFRESH = true then
                    -- Refresh counter (ONLY after initialization complete)
                    if init_done = '1' then
                        if refresh_counter >= REFRESH_INTERVAL then
                            refresh_counter <= 0;
                            need_refresh <= '1';
                        else
                            refresh_counter <= refresh_counter + 1;
                        end if;
                    end if;
                else
                    if refresh_req = '1' then
                        need_refresh <= '1';
                    end if;
                end if;
            end if;
        end if;    
    end process;    

    process(
        -- Current state registers
        state,
        seq_count,
        row_active,
        active_row,
        active_bank,
        need_refresh,
        refresh_count,
        refresh_postponed,
        init_done,
        refresh_counter,
        
        -- Latched address/data/control
        addr_bank_latched,
        addr_row_latched,
        addr_col_latched,
        byte_en_latched,
        din_latched,
        wr_n_latched,
       
        -- External inputs from CPU/system
        req,
        wr_n,
        addr,
        byte_en,
        din,
        
        -- Input from SDRAM
        sdram_dq
    )

    begin    
        state_next <= state;
        seq_count_next <= seq_count;
        cmd_next <= CMD_NOP;
        sdram_addr_next <= (others => '0');
        sdram_ba_next <= "00";
        sdram_dqm_next <= "11";
        sdram_cke_next <= '1';
        sdram_dq_next <= (others => 'Z');
        row_active_next <= row_active;
        active_row_next <= active_row;
        active_bank_next <= active_bank;
        need_refresh_next <= need_refresh;
        refresh_count_next <= refresh_count;
        init_done_next <= init_done;
        ready_next <= '0';
        ack_next <= '0';
        dout_next <= (others => '0');
    
    
        case state is
                   
        --========================================
        -- ST_INIT - Initial power-up stabilization
        --========================================
        -- Purpose: Allow SDRAM power supply and internal clock to stabilize
        --          before issuing any commands beyond NOP.
        --
        -- Signal Requirements:
        --   CKE    = '1'  --> Clock ENABLED (critical for initialization)
        --   CMD    = NOP (or INHIBIT) --> No operation during stabilization
        --   DQM    = '1'  --> Data outputs tri-stated (datasheet requirement)
        --   ADDR   = (don't care)
        --   BA     = (don't care)
        --
        -- Timing: Wait 100µs minimum (INIT_WAIT = FREQ_MHZ * 100) with CKE HIGH
        --
        -- JEDEC Requirement: CKE must transition from LOW to HIGH and remain
        --                    HIGH for at least 100µs before issuing commands
        --
        -- Exit Condition: When seq_count reaches 0, issue PRECHARGE ALL command
        --                 and transition to ST_INIT_PRECHARGE
        --
        -- Note: Some designs hold CKE low during power-up; this design assumes
        --       CKE goes high when entering this state						  
        
        when ST_INIT =>
            cmd_next            <= CMD_NOP;
            sdram_cke_next <= '1';
            sdram_dqm_next <= "11";
            if seq_count = 0 then
                state_next      <= ST_INIT_PRECHARGE;
                cmd_next        <= CMD_PRE;  -- Issue command on transition
                sdram_addr_next <= (10 => '1', others => '0');  -- Set ALL bits!
                seq_count_next  <= TRP_CYCLES;  -- Now wait
            else
                seq_count_next  <= seq_count - 1;
            end if;					  
                    
        --========================================
        -- ST_INIT_PRECHARGE - Precharge all banks
        --========================================
        -- Purpose: Close any potentially open rows in all banks to prepare
        --          for initialization sequence. Ensures known starting state.
        --
        -- Signal Requirements:
        --   CKE    = '1'  --> Clock enabled
        --   CMD    = PRECHARGE (CS_N RAS_N CAS_N WE_N = 0010)
        --   DQM    = '1'  --> Keep outputs tri-stated
        --   A10    = '1'  --> CRITICAL! Precharges ALL banks simultaneously
        --   A9:0   = (don't care when A10=1)
        --   BA     = (don't care when A10=1, command affects all banks)
        --
        -- Command Timing:
        --   Cycle 1: Issue PRECHARGE command (state entry, previous state exit)
        --   Cycles 2-N: Hold NOP while waiting tRP (precharge time ~20ns)
        --   
        -- Timing: Wait tRP cycles (typically 2-3 cycles @ 100MHz)
        --
        -- Exit Condition: When seq_count reaches 0, issue first AUTO REFRESH
        --                 command and transition to ST_INIT_REFRESH with
        --                 refresh_count = 7 (7 more refreshes to go)
        --
        -- JEDEC Note: PRECHARGE ALL must complete before AUTO REFRESH

        when ST_INIT_PRECHARGE =>
            cmd_next               <= CMD_NOP;  -- NOP during wait
            if seq_count = 0 then
                state_next         <= ST_INIT_REFRESH;
                cmd_next           <= CMD_REF;  -- Issue first refresh
                refresh_count_next <= 7;  
                seq_count_next     <= TRFC_CYCLES;
            else
                seq_count_next <= seq_count - 1;
            end if;					  
                    
        --========================================
        -- ST_INIT_REFRESH - Execute 8 auto-refresh cycles
        --========================================
        -- Purpose: Perform mandatory 8 AUTO REFRESH cycles required by JEDEC
        --          specification before MODE REGISTER can be programmed.
        --          Ensures all storage cells are refreshed before operation.
        --
        -- Signal Requirements:
        --   CKE    = '1'  --> Clock enabled
        --   CMD    = AUTO REFRESH (CS_N RAS_N CAS_N WE_N = 0001) for 1 cycle,
        --            then NOP during tRFC wait
        --   DQM    = '1'  --> Keep outputs tri-stated during initialization
        --   ADDR   = (don't care during refresh)
        --   BA     = (don't care during refresh)
        --
        -- Command Timing per Refresh Cycle:
        --   Cycle 1:       Issue CMD_REF (AUTO REFRESH command)
        --   Cycles 2-tRFC: Issue CMD_NOP while waiting for refresh to complete
        --   
        -- Timing: Each refresh requires tRFC cycles (~70ns = 7 cycles @ 100MHz)
        --
        -- Counter Operation:
        --   refresh_count starts at 7 (after first refresh issued in INIT_PRECHARGE)
        --   Decrements after each completed refresh
        --   When refresh_count = 0 after 8th refresh → proceed to MODE REGISTER
        --
        -- Exit Condition: After 8th refresh completes (refresh_count = 0),
        --                 issue LOAD MODE REGISTER command and transition to ST_INIT_MODE
        --
        -- JEDEC Note: Commands are single-cycle pulses separated by NOP commands
        --             during mandatory timing delays. Don't hold commands for
        --             multiple cycles - this is a common mistake!		
        
        when ST_INIT_REFRESH =>
            cmd_next                <= CMD_NOP;  -- NOP during wait
            if seq_count = 0 then
                if refresh_count = 0 then
                    state_next         <= ST_INIT_MODE;
                    cmd_next           <= CMD_MRS;  -- Issue MODE command
                    sdram_ba_next      <= "00";
                    sdram_addr_next    <= MODE_REG(ROW_BITS - 1 downto 0);
                    seq_count_next     <= TMRD_CYCLES;
                else
                    cmd_next           <= CMD_REF;  -- Issue next refresh
                    refresh_count_next <= refresh_count - 1;
                    seq_count_next     <= TRFC_CYCLES;
                end if;
            else
                seq_count_next <= seq_count - 1;
            end if;					


        --========================================
        -- ST_INIT_MODE - Load mode register
        --========================================
        -- Purpose: Configure SDRAM operating parameters by programming the
        --          MODE REGISTER. This sets burst mode, CAS latency, and
        --          other operational characteristics.
        --
        -- Signal Requirements:
        --   CKE    = '1'  --> Clock enabled
        --   CMD    = LOAD MODE REGISTER (CS_N RAS_N CAS_N WE_N = 0000)
        --   DQM    = '0' or '1' --> Can unmask (not critical for MRS)
        --   BA     = "00" --> Selects standard mode register (BA="01" would be extended)
        --   ADDR   = MODE_REG value (operating parameters)
        --
        -- MODE_REG Bit Breakdown: "000" & "1" & "00" & "010" & "0" & "000"
        --   A12:A10 = "000" --> Reserved (must be 000 per JEDEC)
        --   A9      = "1"   --> Write Burst Mode = Single Location Access
        --                       (writes only affect addressed location, not full burst)
        --   A8:A7   = "00"  --> Operating Mode = Standard Operation
        --                       (not test mode or vendor-specific mode)
        --   A6:A4   = "010" --> CAS Latency = 2 cycles
        --                       (data appears 2 clocks after READ command)
        --   A3      = "0"   --> Burst Type = Sequential
        --                       (addresses increment: 0,1,2,3... not interleaved)
        --   A2:A0   = "000" --> Burst Length = 1 word
        --                       (single access per command, not burst of 2/4/8)
        --
        -- Command Timing:
        --   Cycle 1:      Issue LOAD MODE REGISTER command
        --   Cycles 2-tMRD: Issue NOP while waiting tMRD (mode register delay)
        --   
        -- Timing: Wait tMRD cycles (2 cycles fixed per JEDEC)
        --
        -- Exit Condition: After tMRD delay, set init_done flag and transition
        --                 to ST_IDLE. Controller is now ready for normal operation!
        --
        -- JEDEC Note: No commands except NOP/DESELECT allowed during tMRD period
                
        when ST_INIT_MODE =>
            if seq_count = 0 then
                state_next     <= ST_IDLE;
                init_done_next <= '1';
                cmd_next       <= CMD_NOP;  -- NOP during wait
            else 
                seq_count_next <= seq_count - 1;
            end if;

 
        --========================================
        -- ST_IDLE - Wait for CPU request or refresh
        --========================================
        -- Purpose: Default idle state. Monitors for:
        --          1. Pending refresh (need_refresh or refresh_postponed)
        --          2. CPU read/write request (req = '1')
        --          Prioritizes postponed refresh over new CPU requests.
        --
        -- Signal Requirements:
        --   CMD    = NOP --> No operation
        --   DQM    = '1' --> Mask data (no output)
        --   READY  = '1' --> Signal to CPU that controller can accept new request
        --
        -- Decision Logic:
        --
        -- Priority 1: Postponed Refresh (highest priority)
        --   If need_refresh='1' AND refresh_postponed='1':
        --     → Must service delayed refresh immediately
        --     → If row_active='1': Go to PRECHARGE first
        --     → If row_active='0': Go directly to REFRESH
        --
        -- Priority 2: CPU Request
        --   If req='1' (CPU wants access):
        --     → Latch address, data, byte_en, wr_n from CPU
        --     → Parse address into bank, row, column
        --     → Check if refresh is needed (set postponed flag if so)
        --     
        --     With USE_AUTO_PRECHARGE = true:
        --       → Always go to ACTIVATE (simple path)
        --       → Row will auto-close after read/write
        --     
        --     With USE_AUTO_PRECHARGE = false:
        --       → Check if same bank/row already active:
        --         * Same row: Go directly to READ/WRITE (fast!)
        --         * Different row: PRECHARGE → ACTIVATE → READ/WRITE
        --         * No row active: ACTIVATE → READ/WRITE
        --
        -- Priority 3: Normal Refresh
        --   If need_refresh='1' (and not postponed):
        --     → If row_active='1': Go to PRECHARGE first
        --     → If row_active='0': Go directly to REFRESH
        --
        -- Refresh Postponement:
        --   - If CPU request arrives when refresh needed, set postponed flag
        --   - Allows CPU burst to complete before servicing refresh
        --   - Refresh will be highest priority next time through IDLE
        --   - Prevents excessive refresh delays (data retention guarantee)
        
        when ST_IDLE =>
            cmd_next       <= CMD_NOP;
            sdram_dqm_next <= "11";
            ready_next     <= '1';
            seq_count_next <= 0;

            if need_refresh = '1' and refresh_postponed = '1' then
                -- Need to precharge first if row is active
                if row_active = '1' then
                    state_next <= ST_PRECHARGE;
                    ready_next <= '0';
                else
                    state_next <= ST_REFRESH;
                    ready_next <= '0';
                end if;
                -- check if we have a request    
            elsif req = '1' then
                if need_refresh = '1' then
                    refresh_postponed_next <= '1';
                end if;
                -- Latch address
                addr_bank_latched <= addr(addr'high downto addr'high-1);
                addr_row_latched  <= addr(addr'high-2 downto COL_BITS);
                addr_col_latched  <= addr(COL_BITS-1 downto 0);
                byte_en_latched   <= byte_en;
                wr_n_latched      <= wr_n;
                din_latched       <= din;

                if USE_AUTO_PRECHARGE = true then
                    state_next    <= ST_ACTIVATE;
                    ready_next <= '0';
                else
                    -- Check if accessing same row/bank
                    if row_active = '1' and 
                        active_bank = addr(addr'high downto addr'high-1) and
                        active_row  = addr(addr'high-2 downto COL_BITS) then
                        -- Same row! Go directly to read/write
                        if wr_n = '0' then
                            state_next <= ST_WRITE;
                        else
                            state_next <= ST_READ;
                        end if;
                    else
                        -- Different row, need to activate (precharge first if needed)
                        if row_active = '1' then
                            state_next <= ST_PRECHARGE;
                        else
                            state_next <= ST_ACTIVATE;
                        end if;
                    end if;
                    ready_next <= '0';
               end if;
            -- check if a refresh is needed
            elsif need_refresh = '1' then
                -- Need to precharge first if row is active
                if row_active = '1' then
                    state_next <= ST_PRECHARGE;
                    ready_next <= '0';
                else
                    state_next <= ST_REFRESH;
                    ready_next <= '0';
                end if;
            end if;
            
           
        --========================================
        -- ST_REFRESH - Execute auto-refresh cycle
        --========================================
        -- Purpose: Refresh all rows across all banks to maintain data integrity.
        --          Must occur every 7.8µs to meet SDRAM retention specifications.
        --
        -- Signal Requirements:
        --   CMD    = AUTO REFRESH (CS_N RAS_N CAS_N WE_N = 0001) for 1 cycle,
        --            then NOP during tRFC wait
        --   DQM    = (don't care during refresh)
        --   ADDR   = (don't care during refresh)
        --   BA     = (don't care during refresh)
        --
        -- Prerequisite: All banks must be precharged (no active rows)
        --               Controller ensures this before entering this state
        --
        -- Command Timing:
        --   Cycle 1 (seq_count=0):  Issue CMD_REF (AUTO REFRESH)
        --   Cycles 2-tRFC:          Issue CMD_NOP while internal refresh occurs
        --   Cycle tRFC:             Complete, clear flags, return to IDLE
        --
        -- Timing: Wait tRFC cycles (~70ns = 7 cycles @ 100MHz)
        --
        -- Flags Cleared on Completion:
        --   need_refresh = '0'      --> This refresh serviced
        --   refresh_postponed = '0' --> Clear any postponement flag
        --
        -- Exit Condition: Return to ST_IDLE after tRFC delay completes
        --
        -- Refresh Sources:
        --   - USE_AUTO_REFRESH = true:  Internal counter triggers every 7.8µs
        --   - USE_AUTO_REFRESH = false: External refresh_req signal        

        when ST_REFRESH =>
            if seq_count = 0 then
                cmd_next       <= CMD_REF;
                seq_count_next <= seq_count + 1;
            elsif seq_count = TRFC_CYCLES then
                cmd_next               <= CMD_NOP;
                need_refresh_next      <= '0';
                refresh_postponed_next <= '0';
                state_next             <= ST_IDLE;
                seq_count_next         <= 0;
            else
                cmd_next               <= CMD_NOP;
                seq_count_next         <= seq_count + 1;
            end if;

        --========================================
        -- ST_PRECHARGE - Close currently active row
        --========================================
        -- Purpose: Close (precharge) the currently open row to allow access
        --          to a different row, or prepare for refresh operation.
        --
        -- Signal Requirements:
        --   CMD    = PRECHARGE (CS_N RAS_N CAS_N WE_N = 0010)
        --   A10    = '1' --> PRECHARGE ALL banks (safest, closes any open row)
        --   A9:0   = (don't care when A10=1)
        --   BA     = (don't care when A10=1)
        --   DQM    = (don't care)
        --
        -- Command Timing:
        --   Cycle 1 (seq_count=0):  Issue CMD_PRE with A10=1
        --   Cycles 2-tRP:           Issue CMD_NOP while banks precharge
        --   Cycle tRP:              Complete, clear row_active, return to IDLE
        --
        -- Timing: Wait tRP cycles (~20ns = 2 cycles @ 100MHz)
        --
        -- Why A10=1 (All Banks)?
        --   - Simpler: Don't need to track which specific bank to precharge
        --   - Safer: Ensures all banks are in known precharged state
        --   - Cost: Negligible (tRP timing same for single or all banks)
        --
        -- State Management:
        --   row_active = '0' after completion
        --   active_row/active_bank preserved (for debugging)
        --
        -- Exit Condition: ALWAYS returns to ST_IDLE
        --                 IDLE will then decide next action:
        --                 - ACTIVATE for CPU request
        --                 - REFRESH if that's why we precharged
        --
        -- JEDEC Note: Per datasheet, PRECHARGE always returns controller
        --             to IDLE state, never directly to another operation

        when ST_PRECHARGE =>
            if seq_count = 0 then
                cmd_next        <= CMD_PRE;
                sdram_addr_next <= (10 => '1', others => '0');  -- Set ALL bits!
                seq_count_next  <= seq_count + 1;
            elsif seq_count = TRP_CYCLES then
                -- according to ISSI datasheet
                -- precharge always retuns to IDLE
                cmd_next        <= CMD_NOP;
                sdram_addr_next <= (others => '0');   -- Clear Address
                row_active_next <= '0';
                state_next      <= ST_IDLE;
                seq_count_next  <= 0;
            else
                cmd_next        <= CMD_NOP;
                seq_count_next  <= seq_count + 1;
            end if;	
        
        --========================================
        -- ST_ACTIVATE - Open (activate) a row
        --========================================
        -- Purpose: Open a row in a specific bank, loading entire row into
        --          the bank's sense amplifiers. Required before READ/WRITE.
        --
        -- Signal Requirements:
        --   CMD    = BANK ACTIVATE (CS_N RAS_N CAS_N WE_N = 0011)
        --   BA     = Bank select (from addr_bank_latched)
        --   ADDR   = Row address (13 or 12 bits from addr_row_latched)
        --   DQM    = (don't care)
        --
        -- Command Timing:
        --   Cycle 1 (seq_count=0):  Issue CMD_ACT with bank and row address
        --                           Latch bank/row into active tracking
        --   Cycles 2-tRCD:          Issue CMD_NOP while row activates
        --   Cycle tRCD:             Row ready! Transition to ST_READ or ST_WRITE
        --
        -- Timing: Wait tRCD cycles (~20ns = 2 cycles @ 100MHz)
        --         This is RAS-to-CAS delay - time for row to stabilize
        --
        -- State Tracking (when USE_AUTO_PRECHARGE = false):
        --   active_row  = addr_row_latched   --> Remember which row is open
        --   active_bank = addr_bank_latched  --> Remember which bank
        --   row_active  = '1'                --> Flag that a row is open
        --
        -- Exit Path:
        --   Always → ST_READ or ST_WRITE
        --   Auto-precharge behaviour (A10) is handled inside ST_READ/ST_WRITE,
        --   not here. ACTIVATE is identical regardless of USE_AUTO_PRECHARGE.        
        
        when ST_ACTIVATE =>
            if seq_count = 0 then
                cmd_next         <= CMD_ACT;
                sdram_ba_next    <= addr_bank_latched;
                sdram_addr_next  <= std_logic_vector(resize(unsigned(addr_row_latched), ROW_BITS));
                active_row_next  <= addr_row_latched;
                active_bank_next <= addr_bank_latched;
                row_active_next  <= '1';
                seq_count_next   <= seq_count + 1;
            elsif seq_count = TRCD_CYCLES then
                cmd_next         <= CMD_NOP;
                if wr_n_latched = '0' then
                    state_next <= ST_WRITE;
                else
                    state_next <= ST_READ;
                end if;
                seq_count_next <= 0;
            else
                cmd_next         <= CMD_NOP;
                seq_count_next   <= seq_count + 1;
            end if;

        --========================================
        -- ST_READ - Read from active row
        --========================================
        -- Purpose: Read data from currently active row.
        --          Auto-precharge behaviour is selected by USE_AUTO_PRECHARGE.
        --
        -- Signal Requirements:
        --   CMD    = READ (CS_N RAS_N CAS_N WE_N = 0101)
        --   BA     = Bank select
        --   A10    = USE_AUTO_PRECHARGE: '1' = row closes automatically
        --                               '0' = row stays open
        --   A9:0   = Column address
        --   DQM    = NOT byte_en --> Byte selection
        --
        -- Command Timing:
        --   Cycle 1 (seq_count=0):   Issue READ command with A10 as above
        --   Cycles 2-CAS_LAT:        NOP during CAS latency
        --   Cycle CAS_LAT+1:         Data valid, capture and acknowledge
        --                            Issue BST to stop any residual burst
        --
        -- NOTE: The +1 in CAS_LATENCY+1 is required because:
        --   seq_count=0: Issue READ command
        --   seq_count=1: First cycle of CAS latency
        --   seq_count=2: Second cycle of CAS latency (CAS_LATENCY=2)
        --   seq_count=3: Data valid (= CAS_LATENCY+1)
        --   Data becomes valid one cycle AFTER the CAS latency period completes
        --
        -- Row Management after operation:
        --   USE_AUTO_PRECHARGE = true:  Row closed internally by SDRAM (A10=1)
        --                               row_active <= '0', back to ST_IDLE
        --   USE_AUTO_PRECHARGE = false: Row stays open (A10=0)
        --                               row_active STAYS '1', back to ST_IDLE
        --                               active_row/active_bank unchanged
        --                               Next access to same row skips ACTIVATE
        --
        -- Performance:
        --   USE_AUTO_PRECHARGE = true:  ACT + tRCD + READ + CAS_LAT = ~6-8 cycles
        --   USE_AUTO_PRECHARGE = false, same row: READ + CAS_LAT = ~3 cycles


        when ST_READ =>
            if seq_count = 0 then
                cmd_next                             <= CMD_READ;
                sdram_ba_next                        <= addr_bank_latched;
                sdram_addr_next                      <= (others => '0');
                if USE_AUTO_PRECHARGE = true then
                    sdram_addr_next(10)              <= '1';  -- A10=1, with auto-precharge
                else    
                    sdram_addr_next(10)              <= '0';  -- A10=0, No auto-precharge
                end if;
                sdram_addr_next(COL_BITS-1 downto 0) <= addr_col_latched;
                sdram_dqm_next                       <= not byte_en_latched;
                seq_count_next                       <= seq_count + 1;
            elsif seq_count = CAS_LATENCY + 1 then
                -- NOTE: The +1 is REQUIRED because:
                --   seq_count=0: Issue READ command
                --   seq_count=1: First cycle of CAS latency
                --   seq_count=2: Second cycle of CAS latency (CAS_LATENCY=2)
                --   seq_count=3: Data valid (=CAS_LATENCY+1)
                -- Data becomes valid one cycle AFTER the CAS latency period completes            
                cmd_next       <= CMD_BST;   -- was CMD_NOP
                dout_next      <= sdram_dq;
                ack_next       <= '1';
                sdram_dqm_next <= DQM_IDLE; --"11";
                state_next     <= ST_IDLE;
                seq_count_next <= 0;
            else
                cmd_next       <= CMD_NOP;
                seq_count_next <= seq_count + 1;
            end if;

        --========================================
        -- ST_WRITE - Write to active row
        --========================================
        -- Purpose: Write data to currently active row.
        --          Auto-precharge behaviour is selected by USE_AUTO_PRECHARGE.
        --
        -- Signal Requirements:
        --   CMD    = WRITE (CS_N RAS_N CAS_N WE_N = 0100)
        --   BA     = Bank select
        --   A10    = USE_AUTO_PRECHARGE: '1' = row closes automatically
        --                               '0' = row stays open
        --   A9:0   = Column address
        --   DQM    = NOT byte_en --> Byte selection
        --   DQ     = Write data
        --
        -- Command Timing:
        --   Cycle 1 (seq_count=0):  Issue WRITE command with A10 as above
        --                           Drive data on sdram_dq
        --                           Assert ack immediately (no CAS latency on write)
        --   Cycle 2 (seq_count=1):  Issue BST, tri-state sdram_dq
        --   Cycles 3-tRP:           NOP (data write settling)
        --   Cycle tRP:              Complete, return to ST_IDLE
        --
        -- Row Management after operation:
        --   USE_AUTO_PRECHARGE = true:  Row closed internally by SDRAM (A10=1)
        --                               row_active <= '0', back to ST_IDLE
        --   USE_AUTO_PRECHARGE = false: Row stays open (A10=0)
        --                               row_active STAYS '1', back to ST_IDLE
        --                               active_row/active_bank unchanged
        --                               Next access to same row skips ACTIVATE
        --
        -- Performance:
        --   USE_AUTO_PRECHARGE = true:  ACT + tRCD + WRITE + tRP = ~6-8 cycles
        --   USE_AUTO_PRECHARGE = false, same row: WRITE + tRP = ~3 cycles
      

        when ST_WRITE =>
            if seq_count = 0 then
                cmd_next                             <= CMD_WRITE;
                sdram_ba_next                        <= addr_bank_latched;
                sdram_addr_next                      <= (others => '0');
                if USE_AUTO_PRECHARGE = true  then
                    sdram_addr_next(10)              <= '1';  -- A10=1, with auto-precharge
                else        
                    sdram_addr_next(10)              <= '0';  -- A10=0, NO auto-precharge
                end if;
                sdram_addr_next(COL_BITS-1 downto 0) <= addr_col_latched;
                sdram_dqm_next                       <= not byte_en_latched;
                sdram_dq_next                        <= din_latched;
                ack_next                             <= '1';
                seq_count_next                       <= seq_count + 1;
            elsif seq_count = 1 then
                cmd_next       <= CMD_BST;
                sdram_dq_next  <= (others => 'Z');  -- Tri-state after write    
                seq_count_next <= seq_count + 1;
            elsif seq_count = TRP_CYCLES then
                -- Back to idle, row still active
                cmd_next             <= CMD_NOP;
                sdram_dqm_next       <= DQM_IDLE; --"11";
                if USE_AUTO_PRECHARGE = true  then
                    row_active_next  <= '0';  
                end if;
                state_next           <= ST_IDLE;
                seq_count_next       <= 0;
            else
                cmd_next  <= CMD_NOP;
                seq_count_next <= seq_count + 1;
            end if;
        
        when others =>
            state_next <= ST_INIT;
                
    end case;
    end process;

end rtl;