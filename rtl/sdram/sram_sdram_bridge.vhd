--------------------------------------------------------------------------------
-- SRAM to SDRAM Bridge without Write-Through Cache
-- Copyright (c) 2025 Didier Derny
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
-- For commercial licensing inquiries, contact: [ton email si tu veux]
--
-- Full license: https://creativecommons.org/licenses/by-nc-sa/4.0/
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- SRAM to SDRAM Bridge (Non-Cached Version)
--------------------------------------------------------------------------------
-- This module provides a transparent SRAM-like interface to a 16-bit SDRAM
-- controller for 8-bit CPUs (6502, 6800, 6809).
--
-- Theory of Operation:
--
-- 1. CPU Interface (SRAM-like)
--    - 8-bit data bus
--    - E clock input (CPU enable signal)
--    - Standard SRAM control signals (ce_n, we_n, oe_n)
--    - Detects rising edge of E when chip select is active (ce_n = '0')
--
-- 2. Clock Stretching via MRDY
--    - When CPU access is detected, MRDY is immediately pulled low
--    - CPU is blocked until SDRAM operation completes
--    - MRDY returns high when data is ready
--    - This allows slow SDRAM to work with fast CPUs
--
-- 3. Byte-to-Word Conversion
--    - SDRAM is 16-bit wide (2 bytes per word)
--    - Address bit 0 selects which byte:
--      * addr(0) = '0' → lower byte (bits 7:0)
--      * addr(0) = '1' → upper byte (bits 15:8)
--    - sdram_byte_en controls which byte is accessed
--
-- 4. FSM States
--    - IDLE: Wait for CPU access (E rising edge with ce_n low)
--    - WAIT_SDRAM_ACK: Wait for SDRAM controller to complete operation
--
-- 5. Clock Domain Crossing
--    - E signal from CPU clock domain is synchronized using 2-stage FF
--    - Prevents metastability issues
--
-- 6. SDRAM Refresh
--    - Automatic refresh request generation every 7.8µs
--    - Refresh only issued when bus is idle (no active CPU session)
--    - Can be disabled via GENERATE_REFRESH generic
--
-- 7. Compatibility
--    - USE_CACHE, CACHE_SIZE_BYTES, LINE_SIZE_BYTES generics are present
--      for interface compatibility with cached version but are not used
--    - cache_hitp output always returns 0
--
-- Performance:
--    - Each CPU access requires full SDRAM round-trip (~10-15 SDRAM clocks)
--    - No caching, no prefetch
--    - Suitable for CPUs up to ~10-14 MHz depending on SDRAM clock
--
--------------------------------------------------------------------------------

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity sram_sdram_bridge is
    generic (
        ADDR_BITS        : integer := 24;
        SDRAM_MHZ        : integer := 100;
        GENERATE_REFRESH : boolean := true;  -- generate refresh_req  false = don't refresh
        USE_CACHE        : boolean := true;    
        -- Cache parameters
        CACHE_SIZE_BYTES : integer := 1024;  -- 1KB cache
        LINE_SIZE_BYTES  : integer := 16;    -- 16-byte cache lines
        RAM_BLOCK_TYPE   : string  := "M9K" 
    );
    port (
        sdram_clk    : in   std_logic;
        E            : in   std_logic;
        reset_n      : in   std_logic;
        
        -- SRAM-like interface (CPU side)
        sram_ce_n     : in  std_logic;
        sram_we_n     : in  std_logic;
        sram_oe_n     : in  std_logic;
        sram_addr     : in  std_logic_vector(ADDR_BITS-1 downto 0);
        sram_din      : in  std_logic_vector(7 downto 0);
        sram_dout     : out std_logic_vector(7 downto 0);
        
        -- Memory ready output (for clock stretching)
        mrdy          : out std_logic;
        
        -- SDRAM controller interface
        sdram_req     : out std_logic;
        sdram_wr_n    : out std_logic;
        sdram_addr    : out std_logic_vector(ADDR_BITS-2 downto 0);
        sdram_din     : out std_logic_vector(15 downto 0);
        sdram_dout    : in  std_logic_vector(15 downto 0);
        sdram_byte_en : out std_logic_vector(1 downto 0);
        sdram_ready   : in  std_logic;
        sdram_ack     : in  std_logic;
        refresh_req   : out std_logic;

        -- Cache statistics
        cache_hitp    : out unsigned(6 downto 0)  -- 0 to 100%
    );
end sram_sdram_bridge;

architecture rtl of sram_sdram_bridge is

    -- Refresh timing: 7.8µs for 8K refreshes in 64ms
    -- At 100MHz: 780 clocks per refresh
    constant REFRESH_INTERVAL : integer := (SDRAM_MHZ * 78) / 10;  -- 7.8µs
    
    type state_type is (IDLE, WAIT_SDRAM_ACK);
    signal state             : state_type := IDLE;
    signal E_prev            : std_logic;
    signal sdram_ack_prev    : std_logic;
	 signal session_active    : std_logic := '0';
	 signal E_meta, E_sync    : std_logic;
	 signal E_sync_prev       : std_logic;
    
    signal refresh_counter   : integer range 0 to REFRESH_INTERVAL := 0;
    signal refresh_pending   : std_logic := '0';
	
begin

    cache_hitp <= (others => '0');

    process(sdram_clk)
        begin
        if rising_edge(sdram_clk) then
            -- Two-stage synchronizer
            E_meta <= E;           -- First FF (captures metastability)
            E_sync <= E_meta;      -- Second FF (clean output)
            E_sync_prev <= E_sync; -- Edge detection on synchronized signal
		
				sdram_ack_prev <= sdram_ack;
			
            if reset_n = '0' then
                state           <= IDLE;
                mrdy            <= '1';
                sdram_req       <= '0';
                sdram_wr_n      <= '0';
			     	 session_active  <= '0';
                if GENERATE_REFRESH = true then
                    refresh_req     <= '0';
                    refresh_counter <= 0;
                    refresh_pending <= '0';
                end if;
            else
                if GENERATE_REFRESH = true  then
                    -- Refresh counter (runs always)
                    if refresh_counter >= REFRESH_INTERVAL then
                        refresh_counter <= 0;
                        refresh_pending <= '1';  -- Mark that refresh is needed
                    else
                        refresh_counter <= refresh_counter + 1;
                    end if;
                
                    if E_sync = '0' and refresh_pending = '1' and state = IDLE and session_active = '0' then
                        refresh_req <= '1';
                        refresh_pending <= '0';  -- Clear pending flag
                    else
                        refresh_req <= '0';
                    end if;            
                else 
                    refresh_req <= '0';
                end if;
                
                case state is
                    -- ==========================================
                    -- IDLE - Wait for CPU request and initiate SDRAM access
                    -- ==========================================
                    -- Detects rising edge of E signal when chip select is active (ce_n = '0').
                    -- This implements the classical 6800/6809 bus protocol where E rising
                    -- edge indicates valid address and control signals.
                    --
                    -- Operation sequence:
                    --   1. Detect E rising edge with ce_n low → CPU wants memory access
                    --   2. Set session_active flag to remember we're processing a request
                    --   3. Immediately pull MRDY low to BLOCK the CPU (clock stretching)
                    --   4. Wait for SDRAM controller to be ready (sdram_ready = '1')
                    --   5. When ready, issue SDRAM request:
                    --      - Convert 8-bit address to 16-bit SDRAM word address (divide by 2)
                    --      - Set byte_en to select upper or lower byte based on addr(0)
                    --      - Set read/write control (sdram_wr_n)
                    --      - For writes: duplicate data byte to both halves of 16-bit word
                    --      - Assert sdram_req to start operation
                    --   6. Transition to WAIT_SDRAM_ACK
                    --
                    -- Note: CPU remains blocked (MRDY low) until SDRAM completes operation.
                    -- This allows slow SDRAM (~10-15 clocks) to work with fast CPUs.
                
                    when IDLE =>
                        mrdy <= '1';
                        sdram_req <= '0';
                        -- trigger on E rising while cs is low
								if (session_active = '1') or (sram_ce_n = '0' and E_sync = '1' and E_sync_prev = '0') then
                           -- we have detect the rising edge of E combined with CS low
									session_active <= '1';
								   -- we immediately block the cpu cycle
								   mrdy <= '0';
						          -- no we wait until the sdram is ready with the cpu cycle blocked by mrdy
							      if sdram_ready = '1' then 
										sdram_addr  <= sram_addr(ADDR_BITS - 1 downto 1);
										if sram_addr(0) = '0' then
											sdram_byte_en <= "01";
										else
											sdram_byte_en <= "10";
										end if;
										if sram_we_n = '1' then
											sdram_wr_n  <= '1';
										else
											sdram_wr_n  <= '0';
											sdram_din <= sram_din & sram_din;
										end if;
										sdram_req   <= '1';
										state <= WAIT_SDRAM_ACK;
									end if;
								end if;	

							-- ==========================================
							-- WAIT_SDRAM_ACK - Wait for SDRAM operation completion
							-- ==========================================
							-- Waits for SDRAM controller to complete the requested operation.
							-- CPU remains blocked (MRDY low) during this entire time.
							--
							-- When SDRAM acknowledges (sdram_ack rising edge):
							--
							-- For READ operations:
							--   1. SDRAM returns 16-bit word on sdram_dout
							--   2. Extract the correct byte based on original address bit 0:
							--      - addr(0) = '0' → use lower byte (bits 7:0)
							--      - addr(0) = '1' → use upper byte (bits 15:8)
							--   3. Place extracted byte on sram_dout for CPU to read
							--
							-- For WRITE operations:
							--   1. No data to return to CPU
							--   2. SDRAM controller has written the byte
							--
							-- Cleanup and return to IDLE:
							--   1. Deassert sdram_req (operation complete)
							--   2. Return sdram_wr_n to idle state (high)
							--   3. Clear session_active flag
							--   4. Release MRDY (high) to UNBLOCK CPU - CPU can continue
							--   5. Return to IDLE state
							--
							-- Timing: Typical SDRAM access takes 10-15 clocks @ 100-120 MHz
							-- This limits maximum CPU speed to ~10-14 MHz for continuous accesses.
						
							when WAIT_SDRAM_ACK =>
								if sdram_ack = '1' and sdram_ack_prev = '0' then
									sdram_req <= '0';
									sdram_wr_n  <= '1';
									if sram_we_n = '1' then
										-- need to get the data from sdram								
										if sram_addr(0) = '0' then
											sram_dout <= sdram_dout(7 downto 0);
										else
											sram_dout <= sdram_dout(15 downto 8);
										end if;
									end if;
									session_active <= '0';
									mrdy <= '1';
									state <= IDLE;
								end if;

					end case;
				end if;
		end if;
	end process;

end rtl;

