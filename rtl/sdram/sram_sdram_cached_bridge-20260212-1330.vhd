--------------------------------------------------------------------------------
-- SRAM to SDRAM Bridge with Write-Through Cache
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
-- SRAM to SDRAM Bridge (Write-Through Cached Version)
--------------------------------------------------------------------------------
-- This module provides a transparent SRAM-like interface to a 16-bit SDRAM
-- controller with an integrated write-through cache for 8-bit CPUs.
--
-- Theory of Operation:
--
-- 1. Cache Architecture (Direct-Mapped)
--    - Default: 1KB cache with 16-byte lines = 64 cache lines
--    - Each memory address maps to exactly one cache line:
--      * TAG bits: Identify which block is cached
--      * INDEX bits: Select which cache line (6 bits = 64 lines)
--      * OFFSET bits: Select byte within line (4 bits = 16 bytes)
--    - Cache storage in BRAM (M9K blocks) for efficiency
--    - Valid bit per line indicates if cached data is current
--
-- 2. Write-Through Policy
--    - Writes ALWAYS go to both cache AND SDRAM
--    - On write HIT: Update cache line, then write through to SDRAM
--    - On write MISS: Write through to SDRAM only (NO-ALLOCATE)
--    - Guarantees SDRAM always has correct data (cache never "dirty")
--    - Simple and robust - no complex writeback logic needed
--
-- 3. Read Operations
--    - On read HIT: Return data instantly from cache (1 clock)
--    - On read MISS: Fetch entire 16-byte line from SDRAM
--      * FSM loops 16 times to fetch all bytes in the line
--      * Stores complete line in cache
--      * Marks line as valid
--      * Returns requested byte to CPU
--    - Subsequent reads from same line are cache hits
--
-- 4. Line Fetch Mechanism (Read Miss)
--    States: MISS_FETCH_START → MISS_FETCHING (loop 16x) → CACHE_HIT
--    - Fetches all 16 bytes of the cache line sequentially
--    - Each byte requires one SDRAM access (~10 clocks)
--    - Total miss penalty: ~160 clocks for complete line
--    - But next 15 accesses in that line are instant hits!
--
-- 5. Spatial Locality Optimization
--    - Sequential code execution: Fetch 1 instruction → next 15 are cached
--    - Data structures: Accessing one field → nearby fields are cached
--    - Loop code: First iteration fills cache → subsequent iterations hit
--    - Example: Tight loop at $5000-$500F:
--      * First access: miss, fetch 16 bytes (~160 clocks)
--      * Next 15+ accesses: all hits (1 clock each) = ~100% hit rate
--
-- 6. FSM States
--    - IDLE: Wait for CPU access
--    - CACHE_CHECK: Check for hit/miss, decide read or write path
--    - CACHE_HIT: Return data from cache (read only)
--    - MISS_FETCH_START: Begin fetching byte from SDRAM
--    - MISS_FETCHING: Store fetched byte, continue or complete
--    - WAIT_SDRAM_ACK: Wait for write completion or bypass read
--
-- 7. Clock Stretching via MRDY
--    - Same mechanism as non-cached version
--    - Cache hits return immediately (MRDY high after 1 clock)
--    - Cache misses block CPU until line fetch completes
--    - Writes block CPU until SDRAM write completes
--
-- 8. Cache Bypass Mode
--    - When USE_CACHE = false, behaves like non-cached bridge
--    - Allows runtime testing and comparison
--    - All cache logic is synthesized away if USE_CACHE = false
--
-- 9. Cache Statistics
--    - Tracks hit rate over sliding 256-access window
--    - Formula: hit_percent = (hit_counter * 25) >> 6
--    - Approximates (hits * 100) / 256
--    - Output on cache_hitp for real-time monitoring
--
-- 10. SDRAM Refresh
--    - Same automatic refresh as non-cached version
--    - Refresh only issued when bus is idle
--
-- Performance Characteristics:
--    - Cache HIT: 1 clock (instant)
--    - Cache MISS (read): ~160 clocks (fetch entire 16-byte line)
--    - Write (hit or miss): ~10-15 clocks (SDRAM write time)
--    - Expected hit rate on real 6502/6809 code: 80-95%
--    - Expected hit rate on random access: 25-50% (due to spatial locality)
--    - Enables CPUs to run at 10-15 MHz with 120 MHz SDRAM
--
-- Design Notes:
--    - Cache data stored as 1D array for proper BRAM inference
--    - All address calculations captured in IDLE state to prevent
--      metastability when sram_addr changes during operation
--    - NO-ALLOCATE on write miss prevents creating lines with garbage data
--    - Write-through simplifies coherency (SDRAM is always correct)
--
--------------------------------------------------------------------------------


library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity sram_sdram_bridge is
    generic (
        ADDR_BITS        : integer := 24;
        SDRAM_MHZ        : integer := 100;
        GENERATE_REFRESH : boolean := true;
        USE_CACHE        : boolean := true;
        -- Cache parameters
        CACHE_SIZE_BYTES : integer := 1024;   -- 1KB cache
        LINE_SIZE_BYTES  : integer := 16;     -- 16-byte cache lines
		RAM_BLOCK_TYPE   : string  := "M9K, no_rw_check"   -- "M9K", "M4K", "M10K", "AUTO"
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
        cache_hitp    : out unsigned(6 downto 0);  -- 0 to 100%
        debug         : out std_logic_vector(2 downto 0)
    );
end sram_sdram_bridge;

architecture rtl of sram_sdram_bridge is

    -- Refresh timing
    constant REFRESH_INTERVAL : integer := (SDRAM_MHZ * 78) / 10;
    
    -- Cache geometry
    constant NUM_LINES    : integer := CACHE_SIZE_BYTES / LINE_SIZE_BYTES;  -- 64 lines
    constant INDEX_BITS   : integer := 6;   -- log2(64)
    constant OFFSET_BITS  : integer := 4;   -- log2(16)
    constant TAG_BITS     : integer := ADDR_BITS - INDEX_BITS - OFFSET_BITS;
    
    type state_type is (IDLE, CACHE_CHECK, CACHE_HIT, MISS_FETCH_START, MISS_FETCHING, WAIT_SDRAM_ACK);
    signal state             : state_type := IDLE;
    
    -- Synchronizers
    signal E_meta, E_sync    : std_logic;
    signal E_sync_prev       : std_logic;
    signal sdram_ack_prev    : std_logic;
    signal session_active    : std_logic := '0';
    
    -- Refresh
    signal refresh_counter   : integer range 0 to REFRESH_INTERVAL := 0;
    signal refresh_pending   : std_logic := '0';
    
    -- Cache storage (1KB in BRAM) - TABLEAU 1D
    type cache_data_type is array (0 to CACHE_SIZE_BYTES-1) of std_logic_vector(7 downto 0);
    signal cache_data : cache_data_type;
    
    -- Tag storage
    type tag_array_type is array (0 to NUM_LINES-1) of std_logic_vector(TAG_BITS-1 downto 0);
    signal tag_array : tag_array_type := (others => (others => '0'));

    -- Attributes BRAM
    attribute ramstyle : string;
    attribute ramstyle of cache_data : signal is RAM_BLOCK_TYPE; -- was M9K
    attribute ramstyle of tag_array  : signal is RAM_BLOCK_TYPE; -- was M9K 

--	attribute ram_init_file : string;
--	attribute ramstyle : string;
--	attribute ramstyle of cache_data : signal is "M4K, no_rw_check";
    
    -- Valid bits
    signal valid_bits : std_logic_vector(NUM_LINES-1 downto 0) := (others => '0');
    
    -- Saved request
    signal saved_we_n       : std_logic;
    signal saved_addr       : std_logic_vector(ADDR_BITS-1 downto 0);
    signal saved_din        : std_logic_vector(7 downto 0);
    signal saved_tag        : std_logic_vector(TAG_BITS-1 downto 0);
    signal saved_index      : unsigned(INDEX_BITS-1 downto 0);
    signal saved_offset     : unsigned(OFFSET_BITS-1 downto 0);
    signal saved_cache_addr : integer range 0 to CACHE_SIZE_BYTES-1;
    
    -- Line fetch
    signal byte_counter     : unsigned(OFFSET_BITS-1 downto 0);
    signal line_base_addr   : std_logic_vector(ADDR_BITS-1 downto 0);
    signal fetch_cache_addr : integer range 0 to CACHE_SIZE_BYTES-1;
    signal current_fetch_addr : std_logic_vector(ADDR_BITS-1 downto 0);
    
    -- Cache hit detection
    signal is_hit           : std_logic;
    signal is_valid         : std_logic;
    
    -- Statistics - 256-access sliding window
    signal access_counter   : unsigned(7 downto 0) := (others => '0');
    signal hit_counter      : unsigned(7 downto 0) := (others => '0');
    signal hit_percent      : unsigned(6 downto 0) := (others => '0');
    
    signal cache_wr_en   : std_logic := '0';
    signal cache_wr_addr : integer range 0 to CACHE_SIZE_BYTES-1;
    signal cache_wr_data : std_logic_vector(7 downto 0);
    signal read_addr_reg : integer range 0 to CACHE_SIZE_BYTES-1;
	
begin

    -- Output statistics
    cache_hitp <= hit_percent;
    
    -- Hit detection
    is_valid <= valid_bits(to_integer(saved_index));
    is_hit   <= '1' when (is_valid = '1' and 
                          tag_array(to_integer(saved_index)) = saved_tag)
                    else '0';
    
    -- Line base address (zero out offset bits)
    line_base_addr <= saved_addr(ADDR_BITS-1 downto OFFSET_BITS) & (OFFSET_BITS-1 downto 0 => '0');

    debug <= "000" when state = IDLE            and session_active = '0' else
             "001" when state = CACHE_CHECK     else
             "010" when state = CACHE_HIT       else
             "011" when state = MISS_FETCH_START else
             "100" when state = MISS_FETCHING   else
             "101" when state = WAIT_SDRAM_ACK  and saved_we_n = '1' else
             "110" when state = WAIT_SDRAM_ACK  and saved_we_n = '0' else
             "111";

    process(sdram_clk)
        variable current_fetch_addr : std_logic_vector(ADDR_BITS-1 downto 0);
    begin
        if rising_edge(sdram_clk) then
            -- Two-stage synchronizer
            E_meta <= E;
            E_sync <= E_meta;
            E_sync_prev <= E_sync;
            sdram_ack_prev <= sdram_ack;
            
            if reset_n = '0' then
                state           <= IDLE;
                mrdy            <= '1';
                sdram_req       <= '0';
                sdram_wr_n      <= '0';
                session_active  <= '0';
                valid_bits      <= (others => '0');
                access_counter  <= (others => '0');
                hit_counter     <= (others => '0');
                hit_percent     <= (others => '0');
                byte_counter    <= (others => '0');
                if GENERATE_REFRESH = true then
                    refresh_req     <= '0';
                    refresh_counter <= 0;
                    refresh_pending <= '0';
                end if;
            else
                -- Refresh counter logic
                if GENERATE_REFRESH = true then
                    if refresh_counter >= REFRESH_INTERVAL then
                        refresh_counter <= 0;
                        refresh_pending <= '1';
                    else
                        refresh_counter <= refresh_counter + 1;
                    end if;
                    
                    if E_sync = '0' and refresh_pending = '1' and state = IDLE and session_active = '0' then
                        refresh_req <= '1';
                        refresh_pending <= '0';
                    else
                        refresh_req <= '0';
                    end if;            
                else 
                    refresh_req <= '0';
                end if;
                
                case state is
                    -- ==========================================
                    -- IDLE - Wait for CPU request
                    -- ==========================================
                    -- Detects rising edge of E signal when chip select is active.
                    -- When detected:
                    --   1. Captures CPU request (address, data, read/write)
                    --   2. Immediately pulls MRDY low to block CPU
                    --   3. Parses address into TAG, INDEX, OFFSET for cache lookup
                    --   4. Routes to CACHE_CHECK if cache enabled, or direct to SDRAM if bypassed
                    -- All address calculations done HERE to prevent metastability issues
                    -- if sram_addr changes during multi-cycle operations.                    
                    
                    when IDLE =>
                        mrdy <= '1';
                        sdram_req <= '0';
                        
                        if (session_active = '1') or (sram_ce_n = '0' and E_sync = '1' and E_sync_prev = '0') then
                            session_active <= '1';
                            mrdy <= '0';
                            
                            -- Save request
                            saved_we_n   <= sram_we_n;
                            saved_addr   <= sram_addr;
                            saved_din    <= sram_din;
                            
                            if USE_CACHE = true then
                                -- Parse address
                                saved_tag    <= sram_addr(ADDR_BITS-1 downto INDEX_BITS+OFFSET_BITS);
                                saved_index  <= unsigned(sram_addr(INDEX_BITS+OFFSET_BITS-1 downto OFFSET_BITS));
                                saved_offset <= unsigned(sram_addr(OFFSET_BITS-1 downto 0));
                                saved_cache_addr <= to_integer(unsigned(sram_addr(INDEX_BITS+OFFSET_BITS-1 downto 0)));
                                state <= CACHE_CHECK;
                            else
                                -- Bypass cache
                                if sdram_ready = '1' then
                                    sdram_addr <= sram_addr(ADDR_BITS-1 downto 1);
                                    if sram_addr(0) = '0' then
                                        sdram_byte_en <= "01";
                                    else
                                        sdram_byte_en <= "10";
                                    end if;
                                    if sram_we_n = '1' then
                                        sdram_wr_n <= '1';
                                    else
                                        sdram_wr_n <= '0';
                                        sdram_din  <= sram_din & sram_din;
                                    end if;
                                    sdram_req <= '1';
                                    state <= WAIT_SDRAM_ACK;
                                end if;
                            end if;
                        end if;
                    
                    -- ==========================================
                    -- CACHE_CHECK - Determine cache hit or miss
                    -- ==========================================
                    -- Compares saved_tag against tag_array[saved_index] to detect hit/miss.
                    -- Increments access counter for statistics.
                    --
                    -- READ path:
                    --   - HIT: Go to CACHE_HIT to return data instantly
                    --   - MISS: Go to MISS_FETCH_START to load entire 16-byte line
                    --
                    -- WRITE path (write-through):
                    --   - HIT: Update cache line, then write through to SDRAM
                    --   - MISS: NO-ALLOCATE - just write through to SDRAM
                    --           (avoids creating cache lines with partial garbage data)
                    --   - Both cases go to WAIT_SDRAM_ACK
                    --
                    -- Statistics: Every 256 accesses, calculates hit percentage                    
                
                    when CACHE_CHECK =>
                        -- Count this access
                        access_counter <= access_counter + 1;
                        
                        if saved_we_n = '1' then
                            -- READ operation
                            if is_hit = '1' then
                                -- Cache hit on read!
                                hit_counter <= hit_counter + 1;
                                state <= CACHE_HIT;
                            else
                                -- Cache miss - fetch entire line from SDRAM
                                byte_counter <= (others => '0');
                                state <= MISS_FETCH_START;
                            end if;
                        else
                            -- WRITE operation - write-through
                            if is_hit = '1' then
                                hit_counter <= hit_counter + 1;
                                -- Update cache
--                                cache_data(saved_cache_addr) <= saved_din;
                                cache_wr_en     <= '1';
                                cache_wr_addr   <= saved_cache_addr;
                                cache_wr_data   <= saved_din;
                            end if;
                            -- NO-ALLOCATE on write miss - just write through to SDRAM
                            
                            -- Always write through to SDRAM
                            if sdram_ready = '1' then
                                sdram_addr <= saved_addr(ADDR_BITS-1 downto 1);
                                if saved_addr(0) = '0' then
                                    sdram_byte_en <= "01";
                                else
                                    sdram_byte_en <= "10";
                                end if;
                                sdram_din  <= saved_din & saved_din;
                                sdram_wr_n <= '0';
                                sdram_req  <= '1';
                                state      <= WAIT_SDRAM_ACK;
                            end if;
                        end if;
                        
                        -- Every 256 accesses, calculate percentage
                        if access_counter = 255 then
                            hit_percent <= resize((hit_counter * 25) srl 6, 7);
                            hit_counter <= (others => '0');
                        end if;
                    
                    -- ==========================================
                    -- CACHE_HIT - Return cached data to CPU
                    -- ==========================================
                    -- Data is available in cache - return it immediately (1 clock).
                    -- - Outputs cached byte to sram_dout
                    -- - Releases MRDY to unblock CPU
                    -- - Returns to IDLE
                    -- This is the fast path - no SDRAM access needed!   
                    
--                    when CACHE_HIT =>
--                        sram_dout <= cache_data(saved_cache_addr);
--                        session_active <= '0';
--                        mrdy <= '1';
--                        state <= IDLE;

                    when CACHE_HIT =>
                   --     read_addr_reg <= saved_cache_addr;   -- register it one cycle early if needed
                        read_addr_reg <= saved_cache_addr;
                        sram_dout     <= cache_data(read_addr_reg);
                        session_active <= '0';
                        mrdy <= '1';
                        state <= IDLE;
                    
                    -- ==========================================
                    -- MISS_FETCH_START - Begin fetching one byte
                    -- ==========================================
                    -- Read miss detected - must fetch entire 16-byte cache line.
                    -- This state fetches ONE byte at a time from SDRAM.
                    -- Called 16 times (byte_counter 0..15) to fetch complete line.
                    --
                    -- For each byte:
                    --   1. Calculate SDRAM address (line_base + byte_counter)
                    --   2. Calculate where to store in cache (saved_index:byte_counter)
                    --   3. Issue SDRAM read request
                    --   4. Set byte enable based on odd/even address
                    --   5. Go to MISS_FETCHING to wait for data
                    --
                    -- Note: SDRAM address calculation divides byte_counter by 2
                    -- because SDRAM is 16-bit (2 bytes per word)                    

                    when MISS_FETCH_START =>
                        if sdram_ready = '1' then
                            -- Calculate address for current byte in line
                            current_fetch_addr := std_logic_vector(unsigned(line_base_addr) + byte_counter);
                            
                            -- Calculate cache address for this byte
                            fetch_cache_addr <= to_integer(saved_index & byte_counter);
                            
                            sdram_req  <= '1';
                            sdram_wr_n <= '1';  -- Read
                            sdram_addr <= std_logic_vector(unsigned(line_base_addr(ADDR_BITS-1 downto 1)) + 
                                                           resize(byte_counter srl 1, ADDR_BITS-1));
                            if byte_counter(0) = '0' then
                                sdram_byte_en <= "01";
                            else
                                sdram_byte_en <= "10";
                            end if;
                            state <= MISS_FETCHING;
                        end if;

                    -- ==========================================
                    -- MISS_FETCHING - Store byte and continue/complete
                    -- ==========================================
                    -- Waits for SDRAM to acknowledge the read request.
                    -- When data arrives:
                    --   1. Extracts correct byte from 16-bit SDRAM word
                    --      (lower or upper byte based on current_fetch_addr bit 0)
                    --   2. Stores byte in cache at calculated position
                    --   3. Checks if line fetch is complete (byte_counter = 15)
                    --
                    -- If NOT complete (bytes 0-14):
                    --   - Increment byte_counter
                    --   - Loop back to MISS_FETCH_START for next byte
                    --
                    -- If COMPLETE (byte 15):
                    --   - Update tag_array with saved_tag
                    --   - Set valid bit for this cache line
                    --   - Go to CACHE_HIT to return requested byte to CPU
                    --
                    -- Total line fetch time: 16 bytes × ~10 clocks = ~160 clocks                        
                        
                    when MISS_FETCHING =>
                        if sdram_ack = '1' and sdram_ack_prev = '0' then
                            sdram_req <= '0';
                            
                            cache_wr_en   <= '1';
                            cache_wr_addr <= fetch_cache_addr;                            
                            -- Store byte in cache - use saved current_fetch_addr!
--                            if current_fetch_addr(0) = '0' then
--                                cache_data(fetch_cache_addr) <= sdram_dout(7 downto 0);
--                            else
--                                cache_data(fetch_cache_addr) <= sdram_dout(15 downto 8);
--                            end if;
                            
                            if byte_counter = LINE_SIZE_BYTES - 1 then
                                -- Entire line fetched!
                                tag_array(to_integer(saved_index)) <= saved_tag;
                                valid_bits(to_integer(saved_index)) <= '1';
                                
                                -- Return requested byte to CPU
                                state <= CACHE_HIT;
                            else
                                -- Fetch next byte
                                byte_counter <= byte_counter + 1;
                                state <= MISS_FETCH_START;
                            end if;
                        end if;
                        
                    -- ==========================================
                    -- WAIT_SDRAM_ACK - Wait for SDRAM operation
                    -- ==========================================
                    -- Used for two scenarios:
                    --
                    -- 1. WRITE completion (write-through from CACHE_CHECK)
                    --    - Waits for SDRAM to acknowledge write
                    --    - No data to return to CPU
                    --
                    -- 2. BYPASS READ (when USE_CACHE = false from IDLE)
                    --    - Waits for SDRAM to return data
                    --    - Extracts correct byte from 16-bit word
                    --    - Returns data to CPU via sram_dout
                    --
                    -- When SDRAM acknowledges:
                    --   - Deassert sdram_req
                    --   - Return sdram_wr_n to idle state (high)
                    --   - Release MRDY to unblock CPU
                    --   - Return to IDLE                    
                    
                    when WAIT_SDRAM_ACK =>
                        if sdram_ack = '1' and sdram_ack_prev = '0' then
                            sdram_req  <= '0';
                            sdram_wr_n <= '1';
                            
                            if saved_we_n = '1' then
                                -- Bypass read completed
                                if saved_addr(0) = '0' then
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
                
                if cache_wr_en = '1' then
                    cache_data(cache_wr_addr) <= cache_wr_data;
                    cache_wr_en <= '0';           -- self-clear (optional but clean)
                end if;
            end if;
        end if;
    end process;

end rtl;