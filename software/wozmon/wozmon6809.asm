* Woz Monitor for Motorola 6809
* Converted from 6800 version
* Copyright 2011 Eric Smith <eric@brouhaha.com> (original 6800 version)
*
* This program is free software; you can redistribute and/or modify it
* under the terms of the GNU General Public License version 3 as
* published by the Free Software Foundation.

* Zero page variables
XAM         EQU     $0024       * two bytes
ST          EQU     $0026       * two bytes  
H           EQU     $0028
L           EQU     $0029
MODE        EQU     $002B
YSAV        EQU     $002C       * two bytes
INPTR       EQU     $002E       * two bytes

IN          EQU     $0200

* I/O addresses
KBD         EQU     $D010
KBDCR       EQU     $D011
DSP         EQU     $D012
DSPCR       EQU     $D013

START       EQU     $FF00
STACK       EQU     $01FF

            ORG     START

RESET       LDB     #$7F        * Mask for DSP data direction register
            STB     DSP         * Set it up
            LDB     #$A7        * KBD and DSP control register mask
            STB     KBDCR       * Enable interrupts, set CA1, CB1
            STB     DSPCR       * positive edge sense/output mode
            LDS     #STACK

* Get line of input from keyboard, echoing to display
* Note: B contains $A7, so INCB will set negative flag

NOTCR       CMPA    #$DF        * "_"? (back arrow)
            BEQ     BACKSPACE   * Yes (long branch)
            CMPA    #$9B        * ESC?
            BEQ     ESCAPE      * Yes (long branch)
            LEAX    1,X         * Advance text index (6809 advantage)
            INCB
            BPL     NEXTCHAR    * Auto ESC if > 127

ESCAPE      LDA     #$DC        * '\'
            JSR     ECHO        * Output it

GETLINE     LDA     #$8D        * CR
            JSR     ECHO        * Output it
            LDX     #IN+1       * Initialize text index
            LDB     #1

BACKSPACE   LEAX    -1,X        * Back up text index (6809 indexed)
            DECB
            BMI     GETLINE     * Beyond start of line, reinitialize

NEXTCHAR    LDA     KBDCR       * Key ready?
            BPL     NEXTCHAR    * Loop until ready
            LDA     KBD         * Load character, B7 should be '1'
            STA     ,X          * Add to text buffer (6809 indexed)
            BSR     ECHO        * Display character
            CMPA    #$8D        * CR?
            BNE     NOTCR       * No

* Process input line
CR          LDX     #IN+255     * Reset text index (6809 can do IN-1+256)
            STX     INPTR
            CLRA                * For XAM mode, 0->A

SETBLOK     ASLA                * Leaves $56 if setting BLOCK XAM mode
SETMODE     STA     MODE        * $00=XAM, $BA=STOR, $56=BLOK XAM
BLSKIP      INC     INPTR+1     * Advance text index
NEXTITEM    LDX     INPTR
            LDA     ,X          * Get character (6809 indexed)
            CMPA    #$8D        * CR?
            BEQ     GETLINE     * Yes, done this line
            CMPA    #$AE        * "."?
            BEQ     SETBLOK     * Set BLOCK XAM mode
            BLS     BLSKIP      * Skip delimiter
            CMPA    #$BA        * ":"?
            BEQ     SETMODE     * Yes, set STOR mode
            CMPA    #$D2        * "R"?
            BEQ     RUN         * Yes, run user program
            CLR     L           * $00->L
            CLR     H           * and H
            STX     YSAV        * Save for comparison

NEXTHEX     LDX     INPTR
            LDA     ,X          * Get character for hex test
            EORA    #$B0        * Map digits to $0-9
            CMPA    #$09        * Digit?
            BLS     DIG         * Yes
            ADDA    #$89        * Map letter "A"-"F" to $FA-FF
            CMPA    #$F9        * Hex letter?
            BLS     NOTHEX      * No, character not hex

DIG         ASLA                * Hex digit to MSD of A
            ASLA
            ASLA  
	    ASLA
            LDB     #4          * Shift count
HEXSHIFT    ASLA                * Hex digit left, MSB to carry
            ROL     L           * Rotate into LSD
            ROL     H           * Rotate into MSD
            DECB                * Done 4 shifts?
            BNE     HEXSHIFT    * No, loop
            INC     INPTR+1     * Advance text index
            BRA     NEXTHEX     * Check next character for hex

NOTHEX      CMPX    YSAV        * Check if L,H empty (no hex digits)
            BEQ     ESCAPE      * Yes, generate ESC sequence
            TST     MODE        * Test MODE byte
            BPL     NOTSTOR     * B6=0 for STOR, 1 for XAM and BLOCK XAM

* STOR mode
            LDX     ST
            LDA     L           * LSD's of hex data
            STA     ,X+         * Store and increment (6809 auto-increment)
            STX     ST
TONEXTITEM  BRA     NEXTITEM    * Get next command item

PRBYTE      PSHS    A           * Save A for LSD (6809 stack)
            LSRA                * MSD to LSD position
            LSRA
            LSRA
            LSRA
            BSR     PRHEX       * Output hex digit
            PULS    A           * Restore A
PRHEX       ANDA    #$0F        * Mask LSD for hex print
            ORA     #$B0        * Add "0"
            CMPA    #$B9        * Digit?
            BLS     ECHO        * Yes, output it
            ADDA    #$07        * Add offset for letter
ECHO        TST     DSP         * DA bit (B7) cleared yet?
            BMI     ECHO        * No, wait for display
            STA     DSP         * Output character, Sets DA
            RTS                 * Return

RUN         LDX     XAM
            JMP     ,X          * Run at current XAM index (6809 indexed)

NOTSTOR     BNE     XAMNEXT     * MODE=$00 for XAM, $56 for BLOCK XAM
            LDX     H           * Copy hex data to
            STX     ST          * 'store index'
            STX     XAM         * And to 'XAM index'
            CLRA                * Set Z flag to force following branch

NXTPRNT     BNE     PRDATA      * NE means no address to print
            LDA     #$8D        * CR
            BSR     ECHO        * Output it
            LDA     XAM         * 'EXAMine index' high-order byte
            BSR     PRBYTE      * Output it in hex format
            LDA     XAM+1       * Low-order 'EXAMine index' byte
            BSR     PRBYTE      * Output it in hex format
            LDA     #$BA        * ":"
            BSR     ECHO        * Output it

PRDATA      LDA     #$A0        * Blank
            BSR     ECHO        * Output it
            LDX     XAM
            LDA     ,X          * Get data byte at 'eXAMine index'
            BSR     PRBYTE      * Output it in hex format

XAMNEXT     CLR     MODE        * 0->MODE (XAM mode)
            LDX     XAM         * Compare 'eXAMine index' to hex data
            CMPX    H
            BEQ     TONEXTITEM  * Not less, so more data to output
            LEAX    1,X         * Increment using 6809 indexed (saves byte)
            STX     XAM
            LDA     XAM+1       * Check low-order 'examine index' byte
            ANDA    #$07        * For MOD 8 = 0
            BRA     NXTPRNT     * Always taken

* 6809 Vector table
            ORG     $FFF4
*           FDB     $0000       * Reserved
*           FDB     $0000       * SWI3
            FDB     $0000       * SWI2
            FDB     $0000       * FIRQ
            FDB     $0000       * IRQ
            FDB     $0000       * SWI
            FDB     $F000       * NMI
            FDB     $FF00       * RESET
