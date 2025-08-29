* This is a rewrite of the Apple 1 monitor to run on an MC6800
* microprocessor, rather than the MCS6502 microprocessor that
* was standard.  This source code will assemble with the
* AS Macro Assembler; with minor changes it should assemble
* with any MC6800 assembler.
*
* Copyright 2011 Eric Smith <eric@brouhaha.com>
*
* This program is free software; you can redistribute and/or modify it
* under the terms of the GNU General Public License version 3 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
*
* The text of the license may be found online at:
*     http://www.brouhaha.com/~eric/software/GPLv3
* or:
*     http://www.gnu.org/licenses/gpl-3.0.txt


XAM         EQU 	$0024       * two bytes
ST          EQU 	$0026       * two bytes
H           EQU 	$0028
L           EQU 	$0029

MODE        EQU 	$002b
YSAV        EQU 	$002c       * two bytes
INPTR       EQU 	$002e       * two bytes

IN	    	EQU 	$0200
	
KBD	    	EQU 	$d010
KBDCR	    EQU 	$d011
DSP	    	EQU 	$d012
DSPCR	    EQU 	$d013

START       EQU 	$FF00	
STACK       EQU		$01FF
	
	    ORG START

RESET  	    LDAB 	#$7f	    * Mask for DSP data direction register.
	    	STAB 	DSP			* Set it up.
	    	LDAB 	#$a7	    * KBD and DSP control register mask.
	    	STAB 	KBDCR	    * Enable interrupts, set CA1, CB1, for
	    	STAB 	DSPCR	    *  positive edge sense/output mode.
	    	LDS  	#STACK	    

* Note that B contains $a7 here, which means that the incb below will
* set the negative flag, causing the bpl to fall through into escape.
* This saves us a "bra escape" instruction here.

* Get a line of input from the keyboard, echoing to display.
* Normally enter at escape or getline.

NOTCR	    CMPA 	#$df		* "_"?  [NB back arrow]
	    	BEQ  	BACKSPACE	* Yes.
	    	CMPA 	#$9b	    * ESC?
	    	BEQ  	ESCAPE	    * Yes.
	    	INX  	    		* Advance text index.
	    	INCB
	    	BPL  	NEXTCHAR	* Auto ESC if > 127.

ESCAPE      LDAA	#$dc	    * '\'.
	    	JSR		ECHO	    * Output it.

GETLINE     LDAA 	#$8d	    * CR.
	    	JSR  	ECHO	    * Output it.
	    	LDX  	#IN+1	    * Initiallize [sic] text index.
	    	LDAB 	#1
BACKSPACE   DEX					* Back up text index.
	    	DECB
	    	BMI  	GETLINE		* Beyond start of line, reinitialize.

NEXTCHAR    LDAA	KBDCR	    * Key ready?
	    	BPL  	NEXTCHAR    * Loop until ready.
	    	LDAA 	KBD	    	* Load character. B7 should be '1'.
	    	STAA 	0,X         * Add to text buffer.
	    	BSR  	ECHO	    * Display character.
	    	CMPA 	#$8d	    * CR?
	    	BNE  	NOTCR	    * No.

* Process an input line.

CR 	    	LDX  	#IN+256-1	* Reset text index to in-1, +256 so that
	    	STX  	INPTR
	    	CLRA	 		    * For XAM mode. 0->B.

SETBLOK     ASLA				* Leaves $56 if setting BLOCK XAM mode.
SETMODE     STAA 	MODE	    * $00 = XAM, $BA = STOR, $56 = BLOK XAM.
BLSKIP 	    INC  	INPTR+1     * Advance text index.
NEXTITEM    LDX  	INPTR
            LDAA 	0,X         * Get character.
	    	CMPA 	#$8d	    * CR?
	    	BEQ  	GETLINE     * Yes, done this line.
	    	CMPA 	#$ae	    * "."?
	    	BEQ  	SETBLOK     * Set BLOCK XAM mode.
	    	BLS  	BLSKIP	    * Skip delimiter.
	    	CMPA 	#$ba	    * ":"?
	    	BEQ  	SETMODE     * Yes, set STOR mode.
	    	CMPA 	#$d2	    * "R"?
	    	BEQ  	RUN	    	* Yes, run user program.
	    	CLR  	L	    	* $00->L.
	    	CLR  	H	    	*  and H.
	    	STX  	YSAV	    * Save Y for comparison.

NEXTHEX     LDX  	INPTR
            LDAA 	0,X         * Get character for hex test.
	    	EORA 	#$b0		* Map digits to $0-9.
	    	CMPA 	#$09		* Digit?
	    	BLS  	DIG			* Yes.
	    	ADDA 	#$89		* Map letter "A"-"F" to $FA-FF.
	    	CMPA 	#$f9		* Hex letter?
	    	BLS  	NOTHEX		* No, character not hex.

DIG 	    ASLA				* Hex digit to MSD of A.
	    	ASLA
	    	ASLA
	    	ASLA

	    	LDAB 	#$04       	* Shift count.
HEXSHIFT    ASLA           		* Hex digit left, MSB to carry.
	    	ROL  	L			* Rotate into LSD.
	    	ROL  	H	   	    * Rotate into MSD's.
	    	DECB 	  		    * Done 4 shifts?
	    	BNE  	HEXSHIFT    * No, loop.

	    	INC  	INPTR+1     * Advance text index.
	    	BRA  	NEXTHEX     * Always taken. Check next character for hex.

NOTHEX 	    CPX  	YSAV	    * Check if L, H empty (no hex digits).
	    	BEQ  	ESCAPE	    * Yes, generate ESC sequence.
	    	TST  	MODE	    * Test MODE byte.
	    	BPL  	NOTSTOR		* B6=0 for STOR, 1 for XAM and BLOCK XAM

* STOR mode
	    	LDX  	ST
	    	LDAA 	L	    	* LSD's of hex data.
            STAA 	0,X	    	* Store at current 'store index'.
	    	INX
	    	STX  	ST
TONEXTITEM  BRA 	NEXTITEM    * Get next command item.

PRBYTE	    PSHA	    		* Save A for LSD.
	   		LSRA
	    	LSRA
	    	LSRA	    		* MSD to LSD position.
	    	LSRA
	    	BSR  	PRHEX	    * Output hex digit.
	    	PULA	    		* Restore A.
PRHEX	    ANDA 	#$0f	    * Mask LSD for hex print.
	    	ORAA 	#$b0	    * Add "0".
	    	CMPA 	#$b9	    * Digit?
	    	BLS  	ECHO	    * Yes, output it.
	    	ADDA 	#$07	    * Add offset for letter.
ECHO	    TST  	DSP	    	* DA bit (B7) cleared yet?
	    	BMI  	ECHO	    * No, wait for display.
	    	STAA 	DSP	    	* Output character. Sets DA.
	    	RTS		    		* Return.

RUN         LDX  	XAM
	    	JMP  	0,X	   		* Run at current XAM index.

NOTSTOR     BNE  	XAMNEXT     * mode = $00 for XAM, $56 for BLOCK XAM.

	    	LDX  	H	 	    * Copy hex data to
	    	STX  	ST	    	* 'store index'.
	   	 	STX  	XAM	 	    * And to 'XAM index'.
	    	CLRA	  			* set Z flag to force following branch.

NXTPRNT     BNE  	PRDATA	    * NE means no address to print.
	    	LDAA 	#$8d	    * CR.
	    	BSR  	ECHO	    * Output it.
	    	LDAA 	XAM	 	    * 'EXAMine index' high-order byte.
	   		BSR  	PRBYTE	    * Output it in hex format.
	    	LDAA 	XAM+1	    * Low-order 'EXAMine index' byte.
	    	BSR  	PRBYTE	    * Output it in hex format.
	    	LDAA 	#$ba	    * ":".
	    	BSR  	ECHO	    * Output it.

PRDATA      LDAA 	#$a0	    * Blank.
	    	BSR  	ECHO	    * Output it.

	    	LDX  	XAM
	    	LDAA	0,X         * Get data byte at 'eXAMine index'.
	    	BSR		PRBYTE	    * Output it in hex format.

XAMNEXT     CLR  	MODE	    * 0->MODE (XAM mode).
	    	LDX  	XAM	    	* Compare 'eXAMine index' to hex data.
	    	CPX  	H
	    	BEQ  	TONEXTITEM	* Not less, so more data to output.
	    	INX
	    	STX  	XAM
            LDAA 	XAM+1	    * Check low-order 'examine index' byte
	    	ANDA 	#$07	    *  For MOD 8 = 0
	    	BRA 	NXTPRNT    	* always taken

            ORG 	$fff8       * vector table
	    	FDB 	$0000	    * IRQ
	    	FDB 	$0000	    * SWI
	    	FDB 	$f000	    * NMI
	    	FDB		$ff00	    * RESET
