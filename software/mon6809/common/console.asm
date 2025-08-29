				SECTION	"CODE"

*------------------------------------------------------
* ROWCOL: maintain row/colomn on output 
*------------------------------------------------------
ROWCOL			PSHS	A
				ANDA	#$7F
				CMPA	ENDLINE
				BNE		1F
				INC		ROW
				LDA		#$01
				STA		CTAB
				LDA		#$FF
				STA		COL
1				INC		COL
				PULS	A,PC

*------------------------------------------------------
* SETTABS: set hard tabs
*------------------------------------------------------
SETTABS			PSHS	A			;- save A
				STX		TABSPTR		;- store X to tabs pointer
				LDA		,X			;- load first byte
				STA		NTABS		;- store tab count
				INC		NTABS		;- increment tab count
				LDA		#$01		;- load 1
				STA		CTAB		;- into current tab
				PULS	A,PC		;- restore A and return

*------------------------------------------------------
* CLEARTABS: clear hard tabs
*------------------------------------------------------
CLEARTABS		CLR		TABSPTR		;- clear msb tabs pointer
				CLR		TABSPTR+1	;- clear lsb tabs pointer
				CLR		NTABS		;- clear tabs count
				RTS

*------------------------------------------------------
* PROCESSTABS: process tabs
*------------------------------------------------------
PROCESSTABS		PSHS	D,X
				LDX		TABSPTR
				BEQ		1F
				LDB		CTAB
				CMPB	NTABS
				BCS		1F
2				JSR		PRTSPACE
				LDA		COL
				CMPA	B,X
				BNE		2B
				INC		CTAB
1				PULS	D,X,PC						

*------------------------------------------------------
* CONINIT: Initialise the console
*------------------------------------------------------
CONINIT			PSHS	A
				LDA		#$7F
				STA		DSP
				LDA		#%00110110		;- was a7  
				STA		KBDCR
				LDA		#%00100110		;- was a7  
				STA		DSPCR
				JSR		CLEARTABS
				CLR		COL
				CLR		ROW
				PULS	A,PC

*------------------------------------------------------
* CONOUT: output one byte to the console
*------------------------------------------------------
CONOUT			CMPA	#ASCII_HT
				BNE		2F
				BRA		PROCESSTABS
2				BSR		ROWCOL	
				PSHS	D				;- save D register
1				LDB		DSP				;- loard CRB
				BITB	#%10000000		;- bit (B7) cleared yet? 
				BMI		1B				;- No, wait for display.
				ORA		#$80			;- make sure bit 7 is set
				STA		DSP				;- Output character. Sets DA.
				PULS	D,PC			;- restore D register and return

*------------------------------------------------------
* CONIN: input one byte from the console
*------------------------------------------------------
CONIN			LDA		KBDCR			;- Key ready?
				BITA	#%10000000		;- Loop until ready.
				BPL		CONIN			;- no, continue
				LDA		KBD				;- Load character 
				ANDA	#$7F			;- remove bit 7
				RTS

				END
