				SECTION	"CODE"

PRTX			PSHS	X,D
				LDA 	#'X'
				JSR		CONOUT
				LDA		#'='
				JSR		CONOUT
				TFR		X,D
				JSR		PRTWORD
	    		PULS	X,D,PC
		
PRTCC			PSHS	D,X,Y
    			PSHS	CC
	    		LDA		#'C'
				JSR     CONOUT
				LDA		#'C'
    			JSR		CONOUT
	    		LDA		#'='
				JSR		CONOUT
				LDX		#CCFLAGS
				LDY		#$08
				PULS	B
PRTCC1			LDA		,X+
				ROLB
				BCS		PRTCC2
				LDA		#'-'
PRTCC2			JSR		CONOUT
				LEAY	-1,Y
				BNE		PRTCC1
				PULS	D,X,Y,PC

PRTA			PSHS	D
				EXG		A,B
				LDA		#'A'
				JSR		CONOUT
				LDA		#'='
				JSR		CONOUT
	    		EXG		A,B
    			JSR		PRTBYTE
				PULS	D,PC
	
PRTB			PSHS	D
    			TFR		B,A
    			JSR		PRTBYTE
				PULS	D,PC

PRTD			PSHS	D
				JSR		PRTWORD
				PULS	D,PC

PRTSPACE		PSHS	D
				LDA	#' '
				JSR	CONOUT
				PULS	D,PC
	
*-----------------------------------------------------------
*- print the byte in A in hexadecimal
*----------------------------------------------------------- 
PRTWORD			PSHS	D
				JSR		PRTBYTE
				TFR		B,A
				JSR		PRTBYTE
				PULS	D,PC

*-----------------------------------------------------------
*- print the byte in A in hexadecimal
*----------------------------------------------------------- 
PRTBYTE			PSHS	D
				LBSR	BYTE2HEXA
				LBSR	CONOUT
				TFR		B,A	 
				LBSR	CONOUT
				PULS	D,PC

*-----------------------------------------------------------
*- print the prompt
*----------------------------------------------------------- 
PRTPROMPT		PSHS	D
    			TFR		A,B
				LDA		#PROMPT
				LBSR	CONOUT
				TFR		B,A	 
				LBSR	CONOUT
				LBSR	PRTSPACE
				PULS	D,PC
	
*-----------------------------------------------------------
*- print the input buffer
*----------------------------------------------------------- 
PRTINPUT		PSHS	X,D
				LDX		#INPUT
				TFR		X,D
				JSR		PRTWORD
				LDA		#':'
				JSR 	CONOUT
				JSR		PRTSPACE
				CLRB
PRTINPUT1		LDA		,X+
				INCB
				CMPB	LINELENGTH
				BEQ		PRTINPUTEND
				JSR		PRTBYTE
				JSR 	PRTSPACE
    			BRA		PRTINPUT1
PRTINPUTEND		JSR 	PRTCRLF
				PULS	X,D
				
*------------------------------------------------------
* print crlf
*------------------------------------------------------
PRTCRLF			PSHS	D
				LDA		#$0D
				JSR		CONOUT
				IF	! REPLICA1
				LDA		#$0A
				JSR		CONOUT
				ENDIF
				PULS	D,PC

*------------------------------------------------------
* print a null terminated string
* pointed by X register
*------------------------------------------------------
PRTSTR			PSHS	X,A			;- save X and A
PRTSTR1			LDA		,X+			;- load char from string and increment
				BEQ		PRTSTREND	;- if 00 exit
				JSR		CONOUT		;- print char
				BRA		PRTSTR1		;- coninue with next char
PRTSTREND		PULS	A,X,PC		;- restore registers and return

*------------------------------------------------------
* PRTSRC print source address
*------------------------------------------------------
PRTSRC			PSHS	D
				LDD		SRC
				JSR		PRTWORD
				PULS	D,PC

*------------------------------------------------------
* PRTF print with format
*------------------------------------------------------
PRTF			PSHS	D,X,Y
				LDY		#PRTFTAGS
PRTFLOOP		LDA		,X+
				BEQ		PRTFEND
				CMPA	#'%'
				BNE		PRTFOUT
				LDA		,X+
				CLRB
PRTF3			TST		B,Y
				BEQ		PRTFERR
				CMPA	B,Y
				BEQ		PRTF4
				INCB
				BRA		PRTF3
PRTF4			LDX		#PRTFHANDLER
				ASLB
				LDY		B,Y						
				JSR		,Y
				BRA		PRTFLOOP

PRTFOUT			LBSR	CONOUT
				BRA		PRTFLOOP

PRTFEND			ANDCC	#$FE
				PULS	D,X,PC		
PRTFERR			ORCC	#$01
				PULS	D,X,PC

PRTFPERCENT		LDA		#'%'
				LBRA	CONOUT

PRTFMODULE		LDD		MODULE
				LBRA	PRTSTR

PRTFTAGS		FCS		2,/%mM/
PRTFHANDLER		FDB		PRTFPERCENT						
				FDB		PRTFMODULE						
				FDB		PRTFMODULE						

				SECTION	"DATA"
CCFLAGS			FCS		/EFHINZVC/			

                END
