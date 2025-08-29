		SECTION	"CODE"

*-----------------------------------------------------------
*- convert the byte in A into 2 printable hexa character
*----------------------------------------------------------- 
BYTE2HEXA		PSHS	A
				BSR		NIBBLE2HEXA
				TFR		A,B
				PULS	A
				LSRA
				LSRA
				LSRA
				LSRA
*- falltrough
*-----------------------------------------------------------
*- convert the nibble in A into a printable hexa character
*----------------------------------------------------------- 
NIBBLE2HEXA		ANDA	#$0F
				ADDA	#$90
				DAA
				ADCA    #$40
				DAA
				RTS

*-----------------------------------------------------------
*- convert the hexa digit in A into a nibble
*----------------------------------------------------------- 
HEXA2NIBBLE		CMPA	#'0'
				BLT		HEXA2NIBBLE3
				CMPA	#'9'
				BGT		HEXA2NIBBLE2
				ANDA	#$0F
				ANDCC	#$FE
				RTS
HEXA2NIBBLE2	CMPA	#'A'
				BLT	HEXA2NIBBLE3
				CMPA	#'F'
				BGT	HEXA2NIBBLE3
				SUBA	#'A'
				ADDA	#$0A
				ANDCC	#$FE	
				RTS
HEXA2NIBBLE3	ORCC	#$01
				RTS

*-----------------------------------------------------------
*- convert 2 hexa digit in D into a byte
*----------------------------------------------------------- 
HEXA2BYTE		JSR		HEXA2NIBBLE
				PSHS	A
				TFR		B,A
				JSR		HEXA2NIBBLE
				ASLA
				ASLA
				ASLA
				ASLA
				TFR		A,B
				PULS	A
				ROLB
				ROLA
				ROLB
				ROLA
				ROLB
				ROLA
				ROLB
				ROLA
				RTS

*DIV			STD		DIVISOR
*				TFR		X,D
*				LDX		#$0000
*DIVLOOP		SUBD	DIVISOR
*				BCS		DIVEND
*				LEAX	1,X
*				BRA		DIVLOOP
*DIVEND  		ADDD	DIVISOR			
*				RTS

* stack position 0  counter
* stack position 1  CC
* stack position 2  A		
* stack position 3  B
* stack position 4  X
* stack position 6  Y
* stack position 8  PC

DIVISOR		SET	6

DIVXY		PSHS	Y,X,D,CC
			LDB		#10
			PSHS	B
			CLRB
			CLRA
DIVLP		ASL		5,S
			ROL		4,S
			ROLB
			ROLA
			CMPD	DIVISOR,S
			BLO		DIVLT
			SUBD	DIVISOR,S
			INC		5,S
DIVLT		DEC		,S
			BNE		DIVLP
			STD		DIVISOR,S
			LEAS	1,S
			PULS	PC,X,Y,D,CC												

			END

