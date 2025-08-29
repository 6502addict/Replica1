				SECTION	"CODE"
			
*------------------------------------------------------
* GETBYTE get a byte from input buffer
*------------------------------------------------------
GETBYTE			PSHS	X
				JSR	UPCASE
				LDD	,X
				JSR	HEXA2BYTE
				BCS	GETBYTEERR
				ANDCC	#$FE
GETBYTEERR		PULS	X,PC
	
*------------------------------------------------------
* GETWORD  get a word from input buffer
*------------------------------------------------------
GETWORD			PSHS	X			;- save X register
				JSR		UPCASE
				LDD		,X++		;- get 2 hexa bytes in D
				JSR		HEXA2BYTE	;- convert to binary
				BCS		GETWORDERR	;- if convert failed, exit
				PSHS	A			;- save A (fist byte in binary)
				LDD		,X++		;- get next 2 hexa bytes
				JSR		HEXA2BYTE	;- convert to binary
				BCS		GETWORDERR  ;- if convert failed, exit
				TFR		A,B			;- move A to B 
				PULS	A			;- retrive A
				ANDCC	#$FE		;- clear carry
GETWORDERR		PULS	X,PC		;- restore X and return


*------------------------------------------------------
* GETSRC get src and dst address in input buffer
*------------------------------------------------------
GETSRCDST		JSR	FETCHWORD		;- fetch one word
				BCS	GETSRCDSTERR	;- if carry set, error exit
				STD	SRC				;- store soure adress
				JSR	FETCHWORD		;- fetch one word
				BCS	GETSRCDSTERR	;- if carry set, error exit
				STD	DST				;- store destination address
GETSRCDSTERR	RTS					;- return

*------------------------------------------------------
* GETBYTELIST create a list of byte from the input buffer
*------------------------------------------------------
GETBYTELIST		JSR		LEXFETCH	;- fetch a token
				BCS		GETBLISTEND	;- carry set, no more token
				CMPA	#LEX_STRING	;- is the token a symbol
				BEQ		GETBLISTSTR	;- yes get and insert the byte
				JSR		GETBYTE		;- get an hexa byte from input buffer
				BCS		GETBLISTERR	;- if carry set, conversion error, exit
				STA		,Y+			;- store the byte in the memory pointed by Y
				BRA		GETBYTELIST	;- continue with next token
GETBLISTSTR		BSR		STRLEN		;- compute source string length
				TFR		A,B			;- move length to B
GETBLIST1		LDA		,X+			;- load A from 1st string
				STA		,Y+			;- save A to 2nd string
				DECB				;- decrement counter
				BNE 	GETBLIST1	;- if end of string not met continue
				BRA		GETBYTELIST	;- continue with next token
GETBLISTEND		ANDCC	#$FE		;- normal end, clear carry
				STY		SCRATCHEND	;- store scratchpad last address
GETBLISTERR		RTS					;- exit

                END
