				SECTION	"CODE"

*------------------------------------------------------
* compute the lengh of the string pointed by X
*------------------------------------------------------
STRLEN			PSHS	X 			;- save X register
				LDA		#$FF		;- initialize A counter
STRLEN1			INCA				;- increment byte count (A)
				TST		,X+			;- test if end of string (null)
				BNE		STRLEN1		;- no continue
				PULS	X,PC		;- restore X and return

*------------------------------------------------------
* compare 2 strings pointed by X and Y for equality
*------------------------------------------------------
STRCMP			PSHS	A,X,Y		;- save registers
STRCMPA			LDA		,X+			;- load A with char from 1st string
				CMPA	,Y+			;- compare A with char from 2nd string
				BNE		STRCMPE		;- does not match, exit
				TSTA				;- check if end of string met
				BNE		STRCMPA		;- if null found, both string match
STRCMPE			PULS	X,Y,A,PC 	;- restore registers and return
	
*------------------------------------------------------
* copy null terminated string from X to Y
*------------------------------------------------------
STRCPY			PSHS	D,X,Y		;- save registers
				JSR		STRLEN		;- compute source string length
				TFR		A,B			;- move length to B
STRCPYA			LDA		,X+			;- load A from 1st string
				STA		,Y+			;- save A to 2nd string
				DECB				;- decrement count
				BNE		STRCPYA		;- continue until count = zero
				PULS	X,Y,D,PC 	;- restore registers and return

				END
			
