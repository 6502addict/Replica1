				SECTION	"CODE"
				
*------------------------------------------------------
* CMDEDIT modify memory
*------------------------------------------------------
CMDEDIT			JSR		FETCHWORD		;- get source address
				TFR		D,Y				;- transfer D to Y (needed for GETBYTELIST)
				JSR		GETBYTELIST		;- get byte list
				BCS		CMDEDITERR		;- if carry set, error, exit
CMDEDITEND		ANDCC	#$FE
CMDEDITERR		RTS

				END
