				SECTION	"CODE"

*------------------------------------------------------
* parse the command inside the input buffer
*------------------------------------------------------
CMDPARSE		LDY		#CMDTBL		;- initialize Y to start of command table
				JSR		UPCASE		;- turn token into upper case
CMDPARSENXT		TST		,Y			;- test if end of command table
				BEQ		CMDPARSEERR	;- error command not found
				LDX		LEXCUR		;- get addres of current token
				JSR		STRCMP		;- compare the token with the current command
				BEQ		CMDGO		;- if strings are equal execute command
				JSR		CMDNEXT		;- move Y to next entry
				BRA		CMDPARSENXT	;- try next command
	
CMDGO			JSR		CMDNEXT		;- compute next position
				LDY		-2,Y		;- return to the address of the previous command
				JMP		,Y			;- call the routine

CMDPARSEERR		ORCC	#$01 		;- set carry, error command not found
				RTS					;- return to caller
	
CMDNEXT			TFR		Y,X 		;- transfer Y to X 
				JSR		STRLEN		;- compute the string length
				LEAY	A,Y			;- compute Y at end of string
				LEAY	+3,Y		;- skip the cmd address
				RTS					;- return with Y containing the addres to the next entry

                END
