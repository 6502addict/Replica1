				SECTION	"CODE"

*------------------------------------------------------
* fetch a lexem
*------------------------------------------------------
LEXSTART		LDX		#INPUT 		;- initialize lexer to start
				STX		LEXNXT		;- of input buffer
				RTS

LEXFETCH		PSHS	B,Y
				LDX		LEXNXT 		;- retreive next lexer position
				LDA		#LEX_SYMBOL	;- default to lex_symbol
				STA		LEXTYPE		;-
LEXFETCH1		LDA		,X+			;- fetch one byte from input buffer
				BEQ		LEXFETCH3 	;- check for end of input buffer ($00)
				CMPA	#$22		;- double quote
				BEQ		LEXFETCH4 	;- process double quote
				CMPA	#' '		;- space
				BLE		LEXFETCH1	;- skip space and lower chars
				TFR		X,Y			;- save current position in Y register
				LEAY	-1,Y 		;- adjust current position
LEXFETCH2		LDA		,X+			;- fetch one byte of data
				CMPA	#' '		;- check if end of lexeme ' '
				BGT		LEXFETCH2	;- if char above ' ' continue
LEXFETCH7		CLR		,-X			;- replace space char by null
				LEAX	+1,X		;- adjust X position to end of lexem + 1
				STX		LEXNXT		;- store X in LEXNXT to retreive it later
				TFR		Y,X			;- store current position in LEXCUR
				STX		LEXCUR      ;- X also contains the adress of the lexeme found
				LDA		LEXTYPE		;- return lex type in accumulator
				ANDCC	#$FE	    ;- clear carry (no error, continue)
				PULS	B,Y,PC		;- restore registers and return
	
LEXFETCH3		ORCC	#$01 		;- set carry end of input buffer met
				PULS	B,Y,PC
	
LEXFETCH4		LDA		#LEX_STRING ;- set type to LEX_STRING
				STA		LEXTYPE		;- note: quotes are removed
				TFR		X,Y 		;- save current position in Y
LEXFETCH5		LDA		,X+			;- fetch next byte of data
				BEQ		LEXFETCH7	;- null found stop processing
				CMPA	#$22		;- double quote found ?
				BEQ		LEXFETCH7	;- stop processing
				BRA		LEXFETCH5	;- continue with next char

*------------------------------------------------------
* FETCHWORD fetch src address in input buffer
*------------------------------------------------------
FETCHWORD		JSR		LEXFETCH		;- fetch a token
				BCS		FETCHWORDERR	;- if carry set exit, error
				JSR		GETWORD			;- fetch start address from input buffer
				BCS		FETCHWORDERR	;- conversion error, exit
				ANDCC	#$FE			;- clear carry
FETCHWORDERR	RTS

                END
