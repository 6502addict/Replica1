               SECTION	"CODE"

*------------------------------------------------------
* get a line from terminal
*------------------------------------------------------
GETLINE					LDA		#'.'
						JSR		CONOUT
						CLR		LINELENGTH
GETLINE1				JSR		CONIN
						JSR		CONOUT
						STA		,X+
						INC		LINELENGTH
						DECB
						CMPB	#$01
						BLE		GETLINEEND
						IF		REPLICA1
						CMPA	#$0D
						ELSE
						CMPA	#$0A
						ENDIF
						BNE		GETLINE1
GETLINEEND				LDA		#$00
						STA		,X
						RTS
			
