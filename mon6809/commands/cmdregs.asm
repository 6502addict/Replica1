				SECTION	"CODE"
				
*------------------------------------------------------
* CMDREGS display registers
*------------------------------------------------------
CMDREGS         NOP

CMDREGSEND      JSR     PRTCRLF         ;- print cr lf
CMDREGSEXIT		ANDCC	#$FE            ;- clear carry
CMDREGSERR		RTS                     ;- return
			
				END
