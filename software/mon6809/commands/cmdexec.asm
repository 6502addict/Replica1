				SECTION	"CODE"

*------------------------------------------------------
* CMDEXEC  execute user code
*------------------------------------------------------
CMDEXEC			JSR		LEXFETCH
				BCS		CMDEXECERR
				JSR		GETWORD		;- fetch start address from input buffer
				BCS		CMDEDITERR
				TFR		D,X
				JSR		,X
CMDEXECERR		RTS

				END
