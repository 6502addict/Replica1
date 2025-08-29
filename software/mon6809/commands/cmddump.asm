				SECTION	"CODE"

*------------------------------------------------------
* CMDDUMP dump memory
*------------------------------------------------------
CMDDUMP			JSR		GETSRCDST
				BCS		2F
				LDY		SRC
1				LDA		#':'
				JSR		PRTPROMPT
				TFR		Y,D
				JSR		PRTWORD
				JSR		PRTSPACE
				PSHS	Y
				BSR		CMDDUMPHEX
				PULS	Y
				BSR		CMDDUMPASC
				JSR		PRTCRLF	
				CMPY	DST
				BLS		1B
				ANDCC	#$FE
				RTS
2				ORCC	#$01
				RTS

CMDDUMPHEX		LDB		#$08
1				LDA		,Y+
				JSR		PRTBYTE
				JSR		PRTSPACE
				DECB
				BNE		1B
				RTS

CMDDUMPASC		LDB		#$08
1				LDA		,Y+
				CMPA	#' '
				BGE		2F
				LDA		#'.'
2				JSR 	CONOUT
				DECB
				BNE		1B
				RTS

				END
