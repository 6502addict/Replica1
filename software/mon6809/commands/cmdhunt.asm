		SECTION	"CODE"
				
*------------------------------------------------------
* CMDHUNT search a pattern in memory
*------------------------------------------------------
CMDHUNT	  	JSR	GETSRCDST	;- get source and destination address
		BCS	CMDHUNTERR	;- if carry set, error, exit
		LDY     #SCRATCHAREA    ;- store temporary data in scratcharea
                JSR	GETBYTELIST	;- get byte list
		BCS	CMDHUNTERR	;- if carry set, error, exit
                LDX     SRC             ;- load src
                LEAX    -1,X            ;- decrement to compensate next increment
                STX     SRC             ;- store src
CMDHUNT3        LDB     #$08            ;- maximum count of addresses printed
CMDHUNT2        LDX     SRC             ;- load src
                LEAX    1,X             ;- increment current position
                STX     SRC             ;- store src
                LDY     #SCRATCHAREA    ;- set Y to start of scratch area
CMDHUNT1        CMPX    DST             ;- compare with last address
                BHS     CMDHUNTEND      ;- exit if higher or same
                LDA     ,X+             ;- load A with data from SRC
                CMPA    ,Y+             ;- compare A with pattern in scratcharea
                BNE     CMDHUNT2        ;- mismatch increment src and restart
                CMPX    DST             ;- checjing with DST address
                BHS     CMDHUNTEND      ;- if higher or same, exit
                CMPY    SCRATCHEND      ;- check if end of pattern
                BLO     CMDHUNT2        ;- no continue with next byte
                JSR     PRTSRC          ;- display matched address 
                JSR     PRTSPACE        ;- display a space
                DECB                    ;- decrement address count
                BNE     CMDHUNT2        ;- if not yet 0 continue
                JSR     PRTCRLF         ;- print cr lf
                BRA     CMDHUNT3        ;- reinitialise counter and continue
CMDHUNTEND      CMPB    #$08            
                BEQ     CMDHUNTEXIT
                JSR     PRTCRLF
CMDHUNTEXIT	ANDCC	#$FE
CMDHUNTERR	RTS

		END
