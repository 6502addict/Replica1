		SECTION	"CODE"
				
*------------------------------------------------------
* CMDFILL fill memory with a pattern
*------------------------------------------------------
CMDFILL	  	JSR	GETSRCDST		;- get source and destination address
		BCS	CMDFILLERR		;- if carry set, error, exit
		LDY     #SCRATCHAREA    ;- store temporary data in scratcharea
                JSR	GETBYTELIST	;- get byte list
		BCS	CMDFILLERR		;- if carry set, error, exit
                LDX     SRC             ;- load src
CMDFILL2        LDY     #SCRATCHAREA    ;- set Y to start of scratch area
CMDFILL1        LDA     ,Y+             ;- load A with data from pattern
                STA     ,X+             ;- store A 
                CMPX    DST             ;- checjing with DST address
                BHI     CMDFILLEXIT     ;- exit if higher or same
                CMPY    SCRATCHEND      ;- compare with end address of scratch area
                BLO     CMDFILL1        ;- if lower conntinue  (inner loop)
                BRA     CMDFILL2        ;- continue (outer loop)
CMDFILLEXIT	ANDCC	#$FE            ;- clear carry
CMDFILLERR	RTS                     ;- return

		END
