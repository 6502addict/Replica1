				SECTION	"CODE"

*------------------------------------------------------
* CMDCHKSUMON activate checksum
*------------------------------------------------------
CMDCHKSUMON     CLR      CHKSUM		;- clear msb checksum
	        	CLR		CHKSUM+1	;- clear lsb checksum
	        	CLR		DOCHKSUM	;- reset chsksum flag
	        	DEC		DOCHKSUM	;- set checksum on	
                RTS                 ;- return 

*------------------------------------------------------
* CMDCHKSUMOFF deactivate checksum
*------------------------------------------------------
CMDCHKSUMOFF    CLR     DOCHKSUM	;- reset checksum flag
                RTS                 ;- return

*------------------------------------------------------
* COMPUTECHKSUM compute checksum
*------------------------------------------------------
COMPUTECHKSUM   TST		DOCHKSUM	;- test if checksum computation on
	        	BEQ		COMPUTECHKSUM1	;- no skip computation
	        	PSHS	D			;- save D register
	        	TFR		A,B			;- transfer char to lsb of D
	        	CLRA				;- clear msb of D
	        	ADDD	CHKSUM		;- add current checksum
	        	STD		CHKSUM		;- store new checksum
	        	PULS	D			;- restore D
COMPUTECHKSUM1  RTS                 ;- return

	        	END
