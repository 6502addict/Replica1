*------------------------------------------------------
*------------------------------------------------------
* Test program for a Vince Briel REPLICA 1
* modified to run with a 6809
*------------------------------------------------------
*------------------------------------------------------

REPLICA1				EQU		1

SYS_EXIT				EQU		$00
SYS_OUTPUT				EQU		$01
SYS_INPUT				EQU		$02			

LEX_SYMBOL				EQU		0
LEX_STRING				EQU		1
			
ENDLINE					EQU		$0D

KBD						EQU		$D010			;  PIA.A keyboard input
KBDCR					EQU		$D011			;  PIA.A keyboard control register
DSP						EQU		$D012			;  PIA.B display output register
DSPCR					EQU		$D013			;  PIA.B display control register

PROMPT					EQU		'.'
SPACE					EQU		' '			

						INCLUDE	"includes/ascii.inc"

						ORG		$0200
INPUT					RMB		128				;- input buffer
SCRATCHAREA				RMB		128				;- used as volatile storage
			
						ORG		$0300
						SETDP   $03
LEXNXT					RMB		2				;- next lexer location
LEXCUR					RMB		2				;- current lexer location
LEXTYPE					RMB		1				;- token type 0 = SYMBOL, 1 = STRING
LINELENGTH				RMB		1				;- length of the line in the input buffer
SRC						RMB		2				;- source address
DST						RMB		2				;- destination address
CURRENT					RMB		2				;- temporary 16 bits pointer	
SCRATCHEND				RMB		2				;- end address in scratch area
LEN						RMB		2				;- length
MMOVDIR					RMB		1				;- mmov direction
COUNT					RMB		1				;- item count
LOADLEN					RMB		1				;- size of the load chunk
LOADTYPE				RMB 	1				;- type of record
LOADPTRDEFINED			RMB		1				;- $00 = load address undefined, $80 load address defined
LOADADDR				RMB		2				;- address in hexa file
LOADEND					RMB		1				;- completion byte 00=ended, 01=continue
LOADALTADDR				RMB		2				;- alternate load address
XPOS					RMB		1				;- current input buffer position
DOCHKSUM      			RMB 	1				;- checksum flag
CHKSUM			        RMB		2				;- 16 bits chksum

* variable used to manage the tabs in output
CTAB					RMB		1				;- current tab
NTABS					RMB		1				;- maximum count of tabs
TABSPTR					RMB		2				;- hard tabs pointer
* variable containing the output position
ROW						RMB		1				;- current output row
COL						RMB		1				;- current output column
* variable containing the module name
MODULE					RMB		2
* variable used by divide
*DIVIDEND				RMB 	2
*QUOTIENT				RMB		2
*DIVISOR				RMB		2
*REMAINDER				RMB		2
*COUNTER				RMB		1

*------------------------------------------------------
* Initialise the machine
*------------------------------------------------------

						SECTION "CODE"
						ORG		$F800
	
VRESET					LDS		#$7FFF			; SET SYS STACK POINTER
						LDU		#$6FFF			; SET USR STACK POINTER
						LDA		#$03
						TFR		A,DP
						JSR		CONINIT
						JSR		PRTCRLF	
	
		     			LDX		#BANNER 
						LDA		#SYS_OUTPUT
						SWI

*						LDA		#'.'
*						JSR		CONOUT
*						LDX		#256
*						TFR		X,D
*						JSR		PRTWORD
*						JSR		PRTSPACE
*						LDD		#10
*						JSR		PRTWORD
*						JSR		PRTSPACE
*						JSR		DIV
*						JSR		PRTWORD
*						JSR		PRTSPACE
*						TFR		X,D
*						JSR		PRTWORD
*						LDA		#'.'
*						JSR		CONOUT
*						JSR		PRTCRLF

						LDA		#';'
						JSR		CONOUT
						LDX		#256
						TFR		X,D
						JSR		PRTWORD
						JSR		PRTSPACE
						LDY		#0010
						TFR		X,D
						JSR		PRTWORD
						JSR		PRTSPACE
						JSR		DIVXY
						TFR		Y,D
						JSR		PRTWORD
						JSR		PRTSPACE
						TFR		X,D
						JSR		PRTWORD
						LDA		#';'
						JSR		CONOUT
						JSR		PRTCRLF



AGAIN					LDA		#SYS_INPUT	
						LDB		#$80
						LDX		#INPUT
						SWI

						LDA		#SYS_OUTPUT
						LDX		#INPUT
						SWI
	

						JSR		LEXSTART
						JSR		LEXFETCH
						BCS		AGAIN
						JSR		CMDPARSE
						BRA		AGAIN
	
						INCLUDE "common/console.asm"
						INCLUDE "common/getline.asm"
						INCLUDE "common/get.asm"
						INCLUDE "common/strings.asm"
						INCLUDE	"common/converters.asm"
						INCLUDE "common/prt.asm"
						INCLUDE "common/lexer.asm"
						INCLUDE "common/parser.asm"
						INCLUDE "common/misc.asm"
						INCLUDE "common/checksum.asm"
	
	
*------------------------------------------------------
* CMDEXIT  exit from monitor
*------------------------------------------------------
CMDEXIT					CLRA
						SWI

*------------------------------------------------------
* interrupt handlers
*------------------------------------------------------
VSWI					CMPA	#SYS_OUTPUT
						BNE		VSWINXT1
						LBSR	PRTSTR
						RTI
	
VSWINXT1				CMPA	#SYS_INPUT
						BNE		VSWIERR
						LBSR    GETLINE
						RTI

VSWIERR					PSHS	A
						LDX		#UNDEFSYS
						LBSR	PRTSTR
						PULS	A
						LBSR	PRTBYTE
						LBSR	PRTCRLF
VSWIEND					RTI
	
VSWI2					LDX		#SWI2STR
						LBSR	PRTSTR
						RTI

VSWI3					LDX		#SWI3STR
						LBSR	PRTSTR
						RTI

VIRQ					LDX		#IRQSTR
						LBSR	PRTSTR
						RTI

VFIRQ					LDX		#FIRQSTR
						LBSR	PRTSTR
						RTI

VNMI					LDX		#NMISTR
						LBSR	PRTSTR
						RTI



						INCLUDE	"commands/cmddump.asm"
						INCLUDE "commands/cmdedit.asm"
						INCLUDE "commands/cmdexec.asm"
						INCLUDE "commands/cmdhunt.asm"
						INCLUDE "commands/cmdfill.asm"
						INCLUDE "commands/cmdmmov.asm"
						INCLUDE "commands/cmdmcmp.asm"
						INCLUDE "commands/cmdload.asm"
	
						SECTION "DATA"
	
CMDTBL      		    FCC		/C/,$00
            	 	    FDB		CMDMCMP
						FCC		/F/,$00
		                FDB		CMDFILL
						FCC		/G/,$00
						FDB		CMDEXEC
		                FCC		/H/,$00
						FDB		CMDHUNT
						FCC		/L/,$00
						FDB		CMDLOAD
						FCC		/M/,$00
						FDB		CMDDUMP
						FCC		/T/,$00
						FDB		CMDMMOV
						FCC		/:/,$00
						FDB		CMDEDIT
						FCC		/X/,$00
						FDB		CMDEXIT
						FCC		$00
			
NMISTR					FCN		/*NMI*/,$0D,$0A
IRQSTR					FCN		/*IRQ*/,$0D,$0A
FIRQSTR					FCN		/*FIRQ*/,$0D,$0A
SWISTR					FCN		/*SWI*/,$0D,$0A
SWI2STR					FCN		/*SWI2*/,$0D,$0A
SWI3STR					FCN		/*SWI3*/,$0D,$0A
UNDEFSYS				FCN		/Undefined system call $/
BANNER					FCN		/REPLICA 1 6809 MONITOR/,$0D,$0A
MSGEXIT					FCN		/EXITING.../,$0D,$0A
MSGDUMP					FCN																																																																																							/DUMPING.../,$0D,$0A
			
EMOK		            EQU     (MOK			    - ERRORTAB)/2
EBADRECORDTYPE          EQU     (MBADRECORDTYPE     - ERRORTAB)/2
ENORECORDTYPE           EQU     (MNORECORDTYPE      - ERRORTAB)/2
EBADRECORDADDRESS       EQU     (MBADRECORDADDRESS  - ERRORTAB)/2 
ENORECORDLENGTH         EQU     (MNORECORDLENGTH    - ERRORTAB)/2
ENORECORDCHECKSUM       EQU     (MNORECORDCHECKSUM  - ERRORTAB)/2
EBADRECORDCHECKSUM      EQU     (MBADRECORDCHECKSUM - ERRORTAB)/2   
EBADRECORDDATA          EQU     (MBADRECORDDATA     - ERRORTAB)/2    

ERRORTAB                SECTION "ERRORTAB"
MOK				        FDB     MSGOK
MBADRECORDTYPE          FDB     MSGBADRECORDTYPE
MNORECORDTYPE           FDB     MSGNORECORDTYPE
MBADRECORDADDRESS       FDB     MSGBADRECORDADDRESS
MNORECORDLENGTH         FDB     MSGNORECORDLENGTH
MNORECORDCHECKSUM       FDB     MSGNORECORDCHECKSUM
MBADRECORDCHECKSUM      FDB     MSGBADRECORDCHECKSUM
MBADRECORDDATA          FDB     MSGBADRECORDDATA


                        SECTION "ERRORS"
MSGOK					FCC     /OK/
MSGBADRECORDTYPE        FCC     /BAD HEX RECORD TYPE/
MSGNORECORDTYPE         FCC     /NO HEX RECORD TYPE/
MSGBADRECORDADDRESS     FCC     /BAD %M RECORD ADDRESS/
MSGNORECORDLENGTH       FCC     /NO %M RECORD LENGTH/
MSGNORECORDCHECKSUM     FCC     /NO %M RECORD CHECKSUM/
MSGBADRECORDCHECKSUM    FCC     /BAD %M RECORD CHECKSUM/
MSGBADRECORDDATA        FCC     /BAD %M RECORD DATA/



* RESET AND INTERRUPT VECTORS
						SECTION "VECTORS"
						ORG		$FFF0

						FDB		0			;RESERVED VECTOR
						FDB 	VSWI3		;GO TEST SWI3 FOR SUP CALL
						FDB		VSWI2		;SWI2 VECTOR
						FDB		VFIRQ		;FIRQ VECTOR
						FDB		VIRQ		;IRQ VECTOR
						FDB		VSWI		;SOFTWARE INTERRUPT
						FDB		VNMI		;NMI VECTOR DIRECTLY TO NMI RETURN
						FDB		VRESET		;RESTART FOR RESET OR POWERUP

						END 	VRESET
