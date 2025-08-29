                SECTION	"CODE"

*------------------------------------------------------
* CMDLOAD load hexa file
*------------------------------------------------------
CMDLOAD         CLR     LOADPTRDEFINED  ;- set load address to undefined
                JSR     FETCHWORD       ;- fetch a word from input
                BCS     CMDLOADNXTREC   ;- not found, load addres by default
                STD     LOADALTADDR     ;- alternate load address defined
                DEC     LOADPTRDEFINED  ;- set alternate load address defined flag
CMDLOADNXTREC   JSR     GETLINE
                CLR     XPOS
                JSR     LOADINBYTE
                BCS     LOADERROR     
                ANDCC   #$FE            ;- clear carry
                RTS                     ;- return

LOADERROR       RTS

RECORDBADTYPE   LDA     #EBADRECORDTYPE
                BRA     LOADSETERR
RECORDNOTYPE    LDA     #ENORECORDTYPE            
                BRA     LOADSETERR
RECORDNOADDRESS LDA     #EBADRECORDADDRESS
                BRA     LOADSETERR
RECORDNOLENGTH  LDA     #ENORECORDLENGTH
                BRA     LOADSETERR
RECORDNOCHECKSM LDA     #ENORECORDCHECKSUM
                BRA     LOADSETERR
RECORDBADCHKSM  LDA     #EBADRECORDCHECKSUM
                BRA     LOADSETERR
RECORDBADDATA   LDA     #EBADRECORDDATA
LOADSETERR      ORCC    #$01
                RTS

HEXRECORD       RTS
S16RECORD       RTS
MOSRECORD       RTS                        

LOADINBYTE      PSHS    B,X
                LDB     XPOS
                LDX     #INPUT
                LDA     B,X
                BEQ     LOADINBYTEERR
                INCB    
                STB     XPOS
                CMPB    #$80
                BEQ     LOADINBYTEERR
                ANDCC   #$FE
                PULS    B,X,PC
LOADINBYTEERR   ORCC    #$01
                PULS    B,X,PC


LOADINHEXABYTE  PSHS           

                SECTION "DATA"
CMDLOADHANDLER  FCS     /:/
                FDB     HEXRECORD
                FCS     /S/
                FDB     S16RECORD
                FCS     /;/
                FDB     MOSRECORD
                FCB     $00

MODHEX          FCC     /HEX/
MODS16          FCC     /S19/
MODMOS          FCC     /MOS/

                END




