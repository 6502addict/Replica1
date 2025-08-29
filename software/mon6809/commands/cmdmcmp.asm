                SECTION	"CODE"

*------------------------------------------------------
* CMDMCMP compare memory block
*------------------------------------------------------
CMDMCMP         JSR     FETCHWORD       ;- fetch a word from input
                BCS     CMDMCMPERR      ;- error exit with carry set
                STD     SRC             ;- store to source address
                JSR     FETCHWORD       ;- fetch a word from input
                BCS     CMDMCMPERR      ;- error exit with carry set
                SUBD    SRC             ;- substract source address
                STD     LEN             ;- store length
                JSR     FETCHWORD       ;- fetch a word from input
                BCS     CMDMCMPERR      ;- error exit with carry set
                STD     DST             ;- store to destination address
                LDX     SRC             ;- load source address in X
                LDY     DST             ;- load destination address in Y
                CMPX    DST             ;- compare with dst        
                BEQ     CMDMCMPEND      ;- src = dst no need to compare
                LDA     #$08
                STA     COUNT
                LDD     LEN             ;- load length in D
CMDMCMPNEXT     PSHS    D
                LDA     ,X+
                CMPA    ,Y+
                BEQ     CMDMCMPDEC
                TFR     X,D
                JSR     PRTWORD
                JSR     PRTSPACE
                DEC     COUNT
                BPL     CMDMCMPDEC
                JSR     PRTCRLF
                LDA     #$08
                STA     COUNT
CMDMCMPDEC      PULS    D
                SUBD    #$0001
                BPL     CMDMCMPNEXT
                LDA     COUNT
                CMPA    #$08
                BEQ     CMDMCMPEND
                JSR     PRTCRLF
CMDMCMPEND      ANDCC   #$FE            ;- clear carry
CMDMCMPERR      RTS                     ;- return

