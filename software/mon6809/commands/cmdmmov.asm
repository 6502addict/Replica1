                SECTION	"CODE"

*------------------------------------------------------
* CMDMMOV move memory block
*------------------------------------------------------
CMDMMOV         CLR     MMOVDIR         ;- clear movdir flag (default X -> Y)
                JSR     FETCHWORD       ;- fetch a word from input
                BCS     CMDMMOVERR      ;- error exit with carry set
                STD     SRC             ;- store to source address
                JSR     FETCHWORD       ;- fetch a word from input
                BCS     CMDMMOVERR      ;- error exit with carry set
                SUBD    SRC             ;- substract source address
                STD     LEN             ;- store length
                JSR     FETCHWORD       ;- fetch a word from input
                BCS     CMDMMOVERR      ;- error exit with carry set
                STD     DST             ;- store to destination address
                LDX     SRC             ;- load source address in X
                LDY     DST             ;- load destination address in Y
                CMPX    DST             ;- compare with dst        
                BEQ     CMDMMOVEND      ;- src = dst no need to move
                BHI     CMDMOVE         ;- perform forward move
                DEC     MMOVDIR         ;- do transfer backward
                TFR     X,D             ;- move SRC to D
                ADDD    LEN             ;- add length
                TFR     D,X             ;- return SRC to X    
                TFR     Y,D             ;- move DST to D
                ADDD    LEN             ;- add length
                TFR     D,Y             ;- return DST to Y
CMDMOVE         LDD     LEN             ;- load length in D
                JSR     MMOVXFER        ;- do real transfer
CMDMMOVEND      ANDCC   #$FE            ;- clear carry
CMDMMOVERR      RTS                     ;- return

*------------------------------------------------------
* MMOVXFER move memory block X = source, Y = destination
*          D = len  MMOVDIR bit 7 = direction
*------------------------------------------------------
MMOVXFER        PSHS    D,X,Y           ;- LEN, SRC, DST
MMOVNEXT        PSHS    A               ;- save A registers
                LDA     ,X              ;- load byte from SRC
                STA     ,Y              ;- save byte to DST
                TST     MMOVDIR         ;- check direction
                BNE     MMOVDEC         ;- backward, decrement
                LEAX    1,X             ;- increment X
                LEAY    1,Y             ;- increment Y
                BRA     MMOVLEN         ;- compute new length
MMOVDEC         LEAX    -1,X            ;- decrement X
                LEAY    -1,Y            ;- decrement Y
MMOVLEN         PULS    A               ;- restore A
                SUBD    #$0001          ;- decrement length
                BPL     MMOVNEXT        ;- if still positive contine
                ANDCC   #$FE            ;- clear carry
                PULS    D,X,Y,PC        ;- restore registers and return
