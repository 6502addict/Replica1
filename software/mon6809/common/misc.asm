				SECTION	"CODE"

*------------------------------------------------------
* UPCASE convert a lexem to upper case
*------------------------------------------------------
UPCASE  		PSHS    D,X             ;- push registers
                CLRB                    ;- clear byte count (B)
UPCASESTART     LDA     ,X+             ;- load one byte and increment
                BEQ     UPCASEEND       ;- if eq 00 end of input
                CMPA    #$22            ;- if " 
                BEQ     UPCASEEND       ;- exit strings are not converted  
                CMPA    #'a'            ;- if lower than 'a' 
                BLO     UPCASENEXT      ;- no upcase			
                CMPA    #'{'            ;- if above or equal '{'
                BHS     UPCASENEXT      ;- no upcase
                SUBA    #$20            ;- subtract $20 from to convert
                STA     -1,X            ;- write back A where it was
UPCASENEXT      INCB                    ;- increment the byte count
                CMPB    LINELENGTH      ;- reached the end of line ?            
                BLS     UPCASESTART     ;- no continue
UPCASEEND       PULS    D,X,PC          ;- restore registers and return
