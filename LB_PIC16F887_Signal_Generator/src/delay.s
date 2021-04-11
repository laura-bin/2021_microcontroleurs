

#include <xc.inc>
    
; Functions
; =========
global DELAY_4100us	; 4.1 milli seconds delay


; Variables
; =========
PSECT udata_bank0

count1:	    DS 1
count2:	    DS 1


PSECT code

DELAY_4100us:
    movlw   0x43
    movwf   count1
DELAY41:
    movlw   0x65
    movwf   count2
DELAY42:
    decfsz  count2, f
    goto    DELAY42
    decfsz  count1, f
    goto    DELAY41
    return