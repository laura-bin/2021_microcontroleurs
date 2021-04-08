

#include <xc.inc>
    
; Functions
; =========
global DELAY_4ms	; 4 milli seconds delay
global DELAY_10ms	; 10 milli seconds delay


; Variables
; =========
PSECT udata_bank0

count1:	    DS 1
count2:	    DS 1


PSECT code

DELAY_4ms:
    movlw   0x08
    movwf   count1
DELAY41:
    movlw   0xA4
    movwf   count2
DELAY42:
    decfsz  count2, f
    goto    DELAY42
    decfsz  count1, f
    goto    DELAY41
    return


DELAY_10ms:
    movlw   0x15
    movwf   count1
DELAY101:
    movlw   0x9E
    movwf   count2
DELAY102:
    decfsz  count2, f
    goto    DELAY102
    decfsz  count1, f
    goto    DELAY101
    return
