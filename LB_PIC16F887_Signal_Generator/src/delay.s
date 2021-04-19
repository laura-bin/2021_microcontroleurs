; Delay function for 20MHz oscillator
;
; Laura Binacchi - SLP 2020/2021
; ==============================================================================

#include <xc.inc>
    
; Functions
; =========
global DELAY_10ms	; 10 milli seconds delay
global DELAY_40us	; 40 micro seconds delay

; Variables
; =========
PSECT udata_bank0
count1:	    DS 1
count2:	    DS 1


PSECT code

DELAY_10ms:
    movlw   0xA4
    movwf   count1
DELAY101:
    movlw   0x64
    movwf   count2
DELAY102:
    decfsz  count2, f
    goto    DELAY102
    decfsz  count1, f
    goto    DELAY101
    return


DELAY_40us:
    movlw   0x40
    movwf   count1
DELAY401:
    decfsz  count1, f
    goto    DELAY401
    return
