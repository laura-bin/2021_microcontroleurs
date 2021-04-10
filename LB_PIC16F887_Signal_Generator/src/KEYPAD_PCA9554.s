

#include <xc.inc>

#include "config.inc"
#include "delay.inc"
#include "I2C.inc"
    
; Functions
; =========
global	INIT_PCA9554	    ; Initializes the PCA9554 I/0 expander initialization


; Variables
; =========
global	KEYPAD_DATA	    ; Data read from the keypad


PSECT udata_bank0
KEYPAD_DATA:	DS 1


PSECT code

INIT_PCA9554:

    ; PORT -> input
    call    START_I2C
    movlw   PCA9554_KEYPAD_ADD_W
    call    SEND_I2C
    movlw   PCA9554_CONFIG
    call    SEND_I2C
    movlw   0xFF
    call    SEND_I2C
    call    STOP_I2C

    return
