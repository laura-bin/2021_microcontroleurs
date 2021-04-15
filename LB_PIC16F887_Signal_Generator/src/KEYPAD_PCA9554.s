

#include <xc.inc>

#include "config.inc"
#include "delay.inc"
#include "I2C.inc"
    
; Functions
; =========
global	INIT_PCA9554	    ; Initializes the PCA9554 I/0 expander initialization
global	READ_PCA9554	    ; Reads the PCA9554 in the KEYPAD_DATA


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

READ_PCA9554:
    call    START_I2C
    movlw   PCA9554_KEYPAD_ADD_W
    call    SEND_I2C
    movlw   0x00
    call    SEND_I2C
    call    RESTART_I2C
    movlw   PCA9554_KEYPAD_ADD_R
    call    SEND_I2C
    call    WAIT_ACK_I2C
    call    RECEIVE_I2C
    movwf   KEYPAD_DATA
    call    STOP_I2C

    return
