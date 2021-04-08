

#include <xc.inc>

#include "config.inc"
#include "delay.inc"
#include "I2C.inc"
    
; Functions
; =========
global	INIT_PCA9535	    ; Initializes the PCA9535 I/0 expander initialization
global	INIT_LM044L	    ; Initializes the LM044L via PCA9535 I/O expander
global	SEND_CHAR_LCD	    ; Sends a character to the LM044L via PCA9535 I/O expander
global	SEND_COMMAND_LCD    ; Sends a command to the LM044L via PCA9535 I/O expander


; Variables
; =========
global	LCD_DATA	    ; Data displayed or command


PSECT udata_bank0
LCD_DATA:   DS 1


PSECT code

INIT_PCA9535:
    call    START_I2C
    movlw   PCA_LCD_ADD
    call    SEND_I2C

    ; PORT0-1 -> output
    movlw   PCA_PORT0_CONFIG
    call    SEND_I2C
    movlw   0x00
    call    SEND_I2C
    call    SEND_I2C

    ; PORT0-1 -> 0
    movlw   PCA_PORT0_OUT
    call    SEND_I2C
    movlw   0x00
    call    SEND_I2C
    call    SEND_I2C

    call    STOP_I2C

    return


INIT_LM044L:
    ; Initialize the LCD in 8 bits
    movlw	0x38
    movwf	LCD_DATA
    call	SEND_COMMAND_LCD

    call	DELAY_4ms

    ; Initialize the LCD cursor
    movlw	CURSOR_OFF
    movwf	LCD_DATA
    call	SEND_COMMAND_LCD

    ; Writing from left to right
    movlw	0x06
    movwf	LCD_DATA
    call	SEND_COMMAND_LCD

    ; Clear the LCD
    movlw	LCD_CLEAR
    movwf	LCD_DATA
    call	SEND_COMMAND_LCD

    ; Place the cursor on the first line
    movlw	LCD_LINE1
    movwf	LCD_DATA
    call	SEND_COMMAND_LCD

    return


SEND_CHAR_LCD:
    ; Send the data with RS -> 1 & E -> 0
    call    START_I2C
    movlw   PCA_LCD_ADD
    call    SEND_I2C
    movlw   PCA_PORT0_OUT
    call    SEND_I2C
    movlw   LCD_RS
    call    SEND_I2C
    movf    LCD_DATA, w
    call    SEND_I2C
    call    STOP_I2C

    ; Rising edge on the LCD Enable PIN
    call    START_I2C
    movlw   PCA_LCD_ADD
    call    SEND_I2C
    movlw   PCA_PORT0_OUT
    call    SEND_I2C
    movlw   LCD_RS_E
    call    SEND_I2C
    call    STOP_I2C

    call    START_I2C
    movlw   PCA_LCD_ADD
    call    SEND_I2C
    movlw   PCA_PORT0_OUT
    call    SEND_I2C
    movlw   LCD_RS
    call    SEND_I2C
    call    STOP_I2C

    return


SEND_COMMAND_LCD:
    ; Send the command with RS -> 0 & E -> 0
    call    START_I2C
    movlw   PCA_LCD_ADD
    call    SEND_I2C
    movlw   PCA_PORT0_OUT
    call    SEND_I2C
    movlw   0x00
    call    SEND_I2C
    movf    LCD_DATA, w
    call    SEND_I2C
    call    SEND_I2C
    call    STOP_I2C

    ; Rising edge on the LCD Enable PIN
    call    START_I2C
    movlw   PCA_LCD_ADD
    call    SEND_I2C
    movlw   PCA_PORT0_OUT
    call    SEND_I2C
    movlw   LCD_E
    call    SEND_I2C
    call    STOP_I2C

    call    START_I2C
    movlw   PCA_LCD_ADD
    call    SEND_I2C
    movlw   PCA_PORT0_OUT
    call    SEND_I2C
    movlw   0x00
    call    SEND_I2C
    call    STOP_I2C

    return
