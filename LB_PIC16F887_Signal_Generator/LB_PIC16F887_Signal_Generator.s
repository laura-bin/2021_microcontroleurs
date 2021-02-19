; Generateur de signaux digitaux (carres et triangulaires)
; ========================================================
;
; v1:
;   - program structure with run/config modes
;   - LCD connections
;
; PIC16F887 (4MHz clock - 1MHz/1us cycle)
; Proteus : LB_PIC16F887_Signal_Generator_v1
;
; Linker options:
;   -preset_vec=0x00
;   -pinter_vec=0x04
;
; Laura Binacchi - SLP 2020/2021


; PIC configuration
; =================
PROCESSOR 16F887


; Configuration words
; ===================
CONFIG FOSC=HS		; HS Oscillator
CONFIG WDTE=OFF 	; Watchdog timer disabled
CONFIG PWRTE=OFF 	; PWRT disabled
CONFIG CP=OFF		; Program memory unprotected

#include <xc.inc>
#include "config.inc"

#define MODE	    PORTC, 7	; RUN / CONFIG mode button

#define LED_RUN	    PORTC, 0
#define LED_CONFIG  PORTC, 1
    
#define TICK	    PORTC, 2


; Variables
; =========
PSECT udata_bank0
; Context saving variables
w_temp:	DS 1
s_temp:	DS 1

; Delay counters
count1:	DS 1
count2:	DS 1

; Data displayed or command
LCD_data:   DS 1


; Reset vector
; ============
PSECT reset_vec, class=CODE, delta=2
reset_vec:
    goto    init


; Interruption vector
; ===================
PSECT inter_vec, class=CODE, delta=2
inter_vec:
    ; Store w & status registers
    movwf   w_temp
    swapf   STATUS, w
    movwf   s_temp
    
    ; tick
    btfsc   TICK
    goto    tick_off
    goto    tick_on

tick_off:
    bcf	    TICK
    goto    end_inter

tick_on:
    bsf	    TICK
    goto    end_inter

end_inter:
    ; Clear interrupt flag
    bcf	    CCP2IF
    
    ; Restore w & qtatus registers (& Bank Select Bit register)
    swapf   s_temp
    movwf   STATUS
    swapf   w_temp, f
    swapf   w_temp, w
    retfie


PSECT code

; PIC initialization
; ==================
init:
    ; PORTC -> output
    banksel TRISC
    clrf    TRISC
    
    ; RUN/CONFIG -> input
    bsf	    TRISC, 7
    
    ; PORTC -> 0
    banksel PORTC
    clrf    PORTC

    ; I2C bus initialization
    ; ----------------------

    call	init_I2C

    ; PCA9535 I/0 expander initialization
    ; -----------------------------------

    call    start_I2C
    movlw   PCA_LCD_ADD
    call    send_I2C

    ; PORT0-1 -> output
    movlw   PCA_PORT0_CONFIG
    call    send_I2C
    movlw   0x00
    call    send_I2C
    call    send_I2C

    ; PORT0-1 -> 0
    movlw   PCA_PORT0_OUT
    call    send_I2C
    movlw   0x00
    call    send_I2C
    call    send_I2C

    call    stop_I2C

    ; LM044L alphanumeric LCD initialization
    ; --------------------------------------

    ; Initialize the LCD in 8 bits
    movlw	0x38
    movwf	LCD_data
    call	send_command_LCD

    call	delay_4ms

    ; Initialize the LCD cursor
    movlw	CURSOR_OFF
    movwf	LCD_data
    call	send_command_LCD

    ; Writing from left to right
    movlw	0x06
    movwf	LCD_data
    call	send_command_LCD

    ; Clear the LCD
    movlw	LCD_CLEAR
    movwf	LCD_data
    call	send_command_LCD

    ; Place the cursor on the first line
    movlw	LCD_LINE1
    movwf	LCD_data
    call	send_command_LCD

    ; Compare module initialization
    ; -----------------------------

    ; Compare mode, trigger special event (CCP2IF bit is set; CCP2 resets TMR1)
    banksel	CCP2CON
    movlw	0x0B
    movwf	CCP2CON

    ; Timer1 ON, prescale 1:1
    movlw	0x01
    movwf	T1CON

    ; Timer1 -> 0
    clrf	TMR1H
    clrf	TMR1L

    ; CCPR2H-L -> 500us (2kHz)
    banksel	CCPR2H
    movlw	0x01
    movwf	CCPR2H
    movlw	0xF4
    movwf	CCPR2L

    ; RUN/CONFIG mode initialization
    ; ------------------------------

    btfsc	MODE
    goto	init_run_mode
    goto	init_config_mode

init_run_mode:
    call	set_run_mode
    goto	main

init_config_mode:
    call	set_config_mode

; Main loop: RUN/CONFIG button polling
; ====================================
main:
    ; Mode RUN / CONFIG selection
    btfss	MODE
    goto	config_mode

; Run mode : set interruption and generate the signal
; ===================================================
run_mode:
    btfsc	LED_CONFIG
    call	set_run_mode
    goto	main
    
; Config mode : read the inputs to configure the signal
; =====================================================
config_mode:
    btfsc	LED_RUN
    call	set_config_mode
scan_inputs:
    nop
    nop
    nop
    goto	main
    
set_run_mode:
    ; set LEDs
    bsf		LED_RUN
    bcf		LED_CONFIG

    ; set LCD (send character to the last position of the first line)
    movlw	0x93
    movwf	LCD_data
    call	send_command_LCD
    movlw	'R'
    movwf	LCD_data
    call	send_char_LCD

    ; set CCP2 interruption
    banksel	PIE2
    bsf		CCP2IE	    ; Enable CCP2 interruptions
    banksel	INTCON
    bcf		CCP2IF	    ; Clear CCP2 interrupt flag
    bsf		PEIE	    ; Enable peripherical interruptions
    bsf		GIE	    ; Enable global interruptions
    return

set_config_mode:
    ; stop interruptions
    banksel	PIE2
    bcf		CCP2IE	    ; Enable CCP2 interruptions
    banksel	INTCON
    bcf		PEIE
    bcf		GIE

    ; set LEDs
    bsf		LED_CONFIG
    bcf		LED_RUN

    ; set LCD (send character to the last position of the first line)
    movlw	0x93
    movwf	LCD_data
    call	send_command_LCD
    movlw	'C'
    movwf	LCD_data
    call	send_char_LCD
    return


; Sends a character to the LCD
; ============================
send_char_LCD:
    ; Send the data with RS -> 1 & E -> 0
    call    start_I2C
    movlw   PCA_LCD_ADD
    call    send_I2C
    movlw   PCA_PORT0_OUT
    call    send_I2C
    movlw   LCD_RS
    call    send_I2C
    movf    LCD_data, w
    call    send_I2C
    call    stop_I2C

    ; Rising edge on the LCD Enable PIN
    call    start_I2C
    movlw   PCA_LCD_ADD
    call    send_I2C
    movlw   PCA_PORT0_OUT
    call    send_I2C
    movlw   LCD_RS_E
    call    send_I2C
    call    stop_I2C

    call    start_I2C
    movlw   PCA_LCD_ADD
    call    send_I2C
    movlw   PCA_PORT0_OUT
    call    send_I2C
    movlw   LCD_RS
    call    send_I2C
    call    stop_I2C

    return


; Sends a command to the LCD
; ==========================
send_command_LCD:
    ; Send the command with RS -> 0 & E -> 0
    call    start_I2C
    movlw   PCA_LCD_ADD
    call    send_I2C
    movlw   PCA_PORT0_OUT
    call    send_I2C
    movlw   0x00
    call    send_I2C
    movf    LCD_data, w
    call    send_I2C
    call    send_I2C
    call    stop_I2C

    ; Rising edge on the LCD Enable PIN
    call    start_I2C
    movlw   PCA_LCD_ADD
    call    send_I2C
    movlw   PCA_PORT0_OUT
    call    send_I2C
    movlw   LCD_E
    call    send_I2C
    call    stop_I2C

    call    start_I2C
    movlw   PCA_LCD_ADD
    call    send_I2C
    movlw   PCA_PORT0_OUT
    call    send_I2C
    movlw   0x00
    call    send_I2C
    call    stop_I2C

    return


; 4 milliseconds delay
; ====================
delay_4ms:
    movlw   0x08
    movwf   count1
delay41:
    movlw   0xA4
    movwf   count2
delay42:
    decfsz  count2, f
    goto    delay42
    decfsz  count1, f
    goto    delay41
    return

; 10 milliseconds delay
; =====================
delay_10ms:
    movlw   0x15
    movwf   count1
delay101:
    movlw   0x9E
    movwf   count2
delay102:
    decfsz  count2, f
    goto    delay102
    decfsz  count1, f
    goto    delay101
    return


#include "I2C.s"

; End of ASM code
; ===============
END reset_vec