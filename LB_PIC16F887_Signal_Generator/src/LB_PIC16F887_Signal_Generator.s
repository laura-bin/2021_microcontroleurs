; Generateur de signaux digitaux (carres et triangulaires)
; ========================================================
;
; v1:
;   - program structure with run/config modes
;   - LCD connections 
; v2:
;   - 16 bits arithmetic
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
CONFIG WDTE=OFF		; Watchdog timer disabled
CONFIG PWRTE=OFF 	; PWRT disabled
CONFIG CP=OFF		; Program memory unprotected

#include <xc.inc>
#include "16bits_arithmetic.inc"
#include "16bits_bin_bcd_ascii.inc"
#include "config.inc"
#include "I2C.inc"
#include "KEYPAD_PCA9554.inc"
#include "LM044L_PCA9535.inc"
#include "delay.inc"

#define	AMPL_UP		KEYPAD_DATA, 6
#define	AMPL_DOWN	KEYPAD_DATA, 5
#define	DC_UP		KEYPAD_DATA, 4
#define	DC_DOWN		KEYPAD_DATA, 3
#define	FREQ_UP		KEYPAD_DATA, 2
#define	FREQ_DOWN	KEYPAD_DATA, 1
#define	NEXT_WAVE_FORM	KEYPAD_DATA, 0


; Variables
; =========
PSECT udata_bank0
; Context saving variables
w_temp:		DS 1
status_temp:	DS 1
; pclath_temp:	DS 1

; Signal information
; bit 0 : rising edge (1) - falling edge (0)
signal_info:	DS 1
#define	RUN_MODE    signal_info, 0	    ; (1) run mode - (0) config mode
#define	PREV_MODE   signal_info, 1	    ; (1) run mode - (0) config mode	

period:	    DS 2


; signal parameters:
amplitude:		DS 1
duty_cycle:		DS 1
frequency:		DS 2
wave_form:		DS 1    ; (square, triangle or sawtooth)
#define WF_SQUARE	wave_form, 1
#define WF_TRIANGLE	wave_form, 0

    
i:  DS 1
rising_inc: DS 1
falling_inc: DS 1
dir: DS 1 ; sens de l'increment
cpt: DS 1   ; nombre constant d'increments
signal: DS 1	;valeur courante du signal
rising_steps: DS 1  ; nombre de pas du front montant
falling_steps: DS 1  ; nombre de pas du front descendant
samples:	DS 1	; nombre total d'echantillons -> samples
v1: DS 1    ; data send by keypad
v2: DS 1    ; previous v1
app_mode: DS 1	    ; app mode char / mode RUN (1) or config (0)

PSECT udata_bank1
signal_low: DS 50	; signal data vector (low values)

PSECT udata_bank2
signal_high: DS 50	; signal data vector (high values)

    
    
    
    
; sert a cacher al séparation du tableau en 2 (en pas répéter l'alternative)
; macro car on ne perd pas les 4 cycles machines d'office du call
; get the element of the array at given index
GET_SIGNAL  MACRO   INDEX
LOCAL GET_SIGNAL_HIGH		    ; LOCAL ensures the uniqueness of the labels defined in the macro 
LOCAL GET_SIGNAL_END		    ; (a unique tag is added to the label address)
    movlw   50
    subwf   (INDEX), w  
    btfsc   CARRY		    ; if counter is greater than 50
    goto    GET_SIGNAL_HIGH	    ; goto the high half of the array (in bank 2)
    bcf	    STATUS, 7		    ; clear IRP to select bank 1
    movlw   signal_low		    ; move the array pointer value
    movwf   FSR			    ; to FSR
    movf    (INDEX), w
    addwf   FSR, f		    ; add the index value to the pointer
    movf    INDF, w		    ; move the data at this position to the W register
    goto    GET_SIGNAL_END
GET_SIGNAL_HIGH:
    bsf	    STATUS, 7		    ; set IRP to select bank 2
    movlw   BANKMASK(signal_high)   ; get the correct pointer on the array (7bits -> 8bits)
    movwf   FSR			    ; and move it to FSR
    movlw   50
    subwf   (INDEX), w		    ; remove 50 from the index value
    addwf   FSR, f		    ; add the index value to the pointer
    movf    INDF, w		    ; move the data at this position to the W register
GET_SIGNAL_END:
ENDM
    
SET_SIGNAL  MACRO   CPT, VAL
LOCAL SET_SIG1
LOCAL SET_SIG2
    movlw   50
    subwf   CPT, w
    btfsc   CARRY
    goto    SET_SIG1
    bcf	    STATUS, 7
    movlw   tsig1
    movwf   FSR
    movf    CPT, w
    addwf   FSR, f
    movf    VAL, w
    movwf   INDF
    goto    SET_SIG2
SET_SIG1:
    bsf	    STATUS, 7
    movlw   BANKMASK(tsig2)
    movwf   FSR
    movlw   50
    subwf   CPT, w
    addwf   FSR, f
    movf    VAL, w
    movwf   INDF
SET_SIG2:
ENDM

; Reset vector
; ============
PSECT reset_vec, class=CODE, delta=2
reset_vec:
    goto    init

; Interruption vector
; ===================
PSECT inter_vec, class=CODE, delta=2
inter_vec:
    ; save w and STATUS registers
    ; swaps are used because they do not affect the status bits
    movwf   w_temp
    swapf   STATUS, w
    movwf   status_temp
    ; uncomment if PCLATH register is used
    ;movf    PCLATH, w
    ;movwf   pclath_temp
    ;clrf    PCLATH
    
    bsf	    LED_CONFIG
    
    btfsc   CCP2IF
    goto    generate_signal
    goto    read_inputs
    
;    GET_SIGNAL	cpt
;    
;    movwf   DAC0808
;    incf    cpt, f
;    movf    samples, w
;    subwf   cpt, w
;    btfsc   ZERO
;    clrf    cpt
;    goto    end_inter


read_inputs:
    call    READ_PCA9554
    btfss   AMPL_UP
    call    set_amplitude_up
    btfss   AMPL_DOWN
    call    set_amplitude_down
    btfss   DC_UP
    call    set_duty_cycle_up
    btfss   DC_DOWN
    call    set_duty_cycle_down
    btfss   FREQ_UP
    call    set_frequency_up
    btfss   FREQ_DOWN
    call    set_frequency_down
    btfss   NEXT_WAVE_FORM
    call    set_next_wave_form


    ;call	READ_PCA9554
    bcf	    INTF
    goto    end_inter
    
generate_signal:
    btfss   PORTC, 5
    bsf	    PORTC, 5
    btfsc   PORTC, 5
    bcf	    PORTC, 5
    bcf	    CCP2IF
    goto    end_inter 

end_inter:
    ; Clear interrupt flag
    
    ; Restore w and STATUS registers (& Bank Select Bit register)
    ;movf    pclath_temp, w  ;Restore PCLATH
    ;movf    pclath_temp, w  ;Restore PCLATH
    ;movwf   PCLATH	    ;Move W into PCLATH
    
    swapf   status_temp, w  ;Swap STATUS_TEMP register into W
    ;(sets bank to original state)
    movwf   STATUS	    ; Move W into STATUS register
    swapf   w_temp, f	    ;Swap W_TEMP
    swapf   w_temp, w	    ;Swap W_TEMP into W
    retfie


PSECT code

; PIC initialization
; ==================
init:
    ; PORTC -> output
    banksel TRISA
    clrf    TRISA
    ; PORTC -> output
    banksel TRISC
    clrf    TRISC
    ; PORTC -> output
    banksel TRISD
    clrf    TRISD
    
    banksel TRISB
    bsf	    TRISB, 0

    ; RUN/CONFIG -> input
    bsf	    TRISC, 7

    ; PORTB -> digital
    banksel	ANSELH
    clrf	ANSELH
    
    banksel	OPTION_REG
    bcf		INTEDG

    ; PORTC -> 0
    banksel PORTA
    movlw   0x00
    movwf   PORTA
    ; PORTC -> 0
    banksel PORTC
    clrf    PORTC
    ; PORTC -> 0
    banksel PORTD
    clrf    PORTD

    ; I2C bus initialization
    ; ----------------------

    call    INIT_I2C
    call    INIT_PCA9535
    call    INIT_LM044L
    call    INIT_PCA9554

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

    ; CCPR2H-L -> 50*Tcy(0.2us) -> 10us (100kHz)
    banksel	CCPR2H
    clrf	CCPR2H
    movlw	50
    movwf	CCPR2L
    
    ; init parameters
    ; compute the frequency
    INIT16	OP1, 5000
    MOV16	OP2, CCPR2L
    call	DIV16
    MOV16	frequency, RESULT

    call	init_parameters

    call	init_display
    call	display_wave_form
    call	display_frequency
    call	display_duty_cycle
    call	display_amplitude
    
    ; RUN/CONFIG mode initialization
    ; ------------------------------

    btfsc	MODE
    call	init_mode_run
    btfss	MODE
    call	init_mode_config


; Main loop: RUN/CONFIG button polling
; ====================================
main:
    ; Mode RUN / CONFIG selection
    btfss	MODE
    goto	signal_config
    goto	signal_gen


signal_gen:
    btfss   PREV_MODE
    call    init_mode_run
    goto    main

init_mode_run:
    ; disable other interruptions
    banksel	INTCON
    clrf	INTCON

    ; set the LED
    bcf	    LED_CONFIG
    bsf	    LED_RUN

    bsf	    PREV_MODE

    ; set CCP2 interruption
    banksel	PIE2
    bsf		CCP2IE	    ; Enable CCP2 interruptions
    banksel	INTCON
    bcf		CCP2IF	    ; Clear CCP2 interrupt flag
    bsf		PEIE	    ; Enable peripherical interruptions
    bsf		GIE	    ; Enable global interruptions
    return


signal_config:
    btfsc   PREV_MODE
    call    init_mode_config
    goto    main

init_mode_config:
    ; disable other interruptions
    banksel	INTCON
    clrf	INTCON

    ; set the LED
    bcf	    LED_RUN
    bsf	    LED_CONFIG

    bcf	    PREV_MODE
    
    ; enable interruption on RB0
    banksel	INTCON
    bcf		INTF
    bsf		INTE
    bsf		GIE
    
    return


; Initializes the dignal parameters
; =================================
init_parameters:
    ; amplitude -> 100%
    movlw	100
    movwf	amplitude
    
    ; duty cycle -> 50%
    movlw	50
    movwf	duty_cycle
    
    ; frequency -> 1000kHz
    INIT16	frequency, 1000
    
    ; wave form -> square
    clrf	wave_form
    bsf		WF_SQUARE
    return


; Increases the amplitude value (step 10)
; =======================================
set_amplitude_up:
    ; test the max value (100)
    movlw	100
    subwf	amplitude, w
    btfsc	CARRY
    return

    ; if the amplitude is lower than 100, add the step value
    movlw	10
    addwf	amplitude, f

    ; then display the updated amplitude value
    goto	display_amplitude


; Decreases the amplitude value (step 10)
; =======================================
set_amplitude_down:
    ; substract the step value to the amplitude
    movlw	10
    subwf	amplitude, w
    btfss	CARRY
    return

    ; if result is greater than or equal to 0, update the amplitude value with it
    movwf	amplitude

    ; then display the updated amplitude value
    goto	display_amplitude


; Increases the duty cycle value (step 10)
; ========================================
set_duty_cycle_up:
    ; test the max value (100)
    movlw	100
    subwf	duty_cycle, w
    btfsc	CARRY
    return

    ; if the duty cycle is lower than 100, add the step value
    movlw	10
    addwf	duty_cycle, f

    ; then display the updated duty cycle value
    goto	display_duty_cycle


; Decreases duty cycle value (step 5)
; ===================================
set_duty_cycle_down:
    ; substract the step value to the duty cycle
    movlw	10
    subwf	duty_cycle, w
    btfss	CARRY
    return

    ; if result is greater than or equal to 0, update the duty cycle value with it
    movwf	duty_cycle

    ; then display the updated duty cycle value
    goto	display_duty_cycle


; Increases the frequency value (step 50)
; =======================================
set_frequency_up:
    ; test the max value (1050)
    SUBI16	frequency, 1050
    btfsc	CARRY
    goto	end_frequency_up
    goto	add_frequency_step
end_frequency_up:
    ADDI16	frequency, 1050
    return
add_frequency_step:
    ADDI16	frequency, 1100
    goto	display_frequency


; Decreases the frequency value (step 50)
; =======================================
set_frequency_down:
    ; substract the step value to the frequency
    SUBI16	frequency, 50
    btfsc	CARRY

    ; if the result is greater than or equal to 0, display the updated frequency value
    goto	display_frequency

    ; else reverse the substraction and return
    ADDI16	frequency, 50
    return


; Sets the next wave form: square -> triangle -> square...
; ========================================================
set_next_wave_form:
    ; bit rotation
    bcf		CARRY
    rrf		wave_form

    ; if CARRY is set
    btfsc	CARRY

    ; then initialize the wave form to the first
    bsf		WF_SQUARE

    ; display the wave form name
    goto	display_wave_form


; Parameters display initialization
; =================================
init_display:
    movlw	LCD_LINE1
    movwf	LCD_DATA
    call	SEND_COMMAND_LCD
    movlw	'W'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'a'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'v'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'e'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	' '
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'f'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'o'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'r'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'm'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD

    movlw	LCD_LINE2
    movwf	LCD_DATA
    call	SEND_COMMAND_LCD
    movlw	'F'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'r'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'e'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'q'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'u'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'e'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'n'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'c'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'y'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD

    movlw	0xD1
    movwf	LCD_DATA
    call	SEND_COMMAND_LCD
    movlw	'k'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'H'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'z'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD

    movlw	LCD_LINE3
    movwf	LCD_DATA
    call	SEND_COMMAND_LCD
    movlw	'D'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'u'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	't'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'y'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	' '
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'c'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'y'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'c'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'l'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'e'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD

    movlw	0xA7
    movwf	LCD_DATA
    call	SEND_COMMAND_LCD
    movlw	'%'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD

    movlw	LCD_LINE4
    movwf	LCD_DATA
    call	SEND_COMMAND_LCD
    movlw	'A'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'm'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'p'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'l'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'i'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	't'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'u'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'd'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'e'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD

    movlw	0xE7
    movwf	LCD_DATA
    call	SEND_COMMAND_LCD
    movlw	'%'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    return


; Displays the signal wave form
; =============================
display_wave_form:
    btfsc   WF_SQUARE
    goto    display_wave_form_square
    btfsc   WF_TRIANGLE
    goto    display_wave_form_triangle
    
; displays the wave form traingle
; ===============================
display_wave_form_triangle:
    movlw	0x8C
    movwf	LCD_DATA
    call	SEND_COMMAND_LCD
    movlw	't'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'r'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'i'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'a'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'n'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'g'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'l'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'e'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    return

; displays the wave form square
; =============================
display_wave_form_square:
    movlw	0x8C
    movwf	LCD_DATA
    call	SEND_COMMAND_LCD
    movlw	' '
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	' '
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	's'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'q'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'u'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'a'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'r'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'e'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    return

; displays the frequency
; ======================
display_frequency:
    MOV16	VAR16, frequency
    call	HEX16_TO_BCD_ASCII
    movlw	0xCC
    movwf	LCD_DATA
    call	SEND_COMMAND_LCD
    movf	BCD4, w
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movf	BCD3, w
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movf	BCD2, w
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movf	BCD1, w
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movf	BCD0, w
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    return

; displays the duty cycle
; =======================
display_duty_cycle:
    movf	duty_cycle, w
    call	HEX8_TO_BCD_ASCII
    movlw	0xA4
    movwf	LCD_DATA
    call	SEND_COMMAND_LCD
    movf	BCD2, w
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movf	BCD1, w
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movf	BCD0, w
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    return


; Displays the amplitude
; ======================
display_amplitude:
    movf	amplitude, w
    call	HEX8_TO_BCD_ASCII
    movlw	0xE4
    movwf	LCD_DATA
    call	SEND_COMMAND_LCD
    movf	BCD2, w
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movf	BCD1, w
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movf	BCD0, w
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    return

; End of ASM code
; ===============
END reset_vec