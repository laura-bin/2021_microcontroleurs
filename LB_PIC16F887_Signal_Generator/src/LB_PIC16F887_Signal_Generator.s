; Generateur de signaux digitaux (carres et triangulaires)
; ========================================================
;
; PIC16F887 (20MHz clock - 5MHz/0.2us Tcy)
; Proteus : LB_PIC16F887_Signal_Generator_v1
;
; Linker options:
;   -presetVec=0x00
;   -pinterVec=0x04
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
#include "16bits_arithmetic_macros.inc"
#include "16bits_bin_bcd_ascii.inc"
#include "config.inc"
#include "I2C.inc"
#include "LM044L_PCA9535.inc"
#include "PCA9554.inc"
#include "delay.inc"

; signal wave forms types implemented
#define WF_SQUARE	wave_form, 2
#define WF_TRIANGLE	wave_form, 1
#define WF_SAWTOOTH	wave_form, 0

; application previous mode
#define PREV_MODE	sig_info, 0

#define N_SAMPLES	100


; Variables
; =========
PSECT udata_bank0

; Context saving variables
w_temp:		DS 1
status_temp:	DS 1
; pclath_temp:	DS 1

; signal parameters
amplitude:		DS 1
duty_cycle:		DS 1
frequency:		DS 2
wave_form:		DS 1    ; (square, triangle or sawtooth)

; signal information kept in memory
sig_info:		DS 1

    
    
    

signal16: DS 2

i:  DS 1
rising_inc: DS 1
falling_inc: DS 1
dir: DS 1		; sens de l'increment
cpt: DS 1		; nombre constant d'increments
signal: DS 1		; valeur courante du signal
rising_steps: DS 1	; nombre de pas du front montant
falling_steps: DS 1	; nombre de pas du front descendant
samples:	DS 1	; nombre total d'echantillons -> samples
v1: DS 1		; data send by keypad
v2: DS 1		; previous v1
app_mode: DS 1		; app mode char / mode RUN (1) or config (0)


PSECT udata_bank1
signal1: DS 50	    ; signal data vector 1/2

PSECT udata_bank2
signal2: DS 50	    ; signal data vector 2/2



; Get the element of the signal array at given index
; the value is put in the W register
; ==================================================
GET_SIGNAL  MACRO   INDEX
LOCAL GET_SIGNAL2		    ; LOCAL ensures the uniqueness of the labels defined in the macro 
LOCAL GET_SIGNAL_END		    ; by adding a unique tag to the label address
    movlw   50
    subwf   (INDEX), w  
    btfsc   CARRY		    ; if the index is greater than 50
    goto    GET_SIGNAL2		    ; goto the second part of the array (in bank 2)
    bcf	    STATUS, 7		    ; clear IRP to select bank 1
    movlw   signal1		    ; move the array pointer value
    movwf   FSR			    ; to FSR
    movf    (INDEX), w
    addwf   FSR, f		    ; add the index value to the pointer
    movf    INDF, w		    ; move the data at this position to the W register
    goto    GET_SIGNAL_END
GET_SIGNAL2:
    bsf	    STATUS, 7		    ; set IRP to select bank 2
    movlw   BANKMASK(signal2)	    ; get the correct pointer on the array (7bits -> 8bits)
    movwf   FSR			    ; and move it to FSR
    movlw   50
    subwf   (INDEX), w		    ; adjust index
    addwf   FSR, f		    ; and add the index value to the pointer
    movf    INDF, w		    ; move the data at this position to the W register
GET_SIGNAL_END:
ENDM


; Set the element of the signal array at given index
; ==================================================
SET_SIGNAL  MACRO   INDEX, VALUE
LOCAL SET_SIGNAL2
LOCAL SET_SIGNAL_END
    movlw   50
    subwf   INDEX, w
    btfsc   CARRY		    ; if the index is greater than 50
    goto    SET_SIGNAL2		    ; go to the second part of the array (in bank 2)
    bcf	    STATUS, 7		    ; select bank 1
    movlw   signal1		    ; move the pointer of the array
    movwf   FSR			    ; in FSR
    movf    INDEX, w
    addwf   FSR, f		    ; add the index value		
    movf    VALUE, w
    movwf   INDF		    ; set the value
    goto    SET_SIGNAL_END
SET_SIGNAL2:
    bsf	    STATUS, 7		    ; select bank 2
    movlw   BANKMASK(signal2)	    ; move the pointer of the array
    movwf   FSR			    ; in FSR
    movlw   50
    subwf   INDEX, w		    ; adjust index
    addwf   FSR, f		    ; and add the index value to the pointer
    movf    VALUE, w
    movwf   INDF		    ; set the value
SET_SIGNAL_END:
ENDM


; Reset vector
; ============
PSECT resetVec, class=CODE, delta=2
resetVec:
    goto	init

; Interruption vector
; ===================
PSECT interVec, class=CODE, delta=2
interVec:
    ; Save w and STATUS registers
    ; swaps are used because they do not affect the status bits
    movwf	w_temp
    swapf	STATUS, w
    movwf	status_temp
    ; Uncomment if PCLATH register is used
    ;movf    PCLATH, w
    ;movwf   pclath_temp
    ;clrf    PCLATH

    ; Test the interrupt flag
    btfsc   CCP2IF
    goto    generate_signal

read_inputs:
    ; Test the data read by the PCA9554 to set the selected parameter
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

    ; Clear the interrupt flag
    bcf	    INTF
    goto    end_inter
    
generate_signal:
    GET_SIGNAL	cpt
    movwf	DAC0808
    incf	cpt, f
    movlw	N_SAMPLES
    subwf	cpt, w
    btfsc	ZERO
    clrf	cpt

    ;btfss   PORTC, 5
    ;bsf	    PORTC, 5
    ;btfsc   PORTC, 5
    ;bcf	    PORTC, 5
    bcf	    CCP2IF
    goto    end_inter 

end_inter:
    ; Restore w and STATUS registers (& Bank Select Bit register)
    ;movf    pclath_temp, w
    ;movf    pclath_temp, w
    ;movwf   PCLATH
    swapf   status_temp, w
    movwf   STATUS
    swapf   w_temp, f
    swapf   w_temp, w
    retfie


PSECT code

; PIC initialization
; ==================
init:
    ; I/O configuration
    banksel	TRISC
    movlw	0x01	    ; PORTC -> output, except for RC0 (mode switch)
    movwf	TRISC

    clrf	TRISD	    ; PORTD (DAC0808) -> output

    bsf		TRISB, 0    ; RB0 -> input

    banksel	PORTC	    ; LED RUN OFF
    bcf		LED_RUN

    clrf	DAC0808	    ; DAC0808 (PORTD) -> 0

    ; RB0 interruption
    banksel	ANSELH	    ; PORTB -> digital
    clrf	ANSELH
    bcf		INTEDG	    ; RB0 interruption triggered by falling edge

    ; CCP2 interruption
    banksel	CCP2CON
    movlw	0x0B	    ; Compare mode, trigger special event (CCP2IF bit is set; CCP2 resets TMR1)
    movwf	CCP2CON

    movlw	0x01	    ; Timer1 ON, prescale 1:1
    movwf	T1CON

    clrf	TMR1H	    ; Timer1 -> 0
    clrf	TMR1L

    clrf	CCPR2H
    movlw	50	    ; CCPR2H-L -> 50*Tcy(0.2us) -> 10us (100kHz)
    movwf	CCPR2L

    ; Set the signal default parameters
    movlw	100		    ; amplitude -> 100%
    movwf	amplitude

    movlw	50		    ; duty cycle -> 50%
    movwf	duty_cycle

    INIT16	frequency, 1000	    ; frequency -> 1000kHz
    
    clrf	wave_form	    ; wave form -> square
    bsf		WF_SQUARE

    ; Initialize the I2C bus and the attached peripherals
    call	INIT_I2C
    call	INIT_PCA9535
    call	INIT_LM044L
    call	INIT_PCA9554

    ; Display the parameters
    call	init_display
    call	display_wave_form
    call	display_frequency
    call	display_duty_cycle
    call	display_amplitude

    ; Set the selected mode
    btfsc	MODE
    call	init_mode_run
    btfss	MODE
    call	init_mode_config

    goto	main


; Main loop: RUN/CONFIG switch polling
; ====================================
main:
    ; Mode RUN / CONFIG selection
    btfss	MODE
    goto	signal_config
    goto	signal_gen


; Signal generation mode
; ======================
signal_gen:
    btfss   PREV_MODE		; if the mode has just been changed
    call    init_mode_run	; initialize the run mode
    goto    main		; wait for the CCP2 interruption to happen


; Signal configuration mode
; =========================
signal_config:
    btfsc   PREV_MODE		; if the mode has just been changed
    call    init_mode_config	; initialize the configuration mode
    goto    main		; wait for the RB0 interruption to happen


; Run mode initialization
; =======================
init_mode_run:
    banksel	INTCON	    ; disable all the interruptions
    clrf	INTCON

    bsf		LED_RUN	    ; switch on the LED
    bsf		PREV_MODE   ; set the previous mode to RUN
    
    btfsc	WF_SQUARE		; if the selected signal wave form is square
    call	init_square_signal	; init the square signal
    btfsc	WF_TRIANGLE		; if the selected signal wave form is triangle
    call	init_triangle_signal	; init the triangle signal

    ; Enable the CCP2 interruption
    banksel	PIE2
    bsf		CCP2IE	    ; enable CCP2 interruptions
    banksel	INTCON
    bcf		CCP2IF	    ; clear CCP2 interrupt flag
    bsf		PEIE	    ; enable peripherical interruptions
    bsf		GIE	    ; enable global interruptions

    return


; Configuration mode initialization
; =================================
init_mode_config:
    banksel	INTCON	    ; disable all the interruptions
    clrf	INTCON

    bcf		LED_RUN	    ; switch off the LED
    bcf		PREV_MODE   ; set the previous mode to CONFIG

    ; Enable interruption on RB0
    banksel	INTCON
    bcf		INTF
    bsf		INTE
    bsf		GIE

    return


; Initialize a square signal (for i: 0 -> 100)
; ===========================================
init_square_signal:
    clrf	i

    ; init the number of steps composing the high period
    movf	duty_cycle, w
    movwf	samples
    
    ; if samples = 0, go to low period configuration
    movf	samples, f
    btfsc	ZERO
    goto	low_period
    
    ; compute the signal value: max value * amplitude / 100
    movlw	0xFF		    ; max value
    movwf	OPL8
    movf	amplitude, w	    ; * amplitude
    movwf	OPR8
    call	MUL8		    ; = RESULT16

    MOV16	OPL16, RESULT16	    ; RESULT16
    INIT16	OPR16, 100	    ; / 100
    call	DIV16		    ; = RESULT32 (<= 255)
    
    movf	RESULT16, w
    movwf	signal


init_high_period:
    SET_SIGNAL	i, signal
    incf	i, f
    decfsz	samples, f
    goto	init_high_period

low_period:
    ; set the number of steps composing the low period: number of samples - duty cycle
    movlw	100
    movwf	samples
    SUB16	samples, duty_cycle

    ; if samples = 0, stop the configuration here
    movf	samples, f
    btfsc	ZERO
    return

    ; set the signal value to 0
    clrf	signal

init_low_period:
    SET_SIGNAL	i, signal
    incf	i, f
    decfsz	samples, f
    goto	init_low_period

    return
    
; Initialize a triangle signal
; ==========================
init_triangle_signal:
    clrf	i

    ; init the number of steps composing the high period
    movf	duty_cycle, w
    movwf	samples
    
    ; if samples = 0, go to low period configuration
    movf	samples, f
    btfsc	ZERO
    goto	falling_edge

    falling_edge:


    return
    
;init_signal:
;    movlw	N_SAMPLES
;    movwf	i
;    clrf	cpt
;    clrf	signal
;init_signal1:
;    SET_SIGNAL	cpt, signal
;    incf	cpt, f
;    incf	signal
;    decfsz	i, f
;    goto	init_signal1
;    return







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
    btfsc	WF_SAWTOOTH
    goto	max_duty_cycle

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

max_duty_cycle:
    movlw	100
    movwf	duty_cycle
    goto	display_duty_cycle


; Decreases duty cycle value (step 5)
; ===================================
set_duty_cycle_down:
    btfsc	WF_SAWTOOTH
    goto	clear_duty_cycle

    ; substract the step value to the duty cycle
    movlw	10
    subwf	duty_cycle, w
    btfss	CARRY
    return

    ; if result is greater than or equal to 0, update the duty cycle value with it
    movwf	duty_cycle

    ; then display the updated duty cycle value
    goto	display_duty_cycle

clear_duty_cycle:
    clrf	duty_cycle
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

    ; if the wave form is sawtooth
    btfss	WF_SAWTOOTH
    goto	display_wave_form
    call	display_wave_form

    ; set the duty cycle to 100
    movlw	100
    movwf	duty_cycle
    goto	display_duty_cycle


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
    goto    display_wf_square
    btfsc   WF_TRIANGLE
    goto    display_wf_triangle
    btfsc   WF_SAWTOOTH
    goto    display_wf_sawtooth

; displays the wave form sawtooth
; ===============================
display_wf_sawtooth:
    movlw	0x8C
    movwf	LCD_DATA
    call	SEND_COMMAND_LCD
    movlw	's'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'a'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'w'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	't'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'o'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'o'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	't'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    movlw	'h'
    movwf	LCD_DATA
    call	SEND_CHAR_LCD
    return
    
; displays the wave form triangle
; ===============================
display_wf_triangle:
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
display_wf_square:
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
END resetVec
