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
#include "signal_array.inc"

; signal wave forms types implemented
#define WF_SQUARE	wave_form, 2
#define WF_TRIANGLE	wave_form, 1
#define WF_SAWTOOTH	wave_form, 0

; application previous mode
#define PREV_MODE	sig_info, 0


; Variables
; =========
PSECT udata_bank0

; Context saving variables
w_temp:		DS 1
status_temp:	DS 1
pclath_temp:	DS 1

; signal parameters
amplitude:	DS 1
duty_cycle:	DS 1
frequency:	DS 2
wave_form:	DS 1    ; square, triangle or sawtooth (see define)

; signal working variables
sig_info:	DS 1	; signal information kept in memory (see define)
signal:		DS 1	; signal curent value
max:		DS 1	; max signal value (defined by the signal amplitude)
i:		DS 1	; signal array index
samples:	DS 1	; number of samples iteration count
falling_steps:	DS 1	; falling edge steps count (total samples - duty cycle)

PSECT udata_bank1
signal1: DS 50		; signal data vector 1/2

PSECT udata_bank2
signal2: DS 50		; signal data vector 2/2


; Reset vector
; ============
PSECT resetVec, class=CODE, delta=2
resetVec:
    goto	init

; Interruption vector
; ===================
PSECT interVec, class=CODE, delta=2
interVec:
    ; Save W, STATUS an PCLATH registers
    ; swaps are used because they do not affect the status bits
    movwf	w_temp
    swapf	STATUS, w
    movwf	status_temp
    movf	PCLATH, w
    movwf	pclath_temp
    clrf	PCLATH

    ; Test the interrupt flag
    btfsc	CCP2IF
    goto	generate_signal

read_inputs:
    ; Test the data read by the PCA9554 to set the selected parameter
    call	READ_PCA9554
    btfss	AMPL_UP
    call	set_amplitude_up
    btfss	AMPL_DOWN
    call	set_amplitude_down
    btfss	DC_UP
    call	set_duty_cycle_up
    btfss	DC_DOWN
    call	set_duty_cycle_down
    btfss	FREQ_UP
    call	set_frequency_up
    btfss	FREQ_DOWN
    call	set_frequency_down
    btfss	NEXT_WAVE_FORM
    call	set_next_wave_form

    ; Clear the interrupt flag
    bcf		INTF
    goto	end_inter

generate_signal:
    ; Set the signal value on the DAC
    GET_SIGNAL	i	    ; get signal value from the array at index i
    movwf	DAC0808	    ; move it to the DAC
    incf	i, f	    ; increment the index
    movlw	100
    subwf	i, w
    btfsc	ZERO	    ; if i = 100
    clrf	i	    ; i = 0

    ; Clear the interrupt flag
    bcf		CCP2IF

end_inter:
    ; Restore W, STATUS an PCLATH registers (& Bank Select Bit register)
    movf	pclath_temp, w
    movf	pclath_temp, w
    movwf	PCLATH
    swapf	status_temp, w
    movwf	STATUS
    swapf	w_temp, f
    swapf	w_temp, w
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

    INIT16	frequency, 1000	    ; frequency -> 1000Hz
    
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
    goto	init_mode_run
    goto	init_mode_config


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
    btfss	PREV_MODE	    ; if the mode has just been changed
    goto	init_mode_run	    ; initialize the run mode
    goto	main		    ; wait for the CCP2 interruption to happen


; Signal configuration mode
; =========================
signal_config:
    btfsc	PREV_MODE	    ; if the mode has just been changed
    goto	init_mode_config    ; initialize the configuration mode
    goto	main		    ; wait for the RB0 interruption to happen


; Run mode initialization
; =======================
init_mode_run:
    banksel	INTCON		    ; disable all the interruptions
    clrf	INTCON
    bcf		INTF

    bsf		LED_RUN		    ; switch on the LED
    bsf		PREV_MODE	    ; set the previous mode to RUN
    
    btfsc	WF_SQUARE		; if the selected signal wave form is square
    call	init_square_signal	; init the square signal
    btfsc	WF_TRIANGLE		; if the selected signal wave form is triangle
    call	init_triangle_signal	; init the triangle signal
    btfsc	WF_SAWTOOTH		; if the selected signal wave form is sawtooth
    call	init_sawtooth_signal	; init the sawtooth signal
    
    clrf	i		    ; signal array index = 0

    ; Enable the CCP2 interruption
    banksel	PIE2
    bsf		CCP2IE		    ; enable CCP2 interruptions
    banksel	INTCON
    bcf		CCP2IF		    ; clear CCP2 interrupt flag
    bsf		PEIE		    ; enable peripherical interruptions
    bsf		GIE		    ; enable global interruptions

    goto	main


; Configuration mode initialization
; =================================
init_mode_config:
    banksel	INTCON		    ; disable all the interruptions
    clrf	INTCON
    bcf		CCP2IF

    bcf		LED_RUN		    ; switch off the LED
    bcf		PREV_MODE	    ; set the previous mode to CONFIG

    ; Enable interruption on RB0
    banksel	INTCON
    bsf		INTF
    bsf		INTE
    bsf		GIE

    goto	main


; Initialize a square signal
; ==========================
init_square_signal:
    clrf	i

    ; if the high period samples (duty cycle) = 0, go to low period configuration
    movf	duty_cycle, f
    btfsc	ZERO
    goto	low_period

    ; init the high period loop condition
    movf	duty_cycle, w
    movwf	samples

    ; compute the max signal value
    call	compute_max

init_high_period:
    SET_SIGNAL	i, max		    ; array[i] = signal value
    incf	i, f		    ; i++
    decfsz	samples, f	    ; samples--
    goto	init_high_period    ; if samples > 0 -> loop

low_period:
    ; low period samples = 100 - duty cycle
    movlw	100
    movwf	samples
    movf	duty_cycle, w
    subwf	samples, f

    ; if samples = 0, stop the configuration here
    movf	samples, f
    btfsc	ZERO
    return

    ; set the signal value to 0
    clrf	signal

init_low_period:
    SET_SIGNAL	i, signal	    ; array[i] = 0
    incf	i, f		    ; i++
    decfsz	samples, f	    ; samples--
    goto	init_low_period	    ; if samples > 0 -> loop

    return


; Initialize a triangle signal
; ============================
init_triangle_signal:
    clrf	i

    ; if the rising edge samples (duty cycle) = 0, go to sawtooth signal initialization
    movf	duty_cycle, f
    btfsc	ZERO
    goto	init_sawtooth_signal

    ; if the falling edge samples = 0, go to sawtooth signal initialization
    movlw	100		    ; falling edge steps = 100
    movwf	falling_steps
    movf	duty_cycle, w	    ; - duty cycle
    subwf	falling_steps, f

    movf	falling_steps, f
    btfsc	ZERO
    goto	init_sawtooth_signal

    ; compute the max signal value
    call	compute_max

    ; initialize the rising edge loop condition
    movf	duty_cycle, w
    movwf	samples

set_rising_edge:
    ; signal value = max * i / rising edge steps
    movf	max, w		    ; max
    movwf	OPL8
    movf	i, w		    ; * i
    movwf	OPR8
    call	MUL8		    ; = RESULT16

    MOV16	OPL16, RESULT16	    ; RESULT16
    clrf	OPR16+1
    movf	duty_cycle, w
    movwf	OPR16		    ; / rising steps
    call	DIV16		    ; = RESULT16 (<= 255)

    SET_SIGNAL	i, RESULT16	    ; array [i] = signal value
    incf	i, f		    ; i++
    decfsz	samples, f	    ; samples--
    goto	set_rising_edge	    ; if samples != 0 -> loop

    ; initialize the falling edge loop condition
    movf	falling_steps, w
    movwf	samples

set_falling_edge:
    ; signal value = max * samples / falling steps
    movf	max, w		    ; max
    movwf	OPL8
    movf	samples, w	    ; * samples
    movwf	OPR8
    call	MUL8		    ; = RESULT16x

    MOV16	OPL16, RESULT16	    ; RESULT16
    clrf	OPR16+1
    movf	falling_steps, w    ; / falling steps
    movwf	OPR16
    call	DIV16		    ; RESULT16 (<= 255)

    SET_SIGNAL	i, RESULT16	    ; array[i] = signal value
    incf	i, f		    ; i++
    decfsz	samples, f	    ; samples--
    goto	set_falling_edge    ; if samples != 0 -> loop

    return


; Initialize a sawtooth signal
; ============================
init_sawtooth_signal:
    clrf	i

    ; initialize loop condition
    movlw	100		    ; loop for i = 0
    movwf	samples		    ; while i < 100

    ; compute the max signal value
    call	compute_max

    ; set rising or falling sawtooth signl depending on the duty cycle
    movf	duty_cycle, f
    btfsc	ZERO			; if duty cycle = 0
    goto	set_falling_sawtooth	; generate a reversed sawtooth signal

set_rising_sawtooth:
    ; compute the signal value = max * i / 100
    movf	max, w		    ; max
    movwf	OPL8
    movf	i, w		    ; * i
    movwf	OPR8
    call	MUL8		    ; = RESULT16

    MOV16	OPL16, RESULT16	    ; RESULT16
    INIT16	OPR16, 100	    ; / 100
    call	DIV16		    ; = RESULT16 (<= 255)

    SET_SIGNAL	i, RESULT16
    incf	i, f
    decfsz	samples, f
    goto	set_rising_sawtooth

    return

set_falling_sawtooth:
    ; compute the signal value = max * samples / 100
    movf	max, w		    ; max
    movwf	OPL8
    movf	samples, w	    ; * samples
    movwf	OPR8
    call	MUL8		    ; = RESULT16

    MOV16	OPL16, RESULT16	    ; RESULT16
    INIT16	OPR16, 100	    ; / 100
    call	DIV16		    ; = RESULT16 (<= 255)

    SET_SIGNAL	i, RESULT16
    incf	i, f
    decfsz	samples, f
    goto	set_falling_sawtooth

    return


; Compute the max signal value
; the result is placed in the max variable
; ========================================
compute_max:
    ; max signal value = 255 * amplitude / 100
    movlw	0xFF		    ; 255
    movwf	OPL8
    movf	amplitude, w	    ; * amplitude
    movwf	OPR8
    call	MUL8		    ; = RESULT16

    MOV16	OPL16, RESULT16	    ; RESULT16
    INIT16	OPR16, 100	    ; / 100
    call	DIV16		    ; = RESULT16 (<= 255)

    movf	RESULT16, w
    movwf	max		    ; max = RESULT16 lowest byte

    return


; Increases the amplitude value (step 10)
; =======================================
set_amplitude_up:
    ; test the max value (100)
    bcf		CARRY
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
    bcf		CARRY
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
    bcf		CARRY
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
    bcf		CARRY
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


; Increases the frequency value
; by decreasing the timer 1 prescaler value
; =========================================
set_frequency_up:
    btfsc	T1CON, 5
    goto	fu_test_T4		; if 1x
    btfss	T1CON, 4
    return				; if 00
    bcf		T1CON, 4		; if 01 -> 00
    goto	lsl_frequency

fu_test_T4:
    btfss	T1CON, 4
    goto	fu_switch_T45		; if 10 -> 01
    bcf		T1CON, 4		; if 11 -> 10
    goto	lsl_frequency

fu_switch_T45:
    bcf		T1CON, 5
    bsf		T1CON, 4

lsl_frequency:
    LSL16	frequency
    goto	display_frequency


; Decreases the frequency value
; by increasing the timer 1 prescaler value
; =========================================
set_frequency_down:
    btfss	T1CON, 5
    goto	fd_test_T4	    ; if 0x
    btfsc	T1CON, 4
    return			    ; if 11 -> return
    bsf		T1CON, 4	    ; if 10 -> 11
    goto	lsr_frequency

fd_test_T4:
    btfsc	T1CON, 4
    goto	fd_switch_T45	    ; if 01 -> 10
    bsf		T1CON, 4	    ; if 00 -> 01
    goto	lsr_frequency

fd_switch_T45:
    bsf		T1CON, 5    
    bcf		T1CON, 4

lsr_frequency:
    LSR16	frequency	    ; frequency / 2
    goto	display_frequency


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

    ; display the wave form
    call	display_wave_form

    ; if the wave form is sawtooth
    btfss	WF_SAWTOOTH
    return

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

    movlw	0xD2
    movwf	LCD_DATA
    call	SEND_COMMAND_LCD
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
    movlw	0xCE
    movwf	LCD_DATA
    call	SEND_COMMAND_LCD
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
