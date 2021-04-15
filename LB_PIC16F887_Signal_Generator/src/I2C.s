

#include <xc.inc>
    
; Functions
; =========
global	INIT_I2C	; I2C bus initialization
global	START_I2C	; Sends the I2C start condition
global	RESTART_I2C	; Repeats start condition
global	STOP_I2C	; Sends the I2C stop condition
global	SEND_I2C	; Sends data using the I2C bus
global	RECEIVE_I2C	; Receives data using the I2C bus
global	WAIT_ACK_I2C	; Waits for the I2C acknowledgement
global	IDLE_I2C	; Waits for the I2C bus to be available


PSECT code

INIT_I2C:
    ; I2C high speed mode, all indicators -> 0
    banksel SSPSTAT
    clrf    SSPSTAT

    ; Baud rate: 400kHz
    banksel SSPADD
    movlw   0x0A
    movwf   SSPADD

    ; I2C bus clock & data PINs -> input
    bsf	    TRISC, 3	// clock
    bsf	    TRISC, 4	// data

    ; SSP enable bits & acknowledgement status & data -> 0
    banksel SSPCON2
    clrf    SSPCON2

    banksel SSPCON
    clrf    SSPCON
    ; I2C Master mode, clock = FOSC / (4 * (SSPADD+1))
    bsf	    SSPM3
    ; Enable SSP
    bsf	    SSPEN

    ; Clear SSP interrupt flag
    bcf	    SSPIF

    return


START_I2C:
    ; Wait for the I2C to be available
    call    IDLE_I2C

    ; Clear the interrupt flag
    banksel PIR1
    bcf	    SSPIF

    ; Set the start condition
    banksel SSPCON2
    bsf	    SEN

    ; Wait for the end of the transmission
    banksel PIR1
    btfss   SSPIF
    goto    $-1

    return


RESTART_I2C:
    ; Wait for the I2C to be available
    call    IDLE_I2C

    ; Clear the interrupt flag
    banksel PIR1
    bcf	    SSPIF

    ; Set the start condition
    banksel SSPCON2
    bsf	    RSEN

    ; Wait for the end of the transmission
    banksel PIR1
    btfss   SSPIF
    goto    $-1

    return


STOP_I2C:
    ; Wait for the I2C to be available
    call    IDLE_I2C

    ; Clear the interrupt flag
    banksel PIR1
    bcf	    SSPIF

    ; Set the stop condition
    banksel SSPCON2
    bsf	    PEN

    ; Wait for the end of the transmission
    banksel PIR1
    btfss   SSPIF
    goto    $-1

    return


SEND_I2C:
    ; Wait for the I2C to be available
    call    IDLE_I2C

    ; Clear the interrupt flag
    banksel PIR1
    bcf	    SSPIF

    ; Fill the buffer
    banksel SSPBUF
    movwf   SSPBUF

    ; Wait for the end of the transmission
    btfss   SSPIF
    goto    $-1

    return


RECEIVE_I2C:
    ; Wait for the I2C to be available
    call    IDLE_I2C

    ; Set the 
    banksel SSPCON2
    bsf	    RCEN

    ; Wait for the end of the transmission
    btfsc   RCEN
    goto    $-1

    ; Place the received data in the W register
    banksel SSPBUF
    movf    SSPBUF, w

    return


WAIT_ACK_I2C:
    call    IDLE_I2C
    btfsc   ACKSTAT
    goto    $-1

    return


IDLE_I2C:
    banksel SSPCON2
IDLE:
    btfsc   R_W		; Transmit is in progress
    goto    IDLE
    btfsc   SEN		; Start condition enabled
    goto    IDLE
    btfsc   RSEN	; Repeated start condition enabled
    goto    IDLE
    btfsc   PEN		; Stop condition enabled
    goto    IDLE
    btfsc   RCEN	; Receive enabled
    goto    IDLE
    btfsc   ACKEN	; Acknowledgement sequence enabled
    goto    IDLE

    return
