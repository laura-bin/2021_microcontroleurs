#include <xc.inc>


; I2C bus initialization
; ======================
init_I2C:
    ; I2C high speed mode, all indicators -> 0
    banksel SSPSTAT
    clrf    SSPSTAT

    ; Baud rate: 400kHz
    banksel SSPADD
    movlw   0x0A    ; replace by 01 ?
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


; Sends the I2C start condition
; =============================
start_I2C:
    ; Wait for the I2C to be available
    call    idle_I2C

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


; Sends the I2C stop condition
; ============================
stop_I2C:
    ; Wait for the I2C to be available
    call    idle_I2C

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


; Sends data using I2C bus
; ========================
send_I2C:
    ; Wait for the I2C to be available
    call    idle_I2C

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


; Waits for the I2C bus to be available
; =====================================
idle_I2C:
    banksel SSPCON2
idle:
    btfsc   R_W		; Transmit is in progress
    goto    idle
    btfsc   SEN		; Start condition enabled
    goto    idle
    btfsc   RSEN	; Repeated start condition enabled
    goto    idle
    btfsc   PEN		; Stop condition enabled
    goto    idle
    btfsc   RCEN	; Receive enabled (not used here)
    goto    idle
    btfsc   ACKEN	; Acknowledgement sequence enabled (not used here)
    goto    idle
    return