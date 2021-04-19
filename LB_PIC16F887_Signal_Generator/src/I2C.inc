; I2C bus management
;
; Laura Binacchi - SLP 2020/2021
; ==============================================================================

; Functions
; =========
EXTRN	INIT_I2C	; I2C bus initialization
EXTRN	START_I2C	; Sends the I2C start condition
EXTRN	RESTART_I2C	; Repeats start condition
EXTRN	STOP_I2C	; Sends the I2C stop condition
EXTRN	SEND_I2C	; Sends data using the I2C bus
EXTRN	RECEIVE_I2C	; Receives data using the I2C bus
EXTRN	WAIT_ACK_I2C	; Waits for the I2C acknowledgement
EXTRN	IDLE_I2C	; Waits for the I2C bus to be available
