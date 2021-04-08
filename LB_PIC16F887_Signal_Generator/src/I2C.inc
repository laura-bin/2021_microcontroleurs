

; Functions
; =========
EXTRN INIT_I2C	    ; I2C bus initialization
EXTRN START_I2C	    ; Sends the I2C start condition
EXTRN STOP_I2C	    ; Sends the I2C stop condition
EXTRN SEND_I2C	    ; Sends data using I2C bus
EXTRN IDLE_I2C	    ; Waits for the I2C bus to be available
