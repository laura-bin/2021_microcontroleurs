
; Functions
; =========
EXTRN	INIT_PCA9535	    ; Initializes the PCA9535 I/0 expander initialization
EXTRN	INIT_LM044L	    ; Initializes the LM044L via PCA9535 I/O expander
EXTRN	SEND_CHAR_LCD	    ; Sends a character to the LM044L via PCA9535 I/O expander
EXTRN	SEND_COMMAND_LCD    ; Sends a command to the LM044L via PCA9535 I/O expander

; Variables
; =========
EXTRN	LCD_DATA	    ; Data displayed or command
