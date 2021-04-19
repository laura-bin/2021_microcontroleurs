; PCA9554 8 bits I/O expander configured for input reading
;
; Laura Binacchi - SLP 2020/2021
; ==============================================================================

; Functions
; =========
EXTRN	INIT_PCA9554	    ; Initializes the PCA9554 I/0 expander initialization
EXTRN	READ_PCA9554	    ; Reads the PCA9554 in the KEYPAD_DATA

; Variables
; =========
EXTRN	PCA9554_DATA	    ; Data read from the PCA9554
