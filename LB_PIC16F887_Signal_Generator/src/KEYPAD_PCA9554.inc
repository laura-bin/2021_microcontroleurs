
#define	WAVE_FORM   KEYPAD_DATA, 0
#define	FREQ_UP	    KEYPAD_DATA, 1
#define	FREQ_DOWN   KEYPAD_DATA, 2
#define	DC_UP	    KEYPAD_DATA, 3
#define	DC_DOWN	    KEYPAD_DATA, 4
#define	AMPL_UP	    KEYPAD_DATA, 5
#define	AMPL_DOWN   KEYPAD_DATA, 6


; Functions
; =========
EXTRN	INIT_PCA9554	    ; Initializes the PCA9554 I/0 expander initialization


; Variables
; =========
EXTRN	KEYPAD_DATA	    ; Data read from the keypad
