; 16 bits arithmetic functions
; ==============================================================================

; Functions
; =========
EXTRN MUL8	    ; 8 bits multiplication
EXTRN DIV16	    ; 16 bits division
EXTRN MUL16	    ; 16 bits multiplication

; Variables
; =========
EXTRN	OPL8	    ; left operand (8 bits)
EXTRN	OPR8	    ; right operand (8 bits)
EXTRN	OPL16	    ; left operand (16 bits)
EXTRN	OPR16	    ; right operand (16 bits)
EXTRN	RESULT16    ; result (16 bits)
EXTRN	RESULT32    ; result (32 bits)
EXTRN	TEMP16	    ; temporary variable (16 bits)
