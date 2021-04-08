; 16 bits arithmetic unsigned multiplication and division

#include <xc.inc>
;#include "16bits_arithmetic.inc"


; Functions
; =========
global DIV16	    ; 16 bits unsigned division
global MUL16	    ; 16 bits unsigned multiplication


; Variables
; =========
global	OP1	; left operand (16 bits)
global	OP2	; right operand (16 bits)
global	RESULT	; result (32 bits)
global	TEMP8	; temporary variable (8bits)
global	TEMP16	; temporary variable (16 bits)


PSECT udata_bank0
OP1:	DS 2
OP2:	DS 2
RESULT:	DS 4
TEMP8:	DS 1
TEMP16:	DS 2


PSECT code
; Division: RESULT = OP1 / OP2
DIV16:
    movf    OP2, f
    btfss   ZERO
    goto    ZERO_TEST_SKIPPED
    movf    OP2+1, f
    btfsc   ZERO
    return

SUBV16:
    movf    OP2+1, w
    movwf   TEMP8
    movf    OP2, w
    subwf   OP1
    btfss   CARRY
    incf    TEMP8, f
    movf    TEMP8, w
    subwf   OP1+1
    return

ADDV16:
    movf    OP2, w
    addwf   OP1
    btfsc   CARRY
    incf    OP1+1, f
    movf    OP2+1, w
    addwf   OP1+1
    return

ZERO_TEST_SKIPPED:
    movlw   1
    movwf   TEMP16
    clrf    TEMP16+1
    clrf    RESULT
    clrf    RESULT+1
SHIFT_IT16:
    bcf	    CARRY
    rlf	    TEMP16, f
    rlf	    TEMP16+1, f
    bcf	    CARRY
    rlf	    OP2, f
    rlf	    OP2+1, f
    btfss   OP2+1, 7
    goto    SHIFT_IT16
DIVU16LOOP:
    call    SUBV16
    btfsc   CARRY
    goto    COUNTX
    call    ADDV16
    goto    FINALX
COUNTX:
    movf    TEMP16, w
    addwf   RESULT
    btfsc   CARRY
    incf    RESULT+1, f
    movf    TEMP16+1, w
    addwf   RESULT+1
FINALX:
    bcf	    CARRY
    rrf	    OP2+1, f
    rrf	    OP2, f
    bcf	    CARRY
    rrf	    TEMP16+1, f
    rrf	    TEMP16, f
    btfss   CARRY
    goto    DIVU16LOOP
    return


; Multiplication: RESULT = OP1 * OP2
MUL16:
    clrf    RESULT+3
    clrf    RESULT+2
    clrf    RESULT+1
    movlw   0x80
    movwf   RESULT

NEXT_BIT:
    rrf	    OP1+1, f
    rrf	    OP1, f

    btfss   CARRY
    goto    NO_BIT_L
    movf    OP2, w
    addwf   RESULT+1, f

    movf    OP2+1, w
    btfsc   CARRY
    incfsz  OP2+1, w
    addwf   RESULT+2, f
    btfsc   CARRY
    incf    RESULT+3, f
    bcf	    CARRY

NO_BIT_L:
    btfss   OP1, 7
    goto    NO_BIT_H
    movf    OP2, w
    addwf   RESULT+2, f
    movf    OP2+1, w
    btfsc   CARRY
    incfsz  OP2+1, w
    addwf   RESULT+3, f

NO_BIT_H:
    rrf	    RESULT+3, f
    rrf	    RESULT+2, f
    rrf	    RESULT+1, f
    rrf	    RESULT, f

    btfss   CARRY
    goto    NEXT_BIT

    return
