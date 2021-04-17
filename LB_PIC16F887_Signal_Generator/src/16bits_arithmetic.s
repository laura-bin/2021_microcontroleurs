; 16 bits arithmetic unsigned multiplication and division

#include <xc.inc>

; Functions
; =========
global MUL8	    ; 8 bits unsigned multiplication
global DIV16	    ; 16 bits unsigned division
global MUL16	    ; 16 bits unsigned multiplication

; Variables
; =========
global	OPL8	    ; left operand (8 bits)
global	OPR8	    ; right operand (8 bits)
global	OPL16	    ; left operand (16 bits)
global	OPR16	    ; right operand (16 bits)
global	RESULT16    ; result (16 bits)
global	RESULT32    ; result (32 bits)
global	TEMP16	    ; temporary variable (16 bits)


PSECT udata_bank0
OPL8:	    DS 1
OPR8:	    DS 1
OPL16:	    DS 2
OPR16:	    DS 2
RESULT16:   DS 2
RESULT32:   DS 4
TEMP8:	    DS 1
TEMP16:	    DS 2


PSECT code

; RESULT16 = OP_LEFT8 * OP_RIGHT8
MUL8:
    clrf    RESULT16
MUL8LOOP:
    movf    OPL8, w
    btfsc   OPR8, 0
    addwf   RESULT16
    bcf	    CARRY
    rrf	    OPR8, f
    bcf	    CARRY
    rlf	    OPL8, f
    movf    OPR8, f
    btfss   ZERO
    goto    MUL8LOOP
    return


; Division: RESULT32 = OP_LEFT16 / OP_RIGHT16
DIV16:
    movf    OPR16, f
    btfss   ZERO
    goto    ZERO_TEST_SKIPPED
    movf    OPR16+1, f
    btfsc   ZERO
    return

SUBV16:
    movf    OPR16+1, w
    movwf   TEMP8
    movf    OPR16, w
    subwf   OPL16
    btfss   CARRY
    incf    TEMP8, f
    movf    TEMP8, w
    subwf   OPL16+1
    return

ADDV16:
    movf    OPR16, w
    addwf   OPL16
    btfsc   CARRY
    incf    OPL16+1, f
    movf    OPR16+1, w
    addwf   OPL16+1
    return

ZERO_TEST_SKIPPED:
    movlw   1
    movwf   TEMP16
    clrf    TEMP16+1
    clrf    RESULT32
    clrf    RESULT32+1
SHIFT_IT16:
    bcf	    CARRY
    rlf	    TEMP16, f
    rlf	    TEMP16+1, f
    bcf	    CARRY
    rlf	    OPR16, f
    rlf	    OPR16+1, f
    btfss   OPR16+1, 7
    goto    SHIFT_IT16
DIVU16LOOP:
    call    SUBV16
    btfsc   CARRY
    goto    COUNTX
    call    ADDV16
    goto    FINALX
COUNTX:
    movf    TEMP16, w
    addwf   RESULT32
    btfsc   CARRY
    incf    RESULT32+1, f
    movf    TEMP16+1, w
    addwf   RESULT32+1
FINALX:
    bcf	    CARRY
    rrf	    OPR16+1, f
    rrf	    OPR16, f
    bcf	    CARRY
    rrf	    TEMP16+1, f
    rrf	    TEMP16, f
    btfss   CARRY
    goto    DIVU16LOOP
    return


; Multiplication: RESULT32 = OP_LEFT16 * OP_RIGHT16
MUL16:
    clrf    RESULT32+3
    clrf    RESULT32+2
    clrf    RESULT32+1
    movlw   0x80
    movwf   RESULT32

NEXT_BIT:
    rrf	    OPL16+1, f
    rrf	    OPL16, f

    btfss   CARRY
    goto    NO_BIT_L
    movf    OPR16, w
    addwf   RESULT32+1, f

    movf    OPR16+1, w
    btfsc   CARRY
    incfsz  OPR16+1, w
    addwf   RESULT32+2, f
    btfsc   CARRY
    incf    RESULT32+3, f
    bcf	    CARRY

NO_BIT_L:
    btfss   OPL16, 7
    goto    NO_BIT_H
    movf    OPR16, w
    addwf   RESULT32+2, f
    movf    OPR16+1, w
    btfsc   CARRY
    incfsz  OPR16+1, w
    addwf   RESULT32+3, f

NO_BIT_H:
    rrf	    RESULT32+3, f
    rrf	    RESULT32+2, f
    rrf	    RESULT32+1, f
    rrf	    RESULT32, f

    btfss   CARRY
    goto    NEXT_BIT

    return
