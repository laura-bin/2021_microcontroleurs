; 16 bits arithmetic macros in little endian format
; (LSB stored in VALUE and MSB in (VALUE)+1

; ADD16 & SUB16: http://robotics.mcmanis.com/articles/20130117_pic_math.html


; CMP16 needs REGA (2 bytes) to be declared



; Initialize a variable with a constant 16 bits value
; INIT16 VAR, 0x01FA
; INIT16 VAR, 250
; ===================================================
INIT16	MACRO	VAR, CONST
    movlw   low(CONST)
    movwf   (VAR)
    movlw   high(CONST)
    movwf   (VAR)+1
ENDM

; Clears a variable
; =================
CLR16	MACRO	VAR
    clrf    VAR
    clrf    (VAR)+1
ENDM

; Move SRC to DST
; ===============
MOV16	MACRO	DST, SRC
    movf    (SRC), w
    movwf   (DST)
    movf    (SRC)+1, w
    movwf   (DST)+1
ENDM

; Compare VAL1 to VAL2
;   VAL1 = VAL2 -> Z true
;   VAL1 > VAL2 -> C true
;   VAL1 < VAL2 -> C false
; ========================
CMP16	MACRO	VAL1, VAL2
    MOV16   (VAL1), REGA
    SUB16   REGA, (VAL2)
    movf    REGA, w
    iorwf   REGA+1, w
ENDM

; Logical shift to the right (VAR / 2)
; from MSB to LSB, 0 shifted into the MSB
; =======================================
LSR16	MACRO	VAR
    bcf	    CARRY
    rrf	    (VAR)+1, f
    rrf	    VAR, f
ENDM

; Logical shift to the left (VAR x 2)
; from LSB to MSB, 0 shifted into the LSB
; =======================================
LSL16	MACRO	VAR
    bcf	    CARRY
    rlf	    VAR, f
    rlf	    (VAR)+1, f
ENDM

; Unsigned substraction (subwf equivalent)
;   DST = DST - SRC
; ========================================
SUB16   MACRO   DST, SRC
    movf    (SRC), w        ; Get low byte of subtrahend
    subwf   (DST), f        ; Subtract DST(low) - SRC(low)
    movf    (SRC)+1, w      ; Now get high byte of subtrahend
    btfss   CARRY	    ; If there was a borrow, rather than
    incfsz  (SRC)+1, w      ; decrement high byte of dst we inc src
    subwf   (DST)+1, f      ; Subtract the high byte and we're done
ENDM

; Immediate substraction (sublw equivalent)
;   DST = DST - CONST
; =========================================
SUBI16	MACRO   DST, CONST
    movlw   low(CONST)
    subwf   (DST), f
    movlw   high(CONST)
    btfss   CARRY
    movlw   (high(CONST))+1
    subwf   (DST)+1, f
ENDM

; Addition
; DST = DST + SRC
; ===============
ADD16   MACRO   DST, SRC
    movf    (SRC), w	    ; Get low byte
    addwf   (DST), f	    ; Add to destination
    movf    (SRC)+1, w	    ; Get high byte
    btfsc   CARRY	    ; Check for carry
    incf    (SRC)+1, w	    ; Add one for carry
    addwf   (DST)+1, f	    ; Add high byte into DST *loses carry if W is zero*
ENDM

; Immediate addition
; DST = DST + SRC
; ==================
ADDI16	MACRO   DST, CONST
    movlw   low(CONST)
    addwf   DST, f
    movlw   high(CONST)
    btfsc   CARRY
    movlw   (high(CONST))+1
    addwf   (DST)+1, f
ENDM

; Negate a variable (two's complement)
; ====================================
NEG16	MACRO	DST
    comf    (DST)
    comf    (DST)+1
    INC16   DST
ENDM

; Increment a variable (DST++)
; Z is set on exit
; ============================
INC16	MACRO	DST
    incfsz  (DST), w	    ; Increment low byte
    decf    (DST)+1, f	    ; No carry (negates next step)
    incf    (DST)+1, f	    ; Increment high byte
    movwf   (DST)	    ; Store updated low byte back
    iorwf   (DST)+1, w	    ; Set Z flag
ENDM

; Decrement a variable (DST--)
; Z is set on exit
; ============================
DEC16	MACRO	DST
    decf    (DST), f	    ; Decrement low byte
    incfsz  (DST), w	    ; check for underflow
    incf    (DST)+1, f	    ; Update
    decf    (DST)+1, f	    ; Fixup
    movf    (DST), w	    ; Store updated low byte back
    iorwf   (DST)+1, w	    ; Set Z flag
ENDM
