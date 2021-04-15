
#include <xc.inc>

#include "16bits_arithmetic.inc"


; Functions
; =========

; convert a 8 bits hexadecimal value stored in the W register to HEX1-0 ASCII characters
global	HEX_TO_ASCII

; convert a 8 bits hexadecimal value stored in the W register to BCD2-0 ASCII characters
global	HEX8_TO_BCD_ASCII

; convert a 16 bits hexadecimal value stored in the VAR16 variable to BCD4-0 ASCII characters
global	HEX16_TO_BCD_ASCII

; Variables
; =========
global  HEX0		; ASCII hexadecimal least significant nibble
global  HEX1		; ASCII hexadecimal most significant nibble
global  BCD0		; ASCII decimal units
global  BCD1		; ASCII decimal tens
global  BCD2		; ASCII decimal hundreds
global  BCD3		; ASCII decimal thousands
global  BCD4		; ASCII decimal ten thousands
global  VAR16		; 16 bits variable


PSECT udata_bank0
HEX0:	DS 1
HEX1:	DS 1
BCD0:	DS 1
BCD1:	DS 1
BCD2:	DS 1
BCD3:	DS 1
BCD4:	DS 1
VAR16:	DS 2


PSECT code

; 8 bits hexadecimal to ASCII characters
; W -> HEX1-0
; ======================================
HEX_TO_ASCII:
    ; Extract nibbles in MSH & LSH
    movwf	HEX1
    movwf	HEX0
    ; Extract most significant nibble
    movlw	0xF0
    andwf	HEX1, f
    swapf	HEX1, f
    ; Extract least significant nibble
    movlw	0x0F
    andwf	HEX0, f
    
    ; Convert to ASCII
    btfss	HEX1, 3
    goto	HEX1_09	    ; if the nibble starts with 0xxx its value is between 0 and 7
    btfsc	HEX1, 2
    goto	HEX1_AF	    ; if it starts with 11xx, between C and F
    btfsc	HEX1, 1
    goto	HEX1_AF	    ; if it starts with 101x, between A and B
			    ; between 8 and 9

; move the appropriate value to the W register
HEX1_09:
    movlw	0x30
    goto	HEX1_TO_ASCII
HEX1_AF:
    movlw	0x37

; and add it to the nibble value to convert it to the ASCII character
HEX1_TO_ASCII:
    addwf	HEX1, f

; repeat the same operation for the least significant nibble
    btfss	HEX0, 3
    goto	HEX0_09
    btfsc	HEX0, 2
    goto	HEX0_AF
    btfsc	HEX0, 1
    goto	HEX0_AF
    
HEX0_09:
    movlw	0x30
    goto	HEX0_TO_ASCII
HEX0_AF:
    movlw	0x37
HEX0_TO_ASCII:
    addwf	HEX0, f
    return


; 8 bits hexadecimal to ASCII characters
; W -> BCD2-0
; ======================================
HEX8_TO_BCD_ASCII:
    clrf	BCD2
    clrf	BCD1
    movwf	BCD0

HEX8_COUNT_HUNDREDS:
    movlw	100		; substract 100 to the units
    subwf	BCD0, w
    btfss	CARRY		; if the result is lower than 0
    goto	HEX8_COUNT_TENS	; then compute the tens
    movwf	BCD0		; else continue the count
    incf	BCD2, f
    goto	HEX8_COUNT_HUNDREDS

HEX8_COUNT_TENS:
    movlw	10		; substract 10 to the units
    subwf	BCD0, w
    btfss	CARRY		; if the result is lower than 0
    goto	BCD8_TO_ASCII	; then convert the result to ascii
    movwf	BCD0		; else continue the count
    incf	BCD1, f
    goto	HEX8_COUNT_TENS


; 16 bits hexadecimal to ASCII characters
; VAR16 -> BCD4-0
; ======================================
HEX16_TO_BCD_ASCII:
    clrf	BCD4
    clrf	BCD3
    clrf	BCD2
    clrf	BCD1
    clrf	BCD0

HEX16_COUNT_TEN_THOUSANDS:
    SUBI16	VAR16, 10000		; substract 10000 to VAR16
    btfss	CARRY			; if the result is lower than 0
    goto	HEX16_COUNT_THOUSANDS	; count the thousands
    incf	BCD4, f			; else continue the count
    goto	HEX16_COUNT_TEN_THOUSANDS

HEX16_COUNT_THOUSANDS:
    ADDI16	VAR16, 10000
HEX16_COUNT_THOUSANDS_LOOP:
    SUBI16	VAR16, 1000		; substract 1000 to VAR16
    btfss	CARRY			; if the result is lower than 0
    goto	HEX16_COUNT_HUNDREDS	; count the hundreds
    incf	BCD3, f			; else continue the count
    goto	HEX16_COUNT_THOUSANDS_LOOP

HEX16_COUNT_HUNDREDS:
    ADDI16	VAR16, 1000
HEX16_COUNT_HUNDREDS_LOOP:
    SUBI16	VAR16, 100		; substract 100 to VAR16
    btfss	CARRY			; if the result is lower than 0
    goto	HEX16_COUNT_TENS	; count the tens
    incf	BCD2, f			; else continue the count
    goto	HEX16_COUNT_HUNDREDS_LOOP


HEX16_COUNT_TENS:
    ADDI16	VAR16, 100
    movf	VAR16, w
    movwf	BCD0
HEX16_COUNT_TENS_LOOP:
    movlw	10		; substract 10 to the units
    subwf	BCD0, w
    btfss	CARRY		; if the result is lower than 0
    goto	BCD16_TO_ASCII	; then convert the result to ascii
    movwf	BCD0		; else continue the count
    incf	BCD1, f
    goto	HEX16_COUNT_TENS_LOOP

; convert BCD 4-3 to ASCII digits
BCD16_TO_ASCII:
    movf	BCD4, w
    xorlw	0x30
    movwf	BCD4

    movf	BCD3, w
    xorlw	0x30
    movwf	BCD3

; convert BCD 2-0 to ASCII digits
BCD8_TO_ASCII:
    movf	BCD2, w
    xorlw	0x30
    movwf	BCD2

    movf	BCD1, w
    xorlw	0x30
    movwf	BCD1

    movf	BCD0, w
    xorlw	0x30
    movwf	BCD0
    return
