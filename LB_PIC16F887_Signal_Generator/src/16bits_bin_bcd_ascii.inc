; Hexadecimal to ASCII and to BCD ASCII functions and variables
; used to display hexadecimal values, converted to BCD or not, on an LCD display
; 


; Functions
; =========
EXTRN	HEX_TO_ASCII	    ; 8 bits hexadecimal to ASCII conversion
EXTRN	HEX8_TO_BCD_ASCII   ; 8 bits hexadecimal to BCD ASCII conversion
EXTRN	HEX16_TO_BCD_ASCII  ; 16 bits hexadecimal to BCD ASCII conversion


; Variables
; =========
EXTRN	HEX0		; ASCII hexadecimal least significant nibble
EXTRN	HEX1		; ASCII hexadecimal most significant nibble
EXTRN	BCD0		; ASCII decimal units
EXTRN	BCD1		; ASCII decima l tens
EXTRN	BCD2		; ASCII decimal hundreds
EXTRN	BCD3		; ASCII decimal thousands
EXTRN	BCD4		; ASCII decimal ten thousands
EXTRN	VAR16		; 16 bits variable
