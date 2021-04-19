; Signal array access macros
; needs a define of signal1 (50 bytes in bank 1) and signal 2 (50 bytes in bank 2)
;
; Laura Binacchi - SLP 2020/2021
; ==============================================================================

; Get the element of the signal array at given index
; the value is put in the W register
; ==================================================
GET_SIGNAL  MACRO   INDEX
LOCAL GET_SIGNAL2		    ; LOCAL ensures the uniqueness of the labels defined in the macro 
LOCAL GET_SIGNAL_END		    ; by adding a unique tag to the label address
    movlw   50
    subwf   (INDEX), w  
    btfsc   CARRY		    ; if the index is greater than 50
    goto    GET_SIGNAL2		    ; goto the second part of the array (in bank 2)
    bcf	    STATUS, 7		    ; clear IRP to select bank 1
    movlw   signal1		    ; move the array pointer value
    movwf   FSR			    ; to FSR
    movf    (INDEX), w
    addwf   FSR, f		    ; add the index value to the pointer
    movf    INDF, w		    ; move the data at this position to the W register
    goto    GET_SIGNAL_END
GET_SIGNAL2:
    bsf	    STATUS, 7		    ; set IRP to select bank 2
    movlw   BANKMASK(signal2)	    ; get the correct pointer on the array (7bits -> 8bits)
    movwf   FSR			    ; and move it to FSR
    movlw   50
    subwf   (INDEX), w		    ; adjust index
    addwf   FSR, f		    ; and add the index value to the pointer
    movf    INDF, w		    ; move the data at this position to the W register
GET_SIGNAL_END:
ENDM


; Set the element of the signal array at given index
; ==================================================
SET_SIGNAL  MACRO   INDEX, VALUE
LOCAL SET_SIGNAL2
LOCAL SET_SIGNAL_END
    movlw   50
    subwf   INDEX, w
    btfsc   CARRY		    ; if the index is greater than 50
    goto    SET_SIGNAL2		    ; go to the second part of the array (in bank 2)
    bcf	    STATUS, 7		    ; select bank 1
    movlw   signal1		    ; move the pointer of the array
    movwf   FSR			    ; in FSR
    movf    INDEX, w
    addwf   FSR, f		    ; add the index value		
    movf    VALUE, w
    movwf   INDF		    ; set the value
    goto    SET_SIGNAL_END
SET_SIGNAL2:
    bsf	    STATUS, 7		    ; select bank 2
    movlw   BANKMASK(signal2)	    ; move the pointer of the array
    movwf   FSR			    ; in FSR
    movlw   50
    subwf   INDEX, w		    ; adjust index
    addwf   FSR, f		    ; and add the index value to the pointer
    movf    VALUE, w
    movwf   INDF		    ; set the value
SET_SIGNAL_END:
ENDM
