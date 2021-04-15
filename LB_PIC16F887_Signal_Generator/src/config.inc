
#define	I2C_SCL	    PORTC, 3	; I2C clock output PIN
#define	I2C_SDA	    PORTC, 4	; I2C data output PIN


; I/O expander PCA9535 address: 0 0 1 0 A2 A1 A0 R/W
#define PCA9535_LCD_ADD_W       0x40	; write mode
;#define PCA9535_LCD_ADD_R       0x41	; read mode
#define PCA9554_KEYPAD_ADD_W	0x42	; write mode
#define PCA9554_KEYPAD_ADD_R	0x43	; read mode

; I/O expander PCA9535 commands
;#define PCA9535_PORT0_IN	0x00
;#define PCA9535_PORT1_IN	0x01
#define PCA9535_PORT0_OUT       0x02	; connected to LCD RS & EN PINs
#define PCA9535_PORT1_OUT       0x03    ; connected LCD data PINs
;#define PCA9535_PORT0_INV	0x04
;#define PCA9535_PORT1_INV	0x05
#define PCA9535_PORT0_CONFIG	0x06
#define PCA9535_PORT1_CONFIG    0x07

; I/O expander PCA9554 commands
#define PCA9554_IN		0x00
#define PCA9554_OUT		0x01
#define PCA9554_INV		0x02
#define PCA9554_CONFIG		0x03

; LCD RS & EN connections on PCA9535 PORT0
#define LCD_RS              0x01
#define LCD_E               0x02
#define LCD_RS_E            0x03

; LCD LM044L commands
#define LCD_CLEAR	    0x01
#define LCD_LINE1	    0x80
#define LCD_LINE2	    0xC0
#define LCD_LINE3	    0x94
#define LCD_LINE4	    0xD4
#define CURSOR_OFF	    0x0C
#define CURSOR_ON	    0x0E
#define CURSOR_BLINK	    0x0F