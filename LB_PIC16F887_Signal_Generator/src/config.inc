


; I/O expander PCA9535 address: 0 0 1 0 A2 A1 A0 R/W
#define PCA_LCD_ADD         0x40

; I/O expander PCA9535 commands
#define PCA_PORT0_CONFIG    0x06
#define PCA_PORT1_CONFIG    0x07
#define PCA_PORT0_OUT       0x02    // connected to LCD RS & EN PINs
#define PCA_PORT1_OUT       0x03    // connected LCD data PINs

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
