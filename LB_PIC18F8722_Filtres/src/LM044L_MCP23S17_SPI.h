#pragma once
/** ***************************************************************************
 * LCD library for PIC18F8722
 * ==========================
 *
 * Manages the communication with the alphanumeric LCD LM044L
 * via the MCP23S17 I/O expander and using the SPI bus
 *
 * Microcontroleurs 2021 - Laura Binacchi
 ******************************************************************************/

/* Crystal frequency */
#define _XTAL_FREQ  40000000UL      // 10MHz Clock - 10MHz cycle

/* SSP interrupt flag */
#define SSPIF       PIR1bits.SSP1IF

/* SPI connections */
#define SPI_SCK	    PORTCbits.RC3   // clock
#define SPI_SDI	    PORTCbits.RC4   // data in
#define SPI_SDO	    PORTCbits.RC5   // data out
#define SPI_LCD_CS  PORTCbits.RC6   // LCD chip select

/* SPI TRIS registers used for initialization */
#define SPI_SCK_TRIS    TRISCbits.TRISC3
#define SPI_SDI_TRIS    TRISCbits.TRISC4
#define SPI_SDO_TRIS    TRISCbits.TRISC5
#define SPI_LCD_CS_TRIS TRISCbits.TRISC6

/* LCD RS & EN connections on MCP23S17 PORT0 */
#define LCD_RS              0x01
#define LCD_E               0x02
#define LCD_RS_E            0x03

/* MCP23S17 functions */
#define MCP_IODIRA      0x00
#define MCP_IODIRB      0x01
#define MCP_GPIOA       0x12
#define MCP_GPIOB       0x13

/* LCD functions */
#define LCD_CLEAR       0x01
#define LCD_LINE1       0x80
#define LCD_LINE2       0xC0
#define LCD_LINE3       0x94
#define LCD_LINE4       0xD4
#define CURSOR_OFF      0x0C
#define CURSOR_ON       0x0E
#define CURSOR_BLINK    0x0F

/**
 * Initializes the SPI bus
 */
void init_SPI(void);

/**
 * Initializes the SPI bus, 
 * then the MCP23S17 I/0 expander via the SPI bus,
 * then the LM044L LCD via the MCP23S17 I/0 expander
 */
void init_LM044L(void);

/**
 * Sends a command to the LM044L alphanumeric LCD
 * 
 * @param command
 */
void send_command_LCD(char command);

/**
 * Sends a character to the LM044L alphanumeric LCD
 * 
 * @param ch: character to display
 */
void send_char_LCD(char ch);

/**
 * Sends a text to the LM044L alphanumeric LCD
 * 
 * @param text: text to display
 * @param line: line on which display the text
 */
void send_text_LCD(char *text, int line);
