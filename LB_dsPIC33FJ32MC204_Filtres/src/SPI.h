#pragma once

/* Crystal frequency */
#define FOSC    20000000ULL
#define FCY     (FOSC/2)

/* SSP interrupt flag */
#define SSPIF           IFS0bits.SPI1IF

/* SPI connections */
#define SPI_SCK         LATCbits.LATC7  // clock
#define SPI_SDI         LATCbits.LATC4  // data in
#define SPI_SDO         LATCbits.LATC5  // data out
#define SPI_LCD_CS      LATCbits.LATC1  // LCD chip select
#define SPI_DAC_CS      LATCbits.LATC2  // LCD chip select

/* SPI TRIS registers used for initialization */
#define SPI_SCK_TRIS    TRISCbits.TRISC7
#define SPI_SDI_TRIS    TRISCbits.TRISC4
#define SPI_SDO_TRIS    TRISCbits.TRISC5
#define SPI_LCD_CS_TRIS TRISCbits.TRISC1
#define SPI_DAC_CS_TRIS TRISCbits.TRISC2

/* MCP4922 connections */
#define LDAC            LATCbits.LATC0      // DAC load PIN
#define LDAC_TRIS       TRISCbits.TRISC0

/* MCP23S17 functions */
#define MCP_IODIRA      0x00
#define MCP_IODIRB      0x01
#define MCP_GPIOA       0x12
#define MCP_GPIOB       0x13

/* LCD RS & EN connections on MCP23S17 PORT0 */
#define LCD_RS          0x01
#define LCD_E           0x02
#define LCD_RS_E        0x03

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
 * Initializes the SPI bus, 
 * then the MCP4922, 
 * then the MCP23S17 I/0 expander via the SPI bus, 
 * then the LM044L LCD via the MCP23S17 I/0 expander
 */
void init_SPI(void);

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
 * @param row: row [0-3]
 * @param col: col [0-19]
 */
void send_text_LCD(char *text, char row, char col);
