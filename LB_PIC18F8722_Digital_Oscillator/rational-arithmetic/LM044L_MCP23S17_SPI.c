/** ***************************************************************************
 * LCD library for PIC18F8722 (10MHz HSPLL)
 * ========================================
 *
 * Manages the communication with the alphanumeric LCD LM044L
 * via the MCP23S17 I/O expander and using the SPI bus
 *
 * Microcontroleurs 2021 - Laura Binacchi
 ******************************************************************************/

#include <xc.h>

#include "config.h"
#include "LM044L_MCP23S17_SPI.h"
#include "SPI.h"

/* PRIVATE FUNCTION */

/**
 * Sends a data byte to the MCP23S17 I/O expander
 * 
 * @param address
 * @param data
 */
void send(char address, char data) {
    SPI_LCD_CS = 0;
    
    SSPIF = 0;
    SSP1BUF = 0x40;
    while(!SSPIF);
    
    SSPIF = 0;
    SSP1BUF = address;
    while(!SSPIF);
    
    SSPIF = 0;
    SSP1BUF = data;
    while(!SSPIF);
    
    SPI_LCD_CS = 1;
}

/* HEADER IMPLEMENTATION */

void init_LCD(void) {
    // MCP23S17 initialization
    send(MCP_IODIRA, 0x00);
    send(MCP_GPIOA, 0x00);
    send(MCP_IODIRB, 0x00);
    send(MCP_GPIOB, 0x00);
    
    // LM044L initialization
    send_command_LCD(0x33);
    __delay_ms(10);
    send_command_LCD(0x33);
    __delay_ms(10);
    send_command_LCD(0x38);
    __delay_ms(10);
    send_command_LCD(CURSOR_OFF);
    send_command_LCD(0x06);
    
    send_command_LCD(LCD_CLEAR);
    send_command_LCD(LCD_LINE1);
}

void send_command_LCD(char command) {
    send(MCP_GPIOA, 0x00);      // RS -> 0 - EN -> 0
    send(MCP_GPIOB, command);   // send the command
    send(MCP_GPIOA, LCD_E);     // RS -> 0 - EN -> 1
    send(MCP_GPIOA, 0x00);      // RS -> 0 - EN -> 0
    __delay_ms(4);
}

void send_char_LCD(char ch) {
    send(MCP_GPIOA, LCD_RS);    // RS -> 1 - EN -> 0
    send(MCP_GPIOB, ch);        // send the character
    send(MCP_GPIOA, LCD_RS_E);  // RS -> 1 - EN -> 1
    send(MCP_GPIOA, LCD_RS);    // RS -> 1 - EN -> 0
    __delay_us(4);
}

void send_text_LCD(char *text, int line) {
    switch (line) {
        case 1:
            send_command_LCD(LCD_LINE1);
            break;
        case 2:
            send_command_LCD(LCD_LINE2);
            break;
        case 3:
            send_command_LCD(LCD_LINE3);
            break;
        case 4:
            send_command_LCD(LCD_LINE4);
            break;
        default:
            break;
    }
    
    while(*text) {
        send_char_LCD(*text);
        text++;
    }
}