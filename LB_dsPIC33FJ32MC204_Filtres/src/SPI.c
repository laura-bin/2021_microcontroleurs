#include "SPI.h"

#include <xc.h>
#include <libpic30.h>

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
    SPI1BUF = 0x40;
    while(!SSPIF);
    
    SSPIF = 0;
    SPI1BUF = address;
    while(!SSPIF);
    
    SSPIF = 0;
    SPI1BUF = data;
    while(!SSPIF);
    
    SPI_LCD_CS = 1;
}

/* HEADER IMPLEMENTATION */

void init_SPI(void) {
    // SPI bus initialization
    SPI_SCK_TRIS = 0;
    SPI_SDI_TRIS = 1;
    SPI_SDO_TRIS = 0;
    SPI_LCD_CS_TRIS = 0;
    SPI_DAC_CS_TRIS = 0;

    SSPIF = 0;
    SPI1STAT = 0x0000;

    SPI1CON1bits.CKE = 1;       // clock edge
    SPI1CON1bits.MSTEN = 1;     // SPI main mode
    SPI1CON1bits.SPRE = 6;      // secondary prescale 2:1
    SPI1CON1bits.PPRE = 2;      // primary prescale 4:1

    SPI1STATbits.SPIEN = 1;     // enable SPI

    SPI_LCD_CS = 1;
    SPI_DAC_CS = 1;

    // MCP4922 initialization
    LDAC_TRIS = 0;
    LDAC = 1;

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

void send_text_LCD(char *text, char line, char col) {
    switch (line) {
        case 0:
            send_command_LCD(LCD_LINE1 + col);
            break;
        case 1:
            send_command_LCD(LCD_LINE2 + col);
            break;
        case 2:
            send_command_LCD(LCD_LINE3 + col);
            break;
        case 3:
            send_command_LCD(LCD_LINE4 + col);
            break;
        default:
            break;
    }

    while(*text) {
        send_char_LCD(*text);
        text++;
    }
}
