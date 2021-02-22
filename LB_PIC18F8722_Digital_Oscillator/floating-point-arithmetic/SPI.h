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

/* SSP interrupt flag */
#define SSPIF       PIR1bits.SSP1IF

/* SPI connections */
#define SPI_SCK	    PORTCbits.RC3   // clock
#define SPI_SDI	    PORTCbits.RC4   // data in
#define SPI_SDO	    PORTCbits.RC5   // data out
#define SPI_LCD_CS  PORTCbits.RC6   // LCD chip select
#define SPI_DAC_CS  PORTCbits.RC7   // Digital to analog converter chip select

/* SPI TRIS registers used for initialization */
#define SPI_SCK_TRIS    TRISCbits.TRISC3
#define SPI_SDI_TRIS    TRISCbits.TRISC4
#define SPI_SDO_TRIS    TRISCbits.TRISC5
#define SPI_LCD_CS_TRIS TRISCbits.TRISC6
#define SPI_DAC_CS_TRIS TRISCbits.TRISC7

/**
 * Initializes the SPI bus
 */
void init_SPI(void);
