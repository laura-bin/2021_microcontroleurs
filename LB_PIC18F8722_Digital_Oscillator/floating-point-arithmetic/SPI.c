/** ***************************************************************************
 * LCD library for PIC18F8722
 * ==========================
 *
 * Manages the communication with the alphanumeric LCD LM044L
 * via the MCP23S17 I/O expander and using the SPI bus
 *
 * Microcontroleurs 2021 - Laura Binacchi
 ******************************************************************************/

#include <xc.h>

#include "config.h"
#include "SPI.h"

void init_SPI(void) {
    SPI_SCK_TRIS = 0;
    SPI_SDI_TRIS = 1;
    SPI_SDO_TRIS = 0;
    SPI_LCD_CS_TRIS = 0;
    SPI_DAC_CS_TRIS = 0;
    
    SSP1STAT = 0x00;
    SSP1STATbits.CKE = 1;
    SSP1CON1 = 0x00;
    SSP1CON1bits.SSPEN = 1;
    
    SPI_LCD_CS = 1;
    SPI_DAC_CS = 1;
}
