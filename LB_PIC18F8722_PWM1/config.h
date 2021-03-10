#pragma once
/** ***************************************************************************
 * PIC Configuration
 * =================
 *
 * Microcontroleurs 2021 - Laura Binacchi
 ******************************************************************************/

/* Crystal frequency */
#define _XTAL_FREQ  4000000UL      // 20MHz Clock - 5MHz cycle

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

/* NC push buttons */
#define SET_PWM1            PORTCbits.RC2
#define SET_PWM2            PORTCbits.RC7

/* SWITCHES */
#define PWM1_ON             PORTCbits.RC0
#define PWM2_ON             PORTCbits.RC1
#define T2_PRESCALER0       PORTGbits.RG0
#define T2_PRESCALER1       PORTGbits.RG1

/* Hexadecimal switches */
#define PWM_PERIOD          PORTB
#define PWM1_DUTY_CYCLE     PORTD
#define PWM2_DUTY_CYCLE     PORTE
