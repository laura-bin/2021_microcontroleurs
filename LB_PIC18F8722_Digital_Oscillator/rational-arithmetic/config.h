#pragma once
/** ***************************************************************************
 * PIC Configuration
 * =================
 *
 * Microcontroleurs 2021 - Laura Binacchi
 ******************************************************************************/

/* Crystal frequency */
#define _XTAL_FREQ  40000000UL      // 10MHz Clock - 10MHz cycle

/* LCD RS & EN connections on MCP23S17 PORT0 */
#define LCD_RS              0x01
#define LCD_E               0x02
#define LCD_RS_E            0x03

/* Switches */
#define PWM_RUN             PORTCbits.RC0
#define T2_PRESCALER0       PORTGbits.RG0
#define T2_PRESCALER1       PORTGbits.RG1

/* Frequency hexadecimal switch */
#define FREQUENCY           PORTB
#define FREQUENCY_TRIS      TRISB

/* Control tick */
#define TICK                PORTGbits.RG0
#define TICK_TRIS           TRISGbits.TRISG0

/* DAC load PIN*/
#define DAC_LDAC            PORTGbits.RG4
#define DAC_LDAC_TRIS       TRISGbits.TRISG4
