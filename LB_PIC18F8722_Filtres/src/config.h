#pragma once
/** ***************************************************************************
 * PIC Configuration
 * =================
 *
 * Microcontroleurs 2021 - Laura Binacchi
 ******************************************************************************/


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
