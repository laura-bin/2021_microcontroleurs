/** ***************************************************************************
 * PWM signal generator
 * ====================
 * 
 * PWM signal generator using ECCP1-4 modules
 * 
 * PIC18F8722
 * Proteus: LB_PIC18F8722_PWM2
 * 
 * Microcontroleurs 2021 - Laura Binacchi
 ******************************************************************************/

// CONFIG1H
#pragma config OSC = HS         // Oscillator Selection bits (HS oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Two-Speed Start-up disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3L
#pragma config MODE = MC        // Processor Data Memory Mode Select bits (Microcontroller mode)
#pragma config ADDRBW = ADDR20BIT// Address Bus Width Select bits (20-bit Address Bus)
#pragma config DATABW = DATA16BIT// Data Bus Width Select bit (16-bit External Bus mode)
#pragma config WAIT = OFF       // External Bus Data Wait Enable bit (Wait selections are unavailable for table reads and table writes)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (ECCP2 input/output is multiplexed with RC1)
#pragma config ECCPMX = PORTE   // ECCP MUX bit (ECCP1/3 (P1B/P1C/P3B/P3C) are multiplexed onto RE6, RE5, RE4 and RE3 respectively)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RG5 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config BBSIZ = BB2K     // Boot Block Size Select bits (1K word (2 Kbytes) Boot Block size)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit Block 1 (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit Block 2 (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit Block 3 (Block 3 (00C000-00FFFFh) not code-protected)
#pragma config CP4 = OFF        // Code Protection bit Block 4 (Block 4 (010000-013FFFh) not code-protected)
#pragma config CP5 = OFF        // Code Protection bit Block 5 (Block 5 (014000-017FFFh) not code-protected)
#pragma config CP6 = OFF        // Code Protection bit Block 6 (Block 6 (01BFFF-018000h) not code-protected)
#pragma config CP7 = OFF        // Code Protection bit Block 7 (Block 7 (01C000-01FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit Block 3 (Block 3 (00C000-00FFFFh) not write-protected)
#pragma config WRT4 = OFF       // Write Protection bit Block 4 (Block 4 (010000-013FFFh) not write-protected)
#pragma config WRT5 = OFF       // Write Protection bit Block 5 (Block 5 (014000-017FFFh) not write-protected)
#pragma config WRT6 = OFF       // Write Protection bit Block 6 (Block 6 (01BFFF-018000h) not write-protected)
#pragma config WRT7 = OFF       // Write Protection bit Block 7 (Block 7 (01C000-01FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-007FFF, 000FFF or 001FFFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR4 = OFF      // Table Read Protection bit Block 4 (Block 4 (010000-013FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR5 = OFF      // Table Read Protection bit Block 5 (Block 5 (014000-017FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR6 = OFF      // Table Read Protection bit Block 6 (Block 6 (018000-01BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR7 = OFF      // Table Read Protection bit Block 7 (Block 7 (01C000-01FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-007FFF, 000FFF or 001FFFh) not protected from table reads executed in other blocks)

#include <xc.h>
#include <stdio.h>

#include "config.h"
#include "LM044L_MCP23S17_SPI.h"

/**
 * Sets the PWM signals parameters
 */
void set_params(void);

/**
 * Displays the PWM signals parameters
 */
void display_params(void);


void main(void) {
    // switches -> input
    TRISCbits.TRISC0 = 1;   // PWM RUN/CONFIG mode
    TRISGbits.TRISG0 = 1;   // timer 2 prescaler bit 0
    TRISGbits.TRISG1 = 1;   // timer 2 prescaler bit 1

    // Hexadecimal switches -> input
    TRISB = 0xFF;           // PWM period
    TRISD = 0xFF;           // PWM duty cycle
    TRISH = 0xFF;           // PWM CCP1CON value

    // PORTE -> digital
    ADCON1 = 0x0F;

    // ECCP1 P1A-D -> output
    TRISCbits.TRISC2 = 0;
    TRISEbits.TRISE6 = 0;
    TRISEbits.TRISE5 = 0;
    TRISGbits.TRISG4 = 0;
    
    // TIMER2
    T2CON = 0x00;
    T2CONbits.TMR2ON = 1;

    // No interruption
    PIR1bits.CCP1IF = 0;
    PIE1bits.CCP1IE = 0;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 0;

    // PWM signals initialization
    set_params();
    
    // LCD initialization
    init_LCD();
    send_text_LCD("PWM SIGNAL GENERATOR", 1);
    display_params();

    // main program: PWM RUN/CONFIG (button polling)
    while (1) {
        if (PWM_RUN) {
            CCP1CON = PWM_CCP1CON;
        } else {
            CCP1CON = 0x00;
            set_params();
            display_params();
        }

        __delay_ms(100);
    }
}

void set_params(void) {
    // set timer2 prescaler
    T2CONbits.T2CKPS0 = T2_PRESCALER0;
    T2CONbits.T2CKPS1 = T2_PRESCALER1;

    // set period
    PR2 = PWM_PERIOD;

    // set duty cycle
    CCPR1L = PWM_DUTY_CYCLE;
}

void display_params(void) {
    char text[21];
    unsigned long frequency = 0;
    unsigned period = 0;
    unsigned duty_cycle = 0;
    unsigned t2_prescaler = 0;

    if (T2_PRESCALER1) {
        t2_prescaler = 16;
    } else {
        if (T2_PRESCALER0) {
            t2_prescaler = 4;
        } else {
            t2_prescaler = 1;
        }
    }

    // period in microseconds
    period = (PR2 + 1) * t2_prescaler;
    
    // duty cycle in percent
    duty_cycle = 100 * CCPR1L / (PR2 + 1);
    
    // frequency in Hz
    frequency = 1000000 / period;

    sprintf(text, "Frequency %8luHz", frequency);
    send_text_LCD(text, 2);

    sprintf(text, "Period %11uus", period);
    send_text_LCD(text, 3);

    sprintf(text, "Duty cycle%9u%%", duty_cycle);
    send_text_LCD(text, 4);
}
