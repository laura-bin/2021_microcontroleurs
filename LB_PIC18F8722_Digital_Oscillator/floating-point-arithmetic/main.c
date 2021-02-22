/** ***************************************************************************
 * Digital oscillator
 * ==================
 * 
 * Digital oscillator numerique sur MCP4922
 * 
 * Representation de la sinusoide
 * ------------------------------
 * sampling_frequency   frequence d'echantillonnage (Shannon)
 * sampling_period      periode d'echantillonnage
 *                      sampling_period = 1.0 / sampling_frequency
 * frequency            frequence utile du signal
 *                      sampling_frequency >= 2.0 * frequency (theoreme de Shannon)
 * K                    frequence numerique
 *                      K = frequency / sampling_frequency et 0.0 <= K <= 0.5
 * omega                pulsation
 *                      omega = 2.0 * pi * K
 * B                    coefficient de l'oscillateur
 *                      B = 2.0 * cos(omega)
 * Y0                   signal a l'instant courant
 *                      t
 * Y1                   signal a l'instant precedent
 *                      t - 1 * Ts
 * Y2                   signal a l'instant precedent
 *                      t - 2 * Ts
 * 
 * Conditions initiales
 * --------------------
 * Y0 = 0.0
 * Y1 = cos(-1.0 * omega)
 * Y2 = cos(-2.0 * omega)
 * 
 * Equation de l'oscillateur
 * -------------------------
 * Y0 = B * Y1 - Y2
 * 
 * 
 * PIC18F8722
 * Proteus: LB_PIC18F8722_Digital_Oscillator (10MHz)
 * 
 * Microcontroleurs 2021 - Laura Binacchi
 ******************************************************************************/

// CONFIG1H
#pragma config OSC = HSPLL      // Oscillator Selection bits (HS oscillator)
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
#include <math.h>
#include <stdio.h>

#include "config.h"
#include "LM044L_MCP23S17_SPI.h"
#include "SPI.h"

/* Global variables (used by interruption & main program) */
unsigned char num_Y0;   // casted Y0 value
double B, Y0, Y1, Y2;

void __interrupt(high_priority) Int_Vect_High(void) {
    TICK = 1;
    
    // Scaling Y0 to DAC waited values (8 bits -> 0-255)
    num_Y0 = (unsigned char) ceil(125.0 * Y0 + 128.0);
    
    // Send casted Y0 to MCP4922 DACA via SPI bus (mode x2)
    SPI_DAC_CS = 0;
    SSPIF = 0;
    SSPBUF = 0x10;
    while (!SSPIF);
    SSPIF = 0;
    SSPBUF = num_Y0;
    while (!SSPIF);
    SPI_DAC_CS = 1;
    
    // Compute next Y0 value and store previous values in Y1 & Y2
    Y0 = B * Y1 - Y2;
    Y2 = Y1;
    Y1 = Y0;
    
    // Update MCP4922 outputs
    DAC_LDAC = 0; 
    DAC_LDAC = 1;
    
    TICK = 0;
    
    PIR2bits.CCP2IF = 0;
}

/**
 * Main program : initializations and infinite loop
 */
void main(void) {
    char text[21];
    double sampling_frequency, frequency, omega;
    
    // Compare mode: trigger special event
    CCP2CON = 0x0B;
    
    // Timer3 OFF
    T3CON = 0x00;
    
    // Timer1 prescale 1:1
    T1CON = 0x01;
    
    // 500us -> 2kHz
    CCPR2H = 0x13;
    CCPR2L = 0x88;
    
    // Timer1 -> 0
    TMR1H = 0;
    TMR1L = 0;
    
    // Control tick -> 0
    TICK_TRIS = 0;
    TICK = 0;
    
    // LDAC -> 1
    DAC_LDAC_TRIS = 0;
    DAC_LDAC = 1;
    
    // Frequency hexadecimal switch -> input
    FREQUENCY_TRIS = 0xFF;
    
    // Sampling frequency -> 2kHz (single output, floating-point arithmetic)
    sampling_frequency = 2000.0;
    
    // Frequency (max 1kHz, not effective with low frequencies)
    frequency = 100 + FREQUENCY;
    
    // Initialization of the oscillator's variables
    omega = frequency / sampling_frequency * 2.0 * M_PI;
    B = 2.0 * cos(omega);
    Y0 = 0.0;
    Y1 = cos(-1.0 * omega);
    Y2 = cos(-2.0 * omega);
            
    // SPI bus initialization
    init_SPI();
    
    // LCD initialization
    init_LCD();
    send_text_LCD("DIGITAL OSCILLATOR", 1);
    sprintf(text, "Sampling%10.0fHz", sampling_frequency);
    send_text_LCD(text, 2);
    sprintf(text, "Frequency%9.0fHz", frequency);
    send_text_LCD(text, 3);
    // sprintf(text, "K %2.2f", K);
    // send_text_LCD(text, 4);

    // Interruption on CCP2 (high priority)
    RCONbits.IPEN = 1;
    IPR2bits.CCP2IP = 1;
    PIR2bits.CCP2IF = 0;
    PIE2bits.CCP2IE = 1;
    INTCONbits.GIEL = 0;
    INTCONbits.GIEH = 1;

    while (1);
}
