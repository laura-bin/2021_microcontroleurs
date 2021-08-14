/** ***************************************************************************
 * Dossier filtres sur PIC18F8722
 * ==============================
 * 
 * Moving average filter:
 *  - sampling frequency (8 or 16kHz)
 *  - value (2, 4 or 8)
 * 
 * Low-pass filter:
 *  - sampling frequency (8 or 16kHz)
 *  - cutoff frequency
 * 
 * High-pass filter:
 *  - sampling frequency (8 or 16kHz)
 *  - cutoff frequency
 * 
 * Echo filter:
 *  - sampling frequency (8 or 16kHz)
 *  - delay
 *  - number of echoes
 * 
 * PIC18F8722
 * Proteus: PIC18F8722_Filtres (10MHz HSPLL)
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

#include "LM044L_MCP23S17_SPI.h"

#define DAC0808 PORTD                // output signal on the DAC0808
#define SIG_IN  PORTAbits.RA0        // input signal

#define SW_RUN          PORTEbits.RE0   // RUN / CONFIG mode switch
#define PB_PREV_MENU    PORTEbits.RE1   // previous menu entry selection push button (NC)
#define PB_NEXT_MENU    PORTEbits.RE2   // next menu entry selection push button (NC)
#define PB_VALUE_DN     PORTEbits.RE3   // value down push button (NC)
#define PB_VALUE_UP     PORTEbits.RE4   // value up push button (NC)

#define TICK            PORTGbits.RG0   // debug tick measuring the processing time of the interruption

// running modes
#define MODE_NONE       0       // no mode selected
#define MODE_RUN        1       // run mode
#define MODE_CONFIG     2       // filter configuration mode

// menu entries
#define M_FILTER        0       // filter type selection
#define M_SAMPLING      1       // sampling frequency selection
#define M_VALUE         2       // filter value selection
#define M_VALUE2        3       // filter second value selection (used by the echo filter)
#define M_COUNT         3       // count of menu entries
#define M_ECHO_COUNT    4       // count of menu entries for the echo filter

// filters
#define F_MOV_AVG       0       // moving average filter
#define F_LOW_PASS      1       // low-pass filter
#define F_HIGH_PASS     2       // high-pass filter
#define F_ECHO          3       // echo filter
#define F_COUNT         4       // count of filters

// limit values
#define F_MOV_AVG_MIN   1       // moving average filter min value (2^1)
#define F_MOV_AVG_MAX   3       // moving average filter max value (2^3)
#define F_LOW_PASS_MIN  0       // low-pass filter min value
#define F_HIGH_PASS_MIN 0       // high-pass filter min value
#define F_ECHO_MIN      50      // echo filter min delay value
#define F_ECHO_MAX      500     // echo filter max delay value
#define F_ECHO_MIN2     1       // echo filter min number of echoes
#define F_ECHO_MAX2     3       // echo filter max number of echoes
#define SAMPLING_MIN    8000    // sampling frequency min value
#define SAMPLING_MAX    16000   // sampling frequency max value

// steps
#define F_HL_PASS_STEP  100
#define F_ECHO_STEP     10

#define BUF_SIZE        8       // signal buffer size

/* Global variables (used by interruption & main program) */
unsigned char sig_in[BUF_SIZE];     // input buffer
unsigned char sig_out[BUF_SIZE];    // output buffer
int index;                          // buffer index
unsigned temp_out;                  // temporary variable used to compute the output signal value

char prev_mode;             // previous run/config mode value
char menu_entry;            // current menu entry selected
char filter;                // current filter selected
unsigned sampling;          // sampling frequency (8000 or 16000 Hz)
char mov_avg_coef;          // moving average filter coefficient (1-3 -> 2, 4 or 8)
unsigned low_cutoff;        // low-pass filter cutoff frequency
unsigned high_cutoff;       // high-pass filter cutoff frequency
unsigned echo_delay;        // echo filter delay
char echoes;                // echo filter number of echoes (1, 2 or 3)

// update and display the parameters
void display_parameters(void);
void update_sampling_frequency(unsigned new_val);
void update_mov_avg_coef(char new_val);
void update_low_cutoff(unsigned new_val);
void update_high_cutoff(unsigned new_val);
void update_echo_delay(unsigned new_val);
void update_echoes(char new_val);

void init_signal() {
    double frequency, omega;
    int i;
    index = 0;
    for (i = 0; i < 1<<mov_avg_coef; i++) sig_in[i] = 0;
    }

void __interrupt(high_priority) Int_Vect_High(void) {
    unsigned char i;

    TICK = 1;

    // analog to digital conversion, result in X0
    ADCON0bits.NOT_DONE = 1;
    while (ADCON0bits.NOT_DONE);
    sig_in[index] = ADRESH;
        index++;
        if (index == 1<<mov_avg_coef) index = 0;

        temp_out = 0;
        for (i = 0; i < 1<<mov_avg_coef; i++) temp_out += sig_in[i];

        DAC0808 = (unsigned char) (temp_out >> mov_avg_coef);

    TICK = 0;
    PIR2bits.CCP2IF = 0;
}

/**
 * Main program : initializations and infinite loop
 */
void main(void) {
    TRISD = 0x00;

    TRISE = 0xFF;   // PORTE (menu buttons) -> input
    TRISF = 0xFF;
    CMCON = 0x07;

    // AD converter
    ADCON0 = 0X01;      // AN0, ADC ON
    ADCON1 = 0x0B;      // AN3-0 -> analogic
    ADCON2 = 0x01;      // Left justification, 0 Tad, 8 Tosc

    TRISGbits.TRISG0 = 0;
    TRISGbits.TRISG4 = 0;
    PORTGbits.RG0 = 0;

    // CCP2 interruption
    CCP2CON = 0x0B;     // Compare mode: trigger special event
    T3CON   = 0x00;     // Timer3 OFF
    T1CON   = 0x01;     // Timer1 prescale 1:8
    CCPR2H  = 0x04;     // 1250us -> 8kHz
    CCPR2L  = 0xE2;
    TMR1H   = 0;        // Timer1 -> 0
    TMR1L   = 0;

    // initialize the menu default parameters values
    prev_mode = MODE_NONE;              // previous mode selected: none
    sampling = (unsigned) (10000000 / ((CCPR2H << 8) + CCPR2L));    // sampling frequency determined by the CCP2 value
    filter = F_MOV_AVG;                 // filter selected: moving average
    mov_avg_coef = F_MOV_AVG_MIN;       // moving average value: minimum
    low_cutoff = F_LOW_PASS_MIN;        // low-pass cutoff: minimum
    high_cutoff = sampling;             // high-pass cutoff: maximum
    echo_delay = F_ECHO_MIN;            // echo delay: minimum
    echoes = F_ECHO_MIN2;               // number of echoes: minimum

    // LCD initialization
    init_LM044L();
    display_parameters();

    // Interruption on CCP2 (high priority)
    RCONbits.IPEN   = 1;
    IPR2bits.CCP2IP = 1;
    PIR2bits.CCP2IF = 0;
    PIE2bits.CCP2IE = 1;
    INTCONbits.GIEL = 0;
    INTCONbits.GIEH = 1;
    
    while (1) {
        if (SW_RUN) {
            if (prev_mode != MODE_RUN) {            // run mode initialization:
                prev_mode = MODE_RUN;               // set the previous mode to RUN
                send_text_LCD(" ", menu_entry, 19); // erase the menu selector
                init_signal();
                INTCONbits.GIEH = 1;                // activate the interruption
            }
        } else {
            if (prev_mode != MODE_CONFIG) {         // config mode initialization: 
                INTCONbits.GIEH = 0;                // deactivate the interruption
                prev_mode = MODE_CONFIG;            // set the previous mode to CONFIG
                menu_entry = M_FILTER;              // select the first menu
                send_text_LCD("<", menu_entry, 19); // display the menu selector
            }

            if (!PB_PREV_MENU) {                    // previous menu selection:
                while (!PB_PREV_MENU);              // wait the button release
                send_text_LCD(" ", menu_entry, 19); // erase the menu selector
                if (!menu_entry) menu_entry = filter == F_ECHO ? M_ECHO_COUNT-1 : M_COUNT-1;
                else menu_entry--;                  // select the previous menu
                send_text_LCD("<", menu_entry, 19); // display the menu selector

            } else if (!PB_NEXT_MENU) {             // next menu selection:
                while (!PB_NEXT_MENU);              // wait the button release
                send_text_LCD(" ", menu_entry, 19); // erase the menu selector
                if (filter == F_ECHO && menu_entry == M_ECHO_COUNT-1) menu_entry = 0;
                else if (filter != F_ECHO && menu_entry == M_COUNT-1) menu_entry = 0;
                else menu_entry++;                  // select the next menu
                send_text_LCD("<", menu_entry, 19); // display the menu selector
                
            } else if (!PB_VALUE_DN) {              // value down selection:
                while (!PB_VALUE_DN);               // wait the button release
                switch (menu_entry) {
                case M_FILTER:                      // select the previous filter type
                    if (!filter) filter = F_COUNT-1;
                    else filter--;
                    display_parameters();           // display the related parameters
                    break;
                case M_SAMPLING:                    // decrease the sampling frequency
                    if (sampling == SAMPLING_MAX) {
                        CCPR2H = (unsigned char) (CCPR2H << 1);
                        CCPR2L = (unsigned char) (CCPR2L << 1);
                        update_sampling_frequency(sampling >> 1);
                        if (low_cutoff > sampling) update_low_cutoff(sampling);
                        if (high_cutoff > sampling) update_high_cutoff(sampling);
                    }
                    break;
                case M_VALUE:                       // decrease the filter first value, either
                    switch (filter) {
                    case F_MOV_AVG:                 // the moving average value
                        if (mov_avg_coef > F_MOV_AVG_MIN) update_mov_avg_coef(mov_avg_coef-1);
                        break;
                    case F_LOW_PASS:                // the low-pass filter cutoff frequency
                        if (low_cutoff > F_LOW_PASS_MIN) update_low_cutoff(low_cutoff-F_HL_PASS_STEP);
                        break;
                    case F_HIGH_PASS:               // the high-pass filter cutoff frequency
                        if (high_cutoff > F_HIGH_PASS_MIN) update_high_cutoff(high_cutoff-F_HL_PASS_STEP);
                        break;
                    case F_ECHO:                    // the delay of the echo filter
                        if (echo_delay > F_ECHO_MIN) update_echo_delay(echo_delay-F_ECHO_STEP);
                        break;
                    default:
                        break;
                    }
                    break;
                case M_VALUE2:                      // decrease the filter second value (the number of echoes)
                    if (echoes > F_ECHO_MIN2) update_echoes(echoes-1);
                    break;
                default:
                    break;
                }

            } else if (!PB_VALUE_UP) {                     // value up selection
                while (!PB_VALUE_UP);               // wait the button release
                switch (menu_entry) {
                case M_FILTER:                      // select the next filter type
                    if (filter == F_COUNT-1) filter = 0;
                    else filter++;
                    display_parameters();           // display the related parameters
                    break;
                case M_SAMPLING:                    // increase the sampling frequency
                    if (sampling == SAMPLING_MIN) {
                        CCPR2H = CCPR2H >> 1;
                        CCPR2L = CCPR2L >> 1;
                        update_sampling_frequency(sampling << 1);
                    }
                    break;
                case M_VALUE:                       // increase the filter first value, either
                    switch (filter) {
                    case F_MOV_AVG:                 // the moving average value
                        if (mov_avg_coef < F_MOV_AVG_MAX) update_mov_avg_coef(mov_avg_coef+1);
                        break;
                    case F_LOW_PASS:                // the low-pass filter cutoff frequency
                        if (low_cutoff < sampling) update_low_cutoff(low_cutoff+F_HL_PASS_STEP);
                        break;
                    case F_HIGH_PASS:               // the high-pass filter cutoff frequency
                        if (high_cutoff < sampling) update_high_cutoff(high_cutoff+F_HL_PASS_STEP);
                        break;
                    case F_ECHO:                    // the delay of the echo filter
                        if (echo_delay < F_ECHO_MAX) update_echo_delay(echo_delay+F_ECHO_STEP);
                        break;
                    default:
                        break;
                    }
                    break;
                case M_VALUE2:                      // increase the filter second value (the number of echoes)
                    if (echoes < F_ECHO_MAX2) update_echoes(echoes+1);
                    break;
                default:
                    break;
                }
            }
        }
        __delay_ms(100);
    }
}

void display_parameters() {
    char text[19];
    switch (filter) {
    case F_MOV_AVG:
        send_text_LCD("Moving avg filter ", 0, 0);
        sprintf(text, "Sampling %6d Hz", sampling);
        send_text_LCD(text, 1, 0);
        sprintf(text, "Coefficient %6d", 1 << mov_avg_coef);
        send_text_LCD(text, 2, 0);
        send_text_LCD("                  ", 3, 0);
        break;
    case F_LOW_PASS:
        send_text_LCD("Low-pass filter   ", 0, 0);
        sprintf(text, "Sampling %6d Hz", sampling);
        send_text_LCD(text, 1, 0);
        sprintf(text, "Cutoff %8u Hz", low_cutoff);
        send_text_LCD(text, 2, 0);
        send_text_LCD("                  ", 3, 0);
        break;
    case F_HIGH_PASS:
        send_text_LCD("High-pass filter  ", 0, 0);
        sprintf(text, "Sampling %6d Hz", sampling);
        send_text_LCD(text, 1, 0);
        sprintf(text, "Cutoff %8u Hz", high_cutoff);
        send_text_LCD(text, 2, 0);
        send_text_LCD("                  ", 3, 0);
        break;
    case F_ECHO:
        send_text_LCD("Echo filter       ", 0, 0);
        sprintf(text, "Sampling %6d Hz", sampling);
        send_text_LCD(text, 1, 0);
        sprintf(text, "Delay %9u ms", echo_delay);
        send_text_LCD(text, 2, 0);
        sprintf(text, "Echoes %11d", echoes);
        send_text_LCD(text, 3, 0);
        break;
    default:
        break;
    }
}

void update_sampling_frequency(unsigned new_val) {
    char text[19];
    sampling = new_val;
    sprintf(text, "%6d", sampling);
    send_text_LCD(text, 1, 9);
}

void update_mov_avg_coef(char new_val) {
    char text[19];
    mov_avg_coef = new_val;
    if (filter == F_MOV_AVG) {
        sprintf(text, "%6d", 1 << mov_avg_coef);
        send_text_LCD(text, 2, 12);
    }
}

void update_low_cutoff(unsigned new_val) {
    char text[19];
    low_cutoff = new_val;
    if (filter == F_LOW_PASS) {
        sprintf(text, "%8u", low_cutoff);
        send_text_LCD(text, 2, 7);
    }
}

void update_high_cutoff(unsigned new_val) {
    char text[19];
    high_cutoff = new_val;
    if (filter == F_HIGH_PASS) {
        sprintf(text, "%8u", high_cutoff);
        send_text_LCD(text, 2, 7);
    }
}

void update_echo_delay(unsigned new_val) {
    char text[19];
    echo_delay = new_val;
    if (filter == F_ECHO) {
        sprintf(text, "%9u", echo_delay);
        send_text_LCD(text, 2, 6);
    }
}

void update_echoes(char new_val) {
    char text[19];
    echoes = new_val;
    if (filter == F_ECHO) {
        sprintf(text, "%11d", echoes);
        send_text_LCD(text, 3, 7);
    }
}
