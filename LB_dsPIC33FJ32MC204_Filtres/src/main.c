/** ***************************************************************************
 * Dossier filtres
 * ===============
 * 
 * dsPIC33FJ32MC204
 * Proteus: dsPIC_Filtres (20MHz)
 * 
 * Microcontroleurs 2021 - Laura Binacchi
 ******************************************************************************/

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = PRI              // Oscillator Mode (Internal Fast RC (FRC) with divide by N)
#pragma config IESO = OFF               // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = HS              // Primary Oscillator Source (Primary Oscillator Enabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow Only One Re-configuration)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Disabled

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = ON                // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

#include <xc.h>
#include <math.h>
#include <stdio.h>
#include "SPI.h"

#define COMP_VAL            64923       // Timer1 value -> 16Hz
#define COEF_SCALE          7       // filters coefficients scale: 2^7
#define TICK                LATCbits.LATC9  // debug tick measuring the processing time of the interruption

#define F_MOV_AVG           0       // moving average filter
#define F_LOW_PASS          1       // low-pass filter
#define F_HIGH_PASS         2       // high-pass filter
#define F_ECHO              3       // echo filter
#define F_COUNT             4       // count of filters

#define IN_BUF_SIZE         2000    // input signal buffer size
#define OUT_BUF_SIZE        1       // output signal buffer size


// Global variables (used by the interruption & the main program)
int sig_in[10];                     // input signal buffer
int sig_out[OUT_BUF_SIZE];          // output signal buffer
int buf_index;                      // input buffer index

char filter;                        // current filter selected
int sampling;                       // sampling frequency (16kHz)
int mov_avg_steps;                  // moving average filter number of steps
int cutoff;                         // low/high-pass filters cutoff frequency
int echo_delay;                     // echo filter delay
int echoes;                         // echo filter number of echoes

// signed char sig_in[IN_BUF_SIZE];    
// signed char sig_out[OUT_BUF_SIZE];  
// int index[F_ECHO_N_MAX+1];          
// int coef[F_ECHO_N_MAX+1];           // coefficients used to compute the output signal

// Display function
void display_parameters(void);                  // display all the parameters

// void __interrupt(high_priority) Int_Vect_High(void) {
//     int i;
//     long output;    // temporary variable used to compute the output signal value

//     TICK = 1;                       // debug tick ON

//     while (ADCON0bits.NOT_DONE);    // analog to digital conversion, result in the input buffer
//     sig_in[index[0]] = (signed char) (ADRESH - 128);

//     output = 0;
//     switch (filter) {
//     case F_MOV_AVG:                 // moving average filter: Yt = SUM(i=0:coef-1) Xt-i / coef
//         for (i = 0; i < 1<<mov_avg_coef; i++) output += sig_in[i];
//         output = output >> mov_avg_coef;
//         index[0]++;                     // increment the input buffer index
//         if (index[0] == 1<<mov_avg_coef) index[0] = 0;
//         break;
//     case F_LOW_PASS:                    // low-pass filter: Yt = A * (Xt + Xt-1) + B * Yt-1
//         output = coef[0]*(sig_in[0] + sig_in[1]) + coef[1]*sig_out[0];
//         output = output >> COEF_SCALE;
//         sig_in[1] = sig_in[0];             // shift the input signal value in the buffer
//         sig_out[0] = (signed char) output; // store the output value in the buffer
//         break;
//     case F_HIGH_PASS:                   // low-pass filter: Yt = A * (Xt - Xt-1) + B * Yt-1
//         output = coef[0]*(sig_in[0] - sig_in[1]) + coef[1]*sig_out[0];
//         output = output >> COEF_SCALE;
//         sig_in[1] = sig_in[0];             // shift the input signal value in the buffer
//         sig_out[0] = (signed char) output; // store the output value in the buffer
//         break;
//     case F_ECHO:                   // echo filter: Yt = A Xt + B Xt-delay + ...
//         for (i = 0; i <= echoes; i++) output += coef[i] * sig_in[index[i]];
//         output = output >> COEF_SCALE;
//         for (i = 0; i <= echoes; i++) {     // increment the input buffer indexes
//             index[i]++;
//             if (index[i] == IN_BUF_SIZE) index[i] = 0;
//         }
//         break;
//     default:
//         break;
//     }


 //    DAC0808 = (unsigned char) (output + 128);

//     TICK = 0;               // debug tick OFF
//     PIR2bits.CCP2IF = 0;    // interruption flag cleared
// }

void __attribute__((interrupt(auto_psv))) _T1Interrupt(void) {
    char high;
    TMR1 = COMP_VAL;
    TICK = 1;
    
    AD1CON1bits.SAMP = 1;
    AD1CON1bits.SAMP = 0;
    while(!AD1CON1bits.DONE);
    
    SPI_DAC_CS = 0;         // send the output signal value to the DAC
    SSPIF = 0;
    SPI1BUF = (ADC1BUF0 >> 8) | 0x30;         // MCP4922 mode x2
    while (!SSPIF);
    SSPIF = 0;
    SPI1BUF = ADC1BUF0;
    while (!SSPIF);
    SPI_DAC_CS = 1;
    LDAC = 0;               // load the DAC
    LDAC = 1;

    // input buffer = ADC1BUF0;

    TICK = 0;
    _T1IF = 0;
}

void main(void) {
    // SPI configuration
    __builtin_write_OSCCONL(OSCCON & ~(1<<6));  // unlock registers
    RPINR20bits.SCK1R = 20;                     // SPI SDI
    RPOR10bits.RP21R = 7;                       // SPI SDO
    RPOR11bits.RP23R = 8;                       // SPI SCK
    __builtin_write_OSCCONL(OSCCON | (1<<6));   // lock registers
    init_SPI();

    // I/O configuration
    TRISAbits.TRISA0 = 1;                       // AN0 -> input
    TRISCbits.TRISC9 = 0;                       // TICK -> output
    TICK = 0;

    // ADC configuration
    AD1CON1bits.ADON = 0;                       // disable ADC
    AD1CON1bits.AD12B = 1;                      // 1 channel, 12 bits
    AD1CON3bits.SAMC = 4;                       // 4TAD
    AD1CON3bits.ADCS = 3;                       // 1TAD = 3TCY
    AD1PCFGLbits.PCFG0 = 0;                     // AN0 -> analog mode
    AD1CON1bits.ADON = 1;                       // enable ADC

    // Timer1 configuration
    T1CON = 0x8000;                             // Timer1 enabled, internal clock, prescale 1:1
    TMR1 = COMP_VAL;
    _T1IP = 7;                                  // Timer1 priority
    _T1IF = 0;                                  // Timer1 interrupt flag cleared
    
    // Filter parameters initialisation
    sampling = 16;
    filter = F_MOV_AVG;
    mov_avg_steps = 2;
    display_parameters();
    
    // init LCD
    // send text to LCD
    // read parameters
    // init filter
    


    
    
    
    
    // Enable Timer1 interrupt
    _T1IE = 1;
    while (1);
    
// __builtin_write_OSCCONL
// Unlocks and writes its argument to OSCCONL.
// Interrupts may need to be disabled for proper operation.


//     int i;
//     char menu_entry;        // current menu entry selected
//     char prev_mode;         // previous mode selected: run, config or none
////     int cutoff_max;         // high/low pass filter max cutoff value determined by the sampling frequency
//     double omega;           // angular frequency used to initialize some filters coefficients
    // char text[20];

    // PIC configuration
//     TRISE   = 0xFF;         // PORTE (menu buttons) -> input
//     TRISGbits.TRISG0 = 0;   // TICK -> output
//     TICK    = 0;            // TICK -> 0
//     TRISD   = 0;            // DAC0808 -> output
//     DAC0808 = 128;          // DAC0808 -> 0 + offset
//     ADCON0  = 0X01;         // ADC enabled on AN0
//     ADCON1  = 0x0E;         // AN0 -> analogic
//     ADCON2  = 0x01;         // Left justification, 0 Tad, 8 Tosc
//     CCP2CON = 0x0B;         // CCP2 interruption in compare mode with trigger special event
//     CCPR2H  = 0x04;         // 1250us -> 8kHz
//     CCPR2L  = 0xE2;
//     T1CON   = 0x01;         // Timer1 prescale 1:1
//     TMR1H   = 0;            // Timer1 -> 0
//     TMR1L   = 0;
//     RCONbits.IPEN   = 1;    // enable priority levels on interrupts
//     INTCONbits.GIEH = 0;    // high priority interrupts disabled
//     INTCONbits.GIEL = 0;    // low priority interrupts disabled
//     IPR2bits.CCP2IP = 1;    // CCP2 interrupt: high priority
//     PIE2bits.CCP2IE = 1;    // CCP2 interrupt enabled
//     PIR2bits.CCP2IF = 0;    // CCP2 interrupt flag cleared
//     init_SPI();             // SPI bus and peripherals initialization

    // Parameters initialization
//     prev_mode       = MODE_NONE;            // previous mode selected: none
//     sampling        = (int) (10000 / ((CCPR2H << 8) + CCPR2L)); // sampling frequency determined by the CCP2 value
//     cutoff_max      = sampling*1000 >> 1;   // low/high pass filter max value determined by the sampling frequency
//     filter          = F_MOV_AVG;            // filter selected: moving average
//     mov_avg_coef    = F_MOV_AVG_MIN;        // moving average value: minimum
 //    low_cutoff      = F_LOW_PASS_MIN;       // low-pass cutoff value: minimum
//     high_cutoff     = cutoff_max;           // high-pass cutoff value: maximum
//     echo_delay      = F_ECHO_DEL_MIN;       // echo delay: minimum
//     echoes          = F_ECHO_N_MIN;         // number of echoes: minimum

    // Moving average filter test: coef 2-4-8 (16kHz)
    // filter          = F_MOV_AVG;
    // CCPR2H          = CCPR2H >> 1;
    // CCPR2L          = CCPR2L >> 1;
    // sampling        = sampling << 1;
    // mov_avg_coef    = 1;
    // mov_avg_coef    = 2;
    // mov_avg_coef    = 3;

    // Moving average filter test: coef 2-4-8 (8kHz)
    // filter          = F_MOV_AVG;
    // mov_avg_coef    = 1;
    // mov_avg_coef    = 2;
    // mov_avg_coef    = 3;

    // Low-pass filter test (16kHz)
    // filter          = F_LOW_PASS;
    // CCPR2H          = CCPR2H >> 1;
    // CCPR2L          = CCPR2L >> 1;
    // sampling        = sampling << 1;
    // low_cutoff      = 0;
    // low_cutoff      = 500;
    // low_cutoff      = 1000;
    // low_cutoff      = 2000;
    // low_cutoff      = 4000;
    // low_cutoff      = 8000;

    // Low-pass filter test (8kHz)
    // filter          = F_LOW_PASS;
    // low_cutoff      = 0;
    // low_cutoff      = 250;
    // low_cutoff      = 500;
    // low_cutoff      = 1000;
    // low_cutoff      = 2000;
    // low_cutoff      = 4000;

    // High-pass filter test (16kHz)
    // filter          = F_HIGH_PASS;
    // CCPR2H          = CCPR2H >> 1;
    // CCPR2L          = CCPR2L >> 1;
    // sampling        = sampling << 1;
    // high_cutoff     = 0;
    // high_cutoff     = 500;
    // high_cutoff     = 1000;
    // high_cutoff     = 2000;
    // high_cutoff     = 4000;
    // high_cutoff     = 8000;

    // High-pass filter test (8kHz)
    // filter          = F_HIGH_PASS;
    // high_cutoff     = 0;
    // high_cutoff     = 250;
    // high_cutoff     = 500;
    // high_cutoff     = 1000;
    // high_cutoff     = 2000;
    // high_cutoff     = 4000;

    // Echo filter test: 1 echo, delay 200ms (16kHz)
    // filter          = F_ECHO;
    // CCPR2H          = CCPR2H >> 1;
    // CCPR2L          = CCPR2L >> 1;
    // sampling        = sampling << 1;
    // echo_delay      = F_ECHO_DEL_MAX;
    // echoes          = F_ECHO_N_MIN;

    // Echo filter test: 1 echo, delay 400-200-100ms (8kHz)
    // filter          = F_ECHO;
    // echoes          = F_ECHO_N_MIN;
    // echo_delay      = F_ECHO_DEL_MAX;
    // echo_delay      = F_ECHO_DEL_MAX>>1;
    // echo_delay      = F_ECHO_DEL_MAX>>2;

    // Echo filter test: 2 echoes, delay 400ms (8kHz)
    // filter          = F_ECHO;
    // echoes          = 2;
    // echo_delay      = F_ECHO_DEL_MAX;

    // Echo filter test: 3 echoes, delay 400ms (8kHz)
    // filter          = F_ECHO;
    // echoes          = F_ECHO_N_MAX;
    // echo_delay      = F_ECHO_DEL_MAX;

//     display_parameters();

    // Main loop: polling on the RUN/CONFIG switch
//     while (1) {
//         if (SW_RUN) {
//             if (prev_mode != MODE_RUN) {            // run mode initialization:
//                 prev_mode = MODE_RUN;                       // set the previous mode to RUN
//                 send_text_LCD(" ", menu_entry, SELECT_POS); // erase the menu selector

//                 index[0] = 0;                               // input buffer index to 0
//                 for (i = 0; i < IN_BUF_SIZE; i++) sig_in[i] = 0;    // input buffer to 0
//                 for (i = 0; i < OUT_BUF_SIZE; i++) sig_out[i] = 0;  // output buffer to 0

//                 switch (filter) {                   // depending on the filter selected:
//                 case F_LOW_PASS:                        // compute the low-pass filter coefficients
//                     omega = 2.0 * M_PI * (double) low_cutoff / (double) sampling / 1000.0;
//                     coef[0] = (int) round(omega / (2.0 + omega) * pow(2.0, COEF_SCALE));
//                     coef[1] = (int) round((2.0 - omega) / (2.0 + omega) * pow(2.0, COEF_SCALE));
                    // sprintf(text, "%4d %4d", coef[0], coef[1]);
                    // send_text_LCD(text, 3, 0);
//                     break;
//                 case F_HIGH_PASS:                       // or compute the low-pass filter coefficients
//                     omega = 2.0 * M_PI * (double)high_cutoff / (double)sampling / 1000.0;
//                     coef[0] = (int) round(2.0 / (2.0 + omega) * pow(2.0, COEF_SCALE));
//                     coef[1] = (int) round((2.0 - omega) / (2.0 + omega) * pow(2.0, COEF_SCALE));
                    // sprintf(text, "%4d %4d", coef[0], coef[1]);
                    // send_text_LCD(text, 3, 0);
//                     break;
//                 case F_ECHO:                            // or initialize the echoes indexes
//                     for (i = 0; i < echoes; i++) index[echoes-i] = IN_BUF_SIZE - echo_delay/(i+1);
//                     switch (echoes) {                   // and the echoes coefficients (total must be 128, 2^COEF_SCALE)
//                     case 1:
//                         coef[0] = 96;   // original signal
//                         coef[1] = 32;   // echo
//                         break;
//                     case 2:
//                         coef[0] = 64;   // original signal
//                         coef[1] = 40;   // first echo
//                         coef[2] = 24;    // second echo
//                         break;
//                     case 3:
//                         coef[0] = 44;   // original signal
//                         coef[1] = 36;   // first echo
//                         coef[2] = 28;   // second echo
//                         coef[3] = 20;    // third echo
//                         break;
//                     default:
//                         break;
//                     }
//                     break;
//                 default:                            // other filters does not need a specific initialization
//                     break;
//                 }
//                 INTCONbits.GIEH = 1;                // enable high priority interrupts
//             }
//         } else {
//             if (prev_mode != MODE_CONFIG) {         // config mode initialization: 
//                 INTCONbits.GIEH = 0;                        // disable high priority interrupts
//                 prev_mode = MODE_CONFIG;                    // set the previous mode to CONFIG
//                 menu_entry = M_FILTER;                      // select the first menu
//                 send_text_LCD("<", menu_entry, SELECT_POS); // display the menu selector
//             }
// 
//             if (!PB_PREV_MENU) {                        // previous menu selection:
//                 while (!PB_PREV_MENU);                      // wait the button release
//                 send_text_LCD(" ", menu_entry, SELECT_POS); // erase the menu selector
//                 if (!menu_entry) menu_entry = filter == F_ECHO ? M_ECHO_COUNT-1 : M_COUNT-1;
//                 else menu_entry--;                          // select the previous menu
//                 send_text_LCD("<", menu_entry, SELECT_POS); // display the menu selector
// 
//  } else if (!PB_NEXT_MENU) {                 // next menu selection:
//                 while (!PB_NEXT_MENU);                      // wait the button release
//                 send_text_LCD(" ", menu_entry, SELECT_POS); // erase the menu selector
//                 if (filter == F_ECHO && menu_entry == M_ECHO_COUNT-1) menu_entry = 0;
//                 else if (filter != F_ECHO && menu_entry == M_COUNT-1) menu_entry = 0;
//                 else menu_entry++;                          // select the next menu
//                 send_text_LCD("<", menu_entry, SELECT_POS); // display the menu selector

//             } else if (!PB_VALUE_DN) {  // value down selection:
//                 while (!PB_VALUE_DN);       // wait the button release
//                 switch (menu_entry) {
//                 case M_FILTER:              // select the previous filter type
//                     if (!filter) filter = F_COUNT-1;
//                     else filter--;
//                     display_parameters();
//                     break;
//                 case M_SAMPLING:            // decrease the sampling frequency
//                     if (sampling == SAMPLING_MAX) {
//                         CCPR2H = (unsigned char) (CCPR2H << 1);
//                         CCPR2L = (unsigned char) (CCPR2L << 1);
//                         update_sampling_frequency(sampling >> 1);
//                         cutoff_max = sampling*1000 >> 1;
//                         update_echo_delay(echo_delay);
//                         if (low_cutoff > cutoff_max) update_low_cutoff(cutoff_max);
//                         if (high_cutoff > cutoff_max) update_high_cutoff(cutoff_max);
//                     }
//                     break;
//                 case M_VALUE:               // decrease the filter first value, either
//                     switch (filter) {
//                     case F_MOV_AVG:             // the moving average value
//                         if (mov_avg_coef > F_MOV_AVG_MIN) update_mov_avg_coef(mov_avg_coef-F_MOV_AVG_COEF_STEP);
//                         break;
//                     case F_LOW_PASS:            // the low-pass filter cutoff frequency
//                         if (low_cutoff > F_LOW_PASS_MIN) update_low_cutoff(low_cutoff-F_HL_PASS_STEP);
//                         break;
//                     case F_HIGH_PASS:           // the high-pass filter cutoff frequency
//                         if (high_cutoff > F_HIGH_PASS_MIN) update_high_cutoff(high_cutoff-F_HL_PASS_STEP);
//                         break;
//                     case F_ECHO:                // the delay of the echo filter
//                         if (echo_delay > F_ECHO_DEL_MIN) update_echo_delay(echo_delay-F_ECHO_DEL_STEP);
//                         break;
//                     default:
//                         break;
//                     }
//                     break;
//                 case M_VALUE2:              // decrease the filter second value (the number of echoes)
//                     if (echoes > F_ECHO_N_MIN) update_echoes(echoes-1);
//                     break;
//                 default:
//                     break;
//                 }
// 
//             } else if (!PB_VALUE_UP) {  // value up selection:
//                 while (!PB_VALUE_UP);       // wait the button release
//                 switch (menu_entry) {
//                 case M_FILTER:              // select the next filter type
//                     if (filter == F_COUNT-1) filter = 0;
//                     else filter++;
//                     display_parameters();
//                     break;
//                 case M_SAMPLING:            // increase the sampling frequency
//                     if (sampling == SAMPLING_MIN) {
//                         CCPR2H = CCPR2H >> 1;
// //                         CCPR2L = CCPR2L >> 1;
//                         update_sampling_frequency(sampling << 1);
//                         cutoff_max = sampling*1000 >> 1;
//                         update_echo_delay(echo_delay);
//                     }
//                     break;
//                 case M_VALUE:               // increase the filter first value, either
//                     switch (filter) {
//                     case F_MOV_AVG:             // the moving average value
//                         if (mov_avg_coef < F_MOV_AVG_MAX) update_mov_avg_coef(mov_avg_coef+F_MOV_AVG_COEF_STEP);
//                         break;
//                     case F_LOW_PASS:            // the low-pass filter cutoff frequency
//                         if (low_cutoff < cutoff_max) update_low_cutoff(low_cutoff+F_HL_PASS_STEP);
//                         break;
//                     case F_HIGH_PASS:           // the high-pass filter cutoff frequency
//                         if (high_cutoff < cutoff_max) update_high_cutoff(high_cutoff+F_HL_PASS_STEP);
//                         break;
//                     case F_ECHO:                // the delay of the echo filter
//                         if (echo_delay < F_ECHO_DEL_MAX) update_echo_delay(echo_delay+F_ECHO_DEL_STEP);
//                         break;
//                     default:
//                         break;
//                     }
//                     break;
//                 case M_VALUE2:              // increase the filter second value (the number of echoes)
// //                     if (echoes < F_ECHO_N_MAX) update_echoes(echoes+1);
//                     break;
//                 default:
//                     break;
//                 }
//             }
//         }
//         __delay_ms(100);
//     }
}

void display_parameters() {
    char text[20];
    switch (filter) {
    case F_MOV_AVG:
        send_text_LCD("Moving avg filter   ", 0, 0);
        send_text_LCD("Sampling       16kHz", 1, 0);
        sprintf(text, "Steps         %6d", mov_avg_steps);
        send_text_LCD(text, 2, 0);
        send_text_LCD("                    ", 3, 0);
        break;
    case F_LOW_PASS:
        send_text_LCD("Low-pass filter     ", 0, 0);
        send_text_LCD("Sampling       16kHz", 1, 0);
        sprintf(text, "Cutoff    %8dHz", cutoff);
        send_text_LCD(text, 2, 0);
        send_text_LCD("                    ", 3, 0);
        break;
    case F_HIGH_PASS:
        send_text_LCD("High-pass filter    ", 0, 0);
        send_text_LCD("Sampling       16kHz", 1, 0);
        sprintf(text, "Cutoff    %8dHz", cutoff);
        send_text_LCD(text, 2, 0);
        send_text_LCD("                    ", 3, 0);
        break;
    case F_ECHO:
        send_text_LCD("Echo filter         ", 0, 0);
        send_text_LCD("Sampling       16kHz", 1, 0);
        sprintf(text, "Delay    %9dms", echo_delay / sampling);
        send_text_LCD(text, 2, 0);
        sprintf(text, "Echoes   %11d", echoes);
        send_text_LCD(text, 3, 0);
        break;
    default:
        break;
    }
}
