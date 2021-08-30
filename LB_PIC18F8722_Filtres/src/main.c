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
 *  - cutoff frequency (between 0 and sampling/2)
 * 
 * High-pass filter:
 *  - sampling frequency (8 or 16kHz)
 *  - cutoff frequency (between 0 and sampling/2)
 * 
 * Echo filter:
 *  - sampling frequency (8 or 16kHz)
 *  - delay (from 50 to 400ms for 8k and from 25 to 200 ms for 16k)
 *  - number of echoes (1, 2 or 3)
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

#include "SPI.h"

// PIC I/Os
#define SW_RUN              PORTEbits.RE0   // RUN / CONFIG mode switch
#define PB_PREV_MENU        PORTEbits.RE1   // previous menu entry selection push button (NC)
#define PB_NEXT_MENU        PORTEbits.RE2   // next menu entry selection push button (NC)
#define PB_VALUE_DN         PORTEbits.RE3   // value down push button (NC)
#define PB_VALUE_UP         PORTEbits.RE4   // value up push button (NC)
#define TICK                PORTGbits.RG0   // debug tick measuring the processing time of the interruption
#define DAC0808             PORTD           // DAC0808

// Modes
#define MODE_NONE           0       // no mode selected
#define MODE_RUN            1       // signal processing mode
#define MODE_CONFIG         2       // filter configuration mode

// Menu entries
#define M_FILTER            0       // filter type selection
#define M_SAMPLING          1       // sampling frequency selection
#define M_VALUE             2       // filter value selection
#define M_VALUE2            3       // filter second value selection (used by the echo filter)
#define M_COUNT             3       // count of menu entries
#define M_ECHO_COUNT        4       // count of menu entries for the echo filter

// Filters
#define F_MOV_AVG           0       // moving average filter
#define F_LOW_PASS          1       // low-pass filter
#define F_HIGH_PASS         2       // high-pass filter
#define F_ECHO              3       // echo filter
#define F_COUNT             4       // count of filters

// Parameters limit values
#define F_MOV_AVG_MIN       1       // moving average filter min value (2^1)
#define F_MOV_AVG_MAX       3       // moving average filter max value (2^3)
#define F_LOW_PASS_MIN      0       // low-pass filter min value
#define F_HIGH_PASS_MIN     0       // high-pass filter min value
#define F_ECHO_DEL_MIN      800     // echo filter min delay value (100 or 50ms)
#define F_ECHO_DEL_MAX      3200    // echo filter max delay value (400 or 200ms)
#define F_ECHO_N_MIN        1       // echo filter min number of echoes
#define F_ECHO_N_MAX        3       // echo filter max number of echoes
#define SAMPLING_MIN        8       // sampling frequency min value in kHz
#define SAMPLING_MAX        16      // sampling frequency max value in kHz

// Parameters steps
#define F_MOV_AVG_COEF_STEP 1       // moving average coefficient exponent step: 1
#define F_HL_PASS_STEP      100     // high and low-pass filter step: 100Hz
#define F_ECHO_DEL_STEP     400     // echo delay step: 400 spaces in the buffer (50 or 25ms)

#define COEF_SCALE          7       // filters coefficients scale: 2^7

#define SELECT_POS          19      // menu selector position

#define IN_BUF_SIZE  F_ECHO_DEL_MAX+1   // input signal buffer size
#define OUT_BUF_SIZE        1       // output signal buffer size

// Global variables (used by the interruption & the main program)
signed char sig_in[IN_BUF_SIZE];    // input signal buffer
signed char sig_out[OUT_BUF_SIZE];  // output signal buffer
int index[F_ECHO_N_MAX+1];          // input buffer indexes
int coef[F_ECHO_N_MAX+1];           // coefficients used to compute the output signal
char filter;                        // current filter selected
int sampling;                       // sampling frequency (8 or 16 kHz)
char mov_avg_coef;                  // moving average filter coefficient (1-3 -> 2, 4 or 8)
int low_cutoff;                     // low-pass filter cutoff frequency
int high_cutoff;                    // high-pass filter cutoff frequency
int echo_delay;                     // echo filter delay
int echoes;                         // echo filter number of echoes (1, 2 or 3)

// Display function
void display_parameters(void);                  // display all the parameters
void update_sampling_frequency(int new_val);    // update the sampling frequency and display it
void update_mov_avg_coef(char new_val);         // update the moving average coefficient and display it
void update_low_cutoff(int new_val);            // update the low cutoff frequency and display it
void update_high_cutoff(int new_val);           // update the high cutoff frequency and display it
void update_echo_delay(int new_val);            // update the echo delay and display it
void update_echoes(int new_val);                // update the number of echoes and display it

void __interrupt(high_priority) Int_Vect_High(void) {
    int i;
    long output;    // temporary variable used to compute the output signal value

    TICK = 1;                       // debug tick ON

    while (ADCON0bits.NOT_DONE);    // analog to digital conversion, result in the input buffer
    sig_in[index[0]] = (signed char) (ADRESH - 128);

    output = 0;
    switch (filter) {
    case F_MOV_AVG:                 // moving average filter: Yt = SUM(i=0:coef-1) Xt-i / coef
        for (i = 0; i < 1<<mov_avg_coef; i++) output += sig_in[i];
        output = output >> mov_avg_coef;
        index[0]++;                     // increment the input buffer index
        if (index[0] == 1<<mov_avg_coef) index[0] = 0;
        break;
    case F_LOW_PASS:                    // low-pass filter: Yt = A * (Xt + Xt-1) + B * Yt-1
        output = coef[0]*(sig_in[0] + sig_in[1]) + coef[1]*sig_out[0];
        output = output >> COEF_SCALE;
        sig_in[1] = sig_in[0];             // shift the input signal value in the buffer
        sig_out[0] = (signed char) output; // store the output value in the buffer
        break;
    case F_HIGH_PASS:                   // low-pass filter: Yt = A * (Xt - Xt-1) + B * Yt-1
        output = coef[0]*(sig_in[0] - sig_in[1]) + coef[1]*sig_out[0];
        output = output >> COEF_SCALE;
        sig_in[1] = sig_in[0];             // shift the input signal value in the buffer
        sig_out[0] = (signed char) output; // store the output value in the buffer
        break;
    case F_ECHO:                   // echo filter: Yt = A Xt + B Xt-delay + ...
        for (i = 0; i <= echoes; i++) output += coef[i] * sig_in[index[i]];
        output = output >> COEF_SCALE;
        for (i = 0; i <= echoes; i++) {     // increment the input buffer indexes
            index[i]++;
            if (index[i] == IN_BUF_SIZE) index[i] = 0;
        }
        break;
    default:
        break;
    }

    SPI_DAC_CS = 0;         // send the output signal value to the DAC
    SSPIF = 0;
    SSPBUF = 0x10;          // MCP4922 mode x2
    while (!SSPIF);
    SSPIF = 0;
    SSPBUF = (unsigned char) (output + 128);
    while (!SSPIF);
    SPI_DAC_CS = 1;
    LDAC = 0;               // load the DAC
    LDAC = 1;

    DAC0808 = (unsigned char) (output + 128);

    TICK = 0;               // debug tick OFF
    PIR2bits.CCP2IF = 0;    // interruption flag cleared
}

void main(void) {
    int i;
    char menu_entry;        // current menu entry selected
    char prev_mode;         // previous mode selected: run, config or none
    int cutoff_max;         // high/low pass filter max cutoff value determined by the sampling frequency
    double omega;           // angular frequency used to initialize some filters coefficients
    // char text[20];

    // PIC configuration
    TRISE   = 0xFF;         // PORTE (menu buttons) -> input
    TRISGbits.TRISG0 = 0;   // TICK -> output
    TICK    = 0;            // TICK -> 0
    TRISD   = 0;            // DAC0808 -> output
    DAC0808 = 128;          // DAC0808 -> 0 + offset
    ADCON0  = 0X01;         // ADC enabled on AN0
    ADCON1  = 0x0E;         // AN0 -> analogic
    ADCON2  = 0x01;         // Left justification, 0 Tad, 8 Tosc
    CCP2CON = 0x0B;         // CCP2 interruption in compare mode with trigger special event
    CCPR2H  = 0x04;         // 1250us -> 8kHz
    CCPR2L  = 0xE2;
    T1CON   = 0x01;         // Timer1 prescale 1:1
    TMR1H   = 0;            // Timer1 -> 0
    TMR1L   = 0;
    RCONbits.IPEN   = 1;    // enable priority levels on interrupts
    INTCONbits.GIEH = 0;    // high priority interrupts disabled
    INTCONbits.GIEL = 0;    // low priority interrupts disabled
    IPR2bits.CCP2IP = 1;    // CCP2 interrupt: high priority
    PIE2bits.CCP2IE = 1;    // CCP2 interrupt enabled
    PIR2bits.CCP2IF = 0;    // CCP2 interrupt flag cleared
    init_SPI();             // SPI bus and peripherals initialization

    // Parameters initialization
    prev_mode       = MODE_NONE;            // previous mode selected: none
    sampling        = (int) (10000 / ((CCPR2H << 8) + CCPR2L)); // sampling frequency determined by the CCP2 value
    cutoff_max      = sampling*1000 >> 1;   // low/high pass filter max value determined by the sampling frequency
    filter          = F_MOV_AVG;            // filter selected: moving average
    mov_avg_coef    = F_MOV_AVG_MIN;        // moving average value: minimum
    low_cutoff      = F_LOW_PASS_MIN;       // low-pass cutoff value: minimum
    high_cutoff     = cutoff_max;           // high-pass cutoff value: maximum
    echo_delay      = F_ECHO_DEL_MIN;       // echo delay: minimum
    echoes          = F_ECHO_N_MIN;         // number of echoes: minimum

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

    display_parameters();

    // Main loop: polling on the RUN/CONFIG switch
    while (1) {
        if (SW_RUN) {
            if (prev_mode != MODE_RUN) {            // run mode initialization:
                prev_mode = MODE_RUN;                       // set the previous mode to RUN
                send_text_LCD(" ", menu_entry, SELECT_POS); // erase the menu selector

                index[0] = 0;                               // input buffer index to 0
                for (i = 0; i < IN_BUF_SIZE; i++) sig_in[i] = 0;    // input buffer to 0
                for (i = 0; i < OUT_BUF_SIZE; i++) sig_out[i] = 0;  // output buffer to 0

                switch (filter) {                   // depending on the filter selected:
                case F_LOW_PASS:                        // compute the low-pass filter coefficients
                    omega = 2.0 * M_PI * (double) low_cutoff / (double) sampling / 1000.0;
                    coef[0] = (int) floor(omega / (2.0 + omega) * pow(2.0, COEF_SCALE));
                    coef[1] = (int) floor((2.0 - omega) / (2.0 + omega) * pow(2.0, COEF_SCALE));
                    // sprintf(text, "%4d %4d", coef[0], coef[1]);
                    // send_text_LCD(text, 3, 0);
                    break;
                case F_HIGH_PASS:                       // or compute the low-pass filter coefficients
                    omega = 2.0 * M_PI * (double) high_cutoff / (double) sampling / 1000.0;
                    coef[0] = (int) floor(2.0 / (2.0 + omega) * pow(2.0, COEF_SCALE));
                    coef[1] = (int) floor((2.0 - omega) / (2.0 + omega) * pow(2.0, COEF_SCALE));
                    // sprintf(text, "%4d %4d", coef[0], coef[1]);
                    // send_text_LCD(text, 3, 0);
                    break;
                case F_ECHO:                            // or initialize the echoes indexes
                    for (i = 0; i < echoes; i++) index[echoes-i] = IN_BUF_SIZE - echo_delay/(i+1);
                    switch (echoes) {                   // and the echoes coefficients (total must be 128, 2^COEF_SCALE)
                    case 1:
                        coef[0] = 96;   // original signal
                        coef[1] = 32;   // echo
                        break;
                    case 2:
                        coef[0] = 64;   // original signal
                        coef[1] = 40;   // first echo
                        coef[2] = 24;    // second echo
                        break;
                    case 3:
                        coef[0] = 44;   // original signal
                        coef[1] = 36;   // first echo
                        coef[2] = 28;   // second echo
                        coef[3] = 20;    // third echo
                        break;
                    default:
                        break;
                    }
                    break;
                default:                            // other filters does not need a specific initialization
                    break;
                }
                INTCONbits.GIEH = 1;                // enable high priority interrupts
            }
        } else {
            if (prev_mode != MODE_CONFIG) {         // config mode initialization: 
                INTCONbits.GIEH = 0;                        // disable high priority interrupts
                prev_mode = MODE_CONFIG;                    // set the previous mode to CONFIG
                menu_entry = M_FILTER;                      // select the first menu
                send_text_LCD("<", menu_entry, SELECT_POS); // display the menu selector
            }

            if (!PB_PREV_MENU) {                        // previous menu selection:
                while (!PB_PREV_MENU);                      // wait the button release
                send_text_LCD(" ", menu_entry, SELECT_POS); // erase the menu selector
                if (!menu_entry) menu_entry = filter == F_ECHO ? M_ECHO_COUNT-1 : M_COUNT-1;
                else menu_entry--;                          // select the previous menu
                send_text_LCD("<", menu_entry, SELECT_POS); // display the menu selector

            } else if (!PB_NEXT_MENU) {                 // next menu selection:
                while (!PB_NEXT_MENU);                      // wait the button release
                send_text_LCD(" ", menu_entry, SELECT_POS); // erase the menu selector
                if (filter == F_ECHO && menu_entry == M_ECHO_COUNT-1) menu_entry = 0;
                else if (filter != F_ECHO && menu_entry == M_COUNT-1) menu_entry = 0;
                else menu_entry++;                          // select the next menu
                send_text_LCD("<", menu_entry, SELECT_POS); // display the menu selector

            } else if (!PB_VALUE_DN) {  // value down selection:
                while (!PB_VALUE_DN);       // wait the button release
                switch (menu_entry) {
                case M_FILTER:              // select the previous filter type
                    if (!filter) filter = F_COUNT-1;
                    else filter--;
                    display_parameters();
                    break;
                case M_SAMPLING:            // decrease the sampling frequency
                    if (sampling == SAMPLING_MAX) {
                        CCPR2H = (unsigned char) (CCPR2H << 1);
                        CCPR2L = (unsigned char) (CCPR2L << 1);
                        update_sampling_frequency(sampling >> 1);
                        cutoff_max = sampling*1000 >> 1;
                        update_echo_delay(echo_delay);
                        if (low_cutoff > cutoff_max) update_low_cutoff(cutoff_max);
                        if (high_cutoff > cutoff_max) update_high_cutoff(cutoff_max);
                    }
                    break;
                case M_VALUE:               // decrease the filter first value, either
                    switch (filter) {
                    case F_MOV_AVG:             // the moving average value
                        if (mov_avg_coef > F_MOV_AVG_MIN) update_mov_avg_coef(mov_avg_coef-F_MOV_AVG_COEF_STEP);
                        break;
                    case F_LOW_PASS:            // the low-pass filter cutoff frequency
                        if (low_cutoff > F_LOW_PASS_MIN) update_low_cutoff(low_cutoff-F_HL_PASS_STEP);
                        break;
                    case F_HIGH_PASS:           // the high-pass filter cutoff frequency
                        if (high_cutoff > F_HIGH_PASS_MIN) update_high_cutoff(high_cutoff-F_HL_PASS_STEP);
                        break;
                    case F_ECHO:                // the delay of the echo filter
                        if (echo_delay > F_ECHO_DEL_MIN) update_echo_delay(echo_delay-F_ECHO_DEL_STEP);
                        break;
                    default:
                        break;
                    }
                    break;
                case M_VALUE2:              // decrease the filter second value (the number of echoes)
                    if (echoes > F_ECHO_N_MIN) update_echoes(echoes-1);
                    break;
                default:
                    break;
                }

            } else if (!PB_VALUE_UP) {  // value up selection:
                while (!PB_VALUE_UP);       // wait the button release
                switch (menu_entry) {
                case M_FILTER:              // select the next filter type
                    if (filter == F_COUNT-1) filter = 0;
                    else filter++;
                    display_parameters();
                    break;
                case M_SAMPLING:            // increase the sampling frequency
                    if (sampling == SAMPLING_MIN) {
                        CCPR2H = CCPR2H >> 1;
                        CCPR2L = CCPR2L >> 1;
                        update_sampling_frequency(sampling << 1);
                        cutoff_max = sampling*1000 >> 1;
                        update_echo_delay(echo_delay);
                    }
                    break;
                case M_VALUE:               // increase the filter first value, either
                    switch (filter) {
                    case F_MOV_AVG:             // the moving average value
                        if (mov_avg_coef < F_MOV_AVG_MAX) update_mov_avg_coef(mov_avg_coef+F_MOV_AVG_COEF_STEP);
                        break;
                    case F_LOW_PASS:            // the low-pass filter cutoff frequency
                        if (low_cutoff < cutoff_max) update_low_cutoff(low_cutoff+F_HL_PASS_STEP);
                        break;
                    case F_HIGH_PASS:           // the high-pass filter cutoff frequency
                        if (high_cutoff < cutoff_max) update_high_cutoff(high_cutoff+F_HL_PASS_STEP);
                        break;
                    case F_ECHO:                // the delay of the echo filter
                        if (echo_delay < F_ECHO_DEL_MAX) update_echo_delay(echo_delay+F_ECHO_DEL_STEP);
                        break;
                    default:
                        break;
                    }
                    break;
                case M_VALUE2:              // increase the filter second value (the number of echoes)
                    if (echoes < F_ECHO_N_MAX) update_echoes(echoes+1);
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
        send_text_LCD("Moving avg filter ", M_FILTER, 0);
        sprintf(text, "Sampling %6d Hz", sampling*1000);
        send_text_LCD(text, M_SAMPLING, 0);
        sprintf(text, "Steps       %6d", 1 << mov_avg_coef);
        send_text_LCD(text, M_VALUE, 0);
        send_text_LCD("                  ", M_VALUE2, 0);
        break;
    case F_LOW_PASS:
        send_text_LCD("Low-pass filter   ", M_FILTER, 0);
        sprintf(text, "Sampling %6d Hz", sampling*1000);
        send_text_LCD(text, M_SAMPLING, 0);
        sprintf(text, "Cutoff %8u Hz", low_cutoff);
        send_text_LCD(text, M_VALUE, 0);
        send_text_LCD("                  ", M_VALUE2, 0);
        break;
    case F_HIGH_PASS:
        send_text_LCD("High-pass filter  ", M_FILTER, 0);
        sprintf(text, "Sampling %6d Hz", sampling*1000);
        send_text_LCD(text, M_SAMPLING, 0);
        sprintf(text, "Cutoff %8u Hz", high_cutoff);
        send_text_LCD(text, M_VALUE, 0);
        send_text_LCD("                  ", M_VALUE2, 0);
        break;
    case F_ECHO:
        send_text_LCD("Echo filter       ", M_FILTER, 0);
        sprintf(text, "Sampling %6d Hz", sampling*1000);
        send_text_LCD(text, M_SAMPLING, 0);
        sprintf(text, "Delay %9d ms", echo_delay / sampling);
        send_text_LCD(text, M_VALUE, 0);
        sprintf(text, "Echoes %11d", echoes);
        send_text_LCD(text, M_VALUE2, 0);
        break;
    default:
        break;
    }
}

void update_sampling_frequency(int new_val) {
    char text[19];
    sampling = new_val;
    sprintf(text, "%6d", sampling*1000);
    send_text_LCD(text, M_SAMPLING, 9);
}

void update_mov_avg_coef(char new_val) {
    char text[19];
    mov_avg_coef = new_val;
    if (filter == F_MOV_AVG) {
        sprintf(text, "%6d", 1 << mov_avg_coef);
        send_text_LCD(text, M_VALUE, 12);
    }
}

void update_low_cutoff(int new_val) {
    char text[19];
    low_cutoff = new_val;
    if (filter == F_LOW_PASS) {
        sprintf(text, "%8u", low_cutoff);
        send_text_LCD(text, M_VALUE, 7);
    }
}

void update_high_cutoff(int new_val) {
    char text[19];
    high_cutoff = new_val;
    if (filter == F_HIGH_PASS) {
        sprintf(text, "%8u", high_cutoff);
        send_text_LCD(text, M_VALUE, 7);
    }
}

void update_echo_delay(int new_val) {
    char text[19];
    echo_delay = new_val;
    if (filter == F_ECHO) {
        sprintf(text, "%9d", echo_delay / sampling);
        send_text_LCD(text, M_VALUE, 6);
    }
}

void update_echoes(int new_val) {
    char text[19];
    echoes = new_val;
    if (filter == F_ECHO) {
        sprintf(text, "%11d", echoes);
        send_text_LCD(text, M_VALUE2, 7);
    }
}
