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

#define M_2PI               6.28318530717958647692  // 2*pi
#define OFFSET              2048    // offset for the input value on 12 bits
#define COMP_VAL            64923   // Timer1 value -> 16Hz
#define COEF_SCALE          16      // filters coefficients scale: 2^16
#define TICK                LATCbits.LATC9  // debug tick measuring the processing time of the interruption

#define F_MOV_AVG           0       // moving average filter
#define F_LOW_PASS          1       // low-pass filter
#define F_HIGH_PASS         2       // high-pass filter

#define BUF_SIZE            16      // input buffer size -> max steps of the moving average filter

// Global variables (used by the interruption & the main program)
int buf_index;          // input buffer index
int sig_in[BUF_SIZE];   // input signal buffer (16bits)
int sig_out;            // output signal buffer

char filter;            // current filter selected
int sampling;           // sampling frequency (16kHz)
int mov_avg_steps;      // moving average filter number of steps -> max 16 or change the temporary output size
int cutoff;             // low/high-pass filters cutoff frequency
long coef_a;            // low/high-pass filter coefficient a
long coef_b;            // low/high-pass filter coefficient b
int i;
long temp;

// signed char sig_in[IN_BUF_SIZE];    
// signed char sig_out[OUT_BUF_SIZE];  
// int index[F_ECHO_N_MAX+1];          
// int coef[F_ECHO_N_MAX+1];           // coefficients used to compute the output signal

void display_parameters(void);      // display the parameters


void __attribute__((interrupt(auto_psv))) _T1Interrupt(void) {
    TMR1 = COMP_VAL;
    TICK = 1;
    
    AD1CON1bits.SAMP = 1;
    AD1CON1bits.SAMP = 0;
    while(!AD1CON1bits.DONE);
    
    sig_in[buf_index] = ADC1BUF0 - OFFSET;

    switch (filter) {
    case F_MOV_AVG:     // moving average filter: Yt = SUM(i=0:coef-1) Xt-i / coef
        sig_out = 0;
        for (i = 0; i < mov_avg_steps; i++) sig_out += sig_in[i];
        sig_out = sig_out/mov_avg_steps;
        buf_index = (buf_index+1) % mov_avg_steps;
        break;
    case F_LOW_PASS:    // low-pass filter: Yt = A * (Xt + Xt-1) + B * Yt-1
        temp = coef_a * (sig_in[0]+sig_in[1]) + coef_b * sig_out;
        sig_out = temp >> COEF_SCALE;
        sig_in[1] = sig_in[0];
        break;
    case F_HIGH_PASS:   // high-pass filter: Yt = A * (Xt - Xt-1) + B * Yt-1
        temp = coef_a * (sig_in[0]-sig_in[1]) + coef_b * sig_out;
        sig_out = temp >> COEF_SCALE;
        sig_in[1] = sig_in[0];
        break;
    default:
        sig_out = sig_in[buf_index];
        break;
    }

    SPI_DAC_CS = 0;         // send the output signal value to the DAC
    SSPIF = 0;
    SPI1BUF = ((sig_out+OFFSET)>> 8) | 0x30;
    while (!SSPIF);
    SSPIF = 0;
    SPI1BUF = sig_out+OFFSET;
    while (!SSPIF);
    SPI_DAC_CS = 1;
    LDAC = 0;               // load the DAC
    LDAC = 1;

    TICK = 0;
    _T1IF = 0;
}

int main(void) {
    double omega;

    // SPI configuration
    __builtin_write_OSCCONL(OSCCON & ~(1<<6));  // unlock registers
    RPINR20bits.SCK1R   = 20;                   // SPI SDI
    RPOR10bits.RP21R    = 7;                    // SPI SDO
    RPOR11bits.RP23R    = 8;                    // SPI SCK
    __builtin_write_OSCCONL(OSCCON | (1<<6));   // lock registers
    init_SPI();

    // I/O configuration
    TRISAbits.TRISA0    = 1;                    // AN0 -> input
    TRISCbits.TRISC9    = 0;                    // TICK -> output
    TICK = 0;

    // ADC configuration
    AD1CON1bits.ADON    = 0;                    // disable ADC
    AD1CON1bits.AD12B   = 1;                    // 1 channel, 12 bits
    AD1CON3bits.SAMC    = 4;                    // 4TAD
    AD1CON3bits.ADCS    = 3;                    // 1TAD = 3TCY
    AD1PCFGLbits.PCFG0  = 0;                    // AN0 -> analog mode
    AD1CON1bits.ADON    = 1;                    // enable ADC

    // Timer1 configuration
    T1CON = 0x8000;                             // Timer1 enabled, internal clock, prescale 1:1
    TMR1 = COMP_VAL;
    _T1IP = 7;                                  // Timer1 priority
    _T1IF = 0;                                  // Timer1 interrupt flag cleared

    // Filter parameters initialization
    sampling = 16;
    filter = F_MOV_AVG;
    filter = F_LOW_PASS;
    filter = F_HIGH_PASS;
    mov_avg_steps = 16;
    cutoff = 7000;
    display_parameters();
    
    // Filter initialization
    buf_index = 0;
    sig_out = 0;
    for (i = 0; i < BUF_SIZE; i++) sig_in[i] = 0;
    switch (filter) {
    case F_LOW_PASS:
        omega = M_2PI * (double) cutoff / (double) sampling / 1000.0;
        coef_a = (long) floor(omega / (2.0 + omega) * pow(2.0, COEF_SCALE));
        coef_b = (long) floor((2.0 - omega) / (2.0 + omega) * pow(2.0, COEF_SCALE));
        break;
    case F_HIGH_PASS:
        omega = M_2PI * (double) cutoff / (double) sampling / 1000.0;
        coef_a = (long) floor(2.0 / (2.0 + omega) * pow(2.0, COEF_SCALE));
        coef_b = (long) floor((2.0 - omega) / (2.0 + omega) * pow(2.0, COEF_SCALE));
        break;
    default:
        break;
    }

    // Timer1 interrupt enabled
    _T1IE = 1;
    while (1);
    return 0;
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
    default:
        break;
    }
}
