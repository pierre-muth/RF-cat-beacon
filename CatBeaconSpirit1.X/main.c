/*
 * File:   main.c
 * Author: pierremuth.wordpress.com
 * Created on 26 octobre 2018, 21:44
 *    PIC12F1822:
 *                ---u---
 *           VDD -|      |- GND
 *  SpiritSD/RA5 -|      |- RA0/MOSI/icspDATA
 * SpiritCSN/RA4 -|      |- RA1/SCK /icspCLK
 *     VPP/RA3   -|      |- RA2/MISO
 *                 -----
 * 
 *    SPSRFG:
 *          |----------|
 *          |          |
 *          >          < SDN
 *          >          < CS
 *          >          < MOSI
 *          >          < MISO
 *      VDD >___^______< SCK
 *             GND
 */

// PIC12LF1822 Configuration Bit Settings
// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = SWDTEN    // Watchdog Timer Enable (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)
// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

#include <xc.h>
#include <stdint.h>
#include "spirit.h"
#include "spi.h"


// CPU freq
#define _XTAL_FREQ  (8000000UL)


// proto
void init(void);
//unsigned char spiRW(unsigned char command, unsigned char data);
//void writeRegister(unsigned char address, unsigned char value);
//unsigned char readRegister(unsigned char address);
void sendWithSpirit(void);
uint8_t getVoltage(void);

// global var
uint8_t voltageBefore = 0;
uint8_t voltageAfter = 0;

void main(void) {
    init();
    
    while (1) {
        voltageBefore = getVoltage();
        
        TRISAbits.TRISA2 = 1;
        SSP1CON1bits.SSPEN = 1;              // Enable SPI
        SpiritShutDown = 0;
        __delay_ms(1);
        
        sendWithSpirit();
        
        voltageAfter = getVoltage();
        
        SpiritShutDown = 1;
        SSP1CON1bits.SSPEN = 0;              // disable SPI
        TRISAbits.TRISA2 = 0;
        
        SLEEP();
        NOP();
    }
}

void init(){
    OSCCONbits.IRCF = 0b1110;   // 8/32MHz clock
    WDTCONbits.WDTPS = 0b10000; // watchdog for 64 sec sleep

    // pin config
    ANSELA = 0b00000000;    // no analog on port A
    PORTA = 0x00;
    TRISA = 0b00000000;     // input/output
    SpiritCS = 1;
    SpiritShutDown = 1;
    
    // SPI init
    SSP1CON1bits.CKP = 0;                // SPI clk low by default
    SSP1STATbits.CKE = 1;                // SPI clk phase, data on rising edge
    SSP1CON1bits.SSPM = 0b0010;          // SPI clock Fosc/4
    
    // watch dog enable
    WDTCONbits.SWDTEN = 1;

}

void sendWithSpirit(){
    uint8_t tmp[4];
    
    SpiritBaseConfiguration();
    SpiritVcoCalibration();
    
    tmp[0] = 'B';
    tmp[1] = voltageBefore;
    tmp[2] = 'A';
    tmp[3] = voltageAfter;
    
    SpiritSend(4, tmp);
}



uint8_t getVoltage() {
    uint8_t voltage = 0;

    // start voltage reference
    FVRCONbits.ADFVR = 0b01;    // Internal 1.024v ref
    FVRCONbits.FVREN = 1;       // Enable FVR module
    
    // get battery voltage
    while(!FVRCONbits.FVRRDY) {}; // Wait for FVR to be stable
    ADCON1bits.ADFM = 0;        // left justify result
    ADCON0bits.CHS = 0b11111;   // FVR is ADC input
    ADCON1bits.ADPREF = 0b00;   // Positive ref is Vdd (default)
    ADCON1bits.ADCS = 0b101;    // 2us conversion
    ADCON0bits.ADON = 1;        // Turn on ADC module

    __delay_us(20);
    ADCON0bits.GO_nDONE = 1;    // Start a conversion
    while (ADCON0bits.GO_nDONE) {} ;// Wait for it to be completed

    voltage = ADRESH;         // Store the result in adc_val

    // adc, fvr off
    FVREN = 0;                  // disable FVR module
    ADCON0bits.ADON = 0;        // disable ADC
    
    return voltage;

}
