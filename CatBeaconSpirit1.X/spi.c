#include <xc.h>
#include <stdint.h>
#include "spi.h"


void SpiritSpiWriteRegisters(uint8_t address, uint8_t n_regs, uint8_t* buffer) {
    uint8_t temp = 0;
    
    SpiritCS = 0;
    
    SSP1BUF = 0x00;     //write command
    while (SSP1STATbits.BF == 0){;}
    temp = SSP1BUF;
    
    SSP1BUF = address;
    while (SSP1STATbits.BF == 0){;}
    temp = SSP1BUF;
    
    int i;
    for (i = 0; i < n_regs; i++) {
        SSP1BUF = buffer[i];
        while (SSP1STATbits.BF == 0){;}
        temp = SSP1BUF;

    }
    
    SpiritCS = 1;
}
void SpiritSpiReadRegisters(uint8_t address, uint8_t n_regs, uint8_t* buffer){
    uint8_t temp = 0;
    
    SpiritCS = 0;
    
    SSP1BUF = 0x01;     //read command
    while (SSP1STATbits.BF == 0){;}
    temp = SSP1BUF;
    
    SSP1BUF = address;
    while (SSP1STATbits.BF == 0){;}
    temp = SSP1BUF;
    
    int i;
    for (i = 0; i < n_regs; i++) {
        SSP1BUF = 0x00;
        while (SSP1STATbits.BF == 0){;}
        buffer[i] = SSP1BUF;

    }
    
    SpiritCS = 1;
}
void SpiritSpiCommandStrobes(uint8_t cmd_code){
    uint8_t temp = 0;
    
    SpiritCS = 0;
    
    SSP1BUF = 0x80;
    while (SSP1STATbits.BF == 0){;}
    temp = SSP1BUF;

    
    SSP1BUF = cmd_code;
    while (SSP1STATbits.BF == 0){;}
    temp = SSP1BUF;
    
    SSP1BUF = 0;
    while (SSP1STATbits.BF == 0){;}
    temp = SSP1BUF;
    
    SSP1BUF = 0;
    while (SSP1STATbits.BF == 0){;}
    temp = SSP1BUF;

    SpiritCS = 1;
}
