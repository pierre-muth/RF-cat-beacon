/* 
 * File:   spi.h
 * Author: muthi
 *
 * Created on 10 novembre 2018, 14:28
 */
#include <stdint.h>
#ifndef SPI_H
#define	SPI_H

#ifdef	__cplusplus
extern "C" {
#endif
    
// pin names
#define SpiritCS PORTAbits.RA4

void SpiritSpiWriteRegisters(uint8_t address, uint8_t n_regs, uint8_t* buffer);
void SpiritSpiReadRegisters(uint8_t address, uint8_t n_regs, uint8_t* buffer);
void SpiritSpiCommandStrobes(uint8_t cmd_code);


#ifdef	__cplusplus
}
#endif

#endif	/* SPI_H */

