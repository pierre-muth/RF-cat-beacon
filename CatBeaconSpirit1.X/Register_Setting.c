#include <stdint.h>

/**

The SPI interface is platform dependent, this means that it should be implemented according to the used hardware.

At least the 3 functions:
        SpiritSpiWriteRegisters(uint8_t address, uint8_t n_regs, uint8_t* buffer)
        SpiritSpiReadRegisters(uint8_t address, uint8_t n_regs, uint8_t* buffer)
        SpiritSpiCommandStrobes(uint8_t cmd_code)
must be implemented.

An example of implementation (for the SDK_EVAL motherboards of the SPIRIT kit) can be found into the file: Firmware/STM32L/SDK_Eval_STM32L/Source/src/SDK_EVAL_Spi_Driver.c
It is advisable to implement also the 2 FIFO functions to read and write (no reference in this template code but needed in applications using the FIFO of SPIRIT).

These functions must be called in the following order:

 SpiritBaseConfiguration();
 SpiritVcoCalibration();

**/

/* list of the command codes of SPIRIT1 */
#define	COMMAND_TX                                          ((uint8_t)(0x60)) /*!< Start to transmit; valid only from READY */
#define	COMMAND_RX                                          ((uint8_t)(0x61)) /*!< Start to receive; valid only from READY */
#define	COMMAND_READY                                       ((uint8_t)(0x62)) /*!< Go to READY; valid only from STANDBY or SLEEP or LOCK */
#define	COMMAND_STANDBY                                     ((uint8_t)(0x63)) /*!< Go to STANDBY; valid only from READY */
#define	COMMAND_SLEEP                                       ((uint8_t)(0x64)) /*!< Go to SLEEP; valid only from READY */
#define	COMMAND_LOCKRX                                      ((uint8_t)(0x65)) /*!< Go to LOCK state by using the RX configuration of the synth; valid only from READY */
#define	COMMAND_LOCKTX                                      ((uint8_t)(0x66)) /*!< Go to LOCK state by using the TX configuration of the synth; valid only from READY */
#define	COMMAND_SABORT                                      ((uint8_t)(0x67)) /*!< Force exit form TX or RX states and go to READY state; valid only from TX or RX */
#define	COMMAND_SRES                                        ((uint8_t)(0x70)) /*!< Reset of all digital part, except SPI registers */
#define	COMMAND_FLUSHRXFIFO                                 ((uint8_t)(0x71)) /*!< Clean the RX FIFO; valid from all states */
#define	COMMAND_FLUSHTXFIFO                                 ((uint8_t)(0x72)) /*!< Clean the TX FIFO; valid from all states */


/* This is the function that initializes the SPIRIT with the configuration 
that the user has exported using the GUI */
void SpiritBaseConfiguration(void)
{
  uint8_t tmp[7];

  /* Be sure that the registers config is default */
  SpiritSpiCommandStrobes(COMMAND_SRES);

  /* Extra current in after power on fix.
     In some samples, when a supply voltage below 2.6 V is applied to SPIRIT1 from a no power condition,
     an extra current is added to the typical current consumption.
     With this sequence, the extra current is erased.
  */
  tmp[0]=0xCA;SpiritSpiWriteRegisters(0xB2, 1, tmp); 
  tmp[0]=0x04;SpiritSpiWriteRegisters(0xA8, 1, tmp); 
  SpiritSpiReadRegisters(0xA8, 1, tmp);
  tmp[0]=0x00;SpiritSpiWriteRegisters(0xA8, 1, tmp);

  tmp[0] = 0x36; /* reg. IF_OFFSET_ANA (0x07) */
  tmp[1] = 0x06; /* reg. SYNT3 (0x08) */
  tmp[2] = 0x82; /* reg. SYNT2 (0x09) */
  tmp[3] = 0x8F; /* reg. SYNT1 (0x0A) */
  tmp[4] = 0x59; /* reg. SYNT0 (0x0B) */
  tmp[5] = 0x01; /* reg. CH_SPACE (0x0C) */
  tmp[6] = 0xAC; /* reg. IF_OFFSET_DIG (0x0D) */
  SpiritSpiWriteRegisters(0x07, 7, tmp);
  tmp[0] = 0x01; /* reg. PA_POWER[8] (0x10) */
  SpiritSpiWriteRegisters(0x10, 1, tmp);
  tmp[0] = 0xA3; /* reg. MOD1 (0x1A) */
  tmp[1] = 0x08; /* reg. MOD0 (0x1B) */
  tmp[2] = 0x52; /* reg. FDEV0 (0x1C) */
  tmp[3] = 0x62; /* reg. CHFLT (0x1D) */
  tmp[4] = 0xC8; /* reg. AFC2 (0x1E) */
  SpiritSpiWriteRegisters(0x1A, 5, tmp);
  tmp[0] = 0x62; /* reg. AGCCTRL1 (0x25) */
  SpiritSpiWriteRegisters(0x25, 1, tmp);
  tmp[0] = 0x15; /* reg. ANT_SELECT_CONF (0x27) */
  SpiritSpiWriteRegisters(0x27, 1, tmp);
  tmp[0] = 0x3F; /* reg. PCKTCTRL2 (0x32) */
  tmp[1] = 0x50; /* reg. PCKTCTRL1 (0x33) */
  SpiritSpiWriteRegisters(0x32, 2, tmp);
  tmp[0] = 0x41; /* reg. PCKT_FLT_OPTIONS (0x4F) */
  tmp[1] = 0x40; /* reg. PROTOCOL[2] (0x50) */
  tmp[2] = 0x01; /* reg. PROTOCOL[1] (0x51) */
  SpiritSpiWriteRegisters(0x4F, 3, tmp);
  tmp[0] = 0x00; /* reg. RCO_VCO_CALIBR_IN[1] (0x6E) */
  tmp[1] = 0x00; /* reg. RCO_VCO_CALIBR_IN[0] (0x6F) */
  SpiritSpiWriteRegisters(0x6E, 2, tmp);
  tmp[0] = 0xA0; /* reg. SYNTH_CONFIG[0] (0x9F) */
  SpiritSpiWriteRegisters(0x9F, 1, tmp);
  tmp[0] = 0x25; /* reg. VCO_CONFIG (0xA1) */
  SpiritSpiWriteRegisters(0xA1, 1, tmp);
  tmp[0] = 0x35; /* reg. DEM_CONFIG (0xA3) */
  SpiritSpiWriteRegisters(0xA3, 1, tmp);

  /* VCO unwanted calibration workaround. 
     With this sequence, the PA is on after the eventual VCO calibration expires.
  */
  tmp[0]=0x22;SpiritSpiWriteRegisters(0xBC, 1, tmp);

}

/* This is a VCO calibration routine used to recalibrate the VCO of SPIRIT1 in a safe way.
 IMPORTANT: It must be called from READY state. */
void SpiritVcoCalibration(void)
{
  uint8_t tmp[4];
  uint8_t cal_words[2];
  uint8_t state;


    
  SpiritSpiReadRegisters(0x9E, 1, tmp);
  tmp[0] |= 0x80;
  SpiritSpiWriteRegisters(0x9E, 1, tmp); /* REFDIV bit set (to be restored) */

  /* As a consequence we need to double the SYNT word to generate the target frequency */
  tmp[0] = 0x0D;
  tmp[1] = 0x05;
  tmp[2] = 0x1E;
  tmp[3] = 0xB1;
  SpiritSpiWriteRegisters(0x08, 4, tmp);


  tmp[0] = 0x25; SpiritSpiWriteRegisters(0xA1,1,tmp); /* increase VCO current (restore to 0x11) */
  
  SpiritSpiReadRegisters(0x50,1,tmp);
  tmp[0] |= 0x02; 
  SpiritSpiWriteRegisters(0x50,1,tmp); /* enable VCO calibration (to be restored) */
  
  SpiritSpiCommandStrobes(COMMAND_LOCKTX);
  do{
    SpiritSpiReadRegisters(0xC1, 1, &state);
  }while((state&0xFE) != 0x1E); /* wait until LOCK (MC_STATE = 0x0F <<1) */
  SpiritSpiReadRegisters(0xE5, 1, &cal_words[0]); /* calib out word for TX */
  
  SpiritSpiCommandStrobes(COMMAND_READY);
   do{
    SpiritSpiReadRegisters(0xC1, 1, &state);
  }while((state&0xFE) != 0x06); /* wait until READY (MC_STATE = 0x03 <<1) */
  
  SpiritSpiCommandStrobes(COMMAND_LOCKRX);
  do{
    SpiritSpiReadRegisters(0xC1, 1, &state);
  }while((state&0xFE) != 0x1E); /* wait until LOCK (MC_STATE = 0x0F <<1) */
  SpiritSpiReadRegisters(0xE5, 1, &cal_words[1]); /* calib out word for RX */
  
  SpiritSpiCommandStrobes(COMMAND_READY);
   do{
    SpiritSpiReadRegisters(0xC1, 1, &state);
  }while((state&0xFE) != 0x06); /* wait until READY (MC_STATE = 0x03 <<1) */
  
  SpiritSpiReadRegisters(0x50,1,tmp);
  tmp[0] &= 0xFD; 
  SpiritSpiWriteRegisters(0x50,1,tmp); /* VCO calib restored to 0 */

  SpiritSpiReadRegisters(0x9E, 1, tmp);
  tmp[0] &= 0x7F;
  SpiritSpiWriteRegisters(0x9E, 1, tmp); /* REFDIV bit reset */

  
  tmp[0] = 0x06;
  tmp[1] = 0x82;
  tmp[2] = 0x8F;
  tmp[3] = 0x59;
  SpiritSpiWriteRegisters(0x08, 4, tmp); /* SYNTH WORD restored */

  
  SpiritSpiWriteRegisters(0x6E,2,cal_words); /* write both calibration words */

}

