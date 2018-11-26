#include <Arduino.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <WiFiClient.h>

/* list of the command codes of SPIRIT1 */
#define	COMMAND_TX                                  ((uint8_t)(0x60)) /*!< Start to transmit; valid only from READY */
#define	COMMAND_RX                                  ((uint8_t)(0x61)) /*!< Start to receive; valid only from READY */
#define	COMMAND_READY                               ((uint8_t)(0x62)) /*!< Go to READY; valid only from STANDBY or SLEEP or LOCK */
#define	COMMAND_STANDBY                             ((uint8_t)(0x63)) /*!< Go to STANDBY; valid only from READY */
#define	COMMAND_SLEEP                               ((uint8_t)(0x64)) /*!< Go to SLEEP; valid only from READY */
#define	COMMAND_LOCKRX                              ((uint8_t)(0x65)) /*!< Go to LOCK state by using the RX configuration of the synth; valid only from READY */
#define	COMMAND_LOCKTX                              ((uint8_t)(0x66)) /*!< Go to LOCK state by using the TX configuration of the synth; valid only from READY */
#define	COMMAND_SABORT                              ((uint8_t)(0x67)) /*!< Force exit form TX or RX states and go to READY state; valid only from TX or RX */
#define	COMMAND_SRES                                ((uint8_t)(0x70)) /*!< Reset of all digital part, except SPI registers */
#define	COMMAND_FLUSHRXFIFO                         ((uint8_t)(0x71)) /*!< Clean the RX FIFO; valid from all states */
#define	COMMAND_FLUSHTXFIFO                         ((uint8_t)(0x72)) /*!< Clean the TX FIFO; valid from all states */

// Pin definetion of SPSGRF
#define SPSGRF_SCK     5    // GPIO5  -- SPSGRF's SCK
#define SPSGRF_MISO    19   // GPIO19 -- SPSGRF's MISO
#define SPSGRF_MOSI    27   // GPIO27 -- SPSGRF's MOSI
#define SPSGRF_CS      18   // GPIO18 -- SPSGRF's CS
#define SPSGRF_SDN     14   // GPIO14 -- SPSGRF's SDN
#define SPSGRF_GPIO0   26   // GPIO26 -- SPSGRF's GPIO 0

// Pin def for OLED
#define SCL     15    // GPIO15  -- PCD8544's SCL
#define SDA     4     // GPIO4   -- PCD8544's SDA
#define CS     2     // GPIO2 -- PCD8544's SDA
#define DC     16     // GPIO16 -- PCD8544's SDA
#define RST     17     // GPIO17 -- PCD8544's SDA

// U8X8_SSD1305_128X64_ADAFRUIT_HW_I2C u8x8(U8X8_PIN_NONE, SCL, SDA);
U8G2_PCD8544_84X48_F_4W_SW_SPI u8g2(U8G2_R3, SCL, SDA, CS, DC, RST);	

// proto
void SpiritBaseConfiguration();
void SpiritVcoCalibration();
void SpiritSend(uint8_t n_regs, uint8_t* buffer);
uint8_t SpiritRead(uint8_t n_regs, uint8_t* buffer);
void SpiritPutInRX();
void SpiritSpiWriteRegisters(uint8_t address, uint8_t n_regs, uint8_t* buffer);
void SpiritSpiReadRegisters(uint8_t address, uint8_t n_regs, uint8_t* buffer);
void SpiritSpiCommandStrobes(uint8_t cmd_code);
uint8_t spiritLastRSSI();
void WiFiEvent(WiFiEvent_t event);

static volatile bool wifi_connected = false;

void setup() {
  // Serial init
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Spirit1 Receiver start init");

  //SPSGRF SPI init
  SPI.begin(SPSGRF_SCK, SPSGRF_MISO, SPSGRF_MOSI, SPSGRF_CS);
  pinMode(SPSGRF_SDN, OUTPUT);
  pinMode(SPSGRF_CS, OUTPUT);
  digitalWrite(SPSGRF_CS, HIGH);

  //reset Spirit1
  digitalWrite(SPSGRF_SDN, HIGH);
  delay(100);
  digitalWrite(SPSGRF_SDN, LOW);

  // init Spirit1
  SpiritBaseConfiguration();
  SpiritVcoCalibration();

  // oled init
  delay(200);
  u8g2.begin();

 // start wifi
  WiFi.mode(WIFI_MODE_STA); 
  WiFi.onEvent(WiFiEvent);
  
  WiFi.begin("xxxxx", "yyyyy");

  Serial.println("Spirit1 Receiver end init");
}

uint8_t rxBuffer [12];
char rssiChars [12];
char voltageBeforeChars [12];
char voltageAfterChars [12];
char voltageDropChars [12];
char counterChars[12];
uint8_t rawRssi = 0;
int16_t counter = 0;
float rssi = 0;
float vBefore = 0;
float vAfter = 0;
float vDrop = 0;
uint8_t rssiHistory [48];
uint8_t rssiPointer = 0;

void loop() {

  SpiritPutInRX();

  while ( SpiritRead(4, rxBuffer) == 0 ) {
    dtostrf(counter, 7, 0, counterChars);
    u8g2.firstPage();
      do {
        u8g2.setFont(u8g2_font_profont10_tf);
        u8g2.drawStr(0,8, rssiChars);
        u8g2.drawStr(38,8, "dB");
        u8g2.drawStr(0,17, voltageBeforeChars);
        u8g2.drawStr(38,17, "v");
        u8g2.drawStr(0,26, voltageDropChars); 
        u8g2.drawStr(38,26, "v");
        u8g2.drawStr(0,35, counterChars);
        u8g2.drawStr(38,35, "s");
        u8g2.setFont(u8g2_font_m2icon_7_tf);
        if (wifi_connected) u8g2.drawStr(0,35, "D");
        else u8g2.drawStr(0,35, "C");

        uint8_t x = 0, y = 0;
        for (uint8_t i=0; i<48; i++) {
          x = (rssiPointer + i) % 48;
          y = 83 - ((rssiHistory[x]) /4);
          u8g2.drawLine(i, 40, i, y );
        }
        
      } while ( u8g2.nextPage() );
      counter--;
      delay(1000);
  }

  rawRssi = spiritLastRSSI();
  rssiHistory[rssiPointer++] = rawRssi;
  if (rssiPointer >= 48) rssiPointer = 0;
  rssi = (rawRssi/2.0)-130.0;
  dtostrf(rssi, 7, 1, rssiChars);
  Serial.print(rssiChars);
  Serial.print(' ');
  
  vBefore = (1024.0/ (rxBuffer[1]*4.0)) *1.024;
  vAfter = (1024.0/ (rxBuffer[3]*4.0)) *1.024;
  vDrop = vBefore - vAfter;

  dtostrf(vBefore, 7, 3, voltageBeforeChars);
  Serial.print(voltageBeforeChars);
  Serial.print(' ');

  dtostrf(vAfter, 7, 3, voltageAfterChars);
  Serial.println(voltageAfterChars);

  dtostrf(vDrop, 7, 3, voltageDropChars);

  WiFiClient client;
  if (wifi_connected && client.connect("192.168.1.15", 8889)) {
      client.print("rssi: ");
      client.println(rssiChars);
      client.print("vAfter: ");
      client.println(voltageAfterChars);
      client.print("vBefore: ");
      client.println(voltageBeforeChars);
  } 

  counter = 61;

  delay(100);
}


void SpiritBaseConfiguration(void) {
  uint8_t tmp[7];

  /* Be sure that the registers config is default */
  SpiritSpiCommandStrobes(COMMAND_SRES);
    delay(1);
  /* Extra current in after power on fix.
     In some samples, when a supply voltage below 2.6 V is applied to SPIRIT1 from a no power condition,
     an extra current is added to the typical current consumption.
     With this sequence, the extra current is erased.
  */
  tmp[0]=0xCA; SpiritSpiWriteRegisters(0xB2, 1, tmp); 
  tmp[0]=0x04; SpiritSpiWriteRegisters(0xA8, 1, tmp); 
  SpiritSpiReadRegisters(0xA8, 1, tmp);
  tmp[0]=0x00; SpiritSpiWriteRegisters(0xA8, 1, tmp);

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
  tmp[0] = 0x01; /* reg. PCKT_FLT_OPTIONS (0x4F) */
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
  tmp[0]=0x22; SpiritSpiWriteRegisters(0xBC, 1, tmp);

}

/* This is a VCO calibration routine used to recalibrate the VCO of SPIRIT1 in a safe way.
 IMPORTANT: It must be called from READY state. */
void SpiritVcoCalibration(void) {
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

void SpiritSend(uint8_t n_regs, uint8_t* buffer) {
    uint8_t state;
    
    // put to TX fifo
    SpiritSpiWriteRegisters(0xFF, n_regs, buffer);
    
    SpiritSpiCommandStrobes(COMMAND_LOCKTX);
    do{
        SpiritSpiReadRegisters(0xC1, 1, &state);
    }while((state&0xFE) != 0x1E); /* wait until LOCK (MC_STATE = 0x0F <<1) */
    
    SpiritSpiCommandStrobes(COMMAND_TX);
    do{
        SpiritSpiReadRegisters(0xC1, 1, &state);
    }while((state&0xFE) != 0xBE); /* wait until TX (MC_STATE = 0x5F <<1) */
    
    do{
        SpiritSpiReadRegisters(0xC1, 1, &state);
    }while((state&0xFE) != 0x06); /* wait until READY (MC_STATE = 0x03 <<1) */
    
}

void SpiritPutInRX() {
  uint8_t stateC1;

  SpiritSpiCommandStrobes(COMMAND_FLUSHRXFIFO);
  SpiritSpiCommandStrobes(COMMAND_LOCKRX);
  do {
      SpiritSpiReadRegisters(0xC1, 1, &stateC1);
  } while((stateC1&0xFE) != 0x1E); /* wait until LOCK (MC_STATE = 0x0F <<1) */
  
  SpiritSpiCommandStrobes(COMMAND_RX);
  do{
      SpiritSpiReadRegisters(0xC1, 1, &stateC1);
  } while((stateC1&0xFE) != 0x66); /* wait until RX (MC_STATE = 0x33 <<1) */

}

uint8_t SpiritRead(uint8_t n_regs, uint8_t* buffer) {
  uint8_t stateC0;
  uint8_t stateC1;
  uint8_t tmp;
  uint8_t i;

  SpiritSpiReadRegisters(0xC1, 1, &stateC1);
  if ((stateC1&0xFE) == 0x66) return 0; // still in RX

  for (i=0; i<n_regs; i++) {
    SpiritSpiReadRegisters(0xC0, 1, &stateC0);
    if ((stateC0&0x02) == 0x00) { // if fifo not empty
      SpiritSpiReadRegisters(0xFF, 1, &tmp);
      buffer[i] = tmp;
    } 
  }

  SpiritSpiCommandStrobes(COMMAND_FLUSHRXFIFO);
  return i;
}

uint8_t spiritLastRSSI(){
  uint8_t tmp;
  SpiritSpiReadRegisters(0xC8, 1, &tmp);
  return tmp;
}

void SpiritSpiWriteRegisters(uint8_t address, uint8_t n_regs, uint8_t* buffer) {
  uint8_t temp = 0;
    
  digitalWrite(SPSGRF_CS, LOW);
  SPI.beginTransaction(SPISettings(125000, MSBFIRST, SPI_MODE0));

  SPI.transfer(0x00);     //write command
  SPI.transfer(address);
  
  int i;
  for (i = 0; i < n_regs; i++) {
    temp = SPI.transfer(buffer[i]);
    buffer[i] = temp;
  }
  
  SPI.endTransaction();
  digitalWrite(SPSGRF_CS, HIGH);
}

void SpiritSpiReadRegisters(uint8_t address, uint8_t n_regs, uint8_t* buffer){
    
  digitalWrite(SPSGRF_CS, LOW);
  SPI.beginTransaction(SPISettings(125000, MSBFIRST, SPI_MODE0));

  SPI.transfer(0x01);     //read command
  SPI.transfer(address);
  
  int i;
  for (i = 0; i < n_regs; i++) {
    buffer[i] = SPI.transfer(0x00);
  }
  
  SPI.endTransaction();
  digitalWrite(SPSGRF_CS, HIGH);
}

void SpiritSpiCommandStrobes(uint8_t cmd_code) {
    digitalWrite(SPSGRF_CS, LOW);
    SPI.beginTransaction(SPISettings(125000, MSBFIRST, SPI_MODE0));
    SPI.transfer(0x80);
    SPI.transfer(cmd_code);
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    SPI.endTransaction();
    digitalWrite(SPSGRF_CS, HIGH);
}

void WiFiEvent(WiFiEvent_t event) {

    switch (event) {
        case SYSTEM_EVENT_STA_GOT_IP:
            // wifi info
            Serial.println("STA Connected");
            Serial.print("STA IPv4: ");
            Serial.println(WiFi.localIP());
            Serial.print("MAC address: ");
            Serial.println(WiFi.macAddress());
            wifi_connected = true;
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            Serial.println("STA disconnected");
            wifi_connected = false;
            break;
        default:
            break;
    }

}
