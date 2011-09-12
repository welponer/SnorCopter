#define HMC5843

#define COMPASS_ADDRESS 0x3C

/******************************************************/

float magBias[3];
int   magRaw[3];
float magVector[3];
int   magVectorRaw[3];
byte  newMagData = 0;

/******************************************************/

void readCompass() {
  twiMaster.start(COMPASS_ADDRESS | I2C_WRITE);
  twiMaster.write(0x03);
  twiMaster.start(COMPASS_ADDRESS | I2C_READ);
  magRaw[XAXIS] =  (twiMaster.read(0)<<8 | twiMaster.read(0));
  magRaw[YAXIS] = -(twiMaster.read(0)<<8 | twiMaster.read(0));
  magRaw[ZAXIS] = -(twiMaster.read(0)<<8 | twiMaster.read(1));
  
  //twiMaster.stop();  // Not included here, just once at end of 400 Hz ISR to avoid duplication
}

/******************************************************/

void initializeCompass() { 
  delay(100);
  
  twiMaster.start(COMPASS_ADDRESS | I2C_WRITE);
  twiMaster.write(0x00);                         // Configuration Resgister A
  twiMaster.write(0x18);                         // 50 Hz Update Rate

  delay(50);
  
  twiMaster.start(COMPASS_ADDRESS | I2C_WRITE);
  twiMaster.write(0x02);                         // Mode Register
  twiMaster.write(0x00);                         // Continuous Conversion

  delay(50);
  
  readCompass();
  
  twiMaster.stop();
  
  delay(50);
}

/******************************************************/
