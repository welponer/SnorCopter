#define HMC5883

#define COMPASS_ADDRESS 0x3C

/******************************************************/

#include <Magnetometer.h>

/******************************************************/

void readCompass() {
  twiMaster.start(COMPASS_ADDRESS | I2C_WRITE);
  twiMaster.write(0x03);
  twiMaster.start(COMPASS_ADDRESS | I2C_READ);

  rawMag.bytes[3] = twiMaster.read(0);  // Y axis MSB
  rawMag.bytes[2] = twiMaster.read(0);  // Y axis LSB
  rawMag.bytes[5] = twiMaster.read(0);  // Z axis MSB
  rawMag.bytes[4] = twiMaster.read(0);  // Z axis LSB
  rawMag.bytes[1] = twiMaster.read(0);  // X axis MSB
  rawMag.bytes[0] = twiMaster.read(1);  // X axis LSB

  rawMag.value[ZAXIS] = -rawMag.value[ZAXIS];
}

/******************************************************/

void initializeCompass() {
  delay(100);

  twiMaster.start(COMPASS_ADDRESS | I2C_WRITE);
  twiMaster.write(0x00);                         // Configuration Resgister A
  twiMaster.write(0x18);                         // 75 Hz Update Rate

  delay(20);

  twiMaster.start(COMPASS_ADDRESS | I2C_WRITE);
  twiMaster.write(0x02);                         // Mode Register
  twiMaster.write(0x00);                         // Continuous Conversion

  delay(20);

  readCompass();
  mag.value[XAXIS] = float(rawMag.value[XAXIS]) + magBias[XAXIS];
  mag.value[YAXIS] = float(rawMag.value[YAXIS]) + magBias[YAXIS];
  mag.value[ZAXIS] = float(rawMag.value[ZAXIS]) + magBias[ZAXIS];

  delay(20);
}

/******************************************************/
