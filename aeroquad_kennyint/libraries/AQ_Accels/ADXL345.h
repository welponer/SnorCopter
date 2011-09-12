#define ADXL345

#define ACCEL_ADDRESS   0xA6

/******************************************************/

#include <Accel.h>

/******************************************************/

void readAccel() {
  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x32);
  twiMaster.start(ACCEL_ADDRESS | I2C_READ);

  for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
    rawAccel.bytes[axis*2]   = twiMaster.read(0);
    rawAccel.bytes[axis*2+1] = twiMaster.read((axis*2+1) == 5);
    accelSum[axis] += rawAccel.value[axis];
  }
}

/******************************************************/

void computeAccelBias() {
  cli();
  for (int samples = 0; samples < 800; samples++) {
    readAccel();
    delayMicroseconds(2500);
  }

  for (byte axis = 0; axis < 3; axis++) {
    accel.value[axis] = (float(accelSum[axis])/800) * accelScaleFactor[axis];
    accelSum[axis] = 0;
  }

  runTimeAccelBias[XAXIS] =  -accel.value[XAXIS];
  runTimeAccelBias[YAXIS] =  -accel.value[YAXIS];
  runTimeAccelBias[ZAXIS] = -9.8065 - accel.value[ZAXIS];

  oneG = accel.value[ZAXIS] + runTimeAccelBias[ZAXIS];
  sei();
}

/******************************************************/

void initializeAccel(void) {
  byte data;

  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x2D);
  twiMaster.write(1<<3);  // Set device to measure

  delay(10);

  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x31);
  twiMaster.write(0x09);  // Set full range and +/- 4G

  delay(10);

  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x2C);
  twiMaster.write(0x0D);  // Set 800 Hz sampling

  delay(100);

  computeAccelBias();
}

/******************************************************/


