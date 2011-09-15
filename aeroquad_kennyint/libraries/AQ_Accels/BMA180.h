#define BMA180

#define ACCEL_ADDRESS   0x80

/******************************************************/

#include <Accel.h>

/******************************************************/

void readAccelAndSumForAverage() {
  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x02);
  twiMaster.start(ACCEL_ADDRESS | I2C_READ);

  for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
    rawAccel.bytes[axis*2]   = twiMaster.read(0);
    rawAccel.bytes[axis*2+1] = twiMaster.read((axis*2+1) == 5);
    accelSum[axis] += rawAccel.value[axis]>>2;
  }
}

/******************************************************/

void computeAccelBias() {
  cli();
  for (int samples = 0; samples < 800; samples++) {
    readAccelAndSumForAverage();
    delayMicroseconds(2500);
  }

  for (byte axis = 0; axis < 3; axis++) {
    accel.value[axis] = (float(accelSum[axis])/800) * accelScaleFactor[axis];
    accelSum[axis] = 0;
  }

  runTimeAccelBias[XAXIS] = -accel.value[XAXIS];
  runTimeAccelBias[YAXIS] = -accel.value[YAXIS];
  runTimeAccelBias[ZAXIS] = -9.8065 - accel.value[ZAXIS];

  oneG = accel.value[ZAXIS] + runTimeAccelBias[ZAXIS];
  sei();
}

/******************************************************/

void initializeAccel(void) {
  byte data;

  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x10);
  twiMaster.write(0xB6);  // Reset device

  delay(10);

  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x0D);
  twiMaster.write(0x10);  // Enable writting to control registers

  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x20);  // Register bw_tcs (bits 4-7)

  twiMaster.start(ACCEL_ADDRESS | I2C_READ);
  data = twiMaster.read(1);
  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x20);
  twiMaster.write(data & 0x7F);  // Set low pass filter to 1200 Hz (value = 0111xxxx)

  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x35);

  twiMaster.start(ACCEL_ADDRESS | I2C_READ);
  data = twiMaster.read(1);
  data &= 0xF1;
  data |= 0x08;

  twiMaster.start(ACCEL_ADDRESS | I2C_WRITE);
  twiMaster.write(0x35);
  twiMaster.write(data);  // Set range select bits for +/- 4g

  delay(100);

  computeAccelBias();
}

/******************************************************/

