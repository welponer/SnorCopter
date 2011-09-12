#define ITG3200

#ifdef AeroQuad_Mini_6DOF
  #define GYRO_ADDRESS   0xD0
#else
  #define GYRO_ADDRESS   0xD2
#endif

#define gyroScaleFactor radians(1.0/14.375)  //  ITG3200 14.375 LSBs per °/sec


/******************************************************/

#include <Gyro.h>

/******************************************************/

void readGyro() {
  twiMaster.start(GYRO_ADDRESS | I2C_WRITE);
  twiMaster.write(0x1D);
  twiMaster.start(GYRO_ADDRESS | I2C_READ);

  for (byte axis = ROLL; axis < LASTAXIS; axis++) {
    rawGyro.bytes[axis*2+1] = twiMaster.read(0);
    rawGyro.bytes[axis*2]   = twiMaster.read((axis*2+1) == 5);
    gyroSum[axis] += rawGyro.value[axis];
  }
}

/******************************************************/

void computeGyroBias() {
  cli();
  for (int samples = 0; samples < 800; samples++) {
    readGyro();
    delayMicroseconds(2500);
  }

  for (byte axis = ROLL; axis < 3; axis++) {
    runTimeGyroBias[axis] = (float(gyroSum[axis])/800);
    gyroSum[axis] = 0;
  }
  sei();
}

/******************************************************/

void initializeGyro(void) {
  twiMaster.start(GYRO_ADDRESS | I2C_WRITE);  // send a reset to the device
  twiMaster.write(0x3E);
  twiMaster.write(0x80);

  twiMaster.start(GYRO_ADDRESS | I2C_WRITE);  // use internal oscillator
  twiMaster.write(0x3E);
  twiMaster.write(0x01);

  twiMaster.start(GYRO_ADDRESS | I2C_WRITE);  // internal sample rate 8000 Hz
  twiMaster.write(0x16);                      // 256 Hz low pass filter
  twiMaster.write(0x18);                      // +/- 2000 DPS range

  twiMaster.start(GYRO_ADDRESS | I2C_WRITE);  // sample rate divisor
  twiMaster.write(0x15);                      // 500 Hz = 8000 Hz/(divider+1)
  twiMaster.write(0x0F);                      // divider = 15

  twiMaster.start(GYRO_ADDRESS | I2C_WRITE);  // int active high
  twiMaster.write(0x17);                      // push-pull drive
  twiMaster.write(0x31);                      // latched until cleared
                                              // clear upon any register read
  delay(100);                                 // enable interrupt when data is available

  computeGyroBias();
}

/******************************************************/
