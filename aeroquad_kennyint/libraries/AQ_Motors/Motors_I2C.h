#define I2C_ESC

/******************************************************/

#include <Motors.h>

/******************************************************/

byte motorCommandI2C[LASTMOTOR];

/******************************************************/

#define MOTORBASE 0x52

/*  Motor  I2C Address (7 Bit, PhiFun I2C Controller)
    0       0x52
    1       0x54
    2       0x56
    3       0x58
    4       0x5A
    5       0x5C
*/

/******************************************************/

void initializeMotors(void) {
  for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
  {
     twiMaster.start((MOTORBASE + (motor * 2)) | I2C_WRITE);
     twiMaster.write(0);
  }
}

/******************************************************/

void writeMotors(void) {
  for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
	motorCommandI2C[motor] = constrain((motorCommand[motor] - 1000) / 4, 0, 250);
}

/******************************************************/