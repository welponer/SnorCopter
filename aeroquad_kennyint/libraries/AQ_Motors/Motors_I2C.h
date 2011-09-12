#define I2C_ESC

/******************************************************/

#include <Motors.h>

/******************************************************/

byte motorCommandI2C[LASTMOTOR];
byte sendMotorCommands = 0;

/******************************************************/

#define MOTORBASE 0x29

/******************************************************/

void commandAllMotors(int motorCommand) {
  byte temp = constrain((motorCommand - 1000) / 4, 0, 250);
  for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
	motorCommandI2C[motor] = temp;
  sendMotorCommands = 1;
}

/******************************************************/

void initializeMotors(void) {
   for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
 	motorCommandI2C[motor] = 0;
  sendMotorCommands = 1;
}

/******************************************************/

void writeMotors(void) {
  for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
	motorCommandI2C[motor] = constrain((motorCommand[motor] - 1000) / 4, 0, 250);
  sendMotorCommands = 1;
}

/******************************************************/

void pulseMotors(byte quantity) {
  for (byte i = 0; i < quantity; i++) {
  commandAllMotors(MINCOMMAND + 100);
  delay(250);
  commandAllMotors(MINCOMMAND);
  delay(250);
  }
}

/******************************************************/
