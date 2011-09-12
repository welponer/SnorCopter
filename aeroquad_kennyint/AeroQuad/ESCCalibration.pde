//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processCalibrateESC //////////////////////////////
//////////////////////////////////////////////////////////////////////////////


//boolean escsCalibrating = OFF;
//
//void processCalibrateESC(void)
//{
//  switch (calibrateESC) { // used for calibrating ESC's
//  case 1:
//    SERIAL_PRINTLN("Setting MAXCOMMAND");
//    for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
//      motorCommand[motor] = MAXCOMMAND;
//    break;
//  case 3:
//    SERIAL_PRINTLN("Setting Motor Test Command");
//    for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
//      motorCommand[motor] = constrain(testCommand, 1000, 1200);
//    break;
//  case 5:
//    SERIAL_PRINTLN("Setting Individual Motor COmmands");
//    for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
//      motorCommand[motor] = constrain(remoteMotorCommand[motor], 1000, 1200);
//    safetyCheck = ON;
//    break;
//  default:
//    for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
//      motorCommand[motor] = MINCOMMAND;
//  }
//  // Send calibration commands to motors
//  writeMotors(); // Defined in Motors.h
//}

