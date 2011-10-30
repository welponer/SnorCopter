


class Copter {
public:
//  byte flightMode;
//  byte headingHoldConfig;
//float levelAdjust[2];

  Copter(void) {
    flightMode = STABLE;
    headingHoldConfig = ON;
  }

  void initPlatform();

  void initialize(void) {
    levelAdjust[ROLL] = 0.0;
    levelAdjust[PITCH] = 0.0;
  }
  
  void processCopterControl(void) {
    processFlightControl();
  }


/*

  #define ATTITUDE_SCALING (0.75 * PWM2RAD)

  void calculateFlightError(void)
  {
    if (flightMode == ACRO) {
      motorAxisCommandRoll = updatePID(receiver->getSIData(ROLL), gyro->getRadPerSec(ROLL), &PID[ROLL]);
      motorAxisCommandPitch = updatePID(receiver->getSIData(PITCH), -gyro->getRadPerSec(PITCH), &PID[PITCH]);
    }
    else {
      float rollAttitudeCmd = updatePID((receiver->getData(ROLL) - receiver->getZero(ROLL)) * ATTITUDE_SCALING, kinematics->getData(ROLL), &PID[LEVELROLL]);
      float pitchAttitudeCmd = updatePID((receiver->getData(PITCH) - receiver->getZero(PITCH)) * ATTITUDE_SCALING, -kinematics->getData(PITCH), &PID[LEVELPITCH]);
      motorAxisCommandRoll = updatePID(rollAttitudeCmd, gyro->getRadPerSec(ROLL), &PID[LEVELGYROROLL]);
      motorAxisCommandPitch = updatePID(pitchAttitudeCmd, -gyro->getRadPerSec(PITCH), &PID[LEVELGYROPITCH]);
    }
  }

void processFlightControl() {
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    applyMotorCommand();
  } 

  // *********************** process min max motor command *******************
  processMinMaxCommand();

  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers
//  processHardManuevers();
  
  // Apply limits to motor commands
  for (byte motor = 0; motor < LASTMOTOR; motor++) {
    motors->setMotorCommand(motor, constrain(motors->getMotorCommand(motor), motorMinCommand[motor], motorMaxCommand[motor]));
  }

  // If throttle in minimum position, don't apply yaw
  if (receiver->getData(THROTTLE) < MINCHECK) {
    for (byte motor = 0; motor < LASTMOTOR; motor++) {
      motors->setMotorCommand(motor, MINTHROTTLE);
    }
  }

  // ESC Calibration
  if (armed == OFF) {
    processCalibrateESC();
  }

  // *********************** Command Motors **********************
  if (armed == ON && safetyCheck == ON) {
    motors->write();
  }
}

*/

};
