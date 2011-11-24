
#define ATTITUDE_SCALING (0.75 * PWM2RAD)

class Copter {
public:
  float G_Dt;
  int* mmm;
 
  Copter(void) {

    
    G_Dt = 0.002;
    

  }

  void initialize(void) {
   mmm = (int*)malloc( 5);
  }
  
  void initPlatform();
  
  

};




class FlightControl {
public:
  byte armed;
  byte safetyCheck;
  
  byte flightMode;
  
  int throttle;
  int motorAxisCommandRoll;
  int motorAxisCommandPitch;
  int motorAxisCommandYaw;
  
  byte headingHoldConfig;
  byte headingHoldState;
  
  FlightControl(void) {
    armed = OFF;
    safetyCheck = OFF;
    
    flightMode = STABLE;
    
    throttle = 1000;
    motorAxisCommandRoll = 0;
    motorAxisCommandPitch = 0;
    motorAxisCommandYaw = 0;
    
    headingHoldConfig = ON;
    headingHoldState = OFF;
  }
  
  void calculateError(void) {
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
  
  void processControl() {
    calculateError();
  
    processHeading();


  if (armed && safetyCheck) {
    applyMotorCommand();
  } 

  processMinMaxCommand();

  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers
  if (flightMode == ACRO) {
    processHardManuevers();
  }
/* 
  // Apply limits to motor commands
  for (byte motor = 0; motor < motors->motorChannels; motor++) {
    motors->setMotorCommand(motor, constrain(motors->getMotorCommand(motor), motorMinCommand[motor], motorMaxCommand[motor]));
  }
*/
  motors->constrainMotors();
  
  // If throttle in minimum position, don't apply yaw
  if (receiver->getData(THROTTLE) < MINCHECK) {
    for (byte motor = 0; motor < motors->motorChannels; motor++) {
      motors->setMotorCommand(motor, MINTHROTTLE);
    }
  }

  // *********************** Command Motors **********************
  if (armed == ON && safetyCheck == ON) {
    motors->write();
  }
  }

  void processHeading(void) {
    if (headingHoldConfig == ON) {

    #if defined(HeadingMagHold)
      heading = degrees(kinematics->getHeading(YAW));
    #else
      heading = degrees(gyro->getHeading());
    #endif

    // Always center relative heading around absolute heading chosen during yaw command
    // This assumes that an incorrect yaw can't be forced on the AeroQuad >180 or <-180 degrees
    // This is done so that AeroQuad does not accidentally hit transition between 0 and 360 or -180 and 180
    // AKA - THERE IS A BUG HERE - if relative heading is greater than 180 degrees, the PID will swing from negative to positive
    // Doubt that will happen as it would have to be uncommanded.
    relativeHeading = heading - setHeading;
    if (heading <= (setHeading - 180)) relativeHeading += 360;
    if (heading >= (setHeading + 180)) relativeHeading -= 360;

    // Apply heading hold only when flight->throttle high enough to start flight
    if (receiver->getData(throttle) > MINCHECK ) { 
      if ((receiver->getData(YAW) > (MIDCOMMAND + 25)) || (receiver->getData(YAW) < (MIDCOMMAND - 25))) {
        // If commanding yaw, turn off heading hold and store latest heading
        setHeading = heading;
        headingHold = 0;
        PID[HEADING].integratedError = 0;
        headingHoldState = OFF;
        headingTime = currentTime;
      }
      else {
        if (relativeHeading < .25 && relativeHeading > -.25) {
          headingHold = 0;
          PID[HEADING].integratedError = 0;
        }
        else if (headingHoldState == OFF) { // quick fix to soften heading hold on new heading
          if ((currentTime - headingTime) > 500000) {
            headingHoldState = ON;
            headingTime = currentTime;
            setHeading = heading;
            headingHold = 0;
          }
        }
        else {
        // No new yaw input, calculate current heading vs. desired heading heading hold
        // Relative heading is always centered around zero
          headingHold = updatePID(0, relativeHeading, &PID[HEADING]);
          headingTime = currentTime; // quick fix to soften heading hold, wait 100ms before applying heading hold
        }
      }
    }
    else {
      // minimum flight->throttle not reached, use off settings
      setHeading = heading;
      headingHold = 0;
      PID[HEADING].integratedError = 0;
    }
  }
  // NEW SI Version
  float commandedYaw = constrain(receiver->getSIData(YAW) + radians(headingHold), -PI, PI);
  motorAxisCommandYaw = updatePID(commandedYaw, gyro->getRadPerSec(YAW), &PID[YAW]);
  // uses flightAngle unbias rate
  //motors->setMotorAxisCommand(YAW, updatePID(commandedYaw, flightAngle->getGyroUnbias(YAW), &PID[YAW]));
  }
  
  void processAltitudeHold(void) {
    // ****************************** Altitude Adjust *************************
  // Thanks to Honk for his work with altitude hold
  // http://aeroquad.com/showthread.php?792-Problems-with-BMP085-I2C-barometer
  // Thanks to Sherbakov for his work in Z Axis dampening
  // http://aeroquad.com/showthread.php?359-Stable-flight-logic...&p=10325&viewfull=1#post10325
#ifdef AltitudeHold
  if (altitudeHold == ON) {
    throttleAdjust = updatePID(holdAltitude, barometricSensor->getAltitude(), &PID[ALTITUDE]);
    //flight->throttleAdjust = constrain((holdAltitude - altitude.getData()) * PID[ALTITUDE].P, minflight->throttleAdjust, maxflight->throttleAdjust);
    throttleAdjust = constrain(flight->throttleAdjust, minflight->throttleAdjust, maxflight->throttleAdjust);
    if (abs(holdflight->throttle - receiver->getData(flight->throttle)) > PANICSTICK_MOVEMENT) {
      altitudeHold = ALTPANIC; // too rapid of stick movement so PANIC out of ALTHOLD
    } else {
      if (receiver->getData(flight->throttle) > (holdflight->throttle + ALTBUMP)) { // AKA changed to use holdflight->throttle + ALTBUMP - (was MAXCHECK) above 1900
        holdAltitude += 0.01;
      }
      if (receiver->getData(flight->throttle) < (holdflight->throttle - ALTBUMP)) { // AKA change to use holdThorrle - ALTBUMP - (was MINCHECK) below 1100
        holdAltitude -= 0.01;
      }
    }
  }
  else {
    // Altitude hold is off, get flight->throttle from receiver
    holdthrottle = receiver->getData(THROTTLE);
    throttleAdjust = autoDescent; // autoDescent is lowered from BatteryMonitor.h during battery alarm
  }
  // holdThrottle set in FlightCommand.pde if altitude hold is on
  throttle = holdThrottle + throttleAdjust; // holdThrottle is also adjust by BatteryMonitor.h during battery alarm
#else
  // If altitude hold not enabled in AeroQuad.pde, get throttle from receiver
  throttle = receiver->getData(THROTTLE) + autoDescent; //autoDescent is lowered from BatteryMonitor.h while battery critical, otherwise kept 0
#endif
  }
  
  virtual void applyMotorCommand(void) {}
  virtual void processMinMaxCommand(void) {}
  virtual void processHardManuevers(void) {}
};


class FlightControlX4 : public FlightControl {
public:
  #include <FlightControlQuadXMode.h>
};

class FlightControlY6 : public FlightControl {
public:
  #include <FlightControlHexY6.h>
};

















void hundredHz(void) { Serial.print("."); };

void tenHz(void) { Serial.print("x"); };

class FrameTimer {
public:  
  unsigned long frameCounter;
  
unsigned long previousTime;
unsigned long currentTime;
unsigned long deltaTime;


unsigned long oneHZpreviousTime;
unsigned long tenHZpreviousTime;
unsigned long twentyFiveHZpreviousTime;
unsigned long fiftyHZpreviousTime;
unsigned long hundredHZpreviousTime; 



  void (*scheduleHandler[8])(void);
  unsigned long previousTimeHandler[8];

  FrameTimer(void) {
    previousTime = 0;
    currentTime = micros();
    deltaTime = 0;
    frameCounter = 0;
    
    for( int i = 0; i < 8; i++) {
      scheduleHandler[i] = NULL;
      previousTimeHandler[i] = 0;
    }
    
    scheduleHandler[0] = hundredHz;
    scheduleHandler[1] = tenHz;
    
  }
  
  void update(void) { 
    currentTime = micros();
    deltaTime = currentTime - previousTime;
    
    // Main scheduler loop set for 100hz
    //if (deltaTime >= 10000) {
    if (deltaTime >= 100000) {
      frameCounter++;
      
      if( frameCounter % 1 == 0) {  //  100 Hz tasks
  scheduleHandler[0]();
      }
      
      if( frameCounter % 10 == 0) {  //   10 Hz tasks
  scheduleHandler[1]();
      }
      
      previousTime = currentTime;
    }
    if (frameCounter >= 1000) {
      frameCounter = 0;
    }    
  }

};



