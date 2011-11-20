
#define ATTITUDE_SCALING (0.75 * PWM2RAD)

class Copter {
public:
  byte armed;
  byte safetyCheck;
  byte flightMode;
  byte headingHoldConfig;
  byte headingHoldState;
  float G_Dt;
 
  int throttle;
  // defined in FlightControl.pde Flight control needs
  int motorAxisCommandRoll;
  int motorAxisCommandPitch;
  int motorAxisCommandYaw;

 
  Copter(void) {
    armed = OFF;
    safetyCheck = OFF;
    flightMode = STABLE;
    headingHoldConfig = ON;
    headingHoldState = OFF;
    G_Dt = 0.002;
    
    throttle = 1000;
    motorAxisCommandRoll = 0;
    motorAxisCommandPitch = 0;
    motorAxisCommandYaw = 0;
  }

  void initialize(void) {
 
  }
  
  void initPlatform();
  
  

};


class FlightControl {
public:

void process() {
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Calculate Motor Commands *************************
  if (copter->armed && copter->safetyCheck) {
    applyMotorCommand();
  } 

  // *********************** process min max motor command *******************
  processMinMaxCommand();

  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers
  if (copter->flightMode == ACRO) {
    processHardManuevers();
  }
  
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

/*
  // ESC Calibration
  if (armed == OFF) {
    processCalibrateESC();
  }
*/

  // *********************** Command Motors **********************
  if (copter->armed == ON && copter->safetyCheck == ON) {
    motors->write();
  }
}




};


class FlightControlQuadX4 : public FlightControl {
public:


};

















void hundredHz(void) { SerialUSB.print("."); };

void tenHz(void) { SerialUSB.print("x"); };

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
