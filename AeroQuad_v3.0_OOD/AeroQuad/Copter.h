
#define ATTITUDE_SCALING (0.75 * PWM2RAD)

class Copter {
public:
  float G_Dt;

  Copter(void) {
    G_Dt = 0.002; 
  }

  void initialize(void) {

  }
  
  void initPlatform();
  
  

};




class FlightControl {
public:
  Motors *m;
  
  byte armed;
  byte safetyCheck;
  byte flightMode;
  
  int throttle;
  int motorAxisCommandRoll;
  int motorAxisCommandPitch;
  int motorAxisCommandYaw;
  
  byte headingHoldConfig;
  byte headingHoldState;
  float headingHold; // calculated adjustment for quad to go to heading (PID output)
  float heading; // measured heading from yaw gyro (process variable)
  float relativeHeading; // current heading the quad is set to (set point)
  float setHeading;
  unsigned long headingTime;

  FlightControl(Motors *mo) {
    m = mo;
    
    armed = OFF;
    safetyCheck = OFF;
    
    flightMode = STABLE;
    
    throttle = 1000;
    motorAxisCommandRoll = 0;
    motorAxisCommandPitch = 0;
    motorAxisCommandYaw = 0;
    
    headingHoldConfig = ON;
    headingHoldState = OFF;
    headingHold = 0;
    heading = 0;
    relativeHeading = 0;
    setHeading = 0;
    headingTime = micros();
  }
  
  void calculateError(void) {
 
  }
  
  void processControl() {

  }

  void processHeading(void) {
 
  }
  
  void processAltitudeHold(void) {

  }
  
  virtual void applyMotorCommand(void) {}
  void processMinMaxCommand(void) {}
  void processHardManuevers(void) {}
};


class FlightControlX4 : public FlightControl {
public:
  int motorMaxCommand[4];
  int motorMinCommand[4];
  
  FlightControlX4(Motors *mo) : FlightControl(mo) {
  
  }
};

class FlightControlP4 : public FlightControl {
public:
  int motorMaxCommand[4];
  int motorMinCommand[4];
  
  FlightControlP4(Motors *mo) : FlightControl(mo) {
   
  }

  void applyMotorCommand() {
    const int throttleCorrection = abs(motorAxisCommandYaw*2/4);
 /*   motor->motorCommand[0] = (throttle - throttleCorrection) - motorAxisCommandPitch - (YAW_DIRECTION * motorAxisCommandYaw);
    motor->motorCommand[1] = (throttle - throttleCorrection) + motorAxisCommandPitch - (YAW_DIRECTION * motorAxisCommandYaw);
    motor->motorCommand[2] = (throttle - throttleCorrection) - motorAxisCommandRoll  + (YAW_DIRECTION * motorAxisCommandYaw);
    motor->motorCommand[3] = (throttle - throttleCorrection) + motorAxisCommandRoll  + (YAW_DIRECTION * motorAxisCommandYaw);
 */ }
 
};


class FlightControlY6 : public FlightControl {
public:
  int motorMaxCommand[6];
  int motorMinCommand[6];
  
  FlightControlY6(Motors *mo) : FlightControl(mo) {
   
  }
  void applyMotorCommand() {
    const int throttleCorrection = abs(motorAxisCommandYaw*3/6);
  /*  motor->motorCommand[0] = (throttle - throttleCorrection)                        + (motorAxisCommandPitch*4/3) - (YAW_DIRECTION * motorAxisCommandYaw);
    motor->motorCommand[1] = (throttle - throttleCorrection) - motorAxisCommandRoll - (motorAxisCommandPitch*2/3) + (YAW_DIRECTION * motorAxisCommandYaw);  
    motor->motorCommand[2] = (throttle - throttleCorrection) + motorAxisCommandRoll - (motorAxisCommandPitch*2/3) - (YAW_DIRECTION * motorAxisCommandYaw);
    motor->motorCommand[3] = (throttle - throttleCorrection)                        + (motorAxisCommandPitch*4/3) + (YAW_DIRECTION * motorAxisCommandYaw);
    motor->motorCommand[4] = (throttle - throttleCorrection) - motorAxisCommandRoll - (motorAxisCommandPitch*2/3) - (YAW_DIRECTION * motorAxisCommandYaw);
    motor->motorCommand[5] = (throttle - throttleCorrection) + motorAxisCommandRoll - (motorAxisCommandPitch*2/3) + (YAW_DIRECTION * motorAxisCommandYaw);
 */ }

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



