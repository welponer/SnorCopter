


class Copter {
public:
  byte armed;
  byte safetyCheck;
  byte flightMode;
  byte headingHoldConfig;
  byte headingHoldState;
  float G_Dt;
 
 int throttle;
 
  Copter(void) {
    armed = OFF;
    safetyCheck = OFF;
    flightMode = STABLE;
    headingHoldConfig = ON;
    headingHoldState = OFF;
    G_Dt = 0.002;
    
    throttle = 1000;
  }

  void initialize(void) {
 
  }
  
  void initPlatform();
  
  
  void processCopterControl(void) {
    processFlightControl();
  }


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
