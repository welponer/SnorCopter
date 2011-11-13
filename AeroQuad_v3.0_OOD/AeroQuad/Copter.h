


class Copter {
public:
  byte armed;
  byte safetyCheck;
  byte flightMode;
  byte headingHoldConfig;
  byte headingHoldState;
 
  Copter(void) {
    armed = OFF;
    safetyCheck = OFF;
    flightMode = STABLE;
    headingHoldConfig = ON;
    headingHoldState = OFF;
  }

  void initialize(void) {
 
  }
  
  void initPlatform();
  
  
  void processCopterControl(void) {
    processFlightControl();
  }


};


class FrameTimer {
  unsigned long frameCounter;
  
unsigned long previousTime;
unsigned long currentTime;
unsigned long deltaTime;


unsigned long oneHZpreviousTime;
unsigned long tenHZpreviousTime;
unsigned long twentyFiveHZpreviousTime;
unsigned long fiftyHZpreviousTime;
unsigned long hundredHZpreviousTime; 

  FrameTimer(void) {
    previousTime = 0;
    currentTime = micros();
    deltaTime = 0;
    frameCounter = 0;
  
  }
  
  void update(void) { 
    currentTime = micros();
    deltaTime = currentTime - previousTime;
    // Main scheduler loop set for 100hz
    //if (deltaTime >= 10000) {
    if (deltaTime >= 100000) {
      frameCounter++;
      
      if( frameCounter %   1 == 0) {  //  100 Hz tasks

      }
      
      if( frameCounter %  10 == 0) {  //   10 Hz tasks

      }
      
      previousTime = currentTime;
    }
    if (frameCounter >= 1000) {
      frameCounter = 0;
    }    
  }

};
