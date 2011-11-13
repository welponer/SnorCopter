


class Copter {
public:
  byte armed;
  byte safetyCheck;
  byte flightMode;
  byte headingHoldConfig;
  byte headingHoldState;
  float G_Dt;
 
  Copter(void) {
    armed = OFF;
    safetyCheck = OFF;
    flightMode = STABLE;
    headingHoldConfig = ON;
    headingHoldState = OFF;
    G_Dt = 0.002;
  }

  void initialize(void) {
 
  }
  
  void initPlatform();
  
  
  void processCopterControl(void) {
    processFlightControl();
  }


};


void hundredHz(void) {};

void tenHz(void) {};

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

void (*scheduleHandler)(void);


  FrameTimer(void) {
    previousTime = 0;
    currentTime = micros();
    deltaTime = 0;
    frameCounter = 0;
 //   (*timerHandler)[0] = &hundredHz;
 //   (*timerHandler)[1] = &tendHz; 
  }
  
  void update(void) { 
    currentTime = micros();
    deltaTime = currentTime - previousTime;
    
    // Main scheduler loop set for 100hz
    //if (deltaTime >= 10000) {
    if (deltaTime >= 100000) {
      frameCounter++;
      
      if( frameCounter % 1 == 0) {  //  100 Hz tasks
SerialUSB.print(".");
      }
      
      if( frameCounter % 10 == 0) {  //   10 Hz tasks
SerialUSB.println("+");
      }
      
      previousTime = currentTime;
    }
    if (frameCounter >= 1000) {
      frameCounter = 0;
    }    
  }

};
