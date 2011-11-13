


class Copter {
public:
  byte flightMode;
  byte headingHoldConfig;
 
  Copter(void) {
    flightMode = STABLE;
    headingHoldConfig = ON;
  }

  void initialize(void) {
 
  }
  
  void initPlatform();
  
  
  void processCopterControl(void) {
    processFlightControl();
  }


};
