


#define LEDPIN0 A1
#define LEDPIN1 A3
#define LEDPIN2 A5
#define LEDPIN3 A7
#define BUZZERPIN 0
#define BUZZERGND 0



class StatusSignal {
private:
  byte ledStatus[4];
  byte buzzerStatus;

public:
  StatusSignal( void) {
    for (byte i = 0; i<4; i++) ledStatus[i] = 0;
    buzzerStatus = 0;
  }

  void initialize( void) {
    
  }

  void update( void) {
  
  }


};
