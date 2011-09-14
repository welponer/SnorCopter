


#define LED1 0
#define LED2 1
#define LED2 2
#define LED3 3
#define BUZZER 128

#define LEDPIN0 A1
#define LEDPIN1 A3
#define LEDPIN2 A5
#define LEDPIN3 A7
#define BUZZERPIN 0
#define BUZZERGND 0



class StatusSignal {
private:
  byte pinStatus[5];


public:
  StatusSignal( void) {
    for (byte i = 0; i<4; i++) ledStatus[i] = 0;
    buzzerStatus = 0;
  }

  void initialize( void) {
    Serial.println("init StatusSignal");
  }

  void update( void) {
    
  }
  
  void setTiming(byte source, long on, long off) {
  
  }
  
  void setStatus(byte source, byte status = 1) {
    if (status > 1) status = HIGH;
    Serial.println(status, DEC);
  }

  void setLow(byte source) { 
    setStatus(source, 255);
  }

};
