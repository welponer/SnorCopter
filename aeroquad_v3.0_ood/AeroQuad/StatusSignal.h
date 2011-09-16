


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

#define LASTSTATUS 1


class StatusSignal {
private:
  boolean statusStatus[LASTSTATUS];
  byte statusHighTime[LASTSTATUS];
  byte statusLowTime[LASTSTATUS];
  byte statusTime[LASTSTATUS];
  byte  statusPin[LASTSTATUS];
  
public:
  StatusSignal( void) {
    for (byte i = 0; i < LASTSTATUS; i++) {
      statusStatus[i] = true;
      statusLowTime[i] = 10;
      statusHighTime[i] = 50;
      statusTime[i] = 0;
    }
  }

  void initialize( void) {
    Serial.println("init StatusSignal");
  }

  void update( void) {
     for (byte i = 0; i < LASTSTATUS; i++) {
       statusTime[i]++;
       if (statusStatus[i]) {
         if (statusTime[i] > statusHighTime[i]) {
         statusStatus[i] = false;
         statusTime[i] = 0;
         Serial.println("0");
         }
       } else {
         if (statusTime[i] > statusLowTime[i]) {
         statusStatus[i] = true;
         statusTime[i] = 0;
         Serial.println("1");
         }
       }
     }
  }
  
  void setPin(byte index, byte pin) {
    statusPin[index] = pin;
  }
  
  void setTiming(byte index, byte lowTime, byte highTime) {
    statusLowTime[index] = lowTime;
    statusHighTime[index] = highTime;
    statusStatus[index] = false;
  }
  
  void setStatus(byte index, boolean status = false) {
    setTiming(index, -1, -1);
    statusStatus[index] = status;
    Serial.println(status, DEC);
  }


};
