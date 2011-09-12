/******************************************************/

#include <Motors.h>

/******************************************************/

/*  Motor   Mega Pin Port
      0           2  PE4/OC3B
      1           3  PE5/OC3C
      2           5  PE3/OC3A
      3           6  PH3/OC4A
      4           7  PH4/OC4B
      5           8  PH5/OC4C
      6          11  PB5/OC1A
      7          12  PB6/OC1B
*/

#define PWM_FREQUENCY 300   // in Hz
#define PWM_PRESCALER 8
#define PWM_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_FREQUENCY)

/******************************************************/

void commandAllMotors(int _motorCommand) {   // Sends commands to all motors
  #if (LASTMOTOR >= 4)
    OCR3B = _motorCommand * 2 ;
    OCR3C = _motorCommand * 2 ;
    OCR3A = _motorCommand * 2 ;
    OCR4A = _motorCommand * 2 ;
  #endif

  #if (LASTMOTOR >= 6)
    OCR4B = _motorCommand * 2 ;
    OCR4C = _motorCommand * 2 ;
  #endif

  #if (LASTMOTOR == 8)
    OCR1A = _motorCommand * 2 ;
    OCR1B = _motorCommand * 2 ;
  #endif
}

/******************************************************/

void initializeMotors(void) {
  #if (LASTMOTOR >= 4)
    DDRE = DDRE | B00111000;                                  // Set PE4, PE5, and PE3 ports to output
    DDRH = DDRH | B00001000;                                  // Set PH3 port to output
  #endif

  #if (LASTMOTOR >= 6)
    DDRH = DDRH | B00110000;                                  // Set PH4, and PH5 ports to output
  #endif

  #if (LASTMOTOR == 8)
    DDRB = DDRB | B01100000;                                  // Set PB5, and PB6 ports to output
  #endif

  commandAllMotors(1000);                                     // Initialise motors to 1000us (stopped)

  // Init PWM Timer 3                                         // WGMn1 WGMn2 WGMn3  = Mode 14 Fast PWM, TOP = ICRn ,Update of OCRnx at BOTTOM
  TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);    // Clear OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (non-inverting mode)
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);                   // Prescaler set to 8, that gives us a resolution of 0.5us
  ICR3 = PWM_COUNTER_PERIOD;                                  // Clock_speed / ( Prescaler * desired_PWM_Frequency) #defined above.
  // Init PWM Timer 4
  TCCR4A = (1<<WGM41)|(1<<COM4A1);
  TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
  ICR4 = PWM_COUNTER_PERIOD;
  // Init PWM Timer 1
  TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1);
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
  ICR4 = PWM_COUNTER_PERIOD;
}

/******************************************************/

void writeMotors(void) {
  #if (LASTMOTOR >= 4)
    OCR3B = motorCommand[0] * 2;
    OCR3C = motorCommand[1] * 2;
    OCR3A = motorCommand[2] * 2;
    OCR4A = motorCommand[3] * 2;
  #endif

  #if (LASTMOTOR >= 6)
      OCR4B = motorCommand[4] * 2 ;
      OCR4C = motorCommand[5] * 2 ;
    #endif

    #if (LASTMOTOR == 8)
      OCR1A = motorCommand[6] * 2 ;
      OCR1B = motorCommand[7] * 2 ;
  #endif

}

/******************************************************/

void pulseMotors(byte quantity) {
  for (byte i = 0; i < quantity; i++) {
  commandAllMotors(MINCOMMAND + 100);
  delay(250);
  commandAllMotors(MINCOMMAND);
  delay(250);
  }
}

/******************************************************/
