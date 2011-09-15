/******************************************************/

#include <Motors.h>

/******************************************************/

#if (LASTMOTOR == 6)
  uint8_t PWM_MOTOR4PIN_lowState;
  uint8_t PWM_MOTOR4PIN_highState;
  uint8_t PWM_MOTOR5PIN_lowState;
  uint8_t PWM_MOTOR5PIN_highState;
#endif

/*  Motor  328 Pin Port
    0        3     PD3/OC2B
    1        9     PB1/OC1A
    2       10     PB2/OC1B
    3       11     PB3/OC2A
    4        5     TIMER0_COMPA
    5        6     TIMER0_COMPB
*/

#define PWM_FREQUENCY 244   // in Hz
#define PWM_PRESCALER 256
#define PWM_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_FREQUENCY)

/******************************************************/

void initializeMotors(void) {
  DDRB |= B00001110;                                  // Set ports to output PB1-3
  DDRD |= B00001000;                                  // Set port to output PD3

  #if (LASTMOTOR == 6)
    DDRD |= B01100000;                                // Set ports to out put PD5-6
  #endif

  commandAllMotors(1000);                                   // Initialize motors to 1000us (stopped)

  // Init PWM Timer 1  16 bit
  TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS12);
  ICR1 = PWM_COUNTER_PERIOD;
  // Init PWM Timer 2   8bit                               // WGMn1 WGMn2 = Mode ? Fast PWM, TOP = 0xFF ,Update of OCRnx at BOTTOM
  TCCR2A = (1<<WGM20)|(1<<WGM21)|(1<<COM2A1)|(1<<COM2B1);  // Clear OCnA/OCnB on compare match, set OCnA/OCnB at BOTTOM (non-inverting mode)
  TCCR2B = (1<<CS22)|(1<<CS21);                            // Prescaler set to 256, that gives us a resolution of 16us
  // TOP is fixed at 255                                   // Output_PWM_Frequency = 244hz = 16000000/(256*(1+255)) = Clock_Speed / (Prescaler * (1 + TOP))

  #if (LASTMOTOR == 6)
    TCCR0A = 0;            // normal counting mode
    TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
    TIMSK0 |= (1<<OCIE0B);
  #endif
}

/******************************************************/

void writeMotors(void) {
  OCR2B = motorCommand[0] / 16;
  OCR1A = motorCommand[1] / 16;
  OCR1B = motorCommand[2] / 16;
  OCR2A = motorCommand[3] / 16;
  #if (LASTMOTOR == 6)
    PWM_MOTOR4PIN_highState = int(motorCommand[4] / 8) * 2;
	PWM_MOTOR4PIN_lowState  = 255 - PWM_MOTOR4PIN_highState;
	PWM_MOTOR5PIN_highState = int(motorCommand[5] / 8) * 2;
    PWM_MOTOR5PIN_lowState  = 255 - PWM_MOTOR5PIN_highState;
  #endif
}

/******************************************************/

#if (LASTMOTOR ==6)
  ISR(TIMER0_COMPA_vect) {
    static uint8_t state = 0;
    if (state == 0) {
      PORTD |= 1<<5;                    // digital PIN 5 high
      OCR0A+= PWM_MOTOR4PIN_highState;  // 250 x 4 microseconds = 1ms
      state = 1;
    } else if (state == 1) {
      OCR0A+= PWM_MOTOR4PIN_highState;
      state = 2;
    } else if (state == 2) {
      PORTD &= ~(1<<5);                 // digital PIN 5 low
      OCR0A+= PWM_MOTOR4PIN_lowState;
      state = 0;
    }
  }

  ISR(TIMER0_COMPB_vect) {              // the same with digital PIN 6 and OCR0B counter
    static uint8_t state = 0;
    if (state == 0) {
      PORTD |= 1<<6;
      OCR0B+= PWM_MOTOR5PIN_highState;
      state = 1;
    } else if (state == 1) {
      OCR0B+= PWM_MOTOR5PIN_highState;
      state = 2;
    } else if (state == 2) {
      PORTD &= ~(1<<6);
      OCR0B+= PWM_MOTOR5PIN_lowState;
      state = 0;
    }
  }
#endif

/******************************************************/