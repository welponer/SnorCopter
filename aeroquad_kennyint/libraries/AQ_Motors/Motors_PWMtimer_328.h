/******************************************************/

#include <Motors.h>

/******************************************************/

/*  Motor  328 Pin Port
    0        3     PD3/OC2B
    1        9     PB1/OC1A
    2       10     PB2/OC1B
    3       11     PB3/OC2A
*/

#define PWM_FREQUENCY 244   // in Hz
#define PWM_PRESCALER 256
#define PWM_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_FREQUENCY - 1)

/******************************************************/

void initializeMotors(void) {
  DDRB = DDRB | B00001110;                                  // Set ports to output PB1-3
  DDRD = DDRD | B00001000;                                  // Set port to output PD3

  commandAllMotors(1000);                                   // Initialize motors to 1000us (stopped)

  // Init PWM Timer 1  16 bit
  TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);              // COM1A1,COM1A0 = 10 -> Clear OC1A on Compare Match (Set output to low level)
                                                            // COM1B1,COM1B0 = 10 -> Clear OC1B on Compare Match (Set output to low level)

  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS12);                 // WGM13,WGM12,WGM11,WGM10 = 1110 -> Fast PWM, TOP = ICR1, Update OCR1x at BOTTOM
                                                            // CS12,CS11,CS10 = 100 -> Prescaler set to 256
  ICR1 = PWM_COUNTER_PERIOD;                                // ICR1 = 255 = 16000000/256/244 - 1
                                                            // Output_PWM_Frequency = 244 Hz = 16000000/(256*(1 + 255) = Clock_Speed / (Prescaler * (1 + TOP)

  // Init PWM Timer 2   8bit
  TCCR2A = (1<<WGM20)|(1<<WGM21)|(1<<COM2A1)|(1<<COM2B1);   // COM2A1,COM2A0 = 10 -> Clear OC2A on Compare Match, set OC2A at BOTTOM, (non-inverting mode)
                                                            // COM2B1,COM2B0 = 10 -> Clear OC2B on Compare Match, set OC2B at BOTTOM, (non-inverting mode)
                                                            // WGM22,WGM21,WGM20 = 011 -> Fast PWM, Update of OCRx at BOTTOM
  TCCR2B = (1<<CS22)|(1<<CS21);                             // CS22,CS21,CS20 = 110 -> Prescaler set to 256
  // TOP is fixed at 255                                    // Output_PWM_Frequency = 244Hz = 16000000/(256*256) = Clock_Speed / (Prescaler * 256)
}

/******************************************************/

void writeMotors(void) {
  OCR2B = motorCommand[0] / 16;
  OCR1A = motorCommand[1] / 16;
  OCR1B = motorCommand[2] / 16;
  OCR2A = motorCommand[3] / 16;
}

/******************************************************/