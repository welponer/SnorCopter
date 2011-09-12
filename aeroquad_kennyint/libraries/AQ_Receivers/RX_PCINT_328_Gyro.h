/******************************************************/

#include <Receiver.h>

/******************************************************/

volatile uint8_t *port_to_pcmask[] = {
  &PCMSK0,
  &PCMSK1
};

volatile static uint8_t PCintLast[2];

// Channel data
typedef struct {
  byte edge;
  unsigned long riseTime;
  unsigned long fallTime;
  unsigned int  lastGoodWidth;
} tPinTimingData;
volatile static tPinTimingData pinData[5];

// ISR which records time of rising or falling edge of signal
static void measurePulseWidthISR(uint8_t port) {
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint32_t currentTime;
  uint32_t time;

  // get the pin states for the indicated port.
  curr = *portInputRegister(port+2);
  mask = curr ^ PCintLast[port];
  PCintLast[port] = curr;
  // mask is pins that have changed. screen out non pcint pins.
  if ((mask &= *port_to_pcmask[port]) == 0) {
    return;
  }
  currentTime = micros();
  // mask is pcint pins that have changed.
  for (uint8_t i=0; i < 5; i++) {
    bit = 0x01 << i;
    if (bit & mask) {
      // for each pin changed, record time of change
      if (bit & PCintLast[port]) {
        time = currentTime - pinData[i].fallTime;
        pinData[i].riseTime = currentTime;
        if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
          pinData[i].edge = RISING_EDGE;
        else
          pinData[i].edge = FALLING_EDGE; // invalid rising edge detected
      }
      else {
        time = currentTime - pinData[i].riseTime;
        pinData[i].fallTime = currentTime;
        if ((time >= MINONWIDTH) && (time <= MAXONWIDTH) && (pinData[i].edge == RISING_EDGE)) {
          pinData[i].lastGoodWidth = time;
          pinData[i].edge = FALLING_EDGE;
        }
      }
    }
  }
}

ISR(PCINT0_vect, ISR_BLOCK) {
  measurePulseWidthISR(0); // PORT B
}

ISR(PCINT1_vect, ISR_BLOCK) {
  measurePulseWidthISR(1); // PORT C
}

// defines arduino pins used for receiver in arduino pin numbering schema
static byte   receiverPin[LASTCHANNEL] = {1, 2, 3, 0, 4}; // index used for ROLL, PITCH, YAW, THROTTLE, MODE

void initializeReceiver() {
  DDRB   &= 0xEE;  // Set pins PB4 and PB0 as inputs
  DDRC   &= 0xF1;  // Set pins PC3, PC2, and PC1 as inputs

  PORTB  |= 0x11;  // Internal pullups for PB4 and PB0 on
  PORTC  |= 0x0E;  // Internal pullups fot PC3, PC2, and PC1 on

  PCMSK0 |= 0x11;  // Enable PCINT 4 and 1
  PCMSK1 |= 0x0E;  // Enable PCINT 11, 10, and 9

  PCICR  |= 0x03;  // Enable PCINT 1 and PCINT 0

  for (byte channel = ROLL; channel < LASTCHANNEL; channel++) {
    pinData[receiverPin[channel]].edge = FALLING_EDGE;
  }
}

void readReceiver(void) {
  for(byte channel = ROLL; channel < LASTCHANNEL; channel++) {
    byte pin = receiverPin[channel];
    uint8_t oldSREG = SREG;
    cli();
    // Get receiver value read by pin change interrupt handler
    uint16_t lastGoodWidth = pinData[pin].lastGoodWidth;
    SREG = oldSREG;

    receiverData[channel] = lastGoodWidth;
  }
  receiverData[ROLL]  -= MIDCOMMAND;
  receiverData[PITCH] -= MIDCOMMAND;
  receiverData[YAW]   -= MIDCOMMAND;
}

