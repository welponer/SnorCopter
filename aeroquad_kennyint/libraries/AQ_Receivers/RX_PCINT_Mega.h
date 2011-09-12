/******************************************************/

#include <Receiver.h>

/******************************************************/

volatile uint8_t *port_to_pcmask[] = {
  &PCMSK0,
  &PCMSK1,
  &PCMSK2
};

volatile static uint8_t PCintLast[3];

// Channel data
typedef struct {
  byte edge;
  unsigned long riseTime;
  unsigned long fallTime;
  unsigned int lastGoodWidth;
} tPinTimingData;
volatile static tPinTimingData pinData[LASTCHANNEL];

ISR(PCINT2_vect, ISR_BLOCK) {
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint8_t pin;
  uint32_t currentTime;
  uint32_t time;

  curr = *portInputRegister(11);
  mask = curr ^ PCintLast[0];
  PCintLast[0] = curr;

  // mask is pins that have changed. screen out non pcint pins.
  if ((mask &= PCMSK2) == 0) {
    return;
  }

  currentTime = micros();

  // mask is pcint pins that have changed.
  for (uint8_t i=0; i < 8; i++) {
    bit = 0x01 << i;
    if (bit & mask) {
      pin = i;
      // for each pin changed, record time of change
      if (bit & PCintLast[0]) {
        time = currentTime - pinData[pin].fallTime;
        pinData[pin].riseTime = currentTime;
        if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
          pinData[pin].edge = RISING_EDGE;
        else
          pinData[pin].edge = FALLING_EDGE; // invalid rising edge detected
      }
      else {
        time = currentTime - pinData[pin].riseTime;
        pinData[pin].fallTime = currentTime;
        if ((time >= MINONWIDTH) && (time <= MAXONWIDTH) && (pinData[pin].edge == RISING_EDGE)) {
          pinData[pin].lastGoodWidth = time;
          pinData[pin].edge = FALLING_EDGE;
        }
      }
    }
  }
}

static byte receiverPin[6] = {1, 2, 3, 0, 4, 5}; // bit number of PORTK used for ROLL, PITCH, YAW, THROTTLE, MODE, AUX

void initializeReceiver() {
  DDRK = 0;
  PORTK = 0x3F;
  PCMSK2 |= 0x3F;
  PCICR |= 0x1 << 2;

  for (byte channel = ROLL; channel < LASTCHANNEL; channel++) {
    //pinMode(receiverPin[channel], INPUT);
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

