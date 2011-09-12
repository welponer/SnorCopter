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
  unsigned int  lastGoodWidth;
} tPinTimingData;
volatile static tPinTimingData pinData[9];

// Attaches PCINT to Arduino Pin
void attachPinChangeInterrupt(uint8_t pin) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT) {
    return;
  }
  else {
    port -= 2;
    pcmask = port_to_pcmask[port];
  }
  // set the mask
  *pcmask |= bit;
  // enable the interrupt
  PCICR |= 0x01 << port;
}

// ISR which records time of rising or falling edge of signal
static void measurePulseWidthISR(uint8_t port, uint8_t pinoffset) {
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint8_t pin;
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
  for (uint8_t i=0; i < 8; i++) {
    bit = 0x01 << i;
    if (bit & mask) {
      pin = pinoffset + i;
      // for each pin changed, record time of change
      if (bit & PCintLast[port]) {
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

ISR(PCINT0_vect, ISR_BLOCK) {
  measurePulseWidthISR(0, 8); // PORT B
}

ISR(PCINT2_vect, ISR_BLOCK) {
  measurePulseWidthISR(2, 0); // PORT D
}

// defines arduino pins used for receiver in arduino pin numbering schema
static byte receiverPin[6] = {2, 5, 6, 4, 7, 8}; // pins used for ROLL, PITCH, YAW, THROTTLE, MODE, AUX


void initializeReceiver() {
  for (byte channel = ROLL; channel < LASTCHANNEL; channel++) {
    pinMode(receiverPin[channel], INPUT);
    pinData[receiverPin[channel]].edge = FALLING_EDGE;
    attachPinChangeInterrupt(receiverPin[channel]);
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

