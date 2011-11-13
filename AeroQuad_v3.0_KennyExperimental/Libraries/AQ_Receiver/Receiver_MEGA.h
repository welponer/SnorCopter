	/*
  AeroQuad v3.0 - May 2011
  www.AeroQuad.com
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef _AEROQUAD_RECEIVER_MEGA_H_
#define _AEROQUAD_RECEIVER_MEGA_H_


#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#include <WProgram.h>
#include "Receiver.h"

#define RISING_EDGE 1
#define FALLING_EDGE 0
#define MINONWIDTH 950
#define MAXONWIDTH 2075
#define MINOFFWIDTH 12000
#define MAXOFFWIDTH 24000

#include "pins_arduino.h"
#include <AQMath.h>
#include <Axis.h>

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
volatile static tPinTimingData pinData[9];

static void MegaPcIntISR() {
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

SIGNAL(PCINT2_vect) {
  MegaPcIntISR();
}

// #ifdef AeroQuadMega_v1 is undefined in libraries,  @todo Kenny find another solution for this!
#if defined (__AVR_ATmega1280__) 
  // arduino pins 67, 65, 64, 66, 63, 62
  static byte receiverPin[6] = {5, 3, 2, 4, 1, 0}; // bit number of PORTK used for ROLL, PITCH, YAW, THROTTLE, MODE, AUX
#else
  //arduino pins 63, 64, 65, 62, 66, 67
  static byte receiverPin[8] = {1, 2, 3, 0, 4, 5, 6, 7}; // bit number of PORTK used for ROLL, PITCH, YAW, THROTTLE, MODE, AUX
#endif

void initializeReceiver(int nbChannel = 6) {

  initializeReceiverParam(nbChannel);
  
  DDRK = 0;
  PORTK = 0;
  PCMSK2 |= 0xFF;
  PCICR |= 0x1 << 2;

  for (byte channel = ROLL; channel < lastChannel; channel++)
    pinData[receiverPin[channel]].edge = FALLING_EDGE;
}

void readReceiver() {
  
  for(byte channel = ROLL; channel < lastChannel; channel++) {
    byte pin = receiverPin[channel];
    uint8_t oldSREG = SREG;
    cli();
    // Get receiver value read by pin change interrupt handler
    uint16_t lastGoodWidth = pinData[pin].lastGoodWidth;
    SREG = oldSREG;

    // Apply receiver calibration adjustment
    receiverData[channel] = (receiverSlope[channel] * lastGoodWidth) + receiverOffset[channel];
    // Smooth the flight control receiver inputs
    receiverCommandSmooth[channel] = filterSmooth(receiverData[channel], receiverCommandSmooth[channel], receiverSmoothFactor[channel]);
  }

  // Reduce receiver commands using receiverXmitFactor and center around 1500
  for (byte channel = ROLL; channel < THROTTLE; channel++)
    receiverCommand[channel] = ((receiverCommandSmooth[channel] - receiverZero[channel]) * receiverXmitFactor) + receiverZero[channel];
  // No xmitFactor reduction applied for throttle, mode and AUX
  for (byte channel = THROTTLE; channel < lastChannel; channel++)
    receiverCommand[channel] = receiverCommandSmooth[channel];
}
  
void setChannelValue(byte channel,int value) {
}  

#endif

#endif



