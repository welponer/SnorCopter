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

#ifndef _AEROQUAD_RECEIVER_PPM_H_
#define _AEROQUAD_RECEIVER_PPM_H_

#if defined (__AVR_ATmega328P__) || defined(__AVR_ATmegaUNO__)
  #define PPM_PIN_INTERRUPT          attachInterrupt(0, rxInt, RISING); //PIN 0
#else
  #define PPM_PIN_INTERRUPT          attachInterrupt(4, rxInt, RISING);  //PIN 19, also used for Spektrum satellite option
#endif

#include <WProgram.h>
#include "Receiver.h"

#include "pins_arduino.h"
#include <AQMath.h>
#include <Axis.h>


#define ROLL       0
#define PITCH      1
#define YAW        2
#define THROTTLE   3
#define AUX1       4
#define AUX2       5

#define SERIAL_SUM_PPM_1         PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,0,0 //For Graupner/Spektrum
#define SERIAL_SUM_PPM_2         ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,0,0 //For Robe/Hitec/Futaba
#define SERIAL_SUM_PPM_3         PITCH,ROLL,THROTTLE,YAW,AUX1,AUX2,0,0 //For some Hitec/Sanwa/Others

#if defined (SKETCH_SERIAL_SUM_PPM)
  #define SERIAL_SUM_PPM SKETCH_SERIAL_SUM_PPM
#else	
  #define SERIAL_SUM_PPM SERIAL_SUM_PPM_1
#endif

static uint8_t rcChannel[8] = {SERIAL_SUM_PPM};
volatile uint16_t rcValue[8] = {1500,1500,1500,1500,1500,1500,1500,1500}; // interval [1000;2000]

static void rxInt() {
  uint16_t now,diff;
  static uint16_t last = 0;
  static uint8_t chan = 0;

  now = micros();
  diff = now - last;
  last = now;
  if(diff>3000) { 
    chan = 0;
  }
  else {
    if( 900 < diff && diff < 2200 && chan < 8 ) {
	  rcValue[chan] = diff;
	}
    chan++;
  }
}



void initializeReceiver(int nbChannel) {

  initializeReceiverParam(nbChannel);
  PPM_PIN_INTERRUPT
}

void readReceiver() {
  
  for(byte channel = ROLL; channel < lastChannel; channel++) {
    uint8_t oldSREG;
    oldSREG = SREG;
    cli(); // Let's disable interrupts

    receiverData[channel] = (receiverSlope[channel] * rcValue[rcChannel[channel]]) + receiverOffset[channel];

	SREG = oldSREG;
    sei();// Let's enable the interrupts	
	
    // Smooth the flight control receiver inputs
    receiverCommandSmooth[channel] = filterSmooth(receiverData[channel], receiverCommandSmooth[channel], receiverSmoothFactor[channel]);
  }
	
  // Reduce receiver commands using xmitFactor and center around 1500
  for (byte channel = ROLL; channel < lastChannel; channel++)
    if (channel < THROTTLE)
      receiverCommand[channel] = ((receiverCommandSmooth[channel] - receiverZero[channel]) * receiverXmitFactor) + receiverZero[channel];
    else
      // No xmitFactor reduction applied for throttle, mode and
      receiverCommand[channel] = receiverCommandSmooth[channel];
}

#endif



