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

#include <WProgram.h>
#include "Receiver.h"

#define RISING_EDGE 1
#define FALLING_EDGE 0
#define MINONWIDTH 950
#define MAXONWIDTH 2075
#define MINOFFWIDTH 12000
#define MAXOFFWIDTH 24000

#if defined (__AVR_ATmega328P__) || defined(__AVR_ATmegaUNO__)


#include "pins_arduino.h"
#include <AQMath.h>
#include <Axis.h>


#define ROLL       0
#define PITCH      1
#define YAW        2
#define THROTTLE   3
#define AUX1       4
#define AUX2       5

#define SERIAL_SUM_PPM         PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2 //For Graupner/Spektrum

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
//    #if defined(FAILSAFE)
//      if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;   // clear FailSafe counter - added by MIS
//    #endif
  }
}



class Receiver_PPM : public Receiver {
private:
  
  
public:  
  Receiver_PPM() {
  }

  void initialize(void) {
    attachInterrupt(0, rxInt, RISING);
  }

  void read(void) {
  
    
	for(byte channel = ROLL; channel < LASTCHANNEL; channel++) {
	  uint8_t oldSREG;
      oldSREG = SREG;
      cli(); // Let's disable interrupts

	  receiverData[channel] = (mTransmitter[channel] * rcValue[rcChannel[channel]]) + bTransmitter[channel];
	  
	  SREG = oldSREG;
      sei();// Let's enable the interrupts	
	
      // Smooth the flight control transmitter inputs
      transmitterCommandSmooth[channel] = filterSmooth(receiverData[channel], transmitterCommandSmooth[channel], transmitterSmooth[channel]);
	}
	
	
	
    // Reduce transmitter commands using xmitFactor and center around 1500
    for (byte channel = ROLL; channel < LASTCHANNEL; channel++)
      if (channel < THROTTLE)
        transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel]) * xmitFactor) + transmitterZero[channel];
      else
        // No xmitFactor reduction applied for throttle, mode and
        transmitterCommand[channel] = transmitterCommandSmooth[channel];
		
	
  }
};

#endif

#endif



