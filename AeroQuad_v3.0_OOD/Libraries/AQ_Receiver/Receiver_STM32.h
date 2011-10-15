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

#include <WProgram.h>
#include "Receiver.h"

#define MINONWIDTH 950
#define MAXONWIDTH 2075

#include "Receiver.h"
#include <AQMath.h>
#include <Axis.h>

ROLL, PITCH, YAW, THROTTLE, MODE, AUX, AUX2, AUX3
static byte receiverPin[8] = {5, 3, 2, 4, 1, 0, 0, 0};  

typedef struct {
  unsigned int beginTime;
  unsigned int endTime;
  unsigned int lastGood;
} PinTiming;

volatile static PinTiming pinData[8];

volatile unsigned int chan1begin   = 0;
volatile unsigned int chan1end     = 0;
volatile unsigned int RxInPitch    = 0;
volatile unsigned int chan1prev    = 0;


class Receiver_STM32 : public Receiver {
public:  
  Receiver_STM32() {
  }

  void initialize(void) {
    
    for (byte channel = ROLL; channel < LASTCHANNEL; channel++) {
      pinData[channel].beginTime = 0;
      pinData[channel].endTime = 0;
      pinData[channel].lastGood = MINCOMMAND;
    }
  }

  void read(void) {
    for(byte channel = ROLL; channel < LASTCHANNEL; channel++) {
      byte pin = receiverPin[channel];
 
 
 
 
 
      // Apply transmitter calibration adjustment
  //    receiverData[channel] = (mTransmitter[channel] * lastGoodWidth) + bTransmitter[channel];
      // Smooth the flight control transmitter inputs
      transmitterCommandSmooth[channel] = filterSmooth(receiverData[channel], transmitterCommandSmooth[channel], transmitterSmooth[channel]);
    }

    // Reduce transmitter commands using xmitFactor and center around 1500
    for (byte channel = ROLL; channel < THROTTLE; channel++)
      transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel]) * xmitFactor) + transmitterZero[channel];
    // No xmitFactor reduction applied for throttle, mode and AUX
    for (byte channel = THROTTLE; channel < LASTCHANNEL; channel++)
      transmitterCommand[channel] = transmitterCommandSmooth[channel];
  }
  
};
#endif




