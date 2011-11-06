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

#ifndef _AEROQUAD_RECEIVER_APM_H_
#define _AEROQUAD_RECEIVER_APM_H_

#include <WProgram.h>
#include "Receiver.h"
#include <APM_RC.h>

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#include <AQMath.h>
#include <Axis.h>
#include <APM_RC.h>

int receiverPin[6] = {0,0,0,0,0,0};
  
void initializeReceiver(int nbChannel = 6) {

  initializeReceiverParam(nbChannel);
  receiverPin[ROLL] = 0;
  receiverPin[PITCH] = 1;
  receiverPin[YAW] = 3;
  receiverPin[THROTTLE] = 2;
  receiverPin[MODE] = 4;
  receiverPin[AUX] = 5;
}

void readReceiver() {

  for(byte channel = ROLL; channel < lastChannel; channel++) {
    // Apply receiver calibration adjustment
    receiverData[channel] = (receiverSlope[channel] * ((readReceiverChannel(receiverPin[channel])))) + receiverOffset[channel];
    // Smooth the flight control receiver inputs
    receiverCommandSmooth[channel] = filterSmooth(receiverData[channel], receiverCommandSmooth[channel], receiverSmoothFactor[channel]);
  }

  // Reduce receiver commands using receiverXmitFactor and center around 1500
  for (byte channel = ROLL; channel < THROTTLE; channel++)
    receiverCommand[channel] = ((receiverCommandSmooth[channel] - receiverZero[channel]) * receiverXmitFactor) + receiverZero[channel];
  // No receiverXmitFactor reduction applied for throttle, mode and
  for (byte channel = THROTTLE; channel < lastChannel; channel++)
    receiverCommand[channel] = receiverCommandSmooth[channel];
}


  
void setChannelValue(byte channel,int value) {
}
  
#endif

#endif


