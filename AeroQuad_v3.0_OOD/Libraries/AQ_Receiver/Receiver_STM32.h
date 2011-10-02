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


#define RISING_EDGE 1
#define FALLING_EDGE 0
#define MINONWIDTH 950
#define MAXONWIDTH 2075
#define MINOFFWIDTH 12000
#define MAXOFFWIDTH 24000

#include "Receiver.h"
#include <AQMath.h>
#include <Axis.h>


#if defined (__AVR_ATmega1280__)
  // arduino pins 67, 65, 64, 66, 63, 62
  static byte receiverPin[6] = {5, 3, 2, 4, 1, 0}; // bit number of PORTK used for ROLL, PITCH, YAW, THROTTLE, MODE, AUX
#else
 //arduino pins 63, 64, 65, 62, 66, 67
  static byte receiverPin[6] = {1, 2, 3, 0, 4, 5}; // bit number of PORTK used for ROLL, PITCH, YAW, THROTTLE, MODE, AUX
#endif




class Receiver_STM32 : public Receiver {
public:  
  Receiver_STM32() {
  }

  void initialize(void) {
 
 //   for (byte channel = ROLL; channel < LASTCHANNEL; channel++)
 //     pinData[receiverPin[channel]].edge = FALLING_EDGE;
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




