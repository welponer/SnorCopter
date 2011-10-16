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
#define MAXONWIDTH 2050
#define MINOFFWIDTH 12000
#define MAXOFFWIDTH 24000

#include "Receiver.h"
#include <AQMath.h>
#include <Axis.h>

// ROLL, PITCH, YAW, THROTTLE, MODE, AUX, AUX2, AUX3

typedef struct {
  unsigned int beginTime;
  unsigned int lastGood;
  byte pinNumber;
} PinTiming;

volatile static PinTiming pinData[8];



void receiverPulesWidth(byte channel) {
  unsigned int deltaTime;
  if( pinData[channel].beginTime == 0) {
    pinData[channel].beginTime = micros(); 
  } else {
    deltaTime = micros() - pinData[channel].beginTime;
    if((deltaTime <= MAXONWIDTH) && (deltaTime >= MINONWIDTH)) 
    {
      pinData[channel].lastGood = deltaTime;
    } 
    pinData[channel].beginTime = 0;
    
    if (deltaTime > MINOFFWIDTH) 
      pinData[channel].beginTime += deltaTime;
  }  
}

void receiverHandler0(void) { 
  receiverPulesWidth(0);
}

void receiverHandler1(void) { 
  receiverPulesWidth(1);
}
void receiverHandler2(void) { 
  receiverPulesWidth(2);
}
void receiverHandler3(void) { 
  receiverPulesWidth(3);
}
void receiverHandler4(void) { 
  receiverPulesWidth(4);
}
void receiverHandler5(void) { 
  receiverPulesWidth(5);
}
void receiverHandler6(void) { 
  receiverPulesWidth(6);
}
void receiverHandler7(void) { 
  receiverPulesWidth(7);
}


class Receiver_MapleR5 : public Receiver {
private:
  byte receiverChannels;

public:  
  Receiver_MapleR5() {
    receiverChannels = 7;
  }

  void initialize(void) {    
    pinData[0].pinNumber = 31;  // roll (5V)
    pinData[1].pinNumber = 32;  // pitch (5V)
    pinData[2].pinNumber = 33;  // yaw (5V)
    pinData[3].pinNumber = 34;  // throttle (5V)
    pinData[4].pinNumber = 35;  // mode (5V)
    pinData[5].pinNumber = 36;  // aux (5V)
    pinData[6].pinNumber = 37;  // aux1 (5V)
    pinData[7].pinNumber = 1000;  // aux2 (5V)
    
    for (byte channel = ROLL; channel < THROTTLE; channel++) {
      pinData[channel].beginTime = 0;
      pinData[channel].lastGood = MIDCOMMAND;
      pinMode(pinData[channel].pinNumber, INPUT);
    }
    for (byte channel = THROTTLE; channel < receiverChannels; channel++) {
      pinData[channel].beginTime = 0;
      pinData[channel].lastGood = MINCOMMAND;
      pinMode(pinData[channel].pinNumber, INPUT);
    }
    
    attachInterrupt(pinData[0].pinNumber, receiverHandler0, CHANGE);
    attachInterrupt(pinData[1].pinNumber, receiverHandler1, CHANGE);
    attachInterrupt(pinData[2].pinNumber, receiverHandler2, CHANGE);
    attachInterrupt(pinData[3].pinNumber, receiverHandler3, CHANGE);
    attachInterrupt(pinData[4].pinNumber, receiverHandler4, CHANGE);
    if (receiverChannels > 5) {
      attachInterrupt(pinData[5].pinNumber, receiverHandler5, CHANGE);
      attachInterrupt(pinData[6].pinNumber, receiverHandler6, CHANGE);
    }
    if (receiverChannels > 7) {
      attachInterrupt(pinData[7].pinNumber, receiverHandler7, CHANGE);
    }    
  }

  void read(void) {
  Serial.print("reciver: ");
    for(byte channel = ROLL; channel < receiverChannels; channel++) {
    Serial.print(pinData[channel].lastGood); Serial.print("/");
      // Apply transmitter calibration adjustment
      receiverData[channel] = (mTransmitter[channel] * pinData[channel].lastGood) + bTransmitter[channel];
      // Smooth the flight control transmitter inputs
      transmitterCommandSmooth[channel] = filterSmooth(receiverData[channel], transmitterCommandSmooth[channel], transmitterSmooth[channel]);
    }
Serial.println("");
    // Reduce transmitter commands using xmitFactor and center around 1500
    for (byte channel = ROLL; channel < THROTTLE; channel++)
      transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel]) * xmitFactor) + transmitterZero[channel];
    // No xmitFactor reduction applied for throttle, mode and AUX
    for (byte channel = THROTTLE; channel < receiverChannels; channel++)
      transmitterCommand[channel] = transmitterCommandSmooth[channel];
  }
  
};
#endif




