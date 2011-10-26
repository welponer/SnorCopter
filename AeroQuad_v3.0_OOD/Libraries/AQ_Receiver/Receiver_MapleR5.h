/*
  AeroQuad Maple32 v3.0 - October 2011
  Copyright (c) 2011 Mattias Welponer.  All rights reserved.
  An Open Source Maple STM32 based multicopter.
 
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

#ifndef _AEROQUAD_RECEIVER_MAPLE_H_
#define _AEROQUAD_RECEIVER_MAPLE_H_

#include <WProgram.h>
#include "Receiver.h"
#include <wirish.h>

#include "Receiver.h"
#include <AQMath.h>
#include <Axis.h>

// ROLL, PITCH, YAW, THROTTLE, MODE, AUX, AUX2, AUX3

typedef struct {
  uint16 riseTime;
  uint16 highTime;
  int edgeMask;
  timer_gen_reg_map *timerRegs;
  __io uint32 *timerCCR;
  int pinNumber;
} PinTiming;

volatile PinTiming pinTiming[8];

void receiverPulesWidth(byte channel) {
  timer_gen_reg_map *timer = pinTiming[channel].timerRegs;
  uint16 c = *(pinTiming[channel].timerCCR);
  bool rising = (timer->CCER & pinTiming[channel].edgeMask) == 0;
  if(rising) {
	pinTiming[channel].riseTime = c;   // rising edge
  } else {
	pinTiming[channel].highTime = c - pinTiming[channel].riseTime;  // falling edge
  }
  timer->CCER ^= pinTiming[channel].edgeMask;  // invert polarity
}

void timerHandler0(void) { 
  receiverPulesWidth(0);
}
void timerHandler1(void) { 
  receiverPulesWidth(1);
}
void timerHandler2(void) { 
  receiverPulesWidth(2);
}
void timerHandler3(void) { 
  receiverPulesWidth(3);
}
void timerHandler4(void) { 
  receiverPulesWidth(4);
}
void timerHandler5(void) { 
  receiverPulesWidth(5);
}
void timerHandler6(void) { 
  receiverPulesWidth(6);
}
void timerHandler7(void) { 
  receiverPulesWidth(7);
}

voidFuncPtr timerHandler[] = { timerHandler0, timerHandler1, timerHandler2, timerHandler3, 
        timerHandler4, timerHandler5, timerHandler6, timerHandler7 };



class Receiver_MapleR5 : public Receiver {
private:
  byte receiverChannels;

public:  
  Receiver_MapleR5() {
    receiverChannels = 4;
  }

  void setTimer(int channel, timer_dev *timerDev, int timerChannel)
  {
    timer_gen_reg_map *timer = timerDev->regs.gen;
	pinTiming[channel].timerRegs = timer;
	pinTiming[channel].timerCCR = &timer->CCR1 + timerChannel;
		
	int timerEnable = (1 << (4*timerChannel));
    //pinTiming[channel].edgeMask = (1 << (4*timerChannel+1));
	pinTiming[channel].edgeMask = timerEnable << 1;
    
	timer->PSC = 72-1;
	timer->ARR = 0xffff;
	timer->CR1 = 0;
	timer->DIER &= ~(1);
	timer->CCER &= ~timerEnable; // disable timer
		
	timer->CCER &= ~(pinTiming[channel].edgeMask);
    if(timerChannel < 2) {
      timer->CCMR1 &= ~(0xFF << (8*(timerChannel & 1)));   // prescaler 1
      timer->CCMR1 |= 0x61<< (8*(timerChannel & 1));// 6=filter, 1=inputs 1,2,3,4
    } else {
      timer->CCMR2 &= ~(0xFF << (8*(timerChannel & 1)));   // prescaler 1
      timer->CCMR2 |= 0x61<< (8*(timerChannel & 1));// 6=filter, 1=inputs 1,2,3,4
    }
	timer->CCER |= timerEnable; // enable timer
	timer->CR1 = 1;
  }

  void initialize(void) {    
    pinTiming[0].pinNumber = 5;  // roll
    pinTiming[1].pinNumber = 9;  // pitch
    pinTiming[2].pinNumber = 14;  // yaw
    pinTiming[3].pinNumber = 24;  // throttle
    pinTiming[4].pinNumber = -1;  // mode
    pinTiming[5].pinNumber = -1;  // aux
    pinTiming[6].pinNumber = -1;  // aux1
    pinTiming[7].pinNumber = -1;  // aux2

    for (int channel = 0; channel < 8; channel++) {
	  int pin = pinTiming[channel].pinNumber;
	  if (pin != -1) {
	    int timerChannel = PIN_MAP[pin].timer_channel; 
	    timer_dev *timerDev = PIN_MAP[pin].timer_device;
	  
	    pinTiming[channel].highTime = 1500;
        pinTiming[channel].riseTime = 0;
        
	    if (timerDev != NULL) {
          pinMode(pin, INPUT);
          setTimer(channel, timerDev, timerChannel-1);
	      timer_attach_interrupt(timerDev, timerChannel, timerHandler[channel]);
	    } 
	  }
    }
    Serial.println("init Receiver_MapleR5: done"); 
  }

  void read(void) {
    Serial.print("reciver: ");
    for(byte channel = ROLL; channel < receiverChannels; channel++) {
      volatile PinTiming *data = &pinTiming[channel];
      unsigned int highTime = data->highTime;
      Serial.print(highTime); Serial.print("/");
      // Apply transmitter calibration adjustment
      receiverData[channel] = (mTransmitter[channel] * highTime) + bTransmitter[channel];
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




